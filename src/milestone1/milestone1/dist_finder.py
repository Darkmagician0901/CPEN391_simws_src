"""
Distance finder node for wall following.

Subscribes to the LiDAR scan and odometry, measures the car's distance to
the wall and publishes the error for the pid node.

Error sign convention (side-independent):
    error > 0  ->  the car should steer LEFT  (positive steering angle)
    error < 0  ->  the car should steer RIGHT
    right wall: error = desired - D_next
    left wall:  error = D_next - desired
So the pid node can simply do: steering_angle = Kp*e + Ki*∫e + Kd*de/dt,
without knowing which wall is being followed.

All tuning values are ROS 2 parameters, set in config/params.yaml.
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


class DistFinder(Node):
    """
    Compute the wall-following error from LiDAR data.

    Subscription:
        scan_topic (sensor_msgs/LaserScan): LiDAR ranges.
        odom_topic (nav_msgs/Odometry): forward speed, used to scale L.

    Publication:
        error_topic (std_msgs/Float64): steering error in meters
            (see module docstring for the sign convention).

    Params:
        wall_side (str): 'left' or 'right', the wall to follow.
        desired_distance (float, m): target distance to the wall.
        lookahead_time (float, s): L = speed * lookahead_time, how far ahead
            to project the distance.
        min_lookahead, max_lookahead (float, m): bounds on L.
        theta (float, deg): angle between beam b (perpendicular to the car)
            and beam a (tilted toward the front). Must be in (0, 90).
        neighbor_count (int): beams on each side to average when a reading
            is invalid.
    """

    def __init__(self):
        """Declare parameters and set up the subscriber and publisher."""
        super().__init__('dist_finder')

        # Topics
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('error_topic', '/wall_error')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')

        # Defaults only; real values come from config/params.yaml
        self.declare_parameter('wall_side', 'left')
        self.declare_parameter('desired_distance', 1.0)
        self.declare_parameter('theta', 50.0)
        self.declare_parameter('neighbor_count', 2)
        self.declare_parameter('lookahead_time', 0.5)
        self.declare_parameter('min_lookahead', 0.5)
        self.declare_parameter('max_lookahead', 2.0)

        scan_topic = self.get_parameter('scan_topic').value
        error_topic = self.get_parameter('error_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self.scan_subscription = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10)
        self.odom_subscription = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        self.error_publisher = self.create_publisher(Float64, error_topic, 10)

        self.speed = 0.0

    def odom_callback(self, msg):
        """
        Store the current forward speed of the car.

        Args:
            msg (nav_msgs.msg.Odometry): the odometry message.

        Side effects:
            Updates self.speed (m/s, forward is positive).
        """
        self.speed = msg.twist.twist.linear.x

    def get_lookahead(self):
        """
        Compute the lookahead distance L from the current speed.

        Returns:
            float: speed * lookahead_time, clipped to
                [min_lookahead, max_lookahead], in meters.
        """
        lookahead_time = float(self.get_parameter('lookahead_time').value)
        min_lookahead = float(self.get_parameter('min_lookahead').value)
        max_lookahead = float(self.get_parameter('max_lookahead').value)

        lookahead = abs(self.speed) * lookahead_time
        return min(max(lookahead, min_lookahead), max_lookahead)

    def get_range(self, msg, angle):
        """
        Return the range measurement at a given angle.

        Args:
            msg (sensor_msgs.msg.LaserScan): current LiDAR scan message.
            angle (float): angle in RADIANS (0 = straight ahead, positive = left).

        Returns:
            float: range in meters, or inf if no valid reading nearby.
        """
        if angle < msg.angle_min or angle > msg.angle_max:
            raise ValueError(
                f"Angle {angle} rad is out of bounds [{msg.angle_min}, {msg.angle_max}]")

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        # Index of the beam closest to the requested angle
        index = int(round((angle - msg.angle_min) / msg.angle_increment))
        index = min(max(index, 0), len(ranges) - 1)

        valid_reading = lambda r: np.isfinite(r) and msg.range_min <= r <= msg.range_max

        if valid_reading(ranges[index]):
            return float(ranges[index])

        # Use neighbors if the exact beam is invalid
        neighbor_count = int(self.get_parameter('neighbor_count').value)
        low = max(0, index - neighbor_count)
        high = min(len(ranges), index + neighbor_count + 1)
        neighbor_ranges = [ranges[i] for i in range(low, high)
                           if i != index and valid_reading(ranges[i])]
        if not neighbor_ranges:
            return float('inf')
        return float(np.mean(neighbor_ranges))

    def get_error(self, msg, dist):
        """
        Calculate the steering error from two LiDAR beams.

        Geometry (shown for the right wall; the left wall is the mirror image):
            b: beam perpendicular to the car
            a: beam range corresponding to the given theta
            alpha  = atan((a*cos(theta) - b) / (a*sin(theta)))
                     heading angle of the car relative to the wall,
                     positive = pointing away from the wall.
            D_t    = b * cos(alpha)            current distance to the wall
            D_next = D_t + L * sin(alpha)      distance after driving L ahead

        Args:
            msg (sensor_msgs.msg.LaserScan): current LiDAR scan message.
            dist (float): desired distance to the wall in meters.

        Returns:
            float or None: error in meters, or None if the beams are invalid.
        """
        wall_side = self.get_parameter('wall_side').value
        lookahead_distance = self.get_lookahead()
        theta = math.radians(float(self.get_parameter('theta').value))

        if not 0.0 < theta < math.pi / 2:
            self.get_logger().error('theta must be between 0 and 90 degrees')
            return None

        # Use side coefficient to pick the correct angle
        if wall_side == 'left':
            side = 1.0
        elif wall_side == 'right':
            side = -1.0
        else:
            self.get_logger().error(f"wall_side must be 'left' or 'right', got {wall_side}")
            return None

        # b is on the y axis of the car (±90°), a is theta closer to the front
        b = self.get_range(msg, side * math.pi / 2)
        a = self.get_range(msg, side * (math.pi / 2 - theta))

        if not (math.isfinite(a) and math.isfinite(b)):
            # Warn at most once per second to avoid spamming the log
            self.get_logger().warn('No valid wall reading, skipping this scan',
                                   throttle_duration_sec=1.0)
            return None

        # Heading angle and current distance to the wall
        alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
        current_distance = b * math.cos(alpha)
        # Distance to the wall after driving L ahead
        predicted_distance = current_distance + lookahead_distance * math.sin(alpha)

        # Use the side coefficient to flip the sign for left vs right wall.
        # Right wall: too close -> steer left (positive).
        # Left wall: too close -> steer right (negative). Hence the side flip.
        return side * (predicted_distance - dist)

    def scan_callback(self, msg):
        """
        Compute the error for each new scan and publish it.

        Args:
            msg (sensor_msgs.msg.LaserScan): current LiDAR scan message.

        Side effects:
            Publishes one Float64 message on error_topic when the scan is valid.
        """
        desired_distance = float(self.get_parameter('desired_distance').value)
        error = self.get_error(msg, desired_distance)
        if error is None:
            return

        error_msg = Float64()
        error_msg.data = float(error)
        self.error_publisher.publish(error_msg)


def main(args=None):
    """
    Start the dist_finder node and spin until shutdown.

    Args:
        args (list[str] | None): command-line arguments passed to rclpy.init.
    """
    rclpy.init(args=args)
    dist_finder = DistFinder()
    rclpy.spin(dist_finder)
    dist_finder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
