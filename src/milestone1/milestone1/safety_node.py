"""
name and student number: Jeff Chang, 20230629
Safety node for the car, applying AEB to avoid collisions.

This node subscribes to LiDAR and the car odometry and uses their data to
compute time-to-collision (TTC). The idea is to calculate the instantaneous
TTC (iTTC) for the beams covering the future path of the car. We can think of
the car and its movement as a long rectangle in front of it: as long as
nothing is inside this rectangle, it can be ignored.
If the smallest iTTC is below the threshold, the node publishes a
zero-speed command on /drive.

This node is also the only publisher on /drive: the driving node (pid) sends
its commands to /drive_request, and they are only forwarded to /drive while
the node is not braking. So no other node can overrule a brake.

All three tuning values are ROS 2 parameters and can be changed while running:

    ros2 param set /safety_node ttc_threshold 0.6
    ros2 param set /safety_node car_width 0.34
    ros2 param set /safety_node min_closing_speed 0.000001

Always pass floats (0.6, 0.34), not integers, or the change is rejected.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

# Default iTTC threshold (s)
TTC_THRESHOLD = 0.5

# Closing speeds below this value (m/s) are treated as "not approaching".
MIN_CLOSING_SPEED = 1e-6

# Car width (m). A beam is only kept if its hit point lies inside the strip
# the car sweeps when driving straight: |r * sin(theta)| <= CAR_WIDTH / 2.
CAR_WIDTH = 0.34


class SafetyNode(Node):
    """
    Automatic emergency braking (AEB) based on iTTC, and gatekeeper of /drive.

    Subscription:
        scan_topic (sensor_msgs/LaserScan): LiDAR ranges, used to compute iTTC.
        odom_topic (nav_msgs/Odometry): forward speed (twist.twist.linear.x).
        drive_request_topic (ackermann_msgs/AckermannDriveStamped): commands
            from the driving node, forwarded to drive_topic when not braking.

    Publication:
        drive_topic (ackermann_msgs/AckermannDriveStamped): zero-speed command
            while braking, otherwise the forwarded drive requests.

    Params:
        scan_topic, odom_topic, drive_request_topic, drive_topic (str):
            topic names, read once at startup.
        ttc_threshold (float, s): brake when the smallest iTTC is below this.
        car_width (float, m): width of the car's path used to filter beams.
        min_closing_speed (float, m/s): beams closing slower than this are ignored.
    """

    def __init__(self):
        """Declare parameters, set up topics and initialise the braking state."""
        super().__init__('safety_node')

        # Topic names (shared with the other nodes through the global param file)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_request_topic', '/drive_request')
        self.declare_parameter('drive_topic', '/drive')

        self.publisher_ = self.create_publisher(
            AckermannDriveStamped,
            self.get_parameter('drive_topic').value,
            10)
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10)
        self.drive_request_subscription = self.create_subscription(
            AckermannDriveStamped,
            self.get_parameter('drive_request_topic').value,
            self.drive_request_callback,
            10)

        # Tuning params (re-read on every scan, so ros2 param set works live)
        self.declare_parameter('ttc_threshold', TTC_THRESHOLD)
        self.declare_parameter('car_width', CAR_WIDTH)
        self.declare_parameter('min_closing_speed', MIN_CLOSING_SPEED)

        # Current speed, forward is positive (from odometry callback)
        self.speed = 0.0

        # True while the latest scan says we must brake (blocks drive requests)
        self.braking = False

    def odom_callback(self, msg):
        """
        Store the current forward speed of the car.

        Args:
            msg (nav_msgs.msg.Odometry): the odometry message.

        Side effects:
            Updates self.speed (m/s, forward is positive).
        """
        self.speed = msg.twist.twist.linear.x

    def compute_min_ttc(self, msg):
        """
        Compute the smallest iTTC among the beams inside the car's path.

        For beam i with range r_i and angle theta_i (0 = straight ahead):
            range rate   r_dot_i = -speed * cos(theta_i)
            iTTC_i       = r_i / max(-r_dot_i, 0)

        Args:
            msg (sensor_msgs.msg.LaserScan): current LiDAR scan message.

        Returns:
            tuple[float, float]: (min_ttc, angle_of_min_ttc)
                min_ttc: smallest iTTC in seconds (inf if no beam is closing).
                angle_of_min_ttc: angle of that beam in degrees.
        """
        car_width = float(self.get_parameter('car_width').value)
        min_closing_speed = float(self.get_parameter('min_closing_speed').value)

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        # Angle of every beam, computed from the message (no hardcoded count)
        angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment

        # Drop invalid readings (inf, nan, outside the sensor's range)
        valid_range = (
            np.isfinite(ranges)
            & (ranges >= msg.range_min)
            & (ranges <= msg.range_max)
        )

        # Drop beams whose hit point is outside the car's path
        # (sideways offset larger than half the car width)
        in_path = np.abs(ranges * np.sin(angles)) <= car_width / 2.0

        # How fast each beam's distance is shrinking (0 if it is not)
        range_rate = -self.speed * np.cos(angles)
        closing_speed = np.maximum(-range_rate, 0.0)

        effective_beams = valid_range & in_path & (closing_speed > min_closing_speed)

        # iTTC for the effective beams, inf for all others
        ttc = np.full(ranges.size, np.inf)
        for b in effective_beams.nonzero()[0]:
            ttc[b] = ranges[b] / closing_speed[b]

        index = int(np.argmin(ttc))
        return float(ttc[index]), float(np.degrees(angles[index]))

    def scan_callback(self, msg):
        """
        Decide whether to brake based on the smallest iTTC.

        Args:
            msg (sensor_msgs.msg.LaserScan): current LiDAR scan message.

        Side effects:
            Updates self.braking. When braking, publishes a zero-speed
            command on drive_topic and logs a warning (at most every 0.5 s).
        """
        threshold = float(self.get_parameter('ttc_threshold').value)
        min_ttc, angle_of_min_ttc = self.compute_min_ttc(msg)

        self.braking = min_ttc < threshold

        if self.braking:
            self.brake()
            self.get_logger().warn(
                f'BBBRRRAAAKKKEEEDDD: TTC {min_ttc:.3f} s < {threshold:.2f} s '
                f'(ttc from {angle_of_min_ttc:.1f} deg, v={self.speed:.2f} m/s)',
                throttle_duration_sec=0.5)

    def drive_request_callback(self, msg):
        """
        Forward a drive command to drive_topic unless the car is braking.

        Args:
            msg (ackermann_msgs.msg.AckermannDriveStamped): requested command
                from the driving node.

        Side effects:
            Publishes msg on drive_topic when self.braking is False.
        """
        if not self.braking:
            self.publisher_.publish(msg)

    def brake(self):
        """
        Command the car to stop.

        Side effects:
            Publishes one zero-speed AckermannDriveStamped on drive_topic.
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = 0.0
        self.publisher_.publish(drive_msg)


def main(args=None):
    """
    Start the safety node and spin until shutdown.

    Args:
        args (list[str] | None): command-line arguments passed to rclpy.init.
    """
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
