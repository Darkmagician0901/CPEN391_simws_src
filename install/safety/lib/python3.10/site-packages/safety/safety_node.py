"""
name and student number: Jeff Chang, 20230629
Safety node for the car, applying AEB to avoid collision

This node subscribes to LiDAR and the car odometry and use their data to compute
time-to-collision (TTC). Idea here is to calculate the instantaneous ttc (aka iTTC) for 
the chosen beams, particularly the ones covering the future trajectory of the car
(i.e. we can abstractly think of the car and its movement to be on a super long
rectangle, as long as nothing is in this rectangle, we can ignore it).
Then we check min iTTC and then stop if it is below the threshold, the node 
publishes a set-speed-to-zero command on the /drive topic.

All three tuning values are ROS 2 parameters and can be changed while running:

    ros2 param set /safety_node ttc_threshold 0.6
    ros2 param set /safety_node car_width 0.34
    ros2 param set /safety_node min_closing_speed 0.000001

Always pass floats (0.6, 0.34), not integers, or the change is rejected.
"""

# include needed imports
import rclpy
from rclpy.node import Node

import numpy as np #optional 
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# ITTC threshold
TTC_THRESHOLD = 0.5

# Closing speeds below this value (m/s) are treated as "not approaching".
MIN_CLOSING_SPEED = 1e-6

# Car width (m). A beam is only kept if its hit point lies inside the strip
# the car sweeps when driving straight: |r * sin(theta)| <= CAR_WIDTH / 2.
CAR_WIDTH = 0.34


class SafetyNode(Node):
    """
    The safety node implementations are below. 

    Subscription:
        topic scan (sensor_msgs/LaserScan): LiDAR ranges, used to compute iTTC
        topic ego_racecar/odom (nav_msgs/Odometry): forward speed 
        (twist.twist.linear.x).

    Publication:
        topic drive (ackermann_msgs/AckermannDriveStamped): zero-speed command
        sent  when decided to brake.

    Params:
        ttc_threshold (float, s): brake when the smallest iTTC is below this.
        car_width (float, m): width of the car's path used to filter beams.
        min_closing_speed (float, m/s): beams closing slower than this are ignored.
    """

    def __init__(self):
        """
        Constructor for the node
        """
        super().__init__('safety_node')

        # create the required publisher, subscriber ...
        # use the following as a starting point and modify as needed ...

        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.scan_subscription = self.create_subscription(
              LaserScan,
              'scan',
              self.scan_callback,   #choose a descriptive method name for the callback
              10)
        self.odom_subscription = self.create_subscription(
              Odometry,
              'ego_racecar/odom',
              self.odom_callback,  #choose a descriptive method name for the callback
              10)

        # Using params here for easy tuning
        self.declare_parameter('ttc_threshold', TTC_THRESHOLD)
        self.declare_parameter('car_width', CAR_WIDTH)
        self.declare_parameter('min_closing_speed', MIN_CLOSING_SPEED)

        # Current speed, forward is positive (from odometry callback)
        self.speed = 0.0

    def odom_callback(self, msg):
        """
        Return the instant speed of the car (with forward as positive)

        Args:
            msg (nav_msgs.msg.Odometry): the odometry message

        Side effects:
            Updates self.speed.
        """
        self.speed = msg.twist.twist.linear.x

    def compute_min_ttc(self, msg):
        """
        Compute min iTTC from the beams inside the car's path

        For beam i with range r_i and angle theta_i (0 = straight ahead):
            range rate   r_dot_i = -speed * cos(theta_i)
            iTTC_i       = r_i / max(-r_dot_i, 0)

        Args:
            msg (sensor_msgs.msg.LaserScan): current LiDAR scan message

        Returns:
            (min_ttc, angle_of_min_ttc) --> tuple floats
            
                min_ttc (float) is the smallest iTTC from the car's path in seconds (inf if no beam is closing)
                angle_of_min_ttc (float): angle of that beam in degrees
        """
        
        # Read the tuning params
        car_width = float(self.get_parameter('car_width').value)
        min_closing_speed = float(self.get_parameter('min_closing_speed').value)

        # Read range and angle from lidar message
        ranges = np.asarray(msg.ranges, dtype=np.float64)

        # Dynamic angle labelings
        angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment

        # Filter invalid ranges
        valid_range = (
            np.isfinite(ranges)
            & (ranges >= msg.range_min)
            & (ranges <= msg.range_max)
        )

        # Filter out beams whose hit point is outside the car's path
        # (the beams whose y offset larger than half the car width)
        in_path = np.abs(ranges * np.sin(angles)) <= car_width / 2.0

        # Compute closing speed and iTTC for working beams, we set inf for all others
        range_rate = -self.speed * np.cos(angles)
        closing_speed = np.maximum(-range_rate, 0.0)

        effective_beams = valid_range & in_path & (closing_speed > min_closing_speed)

        # caclulate iTTC for the used beams, inf for all others
        ttc = np.full(ranges.size, np.inf)
        for b in effective_beams.nonzero()[0]:
            ttc[b] = ranges[b] / closing_speed[b]

        # Also retrieve the angle of the min ttc beam
        index = int(np.argmin(ttc))

        return float(ttc[index]), float(np.degrees(angles[index]))

    def scan_callback(self, msg):
        """
        Decide whether brake or not based on min ttc

        Args:
            msg (sensor_msgs.msg.LaserScan): current LiDAR scan message

        Side effects:
            Call brake when min iTTC < ttc_threshold.
            Also writes a log message when braking.
        """
        threshold = float(self.get_parameter('ttc_threshold').value)
        min_ttc, angle_of_min_ttc = self.compute_min_ttc(msg)

        if min_ttc < threshold:
            self.brake()
            self.get_logger().warn(
                f'BBBRRRAAAKKKEEEDDD: TTC {min_ttc:.3f} s < {threshold:.2f} s '
                f'(ttc from {angle_of_min_ttc:.1f} deg, v={self.speed:.2f} m/s)',
                throttle_duration_sec=0.5)

    def brake(self):
        """
        Brake the car

        Side effects:
            Publishes one AckermannDriveStamped message on 'drive'.
        """
        # Use ackermann message here to set speed
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        # Set speed to zero
        drive_msg.drive.speed = 0.0
        self.publisher_.publish(drive_msg)
    

def main(args=None):
    """Start the safety node and spin until shutdown.

    Args:
        args (list[str] | None): command-line arguments passed to rclpy.init.
    """
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()