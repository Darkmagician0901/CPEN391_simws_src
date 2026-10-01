"""
PID controller node for wall following.

Subscribes to the wall distance error from dist_finder, runs PID control to
get a steering angle, picks a speed from that angle and publishes the drive
request. The request goes to safety_node, which is the only node
allowed to publish on /drive (so a brake can never be overruled).

Based on the WallFollow template from the CPEN 391 "Reactive Methods for
Navigation" slides; the error computation lives in dist_finder.py.

Sign convention (must match dist_finder.py):
    error > 0 -> steer LEFT (positive steering angle), so the PID output is
    used directly, with NO minus sign in front.

All tuning values are ROS 2 parameters, set in config/params.yaml.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped


class PID(Node):
    """
    PID steering control for wall following.

    Subscription:
        error_topic (std_msgs/Float64): distance error to the wall in meters.

    Publication:
        drive_request_topic (ackermann_msgs/AckermannDriveStamped): requested
            steering angle and speed, forwarded to /drive by safety_node.

    Params:
        kp, ki, kd (float): PID gains.
        max_steering_angle (float, rad): steering output is clipped to this.
        integral_limit (float, m*s): anti-windup clamp on the error integral.
        small_angle (float, deg): at or below this steering angle, drive at
            fast_speed.
        large_angle (float, deg): at or above this steering angle, drive at
            slow_speed. Speed ramps linearly between the two thresholds.
        fast_speed, slow_speed (float, m/s): speeds at the two ends of the ramp.
    """

    def __init__(self):
        """Declare parameters, set up topics and initialise PID history."""
        super().__init__('pid')

        # Topics
        self.declare_parameter('error_topic', '/wall_error')
        self.declare_parameter('drive_request_topic', '/drive_request')

        # PID gains
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('integral_limit', 1.0)

        # Speed ramp
        self.declare_parameter('small_angle', 10.0)
        self.declare_parameter('large_angle', 20.0)
        self.declare_parameter('fast_speed', 1.5)
        self.declare_parameter('slow_speed', 0.5)

        error_topic = self.get_parameter('error_topic').value
        drive_request_topic = self.get_parameter('drive_request_topic').value

        self.error_subscription = self.create_subscription(
            Float64, error_topic, self.error_callback, 10)
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, drive_request_topic, 10)

        # PID state
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    @staticmethod
    def clamp(value, limit):
        """Clamp value to the range [-limit, limit]."""
        return max(-limit, min(value, limit))

    def pid_control(self, error, dt):
        """
        Compute the steering angle from the error with pid.

        Args:
            error (float): current distance error in meters.
            dt (float): seconds since the previous error (0.0 on the first one).

        Returns:
            float: steering angle in radians, clipped to max_steering_angle.

        Side effects:
            Updates self.integral and self.prev_error.
        """
        kp = float(self.get_parameter('kp').value)
        ki = float(self.get_parameter('ki').value)
        kd = float(self.get_parameter('kd').value)
        max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        integral_limit = float(self.get_parameter('integral_limit').value)

        # On the first message there is no dt or previous error, so skip I and D
        derivative = 0.0
        if dt > 0.0:
            # I: accumulate error over time, clamped so it cannot wind up
            self.integral = self.clamp(self.integral + error * dt, integral_limit)
            # D: how fast the error is changing
            derivative = (error - self.prev_error) / dt

        self.prev_error = error

        steering_angle = kp * error + ki * self.integral + kd * derivative
        return self.clamp(steering_angle, max_steering_angle)

    def get_speed(self, steering_angle):
        """
        Pick a speed from the steering angle.

        Speed profile (angles in degrees, compared by magnitude):
            |angle| <= small_angle                -> fast_speed
            small_angle < |angle| < large_angle   -> linear from fast_speed
                                                     down to slow_speed
            |angle| >= large_angle                -> slow_speed

        A continuous profile avoids the speed jumps of a stepped schedule,
        which would otherwise make the speed flicker between bands whenever
        the steering angle hovers near a threshold.

        Args:
            steering_angle (float): steering angle in radians; the sign
                (left/right) is ignored, only the size of the turn matters.

        Returns:
            float: speed in m/s, between slow_speed and fast_speed.
        """
        turn_deg = math.degrees(abs(steering_angle))
        small = float(self.get_parameter('small_angle').value)
        large = float(self.get_parameter('large_angle').value)
        fast = float(self.get_parameter('fast_speed').value)
        slow = float(self.get_parameter('slow_speed').value)

        if turn_deg <= small:
            return fast
        if turn_deg >= large:
            return slow
        # ratio is 0.0 at small_angle, 1.0 at large_angle, linear in between
        ratio = (turn_deg - small) / (large - small)
        return fast + (slow - fast) * ratio

    def error_callback(self, msg):
        """
        Run PID on a new error and publish the resulting drive request.

        Args:
            msg (std_msgs.msg.Float64): distance error from dist_finder.

        Side effects:
            Publishes one AckermannDriveStamped message on drive_request_topic.
        """
        # Time since the previous error; stays 0.0 on the first message
        now = self.get_clock().now()
        dt = 0.0
        if self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        steering_angle = self.pid_control(msg.data, dt)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = now.to_msg()
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(self.get_speed(steering_angle))
        self.drive_publisher.publish(drive_msg)


def main(args=None):
    """
    Start the pid node and spin until shutdown.

    Args:
        args (list[str] | None): command-line arguments passed to rclpy.init.
    """
    rclpy.init(args=args)
    pid = PID()
    rclpy.spin(pid)
    pid.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
