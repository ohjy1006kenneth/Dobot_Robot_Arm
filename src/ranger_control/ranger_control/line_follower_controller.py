#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.time import Time

class PIDLineFollower(Node):
    def __init__(self):
        super().__init__('pid_line_follower')

        self.crack_arrived_prev = False

        # PID parameters
        self.kp = 0.003
        self.ki = 0.000
        self.kd = 0.002

        self.integral = 0.0
        self.last_error = 0

        # Timeout and timing
        self.last_msg_time = self.get_clock().now()
        self.timeout_sec = 0.5

        # Sub & Pub
        self.subscription = self.create_subscription(
            Int32,
            '/line_error',
            self.line_error_callback,
            10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.trigger_pub = self.create_publisher(Bool, '/crack_arrived', 10)

        self.timer = self.create_timer(0.05, self.control_loop)
        self.current_error = 0
        self.received_first = False

    def line_error_callback(self, msg):
        if abs(msg.data) > 10000:
            self.get_logger().warn('Ignoring large error value: likely no line detected')
            return
        self.current_error = msg.data
        self.last_msg_time = self.get_clock().now()
        self.received_first = True

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = now - self.last_msg_time

        twist = Twist()

        if not self.received_first or elapsed.nanoseconds > self.timeout_sec * 1e9:
            # No recent line error — stop the robot
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn("No line error received recently — stopping robot.")

            if not self.crack_arrived_prev:
                self.get_logger().warn("adshglkahgd;lksakgsd")
                self.trigger_pub.publish(Bool(data=True))
                self.crack_arrived_prev = True
                return
            return
        else:
            # PID control logic
            self.trigger_pub.publish(Bool(data=False))
            error = self.current_error
            self.integral += error
            self.integral = max(min(self.integral, 1000), -1000)  # Anti-windup

            derivative = error - self.last_error
            correction = self.kp * error + self.ki * self.integral + self.kd * derivative

            twist.linear.x = -0.1  # Forward
            twist.angular.z = -max(min((correction * 0.1), 1.0), -1.0)  # Clamp correction

            self.last_error = error
            self.get_logger().info(f'cmd_vel: linear.x = {twist.linear.x:.2f}, angular.z = {twist.angular.z:.2f}')

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PIDLineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
