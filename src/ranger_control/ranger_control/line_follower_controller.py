#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist
import math

class PIDLineFollower(Node):
    def __init__(self):
        super().__init__('pid_line_follower')

        # PID params for line following
        self.kp = 0.003
        self.ki = 0.000
        self.kd = 0.002

        self.integral = 0.0
        self.last_error = 0

        # Timing for line error timeout
        self.last_msg_time = self.get_clock().now()
        self.timeout_sec = 0.5

        # Subscriptions
        self.subscription = self.create_subscription(
            Int32,
            '/line_error',
            self.line_error_callback,
            10)

        self.crack_angle_sub = self.create_subscription(
            Int32,
            '/crack_angle',
            self.crack_angle_callback,
            10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.trigger_pub = self.create_publisher(Bool, '/start_repair', 10)

        # Local vars
        self.current_error = 0
        self.received_first = False
        self.crack_angle = 9999  # default no crack detected
        self.crack_trigger_sent = False
        self.alignment_tolerance_deg = 5  # degrees for "aligned"

        # Timer for control loop
        self.timer = self.create_timer(0.05, self.control_loop)

    def line_error_callback(self, msg):
        if abs(msg.data) > 10000:
            self.get_logger().warn('Ignoring large error value: likely no line detected')
            return
        self.current_error = msg.data
        self.last_msg_time = self.get_clock().now()
        self.received_first = True

    def crack_angle_callback(self, msg):
        self.crack_angle = msg.data

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = now - self.last_msg_time

        twist = Twist()

        # Blue line detected → stop and trigger repair
        if self.crack_angle != 9999:
            twist.linear.x = 0.0000000000
            twist.angular.z = 0.000000000

            if not self.crack_trigger_sent:
                self.trigger_pub.publish(Bool(data=True))
                self.crack_trigger_sent = True
                self.cmd_pub.publish(twist)

            #self.cmd_pub.publish(twist)
            return  # Skip normal white line following

        # No blue line detected → reset trigger, do white line following
        self.crack_trigger_sent = False
        self.trigger_pub.publish(Bool(data=False))

        if not self.received_first or elapsed.nanoseconds > self.timeout_sec * 1e9:
            # Timeout or no line error → stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            # PID control for white line
            error = self.current_error
            self.integral += error
            self.integral = max(min(self.integral, 1000), -1000)  # anti-windup

            derivative = error - self.last_error
            correction = self.kp * error + self.ki * self.integral + self.kd * derivative

            twist.linear.x = -0.1  # forward
            twist.angular.z = -max(min((correction * 0.1), 1.0), -1.0)

            self.last_error = error
            
        self.cmd_pub.publish(twist)
        
        self.get_logger().info(f'PID driving: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}')



def main(args=None):
    rclpy.init(args=args)
    node = PIDLineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
