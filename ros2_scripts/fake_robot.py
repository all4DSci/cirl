#!/usr/bin/env python3
"""Simple fake robot integrator for testing controllers.

Subscribes to /cmd_vel_unstamped (Twist) and integrates linear.x and angular.z
to a 2D pose, publishing nav_msgs/Odometry on /odom. This is not a realistic
simulator but is sufficient to see the controller steer and avoid the obstacle.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time


class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot')
        self.sub = self.create_subscription(Twist, '/cmd_vel_unstamped', self.cmd_cb, 10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)

        # state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.w = 0.0

        self.last_time = time.time()
        self.timer = self.create_timer(0.05, self.timer_cb)  # 20 Hz

    def cmd_cb(self, msg: Twist):
        # Accept commands directly as desired velocities
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)

    def timer_cb(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # simple unicycle integration
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt

        # publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        # minimal quaternion from yaw
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w

        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
