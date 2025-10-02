#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class PIController(Node):
    def __init__(self):
        super().__init__('pi_controller')

        # Publisher to TurtleBot4 base
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)

        # Subscriber to odometry
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        # Setpoint (goal position)
        self.goal_x = 1.0   # meters
        self.goal_y = 0.0   # meters

        # Controller gains
        self.Kp_lin = 0.8
        self.Ki_lin = 0.05
        self.Kp_ang = 2.0
        self.Ki_ang = 0.1

        # Integrators
        self.int_lin = 0.0
        self.int_ang = 0.0

        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0
        # use ROS time
        self.last_time = self.get_clock().now()
        self.have_odom = False

    def odom_callback(self, msg: Odometry):
        # Extract position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.have_odom = True

        # Extract orientation (quaternion → yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        # Time delta using ROS time
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time.nanoseconds) * 1e-9
        if dt <= 0.0 or dt > 0.5:
            dt_valid = False
            dt = 0.0
        else:
            dt_valid = True
        self.last_time = now

        if not getattr(self, 'have_odom', False):
            return

        # Compute errors
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance_error = math.sqrt(dx**2 + dy**2)
        target_yaw = math.atan2(dy, dx)
        heading_error = self._angle_wrap(target_yaw - self.yaw)

        if dt_valid:
            self.int_lin += distance_error * dt
            self.int_ang += heading_error * dt

        self.int_lin = max(min(self.int_lin, 2.0), -2.0)
        self.int_ang = max(min(self.int_ang, 2.0), -2.0)

        v = self.Kp_lin * distance_error + self.Ki_lin * self.int_lin
        w = self.Kp_ang * heading_error + self.Ki_ang * self.int_ang

        # Limit velocities
        v = max(min(v, 0.3), -0.3)  # clamp linear speed
        w = max(min(w, 1.5), -1.5)  # clamp angular speed

        # Stop if close enough
        if distance_error < 0.05:
            v, w = 0.0, 0.0

        # Publish command
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def _angle_wrap(self, angle):
        """Wrap angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = PIController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
