#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import matplotlib
import csv
import os
from datetime import datetime

# Note: matplotlib pyplot is imported after backend selection in __init__


class PIControllerCBF(Node):
    def __init__(self):
        super().__init__('pi_controller_cbf')

        # Publisher to TurtleBot4 base
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
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
        # trajectory recording
        self._traj_x = []
        self._traj_y = []
        self._traj_t = []
        # For dt computation (use ROS time)
        self.last_time = self.get_clock().now()

        # wait for first odometry before controlling
        self.have_odom = False

        # Goal (setpoint) in odom frame (can be overridden by ROS params)
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', -0.5)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)

        # live plotting (non-blocking). If True, try to use a GUI backend.
        self.declare_parameter('live_plot', False)
        self.live_plot = bool(self.get_parameter('live_plot').value)

        # Choose matplotlib backend based on live_plot
        try:
            if self.live_plot:
                # try common GUI backends; fall back silently
                try:
                    matplotlib.use('TkAgg')
                except Exception:
                    try:
                        matplotlib.use('Qt5Agg')
                    except Exception:
                        # cannot enable live plotting; fall back
                        matplotlib.use('Agg')
                        self.get_logger().warning('Live plotting requested but no GUI backend available; using Agg (no live display)')
                        self.live_plot = False
            else:
                matplotlib.use('Agg')
        except Exception as e:
            self.get_logger().warning(f'Failed to set matplotlib backend: {e}')
            matplotlib.use('Agg')
            self.live_plot = False

        # import pyplot after backend selection
        import matplotlib.pyplot as plt
        self.plt = plt

        # create live figure if requested
        if self.live_plot:
            try:
                self.fig, self.ax = self.plt.subplots()
                self.line, = self.ax.plot([], [], '-o', markersize=3)
                self.ax.set_aspect('equal', 'box')
                self.ax.grid(True)
                self.ax.set_xlabel('x [m]')
                self.ax.set_ylabel('y [m]')
                self.ax.set_title('Live trajectory')
                # draw initial
                self.fig.canvas.draw()
                self.plt.pause(0.001)
            except Exception as e:
                self.get_logger().warning(f'Live plotting disabled: {e}')
                self.live_plot = False

        # --- CBF parameters (circle constraint) ---
        # Default obstacle placed between typical start (0,0) and the goal
        self.declare_parameter('obs_x', 0.5)
        self.declare_parameter('obs_y', 0.0)
        self.declare_parameter('obs_R', 0.10)
        self.declare_parameter('cbf_alpha', 1.0)

        self.obs_x = float(self.get_parameter('obs_x').value)
        self.obs_y = float(self.get_parameter('obs_y').value)
        self.obs_R = float(self.get_parameter('obs_R').value)
        self.cbf_alpha = float(self.get_parameter('cbf_alpha').value)

        # Log configured scenario so user can tweak parameters
        self.get_logger().info(f'Goal set to ({self.goal_x:.2f}, {self.goal_y:.2f})')
        self.get_logger().info(f'Obstacle at ({self.obs_x:.2f}, {self.obs_y:.2f}) R={self.obs_R:.2f}, alpha={self.cbf_alpha:.2f}')

    def odom_callback(self, msg: Odometry):
        # Extract position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.have_odom = True
        # record for trajectory
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not self._traj_t:
            self._traj_t0 = t
        self._traj_t.append(t - getattr(self, '_traj_t0', t))
        self._traj_x.append(self.current_x)
        self._traj_y.append(self.current_y)

        # Extract orientation (quaternion -> yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        # use ROS time so dt is correct when using simulated time
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time.nanoseconds) * 1e-9
        # guard dt
        if dt <= 0.0 or dt > 0.5:
            # treat as first loop or jump; do not integrate on large/invalid dt
            dt_valid = False
            dt = 0.0
        else:
            dt_valid = True
        self.last_time = now

        if not getattr(self, 'have_odom', False):
            # wait for odom before issuing commands
            return

        # --- Errors ---
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance_error = math.sqrt(dx*dx + dy*dy)
        target_yaw = math.atan2(dy, dx)
        heading_error = self.angle_wrap(target_yaw - self.yaw)

        # --- PI control ---
        # integrate only on valid dt to avoid large first-step kicks
        if dt_valid:
            self.int_lin += distance_error * dt
            self.int_ang += heading_error * dt

        # Anti-windup: clamp integrator magnitude
        self.int_lin = max(min(self.int_lin, 2.0), -2.0)
        self.int_ang = max(min(self.int_ang, 2.0), -2.0)

        v_des = self.Kp_lin * distance_error + self.Ki_lin * self.int_lin
        w_des = self.Kp_ang * heading_error + self.Ki_ang * self.int_ang

        # ---------- CBF safety filter (circular keep-out) ----------
        dxo = self.current_x - self.obs_x
        dyo = self.current_y - self.obs_y
        h = dxo*dxo + dyo*dyo - self.obs_R*self.obs_R  # >=0 safe

        a = 2.0 * (dxo*math.cos(self.yaw) + dyo*math.sin(self.yaw))

        v = v_des
        w = w_des

        # Robust CBF enforcement
        # If a is well-conditioned, use the bound. If a is near-zero, fall back to
        # a safe-speed reduction and a steering away maneuver so we don't divide by tiny values.
        eps_a = 1e-3
        adjusted = False
        if abs(a) > eps_a:
            bound = (-self.cbf_alpha * h) / a
            if a > 0.0:
                if v < bound:
                    v = bound
                    adjusted = True
            else:  # a < 0
                if v > bound:
                    v = bound
                    adjusted = True
        else:
            # fallback when a ~ 0: obstacle is roughly lateral to heading or near singular
            if h < (self.obs_R + 0.3)**2:  # within safety margin (squared)
                # slow down
                v_safe = min(0.05, abs(v_des))
                if abs(v) > v_safe:
                    v = v_safe if v >= 0 else -v_safe
                    adjusted = True
                # steer away from obstacle by adding a corrective angular term
                angle_to_obs = math.atan2(dyo, dxo)
                away_err = self.angle_wrap(angle_to_obs - self.yaw)
                # if obstacle is in front left (away_err > 0) steer right (negative)
                steer = -0.8 * away_err
                w = w + steer
                adjusted = True

        if adjusted:
            self.get_logger().debug(f'CBF adjusted controls: v_des={v_des:.3f} -> v={v:.3f}, w_des={w_des:.3f} -> w={w:.3f}, h={h:.3f}')
        # -----------------------------------------------------------

        # Limit velocities
        v = max(min(v, 0.3), -0.3)
        w = max(min(w, 1.5), -1.5)

        # Stop near goal
        if distance_error < 0.05:
            v, w = 0.0, 0.0

        # Publish command
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def angle_wrap(self, angle):
        while angle > math.pi:
            angle -= 2.0*math.pi
        while angle < -math.pi:
            angle += 2.0*math.pi
        return angle

    def save_trajectory(self):
        if not self._traj_x:
            self.get_logger().info('No trajectory to save')
            return
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        png = f'trajectory_{stamp}.png'
        csvf = f'trajectory_{stamp}.csv'
        # CSV
        with open(csvf, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t','x','y'])
            for t,x,y in zip(self._traj_t, self._traj_x, self._traj_y):
                w.writerow([f'{t:.6f}', f'{x:.6f}', f'{y:.6f}'])
        self.get_logger().info(f'Saved trajectory CSV to {os.path.abspath(csvf)}')
        # PNG
        self.plt.figure()
        self.plt.plot(self._traj_x, self._traj_y, '-o', markersize=3)
        # overlay obstacle and goal
        circle = self.plt.Circle((self.obs_x, self.obs_y), self.obs_R, color='r', alpha=0.4)
        self.plt.gca().add_patch(circle)
        self.plt.plot(self.goal_x, self.goal_y, 'gx', markersize=10, label='goal')
        self.plt.axis('equal')
        self.plt.grid(True)
        self.plt.xlabel('x [m]')
        self.plt.ylabel('y [m]')
        self.plt.title('Robot trajectory')
        self.plt.legend()
        self.plt.savefig(png, dpi=200)
        self.plt.close()
        self.get_logger().info(f'Saved trajectory PNG to {os.path.abspath(png)}')

    def destroy_node(self):
        # save trajectory before shutdown
        try:
            self.save_trajectory()
        except Exception as e:
            self.get_logger().error(f'Failed to save trajectory: {e}')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PIControllerCBF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
