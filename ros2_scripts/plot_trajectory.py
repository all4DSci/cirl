#!/usr/bin/env python3
"""Subscribe to /odom, record x,y positions and save a trajectory plot and CSV on shutdown."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import csv
import os
from datetime import datetime


class TrajectoryRecorder(Node):
    def __init__(self, out_png='trajectory.png', out_csv='trajectory.csv'):
        super().__init__('trajectory_recorder')
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.xs = []
        self.ys = []
        self.ts = []
        self.start_time = None
        self.out_png = out_png
        self.out_csv = out_csv

    def odom_cb(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if self.start_time is None:
            self.start_time = t
        self.ts.append(t - self.start_time)
        self.xs.append(msg.pose.pose.position.x)
        self.ys.append(msg.pose.pose.position.y)

    def save(self):
        if not self.xs:
            self.get_logger().warning('No odometry received, nothing to save')
            return

        # CSV
        csv_path = os.path.abspath(self.out_csv)
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t','x','y'])
            for t,x,y in zip(self.ts, self.xs, self.ys):
                w.writerow([f'{t:.6f}', f'{x:.6f}', f'{y:.6f}'])
        self.get_logger().info(f'Saved trajectory CSV to {csv_path}')

        # PNG
        plt.figure()
        plt.plot(self.xs, self.ys, '-o', markersize=3)
        plt.axis('equal')
        plt.grid(True)
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.title('Robot trajectory')
        png_path = os.path.abspath(self.out_png)
        plt.savefig(png_path, dpi=200)
        plt.close()
        self.get_logger().info(f'Saved trajectory plot to {png_path}')


def main(args=None):
    rclpy.init(args=args)
    # generate timestamped defaults to avoid overwrite
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    node = TrajectoryRecorder(out_png=f'trajectory_{stamp}.png', out_csv=f'trajectory_{stamp}.csv')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
