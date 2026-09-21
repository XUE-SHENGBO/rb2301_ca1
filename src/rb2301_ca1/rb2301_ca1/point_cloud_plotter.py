import numpy as np
import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import qos_profile_sensor_data

class PointCloudPlotter(Node):
    def __init__(self):
        super().__init__("point_cloud_plotter")

        self.points = np.empty((0, 2))
        self.dirty = False

        self.subscription = self.create_subscription(
            PointCloud2,
            "obstacle_points",
            self.points_callback,
            qos_profile_sensor_data,
        )

        plt.ion()
        self.figure_name = "LiDAR Point Cloud"
        self.fig, self.ax = plt.subplots(num=self.figure_name)

        self.cloud = self.ax.scatter([], [], s=8)
        self.ax.scatter(
            0, 0,
            c="red",
            marker=MarkerStyle("^"),
            s=80,
            label="Robot",
        )

        # avoidance_xy 的 X 正方向为机器人左侧，反转横轴以匹配实际左右。
        self.ax.set_xlim(3, -3)
        self.ax.set_ylim(-3, 3)
        self.ax.set_aspect("equal")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title(self.figure_name)
        self.ax.grid(True)
        self.ax.legend()

        plt.show(block=False)

    def points_callback(self, msg):
        # 回调只接收数据，绘图由主循环完成
        points = point_cloud2.read_points_numpy(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )

        self.points = points.reshape(-1, 3)[:, :2]
        self.dirty = True

    def refresh(self):
        if self.dirty:
            self.cloud.set_offsets(self.points)
            self.fig.canvas.draw_idle()
            self.dirty = False

        # 即使没有新数据，也处理窗口拖动、关闭等事件
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPlotter()

    try:
        while rclpy.ok() and plt.fignum_exists(node.figure_name):
            rclpy.spin_once(node, timeout_sec=0.0)
            node.refresh()
            plt.pause(0.05)  # 大约每 50 ms 刷新一次

    except KeyboardInterrupt:
        pass

    finally:
        plt.close(node.fig)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
