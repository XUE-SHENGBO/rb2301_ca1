import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
# 绘图消息发送库
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from rclpy.qos import (
    qos_profile_sensor_data
)

np.set_printoptions(
    2, suppress=True
)  # Print numpy arrays to specified d.p. and suppress scientific notation (e.g. 1e-5)

max_translate_velocity = 0.5 # Can be implemented as parameter
max_turn_velocity = max_translate_velocity * 2 # Can be implemented as parameter
set_logger_level("obstacle_avoidance", level=LoggingSeverity.INFO) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  

timer_freq = 0.05
scan_gap = 10

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("obstacle_avoidance")
        self.get_logger().info("Starting Obstacle Avoidance")

        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  # Publish to cmd_vel node
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # The subscriber to the Lidar ranges.
        self.last_scan = None # Copied laser scan message
        self.last_scan_angles = None

        self.timer = self.create_timer(timer_freq, self.timer_callback)  # Runs at 20Hz. Can be changed. 

        self.state='move_forward'
        self.offset_x = 0
        self.offset_y = 0
        # 每条记录代表一个控制周期；回退时从栈尾反向执行。
        # 与 offset 一样采用指令积分，存在打滑/定时误差，后续可改用 odom。
        self.action_stack = []
        self.decision_points = []
        self.checkpoint_spacing = 0.30  # 前进中每隔 30 cm 留一个可回退点
        self.scan_received_at = None
        self.scan_timeout = 0.5
        self.speed = 0.2

        self.pub_points = self.create_publisher(
            PointCloud2, 
            "obstacle_points",
            qos_profile_sensor_data
        )
        self.scan_stamp = None
        self.move_stack = []

    def move_2D(self, x: float = 0.0, y: float = 0.0, turn: float = 0.0):
        """Publishes a twist command to move in 2D space. +ve x is forwards, +ve y is left, and +ve turn is anticlockwise"""
        twist_msg = Twist()
        x = np.clip(x, -max_translate_velocity, max_translate_velocity)
        y = np.clip(y, -max_translate_velocity, max_translate_velocity)
        turn = np.clip(turn, -max_translate_velocity*2, max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.pub_cmd_vel.publish(twist_msg)

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        indices = np.arange(0, len(msg.ranges), scan_gap)
        self.last_scan = np.asarray(msg.ranges)[indices]
        self.last_scan_angles = msg.angle_min + indices * msg.angle_increment
        self.scan_stamp = msg.header.stamp
        self.scan_received_at = self.get_clock().now().nanoseconds / 1e9

    def transfer(self):
        if self.last_scan is None or self.last_scan_angles is None:
            return
        self.last_scan_xy=[]
        for i in range(len(self.last_scan)):
            if np.isfinite(self.last_scan[i]) and self.last_scan[i] > 0:
                angle = self.last_scan_angles[i]
                x = -self.last_scan[i]*np.sin(angle)
                y = -self.last_scan[i]*np.cos(angle)
                self.last_scan_xy.append((x,y))
# 小车尺寸：前面7cm，左右10cm，后面15cm 
# 
    def front_clear(self):
        for obstacle_dot in self.last_scan_xy:
            x=obstacle_dot[0]
            y=obstacle_dot[1]
            if x>-0.11 and x<0.11 and y>0 and y<0.15:
                return False
        return True

    def left_clear(self):
        if self.offset_x >=1:
            return False
        for obstacle_dot in self.last_scan_xy:
            x=obstacle_dot[0]
            y=obstacle_dot[1]
            if x < 0.15 and x>0 and y>-0.17 and y<0.10:
                return False
        return True

    def right_clear(self):
        if self.offset_x <=-1:
            return False
        for obstacle_dot in self.last_scan_xy:
            x=obstacle_dot[0]
            y=obstacle_dot[1]
            if x >-0.15 and x <0 and y>-0.17 and y<0.10:
                return False
        return True

    def publish_point_cloud(self):
        points = np.asarray(
            self.last_scan_xy, dtype=np.float32
        ).reshape(-1, 2)

        # 排除无效坐标
        points = points[np.isfinite(points).all(axis=1)]

        # (N, 2) → (N, 3)，Z 坐标全部为 0
        xyz = np.zeros((len(points), 3), dtype=np.float32)
        xyz[:, :2] = points

        header = Header()
        if self.scan_stamp is not None:
            header.stamp = self.scan_stamp
        header.frame_id = "avoidance_xy"

        msg = point_cloud2.create_cloud_xyz32(header, xyz)
        self.pub_points.publish(msg)

    def back_clear(self):
        # 后沿在 -0.15 m，留出约 8 cm 的后退检查区域。
        return not any(-0.11 < x < 0.11 and -0.23 < y < 0
                       for x, y in self.last_scan_xy)

    def direction_clear(self, direction):
        return {
            'move_forward': self.front_clear,
            'move_left': self.left_clear,
            'move_right': self.right_clear,
            'move_back': self.back_clear,
        }[direction]()

    def add_decision_point(self):
        index = len(self.action_stack)
        if self.decision_points and self.decision_points[-1]['index'] == index:
            return self.decision_points[-1]
        point = {
            'index': index,
            'y': self.offset_y,
            # 前方是已走过/已经被挡的分支；回到这里后只尝试其他方向。
            'tried': {'move_forward'},
        }
        self.decision_points.append(point)
        return point

    def choose_side(self, point):
        sides = ('move_left', 'move_right') if self.offset_x <= 0 else (
            'move_right', 'move_left')
        for direction in sides:
            if direction not in point['tried']:
                # 堵塞的方向也记为已检查，避免在同一个点来回尝试。
                point['tried'].add(direction)
                if self.direction_clear(direction):
                    return direction
        return None

    def execute_step(self, direction, replay=False):
        velocities = {
            'move_forward': (self.speed, 0.0),
            'move_back': (-self.speed, 0.0),
            'move_left': (0.0, self.speed),
            'move_right': (0.0, -self.speed),
        }
        vx, vy = velocities[direction]
        self.move_2D(vx, vy, 0)
        self.offset_x += vy * timer_freq
        self.offset_y += vx * timer_freq
        if not replay:
            self.action_stack.append(direction)

    def backtrack_step(self):
        # 必须返回决策点才重新选路；途中前方暂时变空不结束回退。
        while self.decision_points:
            point = self.decision_points[-1]
            if len(self.action_stack) > point['index']:
                break
            direction = self.choose_side(point)
            if direction is not None:
                self.state = direction
                self.get_logger().info('回到决策点，尝试新方向：' + direction)
                self.execute_step(direction)
                return
            self.decision_points.pop()

        if not self.action_stack:
            self.state = 'stop'
            self.move_2D()
            self.get_logger().warning('回退路径耗尽，无可用分支，停车')
            return

        inverse = {
            'move_forward': 'move_back',
            'move_left': 'move_right',
            'move_right': 'move_left',
        }
        direction = inverse[self.action_stack[-1]]
        if not self.direction_clear(direction):
            # 不弹栈；保持回退状态，障碍移开后可以继续。
            self.move_2D()
            return
        self.execute_step(direction, replay=True)
        self.action_stack.pop()

    def timer_callback(self):
        """普通行驶记录动作；失败后沿动作栈返回尚有分支的决策点。"""
        now = self.get_clock().now().nanoseconds / 1e9
        if (self.scan_received_at is None or
                now - self.scan_received_at > self.scan_timeout):
            self.move_2D()
            return

        self.transfer()
        self.publish_point_cloud()
        if self.state == 'stop':
            self.move_2D()
            return
        if self.state == 'backtrack':
            self.backtrack_step()
            return

        if self.state == 'move_forward':
            if (not self.decision_points or
                    self.offset_y - self.decision_points[-1]['y'] >=
                    self.checkpoint_spacing):
                self.add_decision_point()
            if not self.front_clear():
                point = self.add_decision_point()
                self.state = self.choose_side(point) or 'backtrack'
        elif self.state in ('move_left', 'move_right'):
            if self.front_clear():
                self.state = 'move_forward'
            elif not self.direction_clear(self.state):
                # 不在死路末端直接反选方向；先撤销这次尝试回到起点。
                self.state = 'backtrack'

        if self.state == 'backtrack':
            self.get_logger().info('当前分支走不通，开始回退')
            self.backtrack_step()
        elif self.direction_clear(self.state):
            self.execute_step(self.state)
        else:
            self.move_2D()

        self.get_logger().debug(f'x偏移：{self.offset_x}')


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
