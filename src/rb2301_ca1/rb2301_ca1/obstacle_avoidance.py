import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

np.set_printoptions(
    2, suppress=True
)  # Print numpy arrays to specified d.p. and suppress scientific notation (e.g. 1e-5)

max_translate_velocity = 0.4 # Can be implemented as parameter
max_turn_velocity = max_translate_velocity * 2 # Can be implemented as parameter

set_logger_level("obstacle_avoidance", level=LoggingSeverity.DEBUG) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  

PI=3.1415926
FULL_SCAN_NUM = 721
downsampling_gap = 10  # maximum 721

front_clear_width = 0.32 # Half-width of the collision corridor in metres

obstacle_avoidance_range = 1.2

scan_angle_increment = downsampling_gap*PI/360

# The generated CA1 field runs from x=0 to x=7.6 m.  Keep a safety
# margin inside the two lateral edges while travelling to the opposite line.
goal_x = 7.6
left_boundary = 1.75
right_boundary = -1.75
control_period = 0.05

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("obstacle_avoidance")
        self.get_logger().info("Starting Obstacle Avoidance")

        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  # Publish to cmd_vel node
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # The subscriber to the Lidar ranges.
        self.last_scan = None # Copied laser scan message
        self.last_scan_angles = None
        self.last_scan_xy = None
        self.sub_odom = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.pose = None
        self.yaw = None
        self.estimated_x = 0.0
        self.estimated_y = 0.0
        self.avoid_direction = 0.0
        self.finished = False
        self.log_counter = 0
        self.timer = self.create_timer(control_period, self.timer_callback)  # Runs at 20Hz. Can be changed.

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
        indices = np.arange(0, len(msg.ranges), downsampling_gap)
        ranges = np.asarray(msg.ranges, dtype=float)[indices]
        # Treat no return as free space, but reject NaN and readings below the
        # sensor's valid minimum range.
        ranges[np.isposinf(ranges)] = msg.range_max
        ranges[~np.isfinite(ranges)] = msg.range_max
        ranges[ranges < msg.range_min] = msg.range_max
        self.last_scan = ranges
        self.last_scan_angles = msg.angle_min + indices * msg.angle_increment

    def odom_callback(self,msg):
        self.pose = msg
        q = msg.pose.pose.orientation

        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def scan_transfer(self):
        # laser_joint is mounted with yaw=pi in the URDF.  Rotate scan points
        # by pi so +x is the vehicle's forward direction and +y is its left.
        x = -self.last_scan * np.cos(self.last_scan_angles)
        y = -self.last_scan * np.sin(self.last_scan_angles)
        self.last_scan_xy = np.column_stack((x, y))

    def front_clear(self):
        for x, y in self.last_scan_xy:
            if 0.0 < x < obstacle_avoidance_range and abs(y) < front_clear_width:
                return False
        return True

    def choose_avoid_direction(self):
        """Return +1 to pass left or -1 to pass right of an obstacle."""
        x = self.last_scan_xy[:, 0]
        y = self.last_scan_xy[:, 1]
        distance = np.hypot(x, y)
        front = (x > 0.0) & (x < 1.5)
        left = distance[front & (y >= 0.0)]
        right = distance[front & (y < 0.0)]
        left_clearance = np.min(left) if left.size else 1.5
        right_clearance = np.min(right) if right.size else 1.5
        return 1.0 if left_clearance >= right_clearance else -1.0
    
    def timer_callback(self):
        """Controller loop"""
        if self.last_scan is None or self.last_scan_angles is None:
            return


        if self.finished:
            self.move_2D(0.0, 0.0, 0.0)
            return
        
        self.scan_transfer()
        
        ######################## MODIFY CODE HERE ########################
        #self.get_logger().debug(str(self.last_scan))
        #self.move_2D(x=0.2, y=0.0, turn=0.0)

        front_is_clear = self.front_clear()

        # Stay inside the lateral limits first.  Otherwise, retain the chosen
        # passing side until the obstacle has cleared to prevent oscillation.
        if self.estimated_y >= left_boundary:
            self.avoid_direction = -1.0
        elif self.estimated_y <= right_boundary:
            self.avoid_direction = 1.0
        elif not front_is_clear and self.avoid_direction == 0.0:
            self.avoid_direction = self.choose_avoid_direction()

        if not front_is_clear:
            # Stop advancing until the body-width corridor is clear.  Continuing
            # forward here can hit a can before lateral motion has moved the
            # whole robot out of its path.
            command_x = 0.0
            command_y = 0.24 * self.avoid_direction
        else:
            self.avoid_direction = 0.0
            command_x = 0.25
            # Gradually return towards the centre line after passing a can.
            command_y = float(np.clip(-0.8 * self.estimated_y, -0.12, 0.12))

        # Hard boundary guard, including a small look-ahead margin.
        if self.estimated_y > left_boundary - 0.10:
            command_y = min(command_y, -0.12)
        elif self.estimated_y < right_boundary + 0.10:
            command_y = max(command_y, 0.12)

        self.move_2D(command_x, command_y, 0.0)
        self.estimated_x += command_x * control_period
        self.estimated_y += command_y * control_period

        if self.estimated_x >= goal_x:
            self.finished = True
            self.move_2D(0.0, 0.0, 0.0)
            self.get_logger().info("Reached the opposite finish line")

        self.log_counter += 1
        if self.log_counter % 20 == 0:
            self.get_logger().info(
                f"estimated position: x={self.estimated_x:.2f}, "
                f"y={self.estimated_y:.2f}, front_clear={front_is_clear}"
            )

        ######################## MODIFY CODE HERE ########################


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
