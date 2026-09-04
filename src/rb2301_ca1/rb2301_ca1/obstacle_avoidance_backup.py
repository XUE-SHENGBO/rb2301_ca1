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

front_clear_width = 0.2 # Half-width of the collision corridor in metres

obstacle_avoidance_range = 0.2

scan_angle_increment = downsampling_gap*PI/360

# The generated CA1 field runs from x=0 to x=7.6 m.  Keep a safety
# margin inside the two lateral edges while travelling to the opposite line.
goal_x = 7.6
left_boundary = 1.75
right_boundary = -1.75
control_period = 0.05

class State:
    def __init__(self,name,vector=None):
        self.name=name
        self.vector = vector
straight = State('straight',None)
turn_left = State('turn_left',(0,1))
turn_right = State('turn_right',(0,-1))
state = (straight,turn_left,turn_right)

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

    def scan_transfer(self):
        # laser_joint is mounted with yaw=pi in the URDF.  Rotate scan points
        # by pi so +x is the vehicle's forward direction and +y is its left.
        last_scan = self.last_scan
        last_scan_angles = self.last_scan_angles
        if last_scan is None or last_scan_angles is None:
            return

        x = -last_scan * np.cos(last_scan_angles)
        y = -last_scan * np.sin(last_scan_angles)
        self.last_scan_xy = np.column_stack((x, y))
    def front_clear(self):
        
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

        

        ######################## MODIFY CODE HERE ########################


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


#if __name__ == "__main__":
#    main()
