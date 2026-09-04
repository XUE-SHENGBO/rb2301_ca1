import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
np.set_printoptions(
    2, suppress=True
)  # Print numpy arrays to specified d.p. and suppress scientific notation (e.g. 1e-5)

max_translate_velocity = 0.4 # Can be implemented as parameter
max_turn_velocity = max_translate_velocity * 2 # Can be implemented as parameter
set_logger_level("obstacle_avoidance", level=LoggingSeverity.DEBUG) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  

PI=3.1415926
MAX_SCAN_NUM=721
scan_gap = 10
scan_num = math.floor(MAX_SCAN_NUM/scan_gap)
scan_increment = scan_gap*(PI/720)

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("obstacle_avoidance")
        self.get_logger().info("Starting Obstacle Avoidance")

        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  # Publish to cmd_vel node
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # The subscriber to the Lidar ranges.
        self.last_scan = None # Copied laser scan message

        self.timer = self.create_timer(0.05, self.timer_callback)  # Runs at 20Hz. Can be changed. 

        self.state='move_forward'

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
        self.last_scan = np.array(msg.ranges)[::scan_gap] # Slices the 721 scan array to return only 36 scans. Feel free to edit

    def transfer(self):
        if self.last_scan is None:
            return
        self.last_scan_xy=[]
        for i in range(0,scan_num):
            if self.last_scan[i] != float('inf'):
                x = -self.last_scan[i]*math.cos(PI/2-i*scan_increment)
                y = -self.last_scan[i]*math.sin(PI/2-i*scan_increment)
                self.last_scan_xy.append((x,y))

    def front_clear(self):
        for obstacle_dot in self.last_scan_xy:
            x=obstacle_dot[0]
            y=obstacle_dot[1]
            if x>-0.15 and x<0.15 and y>-0.2 and y<0.5:
                return False
        return True

    def timer_callback(self):
        """Controller loop"""

        if self.last_scan is None:
            return # Does not run if the laser message is not received.

        self.transfer()
        ######################## MODIFY CODE HERE ########################
        #self.get_logger().debug(str(self.last_scan))

        #self.move_2D(0.2, 0.0, 0.0)
        front_state = self.front_clear()
        if self.state == 'move_forward':
            self.move_2D(0.2,0,0)
            if front_state == False:
                self.state = 'move_left'
        elif self.state == 'move_left':
            self.move_2D(0,0.2,0)
            if front_state == True:
                self.state = 'move_forward'
        elif self.state == 'move_right':   
            self.move_2D(0,-0.2,0)
            if front_state == True:
                self.state = 'move_forward'
        

        ######################## MODIFY CODE HERE ########################


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()