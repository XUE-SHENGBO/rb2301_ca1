import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

np.set_printoptions(
    2, suppress=True
)  # Print numpy arrays to specified d.p. and suppress scientific notation (e.g. 1e-5)

max_translate_velocity = 0.4 # Can be implemented as parameter
max_turn_velocity = max_translate_velocity * 2 # Can be implemented as parameter
set_logger_level("obstacle_avoidance", level=LoggingSeverity.DEBUG) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("obstacle_avoidance")
        self.get_logger().info("Starting Obstacle Avoidance")
        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  # Publish to cmd_vel node
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # The subscriber to the Lidar ranges.
        self.last_scan = None # Copied laser scan message

        self.timer = self.create_timer(0.5, self.timer_callback)  # Runs at 20Hz. Can be changed.

        #New added 
        self.last_direction = 1

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
        self.last_scan = np.array(msg.ranges)[::36] # Slices the 721 scan array to return only 36 scans. Feel free to edit

    def timer_callback(self):

        if self.last_scan is None:
            return

        self.get_logger().debug(str(self.last_scan))
        threshold = 0.25

        # LiDAR sectors
        front = np.concatenate((self.last_scan[-2:], self.last_scan[:3]))
        right = self.last_scan[3:8]
        left = self.last_scan[13:18]

        front_distance = np.min(front)

        # Replace inf with a large distance for comparison
        left_clean = left.copy()
        right_clean = right.copy()

        left_clean[left_clean == np.inf] = 10.0
        right_clean[right_clean == np.inf] = 10.0

        left_space = np.mean(left_clean)
        right_space = np.mean(right_clean)

        # Obstacle ahead
        if front_distance < threshold:
            if left_space - right_space > 0.5:
                self.last_direction = 1
            elif right_space - left_space > 0.5:
                self.last_direction = -1
            self.move_2D(0.0,0.3 * self.last_direction, 0.0)
        else:
            #no obstacle ahead, move forward
            self.move_2D(max_translate_velocity,0.0,0.0)
    


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()