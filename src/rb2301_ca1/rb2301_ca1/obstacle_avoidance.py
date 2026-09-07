import numpy as np
import rclpy
import math 
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

np.set_printoptions(
    2, suppress=True
)  # Print numpy arrays to specified d.p. and suppress scientific notation (e.g. 1e-5)

max_translate_velocity = 0.4 # Can be implemented as parameter
max_turn_velocity = max_translate_velocity * 2 # Can be implemented as parameter

CONTROL_PERIOD = 0.05
FRONT_DETECTION_DISTANCE = 1.4
FRONT_STOP_DISTANCE = 0.12
# Collision half-width is about 0.096 m; laser lies on the robot centerline.
SIDE_DETECTION_DISTANCE = 0.16  # Start correcting about 6 cm from the body.
SIDE_STOP_DISTANCE = 0.12       # Keep about 2 cm from the body.
CRUISE_VELOCITY = 0.4
MAX_LATERAL_VELOCITY = 0.2
LATERAL_ACCELERATION = 0.6
AVOIDANCE_DEADBAND = 0.05
BYPASS_ESCAPE_GAIN = 0.6
BYPASS_RELEASE_DISTANCE = FRONT_DETECTION_DISTANCE + 0.15
BYPASS_CLEAR_SCANS = 3
# Static collision footprint in base_link coordinates, including the wheels.
LASER_X = 0.11423
BODY_REAR_X = -0.034
BODY_FRONT_X = 0.182
SIDE_GUARD_MARGIN = 0.02

set_logger_level("obstacle_avoidance", level=LoggingSeverity.DEBUG) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("obstacle_avoidance")
        self.get_logger().info("Starting Obstacle Avoidance")

        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)  # Publish to cmd_vel node
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # The subscriber to the Lidar ranges.
        self.last_scan = None # Copied laser scan message
        self.last_scan_msg = None
        self.lateral_velocity = 0.0
        self.bypass_direction = 0  # +1 left, -1 right; retained while bypassing.
        self.front_clear_scans = 0
        self.scan_sequence = 0
        self.last_bypass_scan = -1

        self.timer = self.create_timer(CONTROL_PERIOD, self.timer_callback)  # 20 Hz

    def move_2D(self, x: float = 0.0, y: float = 0.0, turn: float = 0.0):
        """Publishes a twist command to move in 2D space. +ve x is forwards, +ve y is left, and +ve turn is anticlockwise"""
        twist_msg = Twist()
        x = np.clip(x, -max_translate_velocity, max_translate_velocity)
        y = np.clip(y, -max_translate_velocity, max_translate_velocity)
        turn = np.clip(turn, -max_translate_velocity*2, max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.pub_cmd_vel.publish(twist_msg)

    def pivot(self):
        if self.last_scan_msg is None:
            return

        ranges, angles, valid = self.scan_in_body_frame(self.last_scan_msg)
        angles = (angles + 180.0) % 360.0 - 180.0

        # Front 60 degrees; the remaining front-facing 60 degrees on each side.
        front_left = (angles >= 0.0) & (angles <= 30.0)
        front_right = (angles >= -30.0) & (angles < 0.0)
        left_side = (angles > 30.0) & (angles <= 90.0)
        right_side = (angles >= -90.0) & (angles < -30.0)

        # Gazebo +inf means no return; NaN/out-of-range data is not free space.
        known = valid | np.isposinf(ranges)
        if any(not np.any(known & zone) for zone in
               (front_left, front_right, left_side, right_side)):
            self.lateral_velocity = 0.0
            self.front_clear_scans = 0
            self.move_2D()
            return

        def nearest(zone, lateral=False):
            selected = valid & zone
            if not np.any(selected):
                return math.inf
            distances = ranges[selected]
            if lateral:
                distances = distances * np.abs(np.sin(np.deg2rad(angles[selected])))
            return float(np.min(distances))

        def urgency(distance, detection, stop):
            return float(np.clip(
                (detection - distance) / (detection - stop), 0.0, 1.0
            ))

        front_left_distance = nearest(front_left)
        front_right_distance = nearest(front_right)
        left_clearance = nearest(left_side, lateral=True)
        right_clearance = nearest(right_side, lateral=True)

        # Check the whole body's lateral sweep, including points behind the laser.
        # A wall ahead of the footprint must not be mistaken for blocked flanks.
        ray_angles = np.deg2rad(angles[valid])
        body_x = LASER_X + ranges[valid] * np.cos(ray_angles)
        body_y = ranges[valid] * np.sin(ray_angles)
        alongside = ((body_x >= BODY_REAR_X - SIDE_GUARD_MARGIN)
                     & (body_x <= BODY_FRONT_X + SIDE_GUARD_MARGIN))
        left_points = body_y[alongside & (body_y > 0.0)]
        right_points = -body_y[alongside & (body_y < 0.0)]
        left_sweep_clearance = float(left_points.min()) if left_points.size else math.inf
        right_sweep_clearance = float(right_points.min()) if right_points.size else math.inf

        front_left_urgency = urgency(
            front_left_distance, FRONT_DETECTION_DISTANCE, FRONT_STOP_DISTANCE)
        front_right_urgency = urgency(
            front_right_distance, FRONT_DETECTION_DISTANCE, FRONT_STOP_DISTANCE)
        left_side_urgency = urgency(
            left_clearance, SIDE_DETECTION_DISTANCE, SIDE_STOP_DISTANCE)
        right_side_urgency = urgency(
            right_clearance, SIDE_DETECTION_DISTANCE, SIDE_STOP_DISTANCE)
        left_sweep_urgency = urgency(
            left_sweep_clearance, SIDE_DETECTION_DISTANCE, SIDE_STOP_DISTANCE)
        right_sweep_urgency = urgency(
            right_sweep_clearance, SIDE_DETECTION_DISTANCE, SIDE_STOP_DISTANCE)

        # Compare normalized urgency, rather than distances with different caps.
        left_urgency = max(front_left_urgency, left_side_urgency, left_sweep_urgency)
        right_urgency = max(front_right_urgency, right_side_urgency, right_sweep_urgency)
        direction = right_urgency - left_urgency  # Positive y moves left.
        front_distance = min(front_left_distance, front_right_distance)
        self.update_bypass_direction(
            front_distance, direction, left_sweep_clearance, right_sweep_clearance
        )
        target_lateral = (
            0.0 if abs(direction) < AVOIDANCE_DEADBAND
            else MAX_LATERAL_VELOCITY * direction
        )
        if self.bypass_direction and front_distance < FRONT_DETECTION_DISTANCE:
            # Symmetric front returns still require a nonzero sideways command.
            escape = BYPASS_ESCAPE_GAIN * max(front_left_urgency, front_right_urgency)
            target_lateral = (
                self.bypass_direction * MAX_LATERAL_VELOCITY
                * max(abs(direction), escape)
            )

        # Limit ordinary changes; clearance constraints below act immediately.
        step = LATERAL_ACCELERATION * CONTROL_PERIOD
        lateral = self.lateral_velocity + float(np.clip(
            target_lateral - self.lateral_velocity, -step, step
        ))
        lateral = float(np.clip(
            lateral,
            -MAX_LATERAL_VELOCITY * (1.0 - right_sweep_urgency),
            MAX_LATERAL_VELOCITY * (1.0 - left_sweep_urgency),
        ))

        limited_front = float(np.clip(
            front_distance, FRONT_STOP_DISTANCE, FRONT_DETECTION_DISTANCE
        ))
        forward = (
            CRUISE_VELOCITY * math.log(limited_front / FRONT_STOP_DISTANCE)
            / math.log(FRONT_DETECTION_DISTANCE / FRONT_STOP_DISTANCE)
        )
        if min(left_clearance, right_clearance) <= SIDE_STOP_DISTANCE:
            forward = 0.0

        self.lateral_velocity = lateral
        self.move_2D(forward, lateral, 0.0)
        self.get_logger().debug(
            f"front={front_distance:.2f}, side_left={left_clearance:.2f}, "
            f"side_right={right_clearance:.2f}, vx={forward:.2f}, vy={lateral:.2f}, "
            f"bypass={self.bypass_direction}"
        )

    def update_bypass_direction(self, front_distance, direction, left_clearance, right_clearance):
        """Commit to one passing side until clear, unless that flank is blocked."""
        if self.scan_sequence != self.last_bypass_scan:
            self.last_bypass_scan = self.scan_sequence
            if front_distance > BYPASS_RELEASE_DISTANCE:
                self.front_clear_scans += 1
            else:
                self.front_clear_scans = 0
            if self.front_clear_scans >= BYPASS_CLEAR_SCANS:
                self.bypass_direction = 0

        left_available = left_clearance > SIDE_STOP_DISTANCE
        right_available = right_clearance > SIDE_STOP_DISTANCE
        if self.bypass_direction == 1 and not left_available:
            if right_clearance >= SIDE_DETECTION_DISTANCE:
                self.bypass_direction = -1
        elif self.bypass_direction == -1 and not right_available:
            if left_clearance >= SIDE_DETECTION_DISTANCE:
                self.bypass_direction = 1

        if self.bypass_direction == 0 and front_distance < FRONT_DETECTION_DISTANCE:
            if left_available and not right_available:
                self.bypass_direction = 1
            elif right_available and not left_available:
                self.bypass_direction = -1
            elif left_available and right_available:
                if abs(direction) >= AVOIDANCE_DEADBAND:
                    self.bypass_direction = 1 if direction > 0.0 else -1
                else:
                    # Compare only nearby clearance; an exact tie consistently picks left.
                    left_room = min(left_clearance, SIDE_DETECTION_DISTANCE)
                    right_room = min(right_clearance, SIDE_DETECTION_DISTANCE)
                    self.bypass_direction = 1 if left_room >= right_room else -1

    def scan_in_body_frame(self, msg, laser_yaw=3.1416):
        """Return raw ranges, body-relative angles in degrees, and valid-ray mask."""
        _ranges = np.asarray(msg.ranges, dtype=float)

        # Ray angles in the laser frame.
        angles = msg.angle_min + np.arange(_ranges.size) * msg.angle_increment

        # Apply mounting yaw, then wrap to [0, 360): forward=0, left=90, right=270.
        # Round conversion noise so exact boundaries fall in the next sector.
        angles = np.round(np.rad2deg(angles + laser_yaw), decimals=10) % 360.0

        # Only average valid readings.
        valid = (
            np.isfinite(_ranges)
            & (_ranges >= msg.range_min)
            & (_ranges <= msg.range_max)
        )

        return _ranges, angles, valid

    def front_sector_averages(self, msg, laser_yaw=3.1416):
        """Average 12 full-circle 30-degree sectors for diagnostics."""
        _ranges, angles, valid = self.scan_in_body_frame(msg, laser_yaw)
        edges = np.linspace(0.0, 360.0, 13)
        averages = np.full(12, np.inf)

        for i, (lo, hi) in enumerate(zip(edges[:-1], edges[1:])):
            # Half-open intervals avoid gaps and double-counting boundary rays.
            mask = valid & (angles >= lo) & (angles < hi)

            if mask.any():
                averages[i] = _ranges[mask].mean()

        return averages

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        self.scan_sequence += 1
        self.last_scan_msg = msg
        self.front_sector = self.front_sector_averages(msg)
        self.last_scan = np.asarray(msg.ranges, dtype=float)

    def timer_callback(self):
        """Controller loop"""

        if self.last_scan is None:
            return # Does not run if the laser message is not received.
        
        ######################## MODIFY CODE HERE ########################
        
        self.get_logger().debug(str(self.front_sector))
        self.pivot()
        

        ######################## MODIFY CODE HERE ########################


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
