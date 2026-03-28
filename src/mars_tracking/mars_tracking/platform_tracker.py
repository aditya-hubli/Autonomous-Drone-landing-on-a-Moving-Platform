#!/usr/bin/env python3
"""Platform tracker - tracks position, estimates velocity, predicts future position."""

import math
from collections import deque
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mars_msgs.msg import PlatformState


class PlatformTracker(Node):
    def __init__(self):
        super().__init__('platform_tracker')

        # Parameters
        self.declare_parameter('history_size', 20)
        self.declare_parameter('prediction_horizon', 1.0)  # seconds ahead
        self.declare_parameter('max_detection_age', 1.0)  # seconds

        history_size = self.get_parameter('history_size').value
        self.prediction_horizon = self.get_parameter('prediction_horizon').value
        self.max_detection_age = self.get_parameter('max_detection_age').value

        # State
        self.position_history = deque(maxlen=history_size)
        self.last_detection_time = None
        self.drone_pose = None

        # Subscribers
        self.aruco_sub = self.create_subscription(
            PoseStamped, '/aruco/detection', self.aruco_callback, 10)
        self.drone_odom_sub = self.create_subscription(
            Odometry, '/drone/odom', self.drone_odom_callback, 10)

        # Publisher
        self.state_pub = self.create_publisher(
            PlatformState, '/platform/tracked_state', 10)

        # Timer for publishing state at 20Hz
        self.create_timer(0.05, self.publish_state)

        self.get_logger().info('Platform tracker initialized')

    def drone_odom_callback(self, msg):
        self.drone_pose = msg.pose.pose

    def aruco_callback(self, msg):
        """Convert camera-relative detection to world-frame position."""
        if self.drone_pose is None:
            return

        # ArUco tvec is in the camera optical frame:
        #   optical +X = right in image
        #   optical +Y = down in image
        #   optical +Z = into the scene (forward)
        #
        # Camera mounted at pitch=-90° (looking straight down):
        #   sensor +X → world -Z,  sensor +Y → world +Y,  sensor +Z → world +X
        #   optical +Z = sensor +X = world -Z
        #   optical +X = -sensor Y = world -Y
        #   optical +Y = -sensor Z = world -X
        #
        # So: world_x = drone_x - cam_y
        #     world_y = drone_y - cam_x
        #     world_z = drone_z - cam_z
        cam_x = msg.pose.position.x
        cam_y = msg.pose.position.y
        cam_z = msg.pose.position.z

        drone_x = self.drone_pose.position.x
        drone_y = self.drone_pose.position.y
        drone_z = self.drone_pose.position.z

        world_x = drone_x - cam_y
        world_y = drone_y - cam_x
        world_z = drone_z - cam_z

        now = self.get_clock().now()
        self.position_history.append({
            'x': world_x,
            'y': world_y,
            'z': world_z,
            'time': now,
        })
        self.last_detection_time = now

    def estimate_velocity(self):
        """Estimate platform velocity from position history."""
        if len(self.position_history) < 2:
            return 0.0, 0.0, 0.0

        # Use last few samples for velocity estimation
        n = min(len(self.position_history), 5)
        recent = list(self.position_history)[-n:]

        dt_total = (recent[-1]['time'] - recent[0]['time']).nanoseconds / 1e9
        if dt_total < 0.01:
            return 0.0, 0.0, 0.0

        vx = (recent[-1]['x'] - recent[0]['x']) / dt_total
        vy = (recent[-1]['y'] - recent[0]['y']) / dt_total
        vz = (recent[-1]['z'] - recent[0]['z']) / dt_total

        return vx, vy, vz

    def predict_position(self, pos_x, pos_y, pos_z, vx, vy, vz):
        """Linear prediction of future platform position."""
        dt = self.prediction_horizon
        pred_x = pos_x + vx * dt
        pred_y = pos_y + vy * dt
        pred_z = pos_z + vz * dt
        return pred_x, pred_y, pred_z

    def publish_state(self):
        """Publish tracked platform state."""
        msg = PlatformState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        now = self.get_clock().now()

        if (self.last_detection_time is not None and
                len(self.position_history) > 0):

            age = (now - self.last_detection_time).nanoseconds / 1e9
            msg.is_detected = age < self.max_detection_age

            latest = self.position_history[-1]
            msg.position.x = latest['x']
            msg.position.y = latest['y']
            msg.position.z = latest['z']

            vx, vy, vz = self.estimate_velocity()
            msg.velocity.x = vx
            msg.velocity.y = vy
            msg.velocity.z = vz

            pred_x, pred_y, pred_z = self.predict_position(
                latest['x'], latest['y'], latest['z'], vx, vy, vz)
            msg.predicted_position.x = pred_x
            msg.predicted_position.y = pred_y
            msg.predicted_position.z = pred_z

            msg.confidence = max(0.0, 1.0 - age / self.max_detection_age)
        else:
            msg.is_detected = False
            msg.confidence = 0.0

        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlatformTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
