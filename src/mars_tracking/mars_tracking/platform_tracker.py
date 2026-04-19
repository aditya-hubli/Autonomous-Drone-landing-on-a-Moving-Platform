#!/usr/bin/env python3
"""Platform tracker - tracks the platform using drone camera images and ArUco detection."""

from collections import deque

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from mars_msgs.msg import PlatformState
import tf2_ros


class PlatformTracker(Node):
    def __init__(self):
        super().__init__('platform_tracker')

        # Parameters
        self.declare_parameter('history_size', 20)
        self.declare_parameter('prediction_horizon', 0.3)  # seconds ahead
        self.declare_parameter('max_detection_age', 1.5)  # seconds
        self.declare_parameter('max_missed_frames', 4)
        self.declare_parameter('velocity_alpha', 0.7)
        self.declare_parameter('camera_topic', '/drone/camera/image_raw')
        self.declare_parameter('camera_frame', 'drone/camera_link')

        history_size = self.get_parameter('history_size').value
        self.prediction_horizon = self.get_parameter('prediction_horizon').value
        self.max_detection_age = self.get_parameter('max_detection_age').value
        self.max_missed_frames = self.get_parameter('max_missed_frames').value
        self.velocity_alpha = self.get_parameter('velocity_alpha').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value

        # State
        self.position_history = deque(maxlen=history_size)
        self.last_detection_time = None
        self.last_position = None
        self.drone_pose = None
        self.filtered_velocity = np.array([0.0, 0.0, 0.0], dtype=float)
        self.last_detection = False
        self.missed_count = 0

        # Camera and detection
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.marker_size = 0.2
        self.camera_matrix = np.array(
            [[600.0, 0.0, 400.0],
             [0.0, 600.0, 300.0],
             [0.0, 0.0, 1.0]], dtype=float)
        self.dist_coeffs = np.zeros((5, 1), dtype=float)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile_sensor_data)
        self.drone_odom_sub = self.create_subscription(
            Odometry, '/drone/odom', self.drone_odom_callback, 10)

        # Publisher
        self.state_pub = self.create_publisher(
            PlatformState, '/platform/tracked_state', 10)

        # Timer for publishing state at 20Hz
        self.create_timer(0.05, self.publish_state)

        self.get_logger().info('Platform tracker initialized with camera-based detection')

    def drone_odom_callback(self, msg):
        self.drone_pose = msg.pose.pose

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'drone/base_link'
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

        now = self.get_clock().now()
        detected = ids is not None and len(ids) > 0

        if not detected:
            self.missed_count += 1
            if self.missed_count <= self.max_missed_frames:
                # Short drop: keep using the last known platform state.
                if self.last_detection:
                    self.get_logger().warn(
                        'Transient marker drop; holding last known platform state',
                        throttle_duration_sec=2.0)
                return

            if self.last_detection:
                self.get_logger().warn(
                    'Platform marker lost after repeated misses; publishing stale state',
                    throttle_duration_sec=2.0)
            self.last_detection = False
            return

        # Reset transient-drop state when a marker is detected again.
        self.missed_count = 0

        # Use the first detected marker for platform pose estimation.
        _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
        if tvecs is None or len(tvecs) == 0:
            return

        cam_x = float(tvecs[0][0][0])
        cam_y = float(tvecs[0][0][1])
        cam_z = float(tvecs[0][0][2])

        pose_camera = PoseStamped()
        pose_camera.header.stamp = now.to_msg()
        pose_camera.header.frame_id = self.camera_frame
        pose_camera.pose.position.x = cam_x
        pose_camera.pose.position.y = cam_y
        pose_camera.pose.position.z = cam_z
        pose_camera.pose.orientation.w = 1.0

        world_x = None
        world_y = None
        world_z = None

        if self.drone_pose is not None:
            drone_x = self.drone_pose.position.x
            drone_y = self.drone_pose.position.y
            drone_z = self.drone_pose.position.z
            world_x = drone_x - cam_y
            world_y = drone_y - cam_x
            world_z = drone_z - cam_z
        else:
            try:
                world_pose = self.tf_buffer.transform(
                    pose_camera,
                    'world',
                    timeout=Duration(seconds=0.2))
                world_x = world_pose.pose.position.x
                world_y = world_pose.pose.position.y
                world_z = world_pose.pose.position.z
            except Exception as exc:
                self.get_logger().warn(
                    f'Unable to transform camera pose to world frame: {exc}')
                self.last_detection = False
                return

        if self.last_position is not None and self.last_detection_time is not None:
            dt = (now - self.last_detection_time).nanoseconds / 1e9
            if dt > 1e-3:
                raw_vx = (world_x - self.last_position[0]) / dt
                raw_vy = (world_y - self.last_position[1]) / dt
                raw_vz = (world_z - self.last_position[2]) / dt
                raw_velocity = np.array([raw_vx, raw_vy, raw_vz], dtype=float)
                self.filtered_velocity = (
                    self.velocity_alpha * raw_velocity +
                    (1.0 - self.velocity_alpha) * self.filtered_velocity)

        self.last_position = np.array([world_x, world_y, world_z], dtype=float)
        self.position_history.append({
            'x': world_x,
            'y': world_y,
            'z': world_z,
            'time': now,
        })
        self.last_detection_time = now

        if not self.last_detection:
            self.get_logger().info('Platform marker detected', throttle_duration_sec=2.0)
        self.last_detection = True

    def predict_position(self, pos_x, pos_y, pos_z, vx, vy, vz):
        dt = self.prediction_horizon
        return pos_x + vx * dt, pos_y + vy * dt, pos_z + vz * dt

    def publish_state(self):
        msg = PlatformState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        if self.last_position is not None:
            age = 0.0
            if self.last_detection_time is not None:
                age = (
                    self.get_clock().now() - self.last_detection_time
                ).nanoseconds / 1e9

            msg.position.x = float(self.last_position[0])
            msg.position.y = float(self.last_position[1])
            msg.position.z = float(self.last_position[2])

            msg.velocity.x = float(self.filtered_velocity[0])
            msg.velocity.y = float(self.filtered_velocity[1])
            msg.velocity.z = float(self.filtered_velocity[2])

            pred_x, pred_y, pred_z = self.predict_position(
                msg.position.x,
                msg.position.y,
                msg.position.z,
                msg.velocity.x,
                msg.velocity.y,
                msg.velocity.z,
            )
            msg.predicted_position.x = pred_x
            msg.predicted_position.y = pred_y
            msg.predicted_position.z = pred_z

            msg.is_detected = age < self.max_detection_age
            if msg.is_detected:
                freshness = max(0.0, min(age, self.max_detection_age) / self.max_detection_age)
                msg.confidence = 0.6 + 0.4 * (1.0 - freshness)
            else:
                msg.confidence = 0.0
        else:
            msg.is_detected = False
            msg.confidence = 0.0
            msg.position.x = 0.0
            msg.position.y = 0.0
            msg.position.z = 0.0
            msg.velocity.x = 0.0
            msg.velocity.y = 0.0
            msg.velocity.z = 0.0
            msg.predicted_position.x = 0.0
            msg.predicted_position.y = 0.0
            msg.predicted_position.z = 0.0

        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlatformTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
