#!/usr/bin/env python3
"""
Platform detector node.

Uses ground-truth odometry to simulate camera-based detection, producing the same
output as a real ArUco/color detector (PoseStamped in camera optical frame).

This works around Gazebo Classic camera rendering issues where spawned models
are invisible to camera sensors.

The output is in camera optical frame coordinates to keep the downstream
tracker/controller pipeline unchanged.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.drone_pos = None
        self.platform_pos = None

        # Camera properties (must match drone model.sdf)
        # Camera mounted at pitch=-90 (looking down) on base_link
        # Horizontal FOV = 2.0944 rad (120°), image 800x600
        self.hfov = 2.0944
        self.vfov = self.hfov * 600.0 / 800.0  # ~90°

        # Subscribers
        self.drone_odom_sub = self.create_subscription(
            Odometry, '/drone/odom', self.drone_odom_cb, 10)
        self.platform_odom_sub = self.create_subscription(
            Odometry, '/platform/odom', self.platform_odom_cb, 10)

        # Publisher — same topic/frame as a real ArUco detector
        self.pose_pub = self.create_publisher(
            PoseStamped, '/aruco/detection', 10)

        # Run detection at 20Hz (matches camera ~30Hz but lighter)
        self.create_timer(0.05, self.detect)

        self.get_logger().info('Platform detector initialized (ground-truth mode)')

    def drone_odom_cb(self, msg):
        self.drone_pos = msg.pose.pose.position

    def platform_odom_cb(self, msg):
        self.platform_pos = msg.pose.pose.position

    def detect(self):
        if self.drone_pos is None or self.platform_pos is None:
            return

        # Vector from drone to platform in world frame
        dx = self.platform_pos.x - self.drone_pos.x
        dy = self.platform_pos.y - self.drone_pos.y
        dz = self.platform_pos.z - self.drone_pos.z  # negative (platform below)

        # Distance below drone (camera looks down, so "depth" = -dz for downward cam)
        depth = -dz  # positive when platform is below drone
        if depth < 0.1:
            return  # platform above or at drone level — not in view

        # Check if platform is within camera FOV
        # Camera looking straight down: angular offset in X and Y
        angle_x = math.atan2(abs(dx), depth)
        angle_y = math.atan2(abs(dy), depth)
        if angle_x > self.hfov / 2 or angle_y > self.vfov / 2:
            return  # outside camera FOV

        # Convert world-frame offset to camera optical frame
        # Camera at pitch=-90° (looking down):
        #   optical X = -world Y (right in image)
        #   optical Y = -world X (down in image — but sign depends on convention)
        #   optical Z = -world Z relative to drone (depth into scene)
        #
        # The tracker does the reverse:
        #   world_x = drone_x - cam_y
        #   world_y = drone_y - cam_x
        #   world_z = drone_z - cam_z
        #
        # So we need: cam_y = -dx, cam_x = -dy, cam_z = -dz = depth
        cam_x = -dy
        cam_y = -dx
        cam_z = depth

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'drone/camera_link'
        pose_msg.pose.position.x = float(cam_x)
        pose_msg.pose.position.y = float(cam_y)
        pose_msg.pose.position.z = float(cam_z)
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)

        self.get_logger().info(
            f'Platform detected: world_offset=({dx:.2f},{dy:.2f},{dz:.2f}) '
            f'cam=({cam_x:.2f},{cam_y:.2f},{cam_z:.2f})',
            throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
