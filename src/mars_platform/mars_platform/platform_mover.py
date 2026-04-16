#!/usr/bin/env python3
"""Platform mover - moves in the XY horizontal plane within a bounding box.

Travels at constant speed in a straight line. When hitting a wall boundary,
the velocity component perpendicular to the wall is reflected, producing a
90-degree rebound (billiard-ball bounce at 45-degree incidence angles).

The planar_move plugin uses body frame for cmd_vel.
We keep yaw=0, so body X = world X, body Y = world Y.
"""

import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class PlatformMover(Node):
    def __init__(self):
        super().__init__('platform_mover')

        self.declare_parameter('max_speed', 0.15)
        self.declare_parameter('boundary', 2.5)

        self.max_speed = self.get_parameter('max_speed').value
        self.boundary = self.get_parameter('boundary').value

        self.cmd_pub = self.create_publisher(Twist, '/platform/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/platform/odom', self.odom_callback, 10)
        self.mission_sub = self.create_subscription(
            String, '/mission/state', self.mission_callback, 10)
        self.stopped = False

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0

        # Start moving at a 45-degree angle for clean 90-degree wall rebounds
        angle = random.choice([
            math.pi / 4,       # +X +Y
            3 * math.pi / 4,   # -X +Y
            -math.pi / 4,      # +X -Y
            -3 * math.pi / 4,  # -X -Y
        ])
        self.vx = self.max_speed * math.cos(angle)
        self.vy = self.max_speed * math.sin(angle)

        self.dt = 0.05
        self.create_timer(self.dt, self.move_callback)

        self.get_logger().info(
            f'Platform mover (XY plane): boundary=±{self.boundary}m, '
            f'speed={self.max_speed}m/s')

    def odom_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def mission_callback(self, msg):
        if msg.data == 'LANDED' and not self.stopped:
            self.stopped = True
            self.cmd_pub.publish(Twist())
            self.get_logger().info('Drone landed — platform stopping.')

    def move_callback(self):
        if self.stopped:
            self.cmd_pub.publish(Twist())
            return

        # Wall rebounds: reflect the velocity component perpendicular to the wall
        if self.pos_x >= self.boundary and self.vx > 0:
            self.vx = -self.vx
            self.get_logger().info('Rebound off +X wall', throttle_duration_sec=1.0)
        elif self.pos_x <= -self.boundary and self.vx < 0:
            self.vx = -self.vx
            self.get_logger().info('Rebound off -X wall', throttle_duration_sec=1.0)

        if self.pos_y >= self.boundary and self.vy > 0:
            self.vy = -self.vy
            self.get_logger().info('Rebound off +Y wall', throttle_duration_sec=1.0)
        elif self.pos_y <= -self.boundary and self.vy < 0:
            self.vy = -self.vy
            self.get_logger().info('Rebound off -Y wall', throttle_duration_sec=1.0)

        cmd = Twist()
        cmd.linear.x = self.vx
        cmd.linear.y = self.vy
        # Keep yaw locked at 0
        cmd.angular.z = -self.yaw * 3.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PlatformMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
