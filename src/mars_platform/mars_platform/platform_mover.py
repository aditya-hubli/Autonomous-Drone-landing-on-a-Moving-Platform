#!/usr/bin/env python3
"""Platform mover - back and forth along X-axis only.

The planar_move plugin uses body frame for cmd_vel.
We keep yaw=0, so body X = world X. No conversion needed.
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

        self.pos_x = 2.0  # match spawn position
        self.yaw = 0.0
        self.target_vx = 0.0
        self.current_vx = 0.0
        self.walk_timer = 0.0
        self.dt = 0.05
        self.create_timer(self.dt, self.move_callback)

        self.get_logger().info(
            f'Platform mover (X-axis): boundary=±{self.boundary}m, speed={self.max_speed}m/s')

    def odom_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def mission_callback(self, msg):
        if msg.data == 'LANDED' and not self.stopped:
            self.stopped = True
            self.cmd_pub.publish(Twist())  # zero velocity
            self.get_logger().info('Drone landed — platform stopping.')

    def _pick_new_target(self):
        """Pick a random target velocity that is never near zero."""
        speed = random.uniform(0.5 * self.max_speed, self.max_speed)
        direction = random.choice([-1, 1])

        # If near a boundary, always head back toward center
        if self.pos_x > self.boundary * 0.6:
            direction = -1
        elif self.pos_x < -self.boundary * 0.6:
            direction = 1

        self.target_vx = speed * direction
        self.walk_timer = random.uniform(2.0, 4.0)

    def move_callback(self):
        if self.stopped:
            self.cmd_pub.publish(Twist())
            return

        cmd = Twist()

        # Pick a new target periodically
        self.walk_timer -= self.dt
        if self.walk_timer <= 0:
            self._pick_new_target()

        # Smooth acceleration (fast enough to actually reach target)
        self.current_vx += 0.1 * (self.target_vx - self.current_vx)
        vx = max(-self.max_speed, min(self.max_speed, self.current_vx))

        # Hard boundary: reverse immediately if past limit
        if self.pos_x > self.boundary and vx > 0:
            vx = -self.max_speed
            self.target_vx = vx
            self.current_vx = vx
        elif self.pos_x < -self.boundary and vx < 0:
            vx = self.max_speed
            self.target_vx = vx
            self.current_vx = vx

        # Body frame: since yaw≈0, linear.x ≈ world X
        cmd.linear.x = vx
        cmd.linear.y = 0.0
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
