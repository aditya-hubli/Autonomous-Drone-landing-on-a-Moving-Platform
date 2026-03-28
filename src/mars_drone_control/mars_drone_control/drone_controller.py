#!/usr/bin/env python3
"""
Drone controller - land on platform moving along X-axis.
Drone plugin uses WORLD frame, so cmd_vel.linear.x = world X directly.

States: TAKEOFF -> SEARCH -> TRACK -> APPROACH -> DESCEND -> LAND -> LANDED
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from mars_msgs.msg import PlatformState


class DroneController(Node):

    TAKEOFF = 'TAKEOFF'
    SEARCH = 'SEARCH'
    TRACK = 'TRACK'
    APPROACH = 'APPROACH'
    DESCEND = 'DESCEND'
    LAND = 'LAND'
    LANDED = 'LANDED'

    def __init__(self):
        super().__init__('drone_controller')

        self.declare_parameter('hover_height', 3.0)
        self.declare_parameter('approach_height', 2.0)
        self.declare_parameter('descend_speed', 0.8)
        self.declare_parameter('land_height', 0.55)
        self.declare_parameter('xy_tolerance', 0.2)

        self.hover_height = self.get_parameter('hover_height').value
        self.approach_height = self.get_parameter('approach_height').value
        self.descend_speed = self.get_parameter('descend_speed').value
        self.land_height = self.get_parameter('land_height').value
        self.xy_tolerance = self.get_parameter('xy_tolerance').value

        self.state = self.TAKEOFF
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.odom_received = False
        self.platform_state = None
        self.sweep_target = 2.5
        self.aligned_count = 0

        self.odom_sub = self.create_subscription(
            Odometry, '/drone/odom', self.odom_cb, 10)
        self.platform_sub = self.create_subscription(
            PlatformState, '/platform/tracked_state', self.platform_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Drone controller started (1D X-axis mode)')

    def odom_cb(self, msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        self.drone_z = msg.pose.pose.position.z
        self.odom_received = True

    def platform_cb(self, msg):
        self.platform_state = msg

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def hold_y_and_height(self, target_z):
        """Return (vy, vz) to keep Y=0 and hold target altitude."""
        vy = self.clamp(-self.drone_y * 2.0, -2.0, 2.0)
        vz = self.clamp((target_z - self.drone_z) * 2.0, -2.0, 2.0)
        return vy, vz

    def send(self, vx=0.0, vy=0.0, vz=0.0):
        cmd = Twist()
        cmd.linear.x = self.clamp(vx, -4.0, 4.0)
        cmd.linear.y = self.clamp(vy, -2.0, 2.0)
        cmd.linear.z = self.clamp(vz, -2.0, 2.0)
        self.cmd_pub.publish(cmd)

    def detected(self):
        return (self.platform_state is not None and
                self.platform_state.is_detected and
                self.platform_state.confidence > 0.3)

    def control_loop(self):
        # Wait for valid odom before doing anything
        if not self.odom_received:
            return

        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        if self.state == self.TAKEOFF:
            self.do_takeoff()
        elif self.state == self.SEARCH:
            self.do_search()
        elif self.state == self.TRACK:
            self.do_track()
        elif self.state == self.APPROACH:
            self.do_approach()
        elif self.state == self.DESCEND:
            self.do_descend()
        elif self.state == self.LAND:
            self.do_land()
        elif self.state == self.LANDED:
            self.send()

    # ── States ──

    def do_takeoff(self):
        if abs(self.hover_height - self.drone_z) < 0.2:
            self.get_logger().info('Takeoff done → SEARCH')
            self.state = self.SEARCH
            return
        vz = self.clamp((self.hover_height - self.drone_z) * 2.0, -1.5, 2.0)
        self.send(vz=vz)

    def do_search(self):
        if self.detected():
            self.get_logger().info('Platform found → TRACK')
            self.state = self.TRACK
            return

        dx = self.sweep_target - self.drone_x
        if abs(dx) < 0.3:
            self.sweep_target *= -1.0

        vx = self.clamp(dx * 2.0, -3.0, 3.0)
        vy, vz = self.hold_y_and_height(self.hover_height)
        self.send(vx=vx, vy=vy, vz=vz)

    def do_track(self):
        if not self.detected():
            self.get_logger().info('Lost platform → SEARCH')
            self.state = self.SEARCH
            self.aligned_count = 0
            return

        ps = self.platform_state
        dx = ps.predicted_position.x - self.drone_x

        vx = self.clamp(dx * 2.0 + ps.velocity.x, -4.0, 4.0)
        vy, vz = self.hold_y_and_height(self.hover_height)
        self.send(vx=vx, vy=vy, vz=vz)

        if abs(dx) < self.xy_tolerance * 3:
            self.aligned_count += 1
        else:
            self.aligned_count = 0

        if self.aligned_count > 10:
            self.get_logger().info('Aligned → APPROACH')
            self.state = self.APPROACH
            self.aligned_count = 0

    def do_approach(self):
        if not self.detected():
            self.get_logger().info('Lost during approach → SEARCH')
            self.state = self.SEARCH
            return

        ps = self.platform_state
        dx = ps.predicted_position.x - self.drone_x

        vx = self.clamp(dx * 2.0 + ps.velocity.x, -4.0, 4.0)
        vy = self.clamp(-self.drone_y * 2.0, -2.0, 2.0)
        vz = self.clamp((self.approach_height - self.drone_z) * 1.5, -1.5, 1.5)
        self.send(vx=vx, vy=vy, vz=vz)

        if (abs(self.drone_z - self.approach_height) < 0.3 and
                abs(dx) < self.xy_tolerance * 2):
            self.aligned_count += 1
        else:
            self.aligned_count = 0

        if self.aligned_count > 8:
            self.get_logger().info('At approach height → DESCEND')
            self.state = self.DESCEND
            self.aligned_count = 0

    def do_descend(self):
        if not self.detected():
            self.get_logger().warn('Lost during descent → TRACK')
            self.state = self.TRACK
            return

        ps = self.platform_state
        dx = (ps.position.x + ps.velocity.x * 0.3) - self.drone_x

        vx = self.clamp(dx * 3.0 + ps.velocity.x, -2.0, 2.0)
        vy = self.clamp(-self.drone_y * 2.0, -2.0, 2.0)

        if abs(dx) < self.xy_tolerance:
            vz = -self.descend_speed
        else:
            vz = 0.1

        self.send(vx=vx, vy=vy, vz=vz)

        if self.drone_z < self.land_height:
            self.get_logger().info('Low enough → LAND')
            self.state = self.LAND

    def do_land(self):
        vx = 0.0
        if self.platform_state and self.platform_state.is_detected:
            ps = self.platform_state
            dx = (ps.position.x + ps.velocity.x * 0.2) - self.drone_x
            vx = self.clamp(dx * 3.0 + ps.velocity.x, -1.0, 1.0)

        vy = self.clamp(-self.drone_y * 2.0, -1.0, 1.0)
        self.send(vx=vx, vy=vy, vz=-0.8)

        if self.drone_z < 0.45:
            self.state = self.LANDED
            self.send()  # zero velocity
            self.get_logger().info('\n' + '='*50)
            self.get_logger().info('   MISSION SUCCESS — LANDED ON PLATFORM!')
            self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
