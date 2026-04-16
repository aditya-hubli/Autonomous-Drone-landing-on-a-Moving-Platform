#!/usr/bin/env python3
"""
Drone controller - land on a platform moving in the XY plane.

The drone starts at an overview height where the full arena bounding box is
visible in the downward camera, then slowly descends while tracking the
platform and lands on it.

Camera geometry (must match drone model.sdf):
  HFOV = 120deg, image 800x600  =>  VFOV = 90deg
  Ground coverage at height h (narrow dim): 2 * h * tan(45deg) = 2h
  To see full +/-2.5m arena: h >= 2.5m.  We use 5.0m for 2x margin.

Drone plugin uses WORLD frame, so cmd_vel.linear.x/y = world X/Y directly.

States: TAKEOFF -> SEARCH -> TRACK -> DESCEND -> LAND -> LANDED
"""

import math
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
    DESCEND = 'DESCEND'
    LAND = 'LAND'
    LANDED = 'LANDED'

    def __init__(self):
        super().__init__('drone_controller')

        # Parameters
        self.declare_parameter('overview_height', 5.0)
        self.declare_parameter('descend_speed', 0.4)
        self.declare_parameter('land_height', 0.55)
        self.declare_parameter('xy_tolerance', 0.2)

        self.overview_height = self.get_parameter('overview_height').value
        self.descend_speed = self.get_parameter('descend_speed').value
        self.land_height = self.get_parameter('land_height').value
        self.xy_tolerance = self.get_parameter('xy_tolerance').value

        # State
        self.state = self.TAKEOFF
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.odom_received = False
        self.platform_state = None
        self.aligned_count = 0

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/drone/odom', self.odom_cb, 10)
        self.platform_sub = self.create_subscription(
            PlatformState, '/platform/tracked_state', self.platform_cb, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)

        # 20 Hz control loop
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            f'Drone controller started — overview height {self.overview_height}m, '
            f'descend {self.descend_speed} m/s')

    # -- helpers --

    def odom_cb(self, msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        self.drone_z = msg.pose.pose.position.z
        self.odom_received = True

    def platform_cb(self, msg):
        self.platform_state = msg

    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def send(self, vx=0.0, vy=0.0, vz=0.0):
        cmd = Twist()
        cmd.linear.x = self.clamp(vx, -4.0, 4.0)
        cmd.linear.y = self.clamp(vy, -4.0, 4.0)
        cmd.linear.z = self.clamp(vz, -2.0, 2.0)
        self.cmd_pub.publish(cmd)

    def detected(self):
        return (self.platform_state is not None
                and self.platform_state.is_detected
                and self.platform_state.confidence > 0.3)

    def xy_error_to_platform(self, prediction_horizon=None):
        """XY error from drone to platform.

        If prediction_horizon is None, use the tracker's predicted_position.
        Otherwise use position + velocity * horizon for a custom look-ahead.
        """
        ps = self.platform_state
        if prediction_horizon is None:
            dx = ps.predicted_position.x - self.drone_x
            dy = ps.predicted_position.y - self.drone_y
        else:
            dx = (ps.position.x + ps.velocity.x * prediction_horizon) - self.drone_x
            dy = (ps.position.y + ps.velocity.y * prediction_horizon) - self.drone_y
        return dx, dy

    @staticmethod
    def xy_dist(dx, dy):
        return math.sqrt(dx * dx + dy * dy)

    def track_xy(self, dx, dy, gain, vel_ff, limit):
        """Proportional + velocity-feedforward XY tracking."""
        ps = self.platform_state
        vx = self.clamp(dx * gain + ps.velocity.x * vel_ff, -limit, limit)
        vy = self.clamp(dy * gain + ps.velocity.y * vel_ff, -limit, limit)
        return vx, vy

    # -- control loop --

    def control_loop(self):
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
        elif self.state == self.DESCEND:
            self.do_descend()
        elif self.state == self.LAND:
            self.do_land()
        elif self.state == self.LANDED:
            self.send()

    # -- states --

    def do_takeoff(self):
        """Climb to overview_height where full arena is in camera view."""
        dz = self.overview_height - self.drone_z
        if abs(dz) < 0.2:
            self.get_logger().info(
                f'At overview height {self.overview_height}m -> SEARCH')
            self.state = self.SEARCH
            return
        vz = self.clamp(dz * 2.0, -1.5, 2.0)
        self.send(vz=vz)

    def do_search(self):
        """Hold at overview height — full arena is visible, detect the platform.

        At 5m the camera covers +/-5m (narrow) which contains the full
        +/-2.5m bounding box.  Detection should be near-instant.
        """
        if self.detected():
            self.get_logger().info('Platform detected -> TRACK')
            self.state = self.TRACK
            self.aligned_count = 0
            return

        # Hold position at overview height
        vz = self.clamp(
            (self.overview_height - self.drone_z) * 2.0, -2.0, 2.0)
        self.send(vz=vz)

    def do_track(self):
        """Track platform XY at overview height until stably aligned."""
        if not self.detected():
            self.get_logger().info('Lost platform -> SEARCH')
            self.state = self.SEARCH
            self.aligned_count = 0
            return

        dx, dy = self.xy_error_to_platform()
        vx, vy = self.track_xy(dx, dy, gain=2.0, vel_ff=1.0, limit=4.0)
        vz = self.clamp(
            (self.overview_height - self.drone_z) * 2.0, -2.0, 2.0)
        self.send(vx=vx, vy=vy, vz=vz)

        if self.xy_dist(dx, dy) < self.xy_tolerance * 3:
            self.aligned_count += 1
        else:
            self.aligned_count = 0

        if self.aligned_count > 15:
            self.get_logger().info('Stably aligned -> DESCEND')
            self.state = self.DESCEND
            self.aligned_count = 0

    def do_descend(self):
        """Slowly descend while tracking XY.  Pause descent if misaligned."""
        if not self.detected():
            self.get_logger().warn('Lost during descent -> SEARCH')
            self.state = self.SEARCH
            self.aligned_count = 0
            return

        # Tighter prediction (0.3s look-ahead) as we get closer
        dx, dy = self.xy_error_to_platform(prediction_horizon=0.3)
        dist = self.xy_dist(dx, dy)

        # Increase gain as altitude decreases for tighter low-altitude tracking
        gain = 2.0 if self.drone_z > 2.0 else 3.0
        vx, vy = self.track_xy(dx, dy, gain=gain, vel_ff=1.0, limit=3.0)

        # Only descend when XY-aligned; hold altitude otherwise
        if dist < self.xy_tolerance:
            vz = -self.descend_speed
        elif dist < self.xy_tolerance * 2:
            vz = -self.descend_speed * 0.5  # slow descent
        else:
            vz = 0.1  # hold / nudge up while re-aligning

        self.send(vx=vx, vy=vy, vz=vz)

        if self.drone_z < self.land_height:
            self.get_logger().info(
                f'Below {self.land_height}m -> LAND')
            self.state = self.LAND

    def do_land(self):
        """Final descent with XY correction."""
        vx, vy = 0.0, 0.0
        if self.platform_state and self.platform_state.is_detected:
            dx, dy = self.xy_error_to_platform(prediction_horizon=0.2)
            ps = self.platform_state
            vx = self.clamp(dx * 3.0 + ps.velocity.x, -1.0, 1.0)
            vy = self.clamp(dy * 3.0 + ps.velocity.y, -1.0, 1.0)

        self.send(vx=vx, vy=vy, vz=-0.8)

        if self.drone_z < 0.45:
            self.state = self.LANDED
            self.send()
            self.get_logger().info('\n' + '=' * 50)
            self.get_logger().info('   MISSION SUCCESS - LANDED ON PLATFORM!')
            self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
