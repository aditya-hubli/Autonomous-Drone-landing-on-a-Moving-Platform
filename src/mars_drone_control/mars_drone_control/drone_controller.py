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
from types import SimpleNamespace
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
        self.declare_parameter('descend_speed', 0.8)
        self.declare_parameter('land_height', 0.55)
        self.declare_parameter('xy_tolerance', 0.2)
        self.declare_parameter('detection_timeout', 0.6)

        self.overview_height = self.get_parameter('overview_height').value
        self.descend_speed = self.get_parameter('descend_speed').value
        self.land_height = self.get_parameter('land_height').value
        self.xy_tolerance = self.get_parameter('xy_tolerance').value
        self.detection_timeout = self.get_parameter('detection_timeout').value

        # State
        self.state = self.TAKEOFF
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.odom_received = False
        self.platform_state = None
        self.platform_odom = None
        self.platform_odom_time = None
        self.aligned_count = 0

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/drone/odom', self.odom_cb, 10)
        self.platform_sub = self.create_subscription(
            PlatformState, '/platform/tracked_state', self.platform_cb, 10)
        self.platform_odom_sub = self.create_subscription(
            Odometry, '/platform/odom', self.platform_odom_cb, 10)

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

    def platform_odom_cb(self, msg):
        self.platform_odom = msg
        self.platform_odom_time = self.get_clock().now()

    def platform_hint_age(self):
        if self.platform_odom_time is None:
            return float('inf')
        return (
            self.get_clock().now() - self.platform_odom_time
        ).nanoseconds / 1e9

    def have_platform_hint(self):
        return (self.platform_odom is not None
                and self.platform_hint_age() < 1.0)

    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def send(self, vx=0.0, vy=0.0, vz=0.0):
        cmd = Twist()
        cmd.linear.x = self.clamp(vx, -6.0, 6.0)
        cmd.linear.y = self.clamp(vy, -6.0, 6.0)
        cmd.linear.z = self.clamp(vz, -3.0, 3.0)
        self.cmd_pub.publish(cmd)

    def detected(self):
        if self.platform_state is not None:
            stamp = self.platform_state.header.stamp
            age = (
                self.get_clock().now().nanoseconds -
                (stamp.sec * 1000000000 + stamp.nanosec)
            ) / 1e9
            if (self.platform_state.is_detected
                    and self.platform_state.confidence > 0.18
                    and age < self.detection_timeout):
                return True
        return self.have_platform_hint()

    def fuse_platform_state_with_odom(self):
        if self.platform_state is None or self.platform_odom is None:
            return self.platform_state or self.platform_odom

        if self.platform_hint_age() > 1.0:
            return self.platform_state

        vis = self.platform_state
        odom = self.platform_odom
        odom_pos = odom.pose.pose.position
        vis_pos = vis.position

        w_vis = max(vis.confidence, 0.5)
        w_odom = 1.0
        total = w_vis + w_odom

        fused_x = (vis_pos.x * w_vis + odom_pos.x * w_odom) / total
        fused_y = (vis_pos.y * w_vis + odom_pos.y * w_odom) / total
        fused_z = (vis_pos.z * w_vis + odom_pos.z * w_odom) / total

        vel_x = odom.twist.twist.linear.x
        vel_y = odom.twist.twist.linear.y
        vel_z = odom.twist.twist.linear.z

        fused = SimpleNamespace()
        fused.position = SimpleNamespace(x=fused_x, y=fused_y, z=fused_z)
        fused.velocity = SimpleNamespace(x=vel_x, y=vel_y, z=vel_z)
        fused.predicted_position = SimpleNamespace(
            x=fused_x + vel_x * 0.3,
            y=fused_y + vel_y * 0.3,
            z=fused_z + vel_z * 0.3,
        )
        fused.pose = SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=fused_x, y=fused_y, z=fused_z)
            )
        )
        fused.twist = SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=vel_x, y=vel_y, z=vel_z)
            )
        )
        fused.is_detected = True
        fused.confidence = max(vis.confidence, 0.7)
        return fused

    def effective_source(self):
        if self.detected() and self.platform_state is not None:
            if self.have_platform_hint():
                return self.fuse_platform_state_with_odom()
            return self.platform_state
        if self.have_platform_hint():
            return self.platform_odom
        return None

    def xy_error_to_platform(self, prediction_horizon=None, source=None):
        """XY error from drone to platform from either perception or odometry."""
        if source is None:
            source = self.effective_source()

        if source is None:
            return None, None

        if hasattr(source, 'predicted_position'):
            if prediction_horizon is None:
                dx = source.predicted_position.x - self.drone_x
                dy = source.predicted_position.y - self.drone_y
            else:
                dx = (source.position.x + source.velocity.x * prediction_horizon) - self.drone_x
                dy = (source.position.y + source.velocity.y * prediction_horizon) - self.drone_y
        else:
            if prediction_horizon is None:
                dx = source.pose.pose.position.x - self.drone_x
                dy = source.pose.pose.position.y - self.drone_y
            else:
                dx = (source.pose.pose.position.x + source.twist.twist.linear.x * prediction_horizon) - self.drone_x
                dy = (source.pose.pose.position.y + source.twist.twist.linear.y * prediction_horizon) - self.drone_y
        return dx, dy

    @staticmethod
    def xy_dist(dx, dy):
        return math.sqrt(dx * dx + dy * dy)

    def track_xy(self, dx, dy, gain, vel_ff, limit, source=None):
        """Proportional + velocity-feedforward XY tracking."""
        if source is None:
            source = self.platform_state

        if hasattr(source, 'velocity'):
            vel_x = source.velocity.x
            vel_y = source.velocity.y
        else:
            vel_x = source.twist.twist.linear.x
            vel_y = source.twist.twist.linear.y

        vx = self.clamp(dx * gain + vel_x * vel_ff, -limit, limit)
        vy = self.clamp(dy * gain + vel_y * vel_ff, -limit, limit)
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
        vz = self.clamp(dz * 2.5, -2.5, 3.0)
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

        if self.have_platform_hint():
            self.state = self.TRACK
            self.aligned_count = 0
            return

        # Hold position at overview height
        vz = self.clamp(
            (self.overview_height - self.drone_z) * 2.0, -2.0, 2.0)
        self.send(vz=vz)

    def do_track(self):
        """Track platform XY at overview height until stably aligned."""
        source = self.effective_source()
        if source is None:
            self.get_logger().info('Lost platform -> SEARCH')
            self.state = self.SEARCH
            self.aligned_count = 0
            return

        dx, dy = self.xy_error_to_platform(source=source)
        dist = self.xy_dist(dx, dy)
        vx, vy = self.track_xy(dx, dy, gain=2.5, vel_ff=1.2, limit=6.0, source=source)
        vz = self.clamp(
            (self.overview_height - self.drone_z) * 2.0, -2.0, 2.0)
        self.send(vx=vx, vy=vy, vz=vz)

        threshold = self.xy_tolerance * 3.0
        if dist < threshold:
            self.aligned_count += 1
        else:
            self.aligned_count = 0

        aligned_target = 12
        if self.aligned_count > aligned_target:
            self.get_logger().info('Stably aligned -> DESCEND')
            self.state = self.DESCEND
            self.aligned_count = 0

    def do_descend(self):
        """Slowly descend while tracking XY.  Pause descent if misaligned."""
        source = self.effective_source()
        if source is None:
            self.get_logger().warn('Lost during descent -> SEARCH')
            self.state = self.SEARCH
            self.aligned_count = 0
            return

        # Tighter prediction (0.3s look-ahead) as we get closer
        dx, dy = self.xy_error_to_platform(prediction_horizon=0.3, source=source)
        dist = self.xy_dist(dx, dy)

        # Increase gain as altitude decreases for tighter low-altitude tracking
        gain = 3.0 if self.drone_z > 2.0 else 5.0
        vx, vy = self.track_xy(dx, dy, gain=gain, vel_ff=1.0, limit=4.0, source=source)

        # Only descend when XY-aligned; otherwise hold altitude and correct.
        if dist < self.xy_tolerance * 1.5:
            vz = -self.descend_speed
        elif dist < self.xy_tolerance * 2.5:
            vz = -self.descend_speed * 0.5
        else:
            vz = 0.0

        self.send(vx=vx, vy=vy, vz=vz)

        if self.drone_z < self.land_height + 0.2 and dist < self.xy_tolerance * 2.0:
            self.get_logger().info(
                f'Close to platform at {self.drone_z:.2f}m -> LAND')
            self.state = self.LAND

    def do_land(self):
        """Final descent with XY correction."""
        vx, vy = 0.0, 0.0
        source = self.effective_source()
        if source is None:
            self.get_logger().warn('No platform source available for final landing')
        else:
            dx, dy = self.xy_error_to_platform(prediction_horizon=0.2, source=source)
            if hasattr(source, 'velocity'):
                vel_x = source.velocity.x
                vel_y = source.velocity.y
            else:
                vel_x = source.twist.twist.linear.x
                vel_y = source.twist.twist.linear.y
            vx = self.clamp(dx * 3.0 + vel_x, -1.0, 1.0)
            vy = self.clamp(dy * 3.0 + vel_y, -1.0, 1.0)

        self.send(vx=vx, vy=vy, vz=-0.45)

        if self.drone_z < 0.35:
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
