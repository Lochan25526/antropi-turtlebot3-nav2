#!/usr/bin/env python3
"""Moves a Gazebo model around so Nav2 has a real dynamic obstacle to avoid.

Gazebo Classic port of ``dynamic_obstacle.py`` from
https://github.com/Lochan25526/ros2-nav2-navigation-sim (Apache-2.0). The
original ran without a physics engine and painted a synthetic blob into a
LaserScan that only the costmaps subscribed to. Here the obstacle is a real
model (``models/moving_obstacle``), so the simulated LiDAR, AMCL and both
costmaps all see it the normal way. The motion logic and the parameter names
of the original are kept; the output side (LaserScan painting) is replaced by
``/gazebo/set_entity_state`` calls.

Modes (parameter ``mode``):

* ``patrol`` (default, new) - walk back and forth between ``patrol_start`` and
  ``patrol_end`` in world coordinates, pausing ``patrol_pause`` seconds at each
  end. Repeatable and independent of what the robot does, which is what a
  demo recording needs.
Both modes pause while the robot is closer than ``yield_distance`` (0.4 m
centre to centre by default), the way a person would, and resume once it has
moved on. The robot still has to detect, re-plan around and wait for the
obstacle; the pause only prevents the kinematically-driven model from being
pushed through a robot that stopped on its track.

* ``crossing`` (from the original) - idle for ``idle_duration`` seconds at
  ``park_pose``, then jump to a line ``cross_distance`` ahead of the robot and
  sweep across its path from ``-cross_width`` to ``+cross_width`` in
  ``cross_duration`` seconds, then park again.

The robot's ground-truth pose comes from ``/gazebo/model_states`` (needs the
``gazebo_ros_state`` world plugin, which ``worlds/lab.world`` loads).
"""
import math

import rclpy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Point, Pose
from rclpy.node import Node
from visualization_msgs.msg import Marker


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class DynamicObstacle(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle')
        self.declare_parameter('mode', 'patrol')
        self.declare_parameter('model_name', 'moving_obstacle')
        self.declare_parameter('robot_model_name', 'burger')
        self.declare_parameter('speed', 0.15)                 # m/s, both modes
        self.declare_parameter('update_rate', 10.0)          # Hz
        self.declare_parameter('yield_distance', 0.4)        # m, pause while the robot is this close (0 = never)
        # patrol
        self.declare_parameter('patrol_start', [-2.7, 1.9])
        self.declare_parameter('patrol_end', [-1.7, 1.9])
        self.declare_parameter('patrol_pause', 1.0)
        # crossing (names from the original node)
        self.declare_parameter('cross_distance', 1.6)
        self.declare_parameter('cross_width', 1.0)
        self.declare_parameter('cross_duration', 0.0)        # 0 -> derived from speed
        self.declare_parameter('idle_duration', 5.0)
        self.declare_parameter('park_pose', [-2.7, 1.9])
        self.declare_parameter('marker_frame', 'map')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.mode = g('mode')
        self.model_name = g('model_name')
        self.robot_name = g('robot_model_name')
        self.speed = float(g('speed'))
        self.period = 1.0 / float(g('update_rate'))
        self.dt = self.period
        self.yield_distance = float(g('yield_distance'))
        self.p_start = [float(v) for v in g('patrol_start')]
        self.p_end = [float(v) for v in g('patrol_end')]
        self.p_pause = float(g('patrol_pause'))
        self.cross_distance = float(g('cross_distance'))
        self.cross_width = float(g('cross_width'))
        self.cross_duration = float(g('cross_duration')) or (2.0 * self.cross_width / self.speed)
        self.idle_duration = float(g('idle_duration'))
        self.park = [float(v) for v in g('park_pose')]
        self.marker_frame = g('marker_frame')

        self.robot_xy = None
        self.robot_yaw = 0.0
        self.model_seen = False
        self.last_tick = None
        self.obs_xy = list(self.p_start if self.mode == 'patrol' else self.park)

        # patrol state
        self.target_is_end = True
        self.pause_left = self.p_pause
        # crossing state
        self.state = 'idle'
        self.state_time = 0.0
        self.anchor = (0.0, 0.0, 0.0)

        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.pending = None
        self.marker_pub = self.create_publisher(Marker, 'moving_obstacle/marker', 1)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.on_states, 10)
        self.timer = self.create_timer(self.period, self.tick)
        self.get_logger().info(
            f"dynamic obstacle '{self.model_name}' mode={self.mode} speed={self.speed} m/s")

    # ------------------------------------------------------------------ inputs
    def on_states(self, msg: ModelStates):
        self.model_seen = self.model_seen or self.model_name in msg.name
        if self.robot_name in msg.name:
            p = msg.pose[msg.name.index(self.robot_name)]
            self.robot_xy = (p.position.x, p.position.y)
            self.robot_yaw = yaw_from_quat(p.orientation)

    # ------------------------------------------------------------------ motion
    def robot_too_close(self):
        if self.robot_xy is None or self.yield_distance <= 0.0:
            return False
        return math.hypot(self.robot_xy[0] - self.obs_xy[0], self.robot_xy[1] - self.obs_xy[1]) < self.yield_distance

    def step_towards(self, target):
        if self.robot_too_close():
            # Behave like a person, not a bulldozer: the model is repositioned
            # kinematically, so without this it would be pushed straight through
            # a robot that stopped on its track.
            return False
        dx, dy = target[0] - self.obs_xy[0], target[1] - self.obs_xy[1]
        d = math.hypot(dx, dy)
        step = self.speed * self.dt
        if d <= step:
            self.obs_xy = list(target)
            return True
        self.obs_xy = [self.obs_xy[0] + dx / d * step, self.obs_xy[1] + dy / d * step]
        return False

    def tick_patrol(self):
        if self.pause_left > 0.0:
            self.pause_left -= self.dt
            return
        if self.step_towards(self.p_end if self.target_is_end else self.p_start):
            self.target_is_end = not self.target_is_end
            self.pause_left = self.p_pause

    def tick_crossing(self):
        self.state_time += self.dt
        if self.state == 'idle':
            self.obs_xy = list(self.park)
            if self.state_time >= self.idle_duration and self.robot_xy is not None:
                self.anchor = (self.robot_xy[0], self.robot_xy[1], self.robot_yaw)
                self.state, self.state_time = 'crossing', 0.0
                self.get_logger().info('obstacle starting to cross the robot path')
            return
        if self.robot_too_close():
            self.state_time -= self.dt          # hold position, resume when clear
        frac = min(self.state_time / self.cross_duration, 1.0)
        lateral = -self.cross_width + 2.0 * self.cross_width * frac
        ax, ay, ayaw = self.anchor
        fx = ax + self.cross_distance * math.cos(ayaw)
        fy = ay + self.cross_distance * math.sin(ayaw)
        self.obs_xy = [fx - lateral * math.sin(ayaw), fy + lateral * math.cos(ayaw)]
        if frac >= 1.0:
            self.state, self.state_time = 'idle', 0.0

    # ------------------------------------------------------------------ output
    def tick(self):
        # Use the real elapsed (sim) time, not the nominal period: if a
        # set_entity_state call is slow and a tick is skipped, the obstacle
        # still covers the right distance instead of slowing down.
        now = self.get_clock().now()
        if self.last_tick is None:
            self.last_tick = now
            return
        self.dt = min((now - self.last_tick).nanoseconds * 1e-9, 0.5)
        self.last_tick = now
        if self.dt <= 0.0:
            return
        if not self.model_seen:
            self.get_logger().info(f"waiting for model '{self.model_name}' in /gazebo/model_states",
                                   throttle_duration_sec=5.0)
            return
        if self.mode == 'crossing':
            self.tick_crossing()
        else:
            self.tick_patrol()
        self.push_pose()
        self.publish_marker()

    def push_pose(self):
        if not self.cli.service_is_ready():
            self.get_logger().info('waiting for /gazebo/set_entity_state', throttle_duration_sec=5.0)
            return
        if self.pending is not None and not self.pending.done():
            return  # previous call still in flight, skip this tick
        req = SetEntityState.Request()
        req.state.name = self.model_name
        req.state.pose = Pose()
        req.state.pose.position.x = self.obs_xy[0]
        req.state.pose.position.y = self.obs_xy[1]
        req.state.pose.position.z = 0.0
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'world'
        self.pending = self.cli.call_async(req)

    def publish_marker(self):
        m = Marker()
        m.header.frame_id = self.marker_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'moving_obstacle', 0, Marker.CYLINDER, Marker.ADD
        m.pose.position = Point(x=self.obs_xy[0], y=self.obs_xy[1], z=0.25)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.3
        m.scale.z = 0.5
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 0.9
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
