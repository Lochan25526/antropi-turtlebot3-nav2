#!/usr/bin/env python3
"""Navigation test harness: drive a route, record what actually happened.

Run against a live simulation (demo.launch.py, usually headless):

    python3 scripts/nav_test.py --name run1 --out docs/results [--scene tb3_house] [x y yaw ...]

Records, at 10 Hz from Gazebo ground truth (/gazebo/model_states):
  * the robot trajectory and the moving-obstacle trajectory,
  * the closest the robot ever got to the moving obstacle and to the static box,
  * how many times the global plan changed (/plan messages),
  * recoveries reported by the BT navigator, and the per-goal result and time.
Writes <out>/<name>.json and <out>/<name>.png (map + trajectories) and exits 0
only if every goal succeeded and no collision happened (centre distance greater
than robot radius + obstacle radius).
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from PIL import Image
from rclpy.node import Node

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from antropi_nav2_demo.nav_wait import wait_for_nav2  # noqa: E402
from antropi_nav2_demo.scenes import load_scene  # noqa: E402

ROBOT_R = 0.105
OBST_R = 0.15      # moving_obstacle cylinder
BOX_HALF = 0.15    # static_box half side


class Recorder(Node):
    def __init__(self, robot='burger'):
        super().__init__('nav_test_recorder')
        self.robot = robot
        self.rob, self.obs, self.box = [], [], None
        self.plans = 0
        self.last_plan = None
        self.create_subscription(ModelStates, '/gazebo/model_states', self.on_states, 10)
        self.create_subscription(Path, '/plan', self.on_plan, 10)

    def on_states(self, m):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.robot in m.name:
            p = m.pose[m.name.index(self.robot)].position
            self.rob.append((t, p.x, p.y))
        if 'moving_obstacle' in m.name:
            p = m.pose[m.name.index('moving_obstacle')].position
            self.obs.append((t, p.x, p.y))
        if 'static_box' in m.name and self.box is None:
            p = m.pose[m.name.index('static_box')].position
            self.box = (p.x, p.y)

    def on_plan(self, m):
        pts = [(round(p.pose.position.x, 2), round(p.pose.position.y, 2)) for p in m.poses[::5]]
        if pts != self.last_plan:
            self.plans += 1
            self.last_plan = pts


def pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--name', default='run')
    ap.add_argument('--out', default='docs/results')
    ap.add_argument('--scene', default='lab', help='scene whose route and map to use (config/scenes/)')
    ap.add_argument('--map', default=None, help='map yaml for the plot (default: the scene map)')
    ap.add_argument('route', nargs='*', type=float)
    a = ap.parse_args()
    scene = load_scene(a.scene)
    route = scene['route'] if not a.route else [tuple(a.route[i:i + 3]) for i in range(0, len(a.route), 3)]
    a.map = a.map or scene['map_path']

    rclpy.init()
    rec = Recorder()
    rec_exec = rclpy.executors.SingleThreadedExecutor()   # own executor: BasicNavigator spins the global one
    rec_exec.add_node(rec)
    th = threading.Thread(target=rec_exec.spin, daemon=True)
    th.start()
    nav = BasicNavigator()
    wait_for_nav2(nav)

    goals = []
    for i, (x, y, yaw) in enumerate(route, 1):
        t0 = time.time()
        plans0 = rec.plans
        nav.goToPose(pose(nav, x, y, yaw))
        recov = 0
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb:
                recov = fb.number_of_recoveries
            time.sleep(0.2)
        res = nav.getResult()
        goals.append({'goal': [x, y, yaw], 'result': res.name if hasattr(res, 'name') else str(res),
                      'time_s': round(time.time() - t0, 1), 'recoveries': recov,
                      'replans': rec.plans - plans0})
        print(f"goal {i}: {goals[-1]}", flush=True)

    rob = np.array(rec.rob)
    obs = np.array(rec.obs)
    summary = {'name': a.name, 'route': route, 'goals': goals,
               'robot_path_length_m': float(np.sum(np.hypot(np.diff(rob[:, 1]), np.diff(rob[:, 2])))) if len(rob) > 1 else 0.0,
               'total_replans': rec.plans, 'samples': int(len(rob))}
    if len(obs):
        # closest approach between robot and obstacle at matching sample times
        ro = np.interp(obs[:, 0], rob[:, 0], rob[:, 1]), np.interp(obs[:, 0], rob[:, 0], rob[:, 2])
        d = np.hypot(ro[0] - obs[:, 1], ro[1] - obs[:, 2])
        d_obs = d
        summary['moving_obstacle'] = {'min_centre_distance_m': round(float(d.min()), 3),
                                      'clearance_m': round(float(d.min()) - ROBOT_R - OBST_R, 3),
                                      'obstacle_travel_m': round(float(np.sum(np.hypot(np.diff(obs[:, 1]), np.diff(obs[:, 2])))), 2),
                                      'collision': bool(d.min() < ROBOT_R + OBST_R)}
    if rec.box is not None:
        d = np.hypot(rob[:, 1] - rec.box[0], rob[:, 2] - rec.box[1])
        summary['static_box'] = {'pose': rec.box, 'min_centre_distance_m': round(float(d.min()), 3),
                                 'clearance_m': round(float(d.min()) - ROBOT_R - BOX_HALF, 3),
                                 'collision': bool(d.min() < ROBOT_R + BOX_HALF)}
    ok = all(g['result'] == 'SUCCEEDED' for g in goals) and not summary.get('moving_obstacle', {}).get('collision') \
        and not summary.get('static_box', {}).get('collision')
    summary['pass'] = bool(ok)

    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, a.name + '.json'), 'w') as f:
        json.dump(summary, f, indent=2)

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        meta = yaml.safe_load(open(a.map))
        img = np.array(Image.open(os.path.join(os.path.dirname(a.map), meta['image'])))
        H, W = img.shape
        res, (ox, oy, _) = meta['resolution'], meta['origin']
        fig, ax = plt.subplots(figsize=(10, 7.5))
        ax.imshow(img, cmap='gray', extent=[ox, ox + W * res, oy, oy + H * res])
        ax.plot(rob[:, 1], rob[:, 2], 'b-', lw=1.5, label='robot (ground truth)')
        if len(obs):
            ax.plot(obs[:, 1], obs[:, 2], color='orange', lw=1, label='moving obstacle')
        if rec.box is not None:
            ax.add_patch(plt.Rectangle((rec.box[0] - BOX_HALF, rec.box[1] - BOX_HALF), 2 * BOX_HALF, 2 * BOX_HALF, color='red', label='static box'))
        for i, (x, y, _) in enumerate(route, 1):
            ax.plot(x, y, 'g*', ms=12)
            ax.annotate(f'G{i}', (x, y), xytext=(4, 4), textcoords='offset points', color='g')
        ax.plot(rob[0, 1], rob[0, 2], 'ko', label='start')
        xs = list(rob[:, 1]) + [r[0] for r in route]
        ys = list(rob[:, 2]) + [r[1] for r in route]
        ax.set_xlim(min(xs) - 0.8, max(xs) + 0.8)
        ax.set_ylim(min(ys) - 0.8, max(ys) + 0.8)
        ax.set_title(f"{a.name}: {'PASS' if ok else 'FAIL'}, {len(goals)} goals, {rec.plans} plans, "
                     f"path {summary['robot_path_length_m']:.1f} m")
        ax.legend(loc='lower left', fontsize=8)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        fig.savefig(os.path.join(a.out, a.name + '.png'), dpi=110, bbox_inches='tight')
        # time series: robot-obstacle distance and robot speed, to show the reaction
        if len(obs) and len(rob) > 2:
            t = obs[:, 0] - rob[0, 0]
            v = np.hypot(np.diff(rob[:, 1]), np.diff(rob[:, 2])) / np.maximum(np.diff(rob[:, 0]), 1e-3)
            fig2, (a1, a2) = plt.subplots(2, 1, figsize=(10, 5), sharex=True)
            a1.plot(t, d_obs, 'k-')
            a1.axhline(ROBOT_R + OBST_R, color='r', ls='--', label='contact distance')
            a1.set_ylabel('robot-obstacle\ncentre distance [m]')
            a1.legend(loc='upper right', fontsize=8)
            a2.plot(rob[1:, 0] - rob[0, 0], np.convolve(v, np.ones(5) / 5, mode='same'), 'b-')
            a2.set_ylabel('robot speed [m/s]')
            a2.set_xlabel('time since first goal [s]')
            fig2.savefig(os.path.join(a.out, a.name + '_timeseries.png'), dpi=110, bbox_inches='tight')
    except Exception as e:  # plotting is optional
        print('plot skipped:', e)

    print(json.dumps(summary, indent=2))
    rec_exec.shutdown()
    nav.destroy_node()
    rec.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
