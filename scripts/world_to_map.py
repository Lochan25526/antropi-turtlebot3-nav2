#!/usr/bin/env python3
"""Build a Nav2 occupancy grid of a running Gazebo world with a ground-truth LiDAR sweep.

A collision-free LiDAR model (models/mapper_lidar) is spawned and teleported over a grid of
poses. Each scan is integrated into an occupancy grid using the exact pose we asked for, so
the result is what a perfect SLAM run would produce at TurtleBot3 LDS height. The sweep
grows outwards from the seed pose and only visits poses that the map so far shows as free
with some clearance, so it never leaves the reachable interior.

Run against a world started on its own (no robot needed):

    ros2 launch gazebo_ros gzserver.launch.py world:=$(pwd)/worlds/tb3_house.world
    python3 scripts/world_to_map.py --seed 0 0 --bounds -8 -6 8 6 --out maps/tb3_house

The world must load the gazebo_ros_state plugin (all worlds in worlds/ do).
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_SDF = os.path.join(HERE, '..', 'models', 'mapper_lidar', 'model.sdf')
NAME = 'mapper_lidar'
Z = 0.18


class Mapper(Node):
    def __init__(self, a):
        super().__init__('world_to_map')
        self.a = a
        self.res = a.res
        self.x0, self.y0, self.x1, self.y1 = a.bounds
        self.w = int(math.ceil((self.x1 - self.x0) / self.res))
        self.h = int(math.ceil((self.y1 - self.y0) / self.res))
        self.hits = np.zeros((self.h, self.w), np.int32)
        self.free = np.zeros((self.h, self.w), np.int32)
        self.scan = None
        self.create_subscription(LaserScan, 'mapper/scan', self.on_scan, qos_profile_sensor_data)
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.state_cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')

    # grid helpers (row 0 = top = max y, like a map .pgm)
    def to_cell(self, x, y):
        return (self.h - 1 - int((y - self.y0) / self.res), int((x - self.x0) / self.res))

    def on_scan(self, m):
        self.scan = m

    def call(self, cli, req, timeout=10.0):
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'waiting for {cli.srv_name}')
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def spawn(self, x, y):
        req = SpawnEntity.Request()
        req.name = NAME
        req.xml = open(MODEL_SDF).read()
        req.initial_pose.position.x, req.initial_pose.position.y, req.initial_pose.position.z = x, y, Z
        req.initial_pose.orientation.w = 1.0
        r = self.call(self.spawn_cli, req)
        if r is None or not r.success:
            sys.exit(f'spawn failed: {r.status_message if r else "no response"}')

    def move(self, x, y):
        req = SetEntityState.Request()
        req.state.name = NAME
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = x, y, Z
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'world'
        self.call(self.state_cli, req)

    def fresh_scan(self, n=2, timeout=3.0):
        """Wait for n new scans after a move (the first may predate the teleport)."""
        got, t0 = 0, time.time()
        last = self.scan
        while got < n and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.scan is not last:
                last, got = self.scan, got + 1
        return self.scan

    def integrate(self, x, y, m):
        n = len(m.ranges)
        ang = m.angle_min + np.arange(n) * m.angle_increment
        r = np.array(m.ranges, dtype=np.float64)
        hit = np.isfinite(r) & (r < m.range_max - 1e-3) & (r > m.range_min)
        reach = np.where(hit, r, min(m.range_max, self.a.max_range))
        reach = np.minimum(reach, self.a.max_range)
        step = self.res * 0.5
        k = np.arange(0, self.a.max_range, step)
        d = np.minimum(k[None, :], reach[:, None])            # distance along each beam, clipped
        valid = k[None, :] < (reach[:, None] - self.res)       # free cells stop one cell before the hit
        px = x + d * np.cos(ang)[:, None]
        py = y + d * np.sin(ang)[:, None]
        cols = ((px - self.x0) / self.res).astype(int)
        rows = (self.h - 1 - ((py - self.y0) / self.res)).astype(int)
        inb = (rows >= 0) & (rows < self.h) & (cols >= 0) & (cols < self.w)
        fr = valid & inb
        np.add.at(self.free, (rows[fr], cols[fr]), 1)
        hx = x + r[hit] * np.cos(ang[hit])
        hy = y + r[hit] * np.sin(ang[hit])
        hc = ((hx - self.x0) / self.res).astype(int)
        hr = (self.h - 1 - ((hy - self.y0) / self.res)).astype(int)
        ok = (hr >= 0) & (hr < self.h) & (hc >= 0) & (hc < self.w)
        np.add.at(self.hits, (hr[ok], hc[ok]), 1)

    def occupancy(self):
        tot = self.hits + self.free
        occ = (self.hits >= 1) & (self.hits >= 0.15 * tot)
        free = (self.free >= 1) & ~occ
        img = np.full((self.h, self.w), 205, np.uint8)
        img[free] = 254
        img[occ] = 0
        return img

    def pose_is_clear(self, x, y):
        r, c = self.to_cell(x, y)
        n = int(math.ceil(self.a.min_clear / self.res))
        r0, r1, c0, c1 = max(r - n, 0), min(r + n + 1, self.h), max(c - n, 0), min(c + n + 1, self.w)
        if r0 >= r1 or c0 >= c1:
            return False
        win_h, win_f = self.hits[r0:r1, c0:c1], self.free[r0:r1, c0:c1]
        return bool((win_h == 0).all() and (win_f > 0).mean() > 0.9)

    def run(self):
        a = self.a
        self.spawn(a.seed[0], a.seed[1])
        self.fresh_scan(n=3)
        visited, queue, poses = set(), [tuple(a.seed)], []
        visited.add((0, 0))
        order = [(0, 0)]
        t0 = time.time()
        while queue and len(poses) < a.max_poses:
            x, y = queue.pop(0)
            self.move(x, y)
            m = self.fresh_scan()
            if m is None:
                continue
            self.integrate(x, y, m)
            poses.append((x, y))
            i0 = round((x - a.seed[0]) / a.step)
            j0 = round((y - a.seed[1]) / a.step)
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                ij = (i0 + di, j0 + dj)
                nx, ny = a.seed[0] + ij[0] * a.step, a.seed[1] + ij[1] * a.step
                if ij in visited or not (self.x0 < nx < self.x1 and self.y0 < ny < self.y1):
                    continue
                if self.pose_is_clear(nx, ny):
                    visited.add(ij)
                    queue.append((nx, ny))
            if len(poses) % 25 == 0:
                self.get_logger().info(f'{len(poses)} poses, {len(queue)} queued, {time.time() - t0:.0f} s')
        img = self.occupancy()
        os.makedirs(os.path.dirname(os.path.abspath(a.out)) or '.', exist_ok=True)
        Image.fromarray(img).save(a.out + '.pgm')
        with open(a.out + '.yaml', 'w') as f:
            f.write(f"image: {os.path.basename(a.out)}.pgm\nmode: trinary\nresolution: {self.res}\n"
                    f"origin: [{self.x0}, {self.y0}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
        # preview with the sweep poses
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(10, 10 * self.h / self.w))
            ax.imshow(img, cmap='gray', extent=[self.x0, self.x1, self.y0, self.y1])
            px, py = zip(*poses)
            ax.plot(px, py, 'c.', ms=3)
            ax.set_title(f'{os.path.basename(a.out)}: {len(poses)} sweep poses, {int((img == 0).sum())} occupied cells')
            fig.savefig(a.out + '_sweep.png', dpi=100, bbox_inches='tight')
        except Exception as e:
            print('preview skipped:', e)
        print(f'{a.out}.pgm {self.w}x{self.h} @ {self.res} m, origin ({self.x0}, {self.y0}); '
              f'{len(poses)} poses in {time.time() - t0:.0f} s; occupied={int((img == 0).sum())} '
              f'free={int((img == 254).sum())} unknown={int((img == 205).sum())}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seed', nargs=2, type=float, required=True, metavar=('X', 'Y'), help='a free pose to start from')
    ap.add_argument('--bounds', nargs=4, type=float, required=True, metavar=('XMIN', 'YMIN', 'XMAX', 'YMAX'))
    ap.add_argument('--out', required=True, help='output path without extension, e.g. maps/tb3_house')
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--step', type=float, default=0.5, help='sweep pose spacing')
    ap.add_argument('--min-clear', type=float, default=0.25, help='clearance a pose needs before it is visited')
    ap.add_argument('--max-range', type=float, default=8.0)
    ap.add_argument('--max-poses', type=int, default=4000)
    a = ap.parse_args()
    rclpy.init()
    node = Mapper(a)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
