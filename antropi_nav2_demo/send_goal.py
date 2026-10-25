#!/usr/bin/env python3
"""Send one or more NavigateToPose goals and report what happened.

    ros2 run antropi_nav2_demo send_goal                        # demo route of the lab scene
    ros2 run antropi_nav2_demo send_goal --scene tb3_house      # demo route of another scene
    ros2 run antropi_nav2_demo send_goal -- -4.3 1.3 3.14       # explicit x y yaw [x y yaw ...]

Waits for Nav2 to be active, sends each pose in turn, prints distance
remaining while driving and the final result. Exit code 0 only if every goal
succeeded, so the same script doubles as the repeatability test.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from antropi_nav2_demo.nav_wait import wait_for_nav2
from antropi_nav2_demo.scenes import load_scene


def pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def parse(argv):
    scene = 'lab'
    if '--scene' in argv:
        i = argv.index('--scene')
        scene, argv = argv[i + 1], argv[:i] + argv[i + 2:]
    vals = [float(a) for a in argv if a != '--']
    if not vals:
        return load_scene(scene)['route']
    if len(vals) % 3:
        sys.exit('usage: send_goal [x y yaw]...')
    return [tuple(vals[i:i + 3]) for i in range(0, len(vals), 3)]


def main():
    route = parse(sys.argv[1:])
    rclpy.init()
    nav = BasicNavigator()
    wait_for_nav2(nav)
    ok = True
    for i, (x, y, yaw) in enumerate(route, 1):
        nav.get_logger().info(f'goal {i}/{len(route)}: x={x:.2f} y={y:.2f} yaw={yaw:.2f}')
        t0 = time.time()
        nav.goToPose(pose(nav, x, y, yaw))
        last = 0.0
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb and time.time() - last > 2.0:
                last = time.time()
                nav.get_logger().info(
                    f'  {fb.distance_remaining:.2f} m remaining, recoveries={fb.number_of_recoveries}')
            time.sleep(0.2)
        res = nav.getResult()
        dt = time.time() - t0
        name = {TaskResult.SUCCEEDED: 'SUCCEEDED', TaskResult.CANCELED: 'CANCELED',
                TaskResult.FAILED: 'FAILED'}.get(res, str(res))
        nav.get_logger().info(f'goal {i} {name} after {dt:.1f} s')
        ok &= res == TaskResult.SUCCEEDED
    nav.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
