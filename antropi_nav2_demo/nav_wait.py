"""Wait for Nav2 without touching AMCL's pose.

``BasicNavigator.waitUntilNav2Active(localizer='amcl')`` publishes its own
``initial_pose`` (all zeros unless you set one) to ``/initialpose`` whenever it
has not yet received an ``/amcl_pose``. If that happens after AMCL already
seeded itself from ``set_initial_pose`` (as this package configures), the robot
is teleported to the map origin in AMCL's eyes and every plan fails with
"robot's start position is off the global costmap". Observed once in testing,
hence this helper: it waits for the lifecycle nodes and for the first AMCL pose
and never publishes anything.
"""
import time

import rclpy


def wait_for_nav2(nav, timeout_s=60.0):
    """Block until amcl and bt_navigator are active and AMCL has published a pose."""
    nav._waitForNodeToActivate('amcl')          # lifecycle get_state polling, no side effects
    nav._waitForNodeToActivate('bt_navigator')
    t0 = time.time()
    while not nav.initial_pose_received:         # set by BasicNavigator's /amcl_pose callback
        if time.time() - t0 > timeout_s:
            raise TimeoutError('no /amcl_pose received: give AMCL an initial pose '
                               '(RViz "2D Pose Estimate") or check amcl.set_initial_pose')
        rclpy.spin_once(nav, timeout_sec=0.5)
    nav.get_logger().info('Nav2 is active and AMCL has a pose estimate')
