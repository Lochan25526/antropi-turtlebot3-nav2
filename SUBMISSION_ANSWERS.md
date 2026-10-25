# Submission answers

## 1. What was your approach?

I started by checking what was already on my machine rather than installing anything: Ubuntu
22.04, ROS 2 Humble, Gazebo Classic 11 with the ROS bridge, the TurtleBot3 simulation
packages and Nav2 1.1.20 were all there from apt, so the simplest stable combination was
also the one I had. I kept Gazebo Classic on purpose. Mixing in Ignition packages, which
one of my older projects used, would have meant a different sensor bridge and a different
spawn mechanism for no gain.

The part I cared about most was not using a stock world. During my internship I built a
LiDAR-inertial SLAM stack on a Raspberry Pi robot and mapped our lab with it, and I later
used that same map in a physics-free Nav2 testbed with a scripted dynamic obstacle. For this
assignment I wanted the TurtleBot3 to drive in that lab. There is no Gazebo world for it, so I
wrote a small generator that turns every occupied cell of the occupancy grid into a static box.
That gives a world whose walls match the map by construction, which is exactly what AMCL
wants: the simulated LiDAR sees the same geometry the particle filter is scoring against.

The robot is the unmodified TurtleBot3 Burger from `turtlebot3_gazebo`. Its diff-drive
plugin takes `/cmd_vel` and publishes `/odom` and the `odom -> base_footprint` transform;
`robot_state_publisher` adds the rest of the tree from the URDF. Nav2 is brought up through
`nav2_bringup` with the standard plugins: map server, AMCL, NavFn for global planning, DWB
as the local controller, the behaviour server for spin and back-up recoveries, and the BT
navigator with the default replanning-and-recovery tree. I seeded AMCL with the spawn pose
so the demo does not depend on a manual pose estimate.

The dynamic obstacle is my internship node ported to Gazebo. The original painted a fake
blob into a LaserScan that only the costmaps subscribed to. Here it repositions a real
cylinder through the `gazebo_ros_state` service, so the LiDAR, both costmaps and the
recording all see the same object. I kept its crossing state machine and parameter names and
added a plain patrol mode for a repeatable demo. An unmapped box on the route covers the
static case. A small test harness drives the route, records Gazebo ground truth, counts
re-plans and recoveries and checks the closest approach to each obstacle, so every claim in
the README comes from a logged run.

## 2. What challenges did you encounter?

The map's unknown space loaded as free. SLAM Toolbox saves the grey unknown pixels as 205
and writes `free_thresh: 0.25`; the map server converts 205 to an occupancy of 0.196, which
is below 0.25, so the whole outside of the building was free space and NavFn was willing to
plan through it. Lowering the threshold to 0.196 and setting `allow_unknown: false` fixed it.

My first obstacle layout trapped the robot. I put the box 0.7 m in front of the spawn pose in
a spot with a wall on one side and a chair leg on the other. The global plan flipped between
two routes every second as the cylinder moved, the controller kept turning around, and the
first leg took 131 s with nine recoveries. Looking at the distance transform of the map
instead of eyeballing it gave me a layout with real clearance.

Then the planner refused the obvious lane. With the stock TurtleBot3 inflation (0.55 m,
cost scaling 3.0) the 0.6 m gaps between chair-leg specks accumulated so much cost that
NavFn preferred an 8 m detour around the far side of the room over a 3 m lane, so the robot
never met the obstacles at all. A 0.3 m radius with scaling 6.0 keeps those lanes cheap
while still keeping the footprint out of the lethal band.

The simple commander undid my AMCL seed. `waitUntilNav2Active` publishes its own default
initial pose (all zeros) to `/initialpose` if it has not yet received an AMCL pose, and once
it won that race: AMCL jumped to the map origin, every plan failed with "start position is
off the global costmap", and I lost a while believing the launch was at fault. The scripts
now wait on the lifecycle states and the first `/amcl_pose` without publishing anything.

Smaller ones: the obstacle moved at half the commanded speed because the service call
sometimes took longer than the 10 Hz tick and I was advancing by a fixed step, so I switched
to real elapsed simulation time. The parameter file I started from still used the Foxy-era
`recoveries_server` name, which Nav2 1.1 silently ignores, so the behaviour server was
running on defaults until I renamed the section. And the composed Nav2 container needs about
15 s and a SIGKILL to shut down on Humble, which made restarts untidy; running the nodes as
separate processes fixed that.

## 3. What did you learn?

The most useful lesson was that localisation, costmaps and planning are only as good as the
agreement between the map and the world. Generating the world from the map removed a whole
class of problems; the remaining ones were all about cost, not geometry.

Inflation parameters are route-selection parameters, not just safety margins. A gentle,
wide inflation makes a cluttered real-world map look impassable to a grid planner even when
the robot fits comfortably, and the symptom is a strange detour rather than an error.

Replanning at 1 Hz against a moving obstacle can oscillate. If two homotopy classes have
similar cost, a small change in the obstacle position flips the plan and the controller never
commits. The fix was to give the robot one clearly better route, so it waits for the obstacle
instead of dithering, which is also what you want to watch in a demo.

Everything in a ROS 2 system that publishes to `/initialpose` is a localisation input, tools
included. I now treat helper libraries with the same suspicion as nodes.

Finally, instrumenting a test beats watching it. The ground-truth recorder told me the
robot was 3 cm from the cylinder in one run, which no amount of staring at RViz would have
shown, and it is why the obstacle now yields.

## 4. What would you improve next?

- Replace the teleported obstacle with a physics-driven one (a kinematic model driven by a
  planar-move plugin or an actor with collisions), so it can be pushed and the yield rule is
  not needed.
- Use the Smac Hybrid-A* planner and MPPI controller, which handle narrow lanes and moving
  obstacles better than NavFn plus DWB, and compare the same logged metrics.
- Add a cost-aware replanning trigger instead of the fixed 1 Hz, for example replan only
  when the current path is blocked or a shorter one appears by some margin.
- Turn `nav_test.py` into a proper regression suite: several start and goal pairs, N trials
  each, and a pass criterion on time, recoveries and clearance, run headless in CI.
- Clean the map a little before generating the world, for example remove isolated specks
  under three cells, with the cleaned map used for both so they still agree.
