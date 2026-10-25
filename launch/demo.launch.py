"""One-command TurtleBot3 + Nav2 demo in one of the bundled scenes.

    ros2 launch antropi_nav2_demo demo.launch.py                      # internship lab (default)
    ros2 launch antropi_nav2_demo demo.launch.py scene:=tb3_house     # tb3_world | tb3_house | aws_small_warehouse
    ros2 launch antropi_nav2_demo demo.launch.py gui:=false rviz:=false   # headless
    ros2 launch antropi_nav2_demo demo.launch.py obstacle_mode:=crossing

A scene file (config/scenes/<scene>.yaml) supplies the world, the map, the spawn pose,
the obstacle placement and the RViz view; every one of them can still be overridden
with the launch arguments below. The AMCL initial pose is rewritten to the spawn pose,
so the robot is localised without any click.

Starts: Gazebo (server, optional client), robot_state_publisher and the TurtleBot3
spawn, the unmapped box, the moving obstacle and its driver, Nav2 via nav2_bringup
(map_server, AMCL, planner, controller, behaviours, BT navigator, lifecycle managers)
and RViz.
"""
import os
import tempfile

import yaml
from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from antropi_nav2_demo.scenes import load_scene


def _arg(context, name, fallback):
    v = LaunchConfiguration(name).perform(context)
    return v if v != '' else str(fallback)


def _write_params(params_file, map_path, spawn, out_dir):
    """Copy the Nav2 params with the AMCL seed set to the spawn pose and the map path filled in."""
    with open(params_file) as f:
        p = yaml.safe_load(f)
    amcl = p['amcl']['ros__parameters']
    amcl['set_initial_pose'] = True
    amcl['initial_pose'] = {'x': spawn['x'], 'y': spawn['y'], 'z': 0.0, 'yaw': spawn['yaw']}
    p['map_server']['ros__parameters']['yaml_filename'] = map_path
    out = os.path.join(out_dir, 'nav2_params.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(p, f, sort_keys=False)
    return out


def _write_rviz(rviz_file, view, out_dir):
    with open(rviz_file) as f:
        r = yaml.safe_load(f)
    cur = r['Visualization Manager']['Views']['Current']
    cur['X'], cur['Y'], cur['Scale'] = float(view['x']), float(view['y']), float(view['scale'])
    out = os.path.join(out_dir, 'nav2_demo.rviz')
    with open(out, 'w') as f:
        yaml.safe_dump(r, f, sort_keys=False)
    return out


def setup(context, *args, **kwargs):
    pkg = get_package_share_directory('antropi_nav2_demo')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    scene = load_scene(LaunchConfiguration('scene').perform(context))
    model = LaunchConfiguration('model').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    world = _arg(context, 'world', scene['world_path'])
    map_path = _arg(context, 'map', scene['map_path'])
    spawn = {'x': float(_arg(context, 'x_pose', scene['spawn']['x'])),
             'y': float(_arg(context, 'y_pose', scene['spawn']['y'])),
             'yaw': float(_arg(context, 'yaw', scene['spawn']['yaw']))}
    patrol = [float(v) for v in _arg(context, 'patrol', ','.join(str(v) for v in scene['patrol'])).split(',')]
    box = {'x': _arg(context, 'static_x', scene['static_box']['x']),
           'y': _arg(context, 'static_y', scene['static_box']['y'])}

    run_dir = tempfile.mkdtemp(prefix=f"antropi_nav2_{scene['name']}_")
    params = _write_params(LaunchConfiguration('params_file').perform(context), map_path, spawn, run_dir)
    rviz_cfg = _write_rviz(LaunchConfiguration('rviz_config').perform(context), scene['rviz'], run_dir)

    with open(os.path.join(tb3_gazebo, 'urdf', f'turtlebot3_{model}.urdf')) as f:
        robot_description = f.read()
    robot_sdf = os.path.join(tb3_gazebo, 'models', f'turtlebot3_{model}', 'model.sdf')
    obstacle_sdf = os.path.join(pkg, 'models', 'moving_obstacle', 'model.sdf')
    static_box_sdf = os.path.join(pkg, 'models', 'static_box', 'model.sdf')

    return [
        # Gazebo Classic server + client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world, 'verbose': LaunchConfiguration('verbose')}.items()),
        # gzclient occasionally aborts on start-up (boost shared_ptr<Camera> assertion, a Gazebo 11
        # race with the server's scene). Give the server a head start and respawn the client if
        # it dies; a second start always came up in testing.
        TimerAction(period=5.0, actions=[ExecuteProcess(
            cmd=['gzclient'], output='log', respawn=True, respawn_delay=3.0,
            condition=IfCondition(LaunchConfiguration('gui')))]),

        # TurtleBot3
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]),
        Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_turtlebot3', output='screen',
             arguments=['-entity', model, '-file', robot_sdf,
                        '-x', str(spawn['x']), '-y', str(spawn['y']), '-z', '0.01', '-Y', str(spawn['yaw'])]),

        # Unmapped static obstacle on the demo route
        Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_static_box', output='screen',
             condition=IfCondition(LaunchConfiguration('static_obstacle')),
             arguments=['-entity', 'static_box', '-file', static_box_sdf, '-x', box['x'], '-y', box['y'], '-z', '0.0']),

        # Moving obstacle (model + driver)
        Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_moving_obstacle', output='screen',
             condition=IfCondition(LaunchConfiguration('obstacle')),
             arguments=['-entity', 'moving_obstacle', '-file', obstacle_sdf,
                        '-x', str(patrol[0]), '-y', str(patrol[1]), '-z', '0.0']),
        Node(package='antropi_nav2_demo', executable='dynamic_obstacle', name='dynamic_obstacle', output='screen',
             condition=IfCondition(LaunchConfiguration('obstacle')),
             parameters=[{'use_sim_time': use_sim_time,
                          'mode': LaunchConfiguration('obstacle_mode'),
                          'speed': LaunchConfiguration('obstacle_speed'),
                          'patrol_start': patrol[0:2], 'patrol_end': patrol[2:4], 'park_pose': patrol[0:2],
                          'robot_model_name': model}]),

        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={'map': map_path, 'params_file': params, 'use_sim_time': use_sim_time,
                              'autostart': 'true',
                              # separate processes: clearer logs and a fast, clean Ctrl-C shutdown
                              # (the composed container needs ~15 s and a SIGKILL on Humble)
                              'use_composition': 'False'}.items()),

        # RViz
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', rviz_cfg], parameters=[{'use_sim_time': use_sim_time}]),
    ]


def generate_launch_description():
    pkg = get_package_share_directory('antropi_nav2_demo')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    model_dirs = [os.path.join(pkg, 'models'), os.path.join(tb3_gazebo, 'models')]
    # The AWS warehouse models point at their visual meshes with paths relative to the package
    # share directory ("models/<name>/meshes/<name>_visual.DAE"), which Gazebo resolves through
    # GAZEBO_RESOURCE_PATH. Without it gzclient draws an empty warehouse while gzserver, which
    # only needs the model:// collision meshes, works fine.
    resource_dirs = []
    try:  # optional: only the aws_small_warehouse scene needs it
        aws = get_package_share_directory('aws_robomaker_small_warehouse_world')
        model_dirs.append(os.path.join(aws, 'models'))
        resource_dirs.append(aws)
    except PackageNotFoundError:
        pass
    model_path = os.pathsep.join(model_dirs + [os.environ.get('GAZEBO_MODEL_PATH', '')]).rstrip(os.pathsep)
    resource_path = os.pathsep.join(resource_dirs + [os.environ.get('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11')])

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', resource_path),
        DeclareLaunchArgument('scene', default_value='lab', description='lab | tb3_world | tb3_house | aws_small_warehouse (config/scenes/)'),
        DeclareLaunchArgument('model', default_value='burger', description='TurtleBot3 model: burger | waffle | waffle_pi'),
        DeclareLaunchArgument('world', default_value='', description='override the scene world file'),
        DeclareLaunchArgument('map', default_value='', description='override the scene map yaml'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(pkg, 'rviz', 'nav2_demo.rviz')),
        DeclareLaunchArgument('x_pose', default_value='', description='override the scene spawn x (map frame)'),
        DeclareLaunchArgument('y_pose', default_value=''),
        DeclareLaunchArgument('yaw', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true', description='start the Gazebo client window'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false', description='verbose gzserver output'),
        DeclareLaunchArgument('static_obstacle', default_value='true', description='spawn an unmapped static box on the route'),
        DeclareLaunchArgument('static_x', default_value='', description='override the scene box x'),
        DeclareLaunchArgument('static_y', default_value=''),
        DeclareLaunchArgument('obstacle', default_value='true', description='spawn and drive the moving obstacle'),
        DeclareLaunchArgument('obstacle_mode', default_value='patrol', description='patrol | crossing'),
        DeclareLaunchArgument('obstacle_speed', default_value='0.15', description='obstacle speed in m/s'),
        DeclareLaunchArgument('patrol', default_value='', description='override the scene patrol segment x0,y0,x1,y1'),
        OpaqueFunction(function=setup),
    ])
