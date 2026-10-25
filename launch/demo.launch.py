"""One-command TurtleBot3 + Nav2 demo in the generated lab world.

    ros2 launch antropi_nav2_demo demo.launch.py
    ros2 launch antropi_nav2_demo demo.launch.py gui:=false rviz:=false   # headless
    ros2 launch antropi_nav2_demo demo.launch.py obstacle_mode:=crossing
    ros2 launch antropi_nav2_demo demo.launch.py obstacle:=false static_obstacle:=false

Starts, in order of dependency: Gazebo (server, optional client), the TurtleBot3
robot_state_publisher and spawn, the moving obstacle and its driver, Nav2 via
nav2_bringup (map_server, AMCL, planner, controller, recoveries, BT navigator,
lifecycle managers) and RViz. The AMCL initial pose in config/nav2_params.yaml
matches the default spawn pose, so the robot is localised without any click.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context, *args, **kwargs):
    pkg = get_package_share_directory('antropi_nav2_demo')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    model = LaunchConfiguration('model').perform(context)
    patrol = [float(v) for v in LaunchConfiguration('patrol').perform(context).split(',')]
    use_sim_time = LaunchConfiguration('use_sim_time')
    x, y, yaw = (LaunchConfiguration(k) for k in ('x_pose', 'y_pose', 'yaw'))

    with open(os.path.join(tb3_gazebo, 'urdf', f'turtlebot3_{model}.urdf')) as f:
        robot_description = f.read()
    robot_sdf = os.path.join(tb3_gazebo, 'models', f'turtlebot3_{model}', 'model.sdf')
    obstacle_sdf = os.path.join(pkg, 'models', 'moving_obstacle', 'model.sdf')
    static_box_sdf = os.path.join(pkg, 'models', 'static_box', 'model.sdf')

    actions = [
        # Gazebo Classic server + client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'),
                              'verbose': LaunchConfiguration('verbose')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))),

        # TurtleBot3
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]),
        Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_turtlebot3',
             output='screen',
             arguments=['-entity', model, '-file', robot_sdf,
                        '-x', x, '-y', y, '-z', '0.01', '-Y', yaw]),

        # Unmapped static obstacle on the first leg of the demo route
        Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_static_box',
             output='screen', condition=IfCondition(LaunchConfiguration('static_obstacle')),
             arguments=['-entity', 'static_box', '-file', static_box_sdf,
                        '-x', LaunchConfiguration('static_x'), '-y', LaunchConfiguration('static_y'), '-z', '0.0']),

        # Moving obstacle (model + driver)
        Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_moving_obstacle',
             output='screen', condition=IfCondition(LaunchConfiguration('obstacle')),
             arguments=['-entity', 'moving_obstacle', '-file', obstacle_sdf,
                        '-x', str(patrol[0]), '-y', str(patrol[1]), '-z', '0.0']),
        Node(package='antropi_nav2_demo', executable='dynamic_obstacle', name='dynamic_obstacle',
             output='screen', condition=IfCondition(LaunchConfiguration('obstacle')),
             parameters=[{'use_sim_time': use_sim_time,
                          'mode': LaunchConfiguration('obstacle_mode'),
                          'speed': LaunchConfiguration('obstacle_speed'),
                          'patrol_start': patrol[0:2], 'patrol_end': patrol[2:4],
                          'park_pose': patrol[0:2],
                          'robot_model_name': model}]),

        # Nav2 (map server, AMCL, planner, controller, recoveries, BT navigator, lifecycle)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={'map': LaunchConfiguration('map'),
                              'params_file': LaunchConfiguration('params_file'),
                              'use_sim_time': use_sim_time,
                              'autostart': 'true',
                              # separate processes: clearer logs and a fast, clean Ctrl-C shutdown
                              # (the composed container needs ~15 s and a SIGKILL on Humble)
                              'use_composition': 'False'}.items()),

        # RViz
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', LaunchConfiguration('rviz_config')],
             parameters=[{'use_sim_time': use_sim_time}]),
    ]
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('antropi_nav2_demo')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    model_path = os.pathsep.join(filter(None, [
        os.path.join(pkg, 'models'), os.path.join(tb3_gazebo, 'models'),
        os.environ.get('GAZEBO_MODEL_PATH', '')]))

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        DeclareLaunchArgument('model', default_value='burger', description='TurtleBot3 model: burger | waffle | waffle_pi'),
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'lab.world')),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'lab_map.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(pkg, 'rviz', 'nav2_demo.rviz')),
        DeclareLaunchArgument('x_pose', default_value='-1.9', description='spawn x (map frame); keep in sync with amcl.initial_pose'),
        DeclareLaunchArgument('y_pose', default_value='2.75'),
        DeclareLaunchArgument('yaw', default_value='-1.5708'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true', description='start the Gazebo client window'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false', description='verbose gzserver output'),
        DeclareLaunchArgument('static_obstacle', default_value='true', description='spawn an unmapped static box on the route'),
        DeclareLaunchArgument('static_x', default_value='-2.0'),
        DeclareLaunchArgument('static_y', default_value='0.4'),
        DeclareLaunchArgument('obstacle', default_value='true', description='spawn and drive the moving obstacle'),
        DeclareLaunchArgument('obstacle_mode', default_value='patrol', description='patrol | crossing'),
        DeclareLaunchArgument('obstacle_speed', default_value='0.15', description='obstacle speed in m/s'),
        DeclareLaunchArgument('patrol', default_value='-2.7,1.9,-1.7,1.9', description='patrol segment x0,y0,x1,y1 (map frame)'),
        OpaqueFunction(function=setup),
    ])
