import os
from glob import glob

from setuptools import setup

package_name = 'antropi_nav2_demo'


def tree(src_root):
    """All files under src_root as (install_dir, [files]) pairs, keeping the directory layout."""
    out = []
    for d, _, files in os.walk(src_root):
        if files:
            out.append((os.path.join('share', package_name, d), [os.path.join(d, f) for f in files]))
    return out

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'scenes'), glob('config/scenes/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        *tree('models'),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lochan Bhaskaruni',
    maintainer_email='lochan25526@gmail.com',
    description='TurtleBot3 + Nav2 demo in a Gazebo world generated from a real SLAM map, with a moving obstacle.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dynamic_obstacle = antropi_nav2_demo.dynamic_obstacle:main',
            'send_goal = antropi_nav2_demo.send_goal:main',
        ],
    },
)
