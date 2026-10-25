"""Scene files: one YAML per world in config/scenes/ with the spawn pose, demo route,
obstacle placement and RViz view. Shared by the launch file and the goal scripts."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory


def scene_dir():
    return os.path.join(get_package_share_directory('antropi_nav2_demo'), 'config', 'scenes')


def list_scenes():
    return sorted(f[:-5] for f in os.listdir(scene_dir()) if f.endswith('.yaml'))


def load_scene(name):
    path = name if name.endswith('.yaml') else os.path.join(scene_dir(), name + '.yaml')
    if not os.path.exists(path):
        raise FileNotFoundError(f"scene '{name}' not found; available: {', '.join(list_scenes())}")
    with open(path) as f:
        s = yaml.safe_load(f)
    pkg = get_package_share_directory('antropi_nav2_demo')
    s['world_path'] = s['world'] if os.path.isabs(s['world']) else os.path.join(pkg, 'worlds', s['world'])
    s['map_path'] = s['map'] if os.path.isabs(s['map']) else os.path.join(pkg, 'maps', s['map'])
    s['route'] = [tuple(float(v) for v in g) for g in s['route']]
    return s
