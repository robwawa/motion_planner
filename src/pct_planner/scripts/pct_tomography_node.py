#!/usr/bin/env python3
"""ROS executable wrapper for the existing tomography implementation."""

import os
import runpy
import sys

import rospkg


def package_path():
    try:
        return rospkg.RosPack().get_path('pct_planner')
    except rospkg.common.ResourceNotFound:
        # Also support running directly from this repository before it is
        # placed under a catkin workspace's src/ directory.
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    package_root = package_path()
    script_dir = os.path.join(package_root, 'tomography', 'scripts')
    config_dir = os.path.join(package_root, 'tomography')
    traversability_config_dir = os.path.join(config_dir, 'config')
    sys.path.insert(0, script_dir)
    sys.path.insert(0, config_dir)
    sys.path.insert(0, traversability_config_dir)
    script_path = os.path.join(script_dir, 'tomography.py')
    # roslaunch appends private remapping arguments such as __name:=... and
    # __log:=....  They are meaningful to rospy but not to argparse.
    sys.argv = [script_path] + [arg for arg in sys.argv[1:] if ':=' not in arg]
    runpy.run_path(script_path, run_name='__main__')


if __name__ == '__main__':
    main()
