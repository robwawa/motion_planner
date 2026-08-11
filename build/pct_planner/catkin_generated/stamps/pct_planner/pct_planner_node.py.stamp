#!/usr/bin/env python3
"""ROS executable wrapper for the planner, with tomogram readiness waiting."""

import argparse
import os
import runpy
import sys
import time

import rospkg


def package_path():
    try:
        return rospkg.RosPack().get_path('pct_planner')
    except rospkg.common.ResourceNotFound:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--scene', default='Spiral')
    parser.add_argument('--wait-timeout', type=float, default=300.0)
    args, _ = parser.parse_known_args()

    package_root = package_path()
    tomo_names = {
        'Spiral': 'spiral0.3_2',
        'Building': 'building2_9',
        'Plaza': 'plaza3_10',
    }
    if args.scene not in tomo_names:
        raise ValueError('Unknown scene: {}'.format(args.scene))

    tomo_path = os.path.join(package_root, 'rsc', 'tomogram',
                             tomo_names[args.scene] + '.pickle')
    deadline = time.time() + args.wait_timeout
    while not os.path.isfile(tomo_path):
        if time.time() >= deadline:
            raise RuntimeError('Timed out waiting for tomogram: {}'.format(tomo_path))
        print('Waiting for tomogram: {}'.format(tomo_path), flush=True)
        time.sleep(1.0)

    script_dir = os.path.join(package_root, 'planner', 'scripts')
    planner_root = os.path.join(package_root, 'planner')
    lib_dir = os.path.join(planner_root, 'lib')
    native_dirs = [
        os.path.join(lib_dir, 'build', 'src', 'a_star'),
        os.path.join(lib_dir, 'build', 'src', 'map_manager'),
        os.path.join(lib_dir, 'build', 'src', 'trajectory_optimization'),
        os.path.join(lib_dir, 'build', 'src', 'ele_planner'),
        os.path.join(lib_dir, 'build', 'src', 'common', 'smoothing'),
        os.path.join(lib_dir, '3rdparty', 'gtsam-4.1.1', 'install', 'lib'),
    ]
    old_ld = os.environ.get('LD_LIBRARY_PATH', '')
    os.environ['LD_LIBRARY_PATH'] = ':'.join(native_dirs + ([old_ld] if old_ld else []))
    sys.path.insert(0, script_dir)
    sys.path.insert(0, planner_root)

    sys.argv = [os.path.join(script_dir, 'plan.py'), '--scene', args.scene]
    runpy.run_path(os.path.join(script_dir, 'plan.py'), run_name='__main__')


if __name__ == '__main__':
    main()
