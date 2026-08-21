import sys
import argparse
import numpy as np

import rospy
from nav_msgs.msg import Path

from utils import *
from planner_wrapper import TomogramPlanner

sys.path.append('../')
from config import Config

parser = argparse.ArgumentParser()
parser.add_argument('--tomogram-name', default='building2_9',
                    help='Tomogram basename under rsc/tomogram, without .pickle')
parser.add_argument('--start', nargs=2, type=float, default=(5.0, 5.0),
                    metavar=('X', 'Y'))
parser.add_argument('--goal', nargs=2, type=float, default=(-6.0, -1.0),
                    metavar=('X', 'Y'))
args = parser.parse_args()

cfg = Config()
tomo_file = args.tomogram_name
start_pos = np.asarray(args.start, dtype=np.float32)
end_pos = np.asarray(args.goal, dtype=np.float32)

path_pub = rospy.Publisher("/pct_path", Path, latch=True, queue_size=1)
planner = TomogramPlanner(cfg)

def pct_plan():
    planner.loadTomogram(tomo_file)

    traj_3d = planner.plan(start_pos, end_pos)
    if traj_3d is not None:
        path_pub.publish(traj2ros(traj_3d))
        print("Trajectory published")


if __name__ == '__main__':
    rospy.init_node("pct_planner", anonymous=True)

    pct_plan()

    rospy.spin()
