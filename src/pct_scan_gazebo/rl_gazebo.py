#!/usr/bin/env python3
"""TorchScript policy runner for the A1 Gazebo simulation only."""
import threading
import rospy
import torch
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pct_scan_gazebo.msg import MotorCmd, MotorState

JOINTS = ("FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
          "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf")
REINDEX = torch.tensor([3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8])
DEFAULT = torch.tensor([-.15, .55, -1.5, .15, .55, -1.5, -.15, .7, -1.5, .15, .7, -1.5])


class Policy:
    def __init__(self):
        self.lock = threading.Lock()
        self.q = torch.zeros(12); self.dq = torch.zeros(12)
        self.quat = torch.tensor([0., 0., 0., 1.]); self.omega = torch.zeros(3); self.cmd = torch.zeros(3)
        self.have_joints = self.have_odom = False
        model_path = rospy.get_param("~model")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = torch.jit.load(model_path, map_location=self.device).eval()
        self.history = torch.zeros(5, 45, device=self.device); self.actions = torch.zeros(12)
        robot = rospy.get_param("~robot_name", "a1")
        self.pubs = [rospy.Publisher(f"/{robot}_gazebo/{name}_controller/command", MotorCmd, queue_size=1) for name in JOINTS]
        for index, name in enumerate(JOINTS):
            rospy.Subscriber(f"/{robot}_gazebo/{name}_controller/state", MotorState, self.joint_cb, index, queue_size=1)
        rospy.Subscriber("/ground_truth/base_w", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb, queue_size=1)

    def joint_cb(self, msg, index):
        with self.lock:
            self.q[index], self.dq[index], self.have_joints = msg.q, msg.dq, True

    def odom_cb(self, msg):
        with self.lock:
            p = msg.pose.pose.orientation; w = msg.twist.twist.angular
            self.quat = torch.tensor([p.x, p.y, p.z, p.w]); self.omega = torch.tensor([w.x, w.y, w.z]); self.have_odom = True

    def cmd_cb(self, msg):
        with self.lock: self.cmd = torch.tensor([msg.linear.x, msg.linear.y, msg.angular.z])

    @staticmethod
    def inv_rotate(q, v):
        xyz, w = q[:3], q[3]
        return v * (2 * w * w - 1) - 2 * w * torch.cross(xyz, v, dim=0) + 2 * xyz * torch.dot(xyz, v)

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            with self.lock:
                if not (self.have_joints and self.have_odom): rate.sleep(); continue
                q, dq, quat, omega, cmd = self.q.clone(), self.dq.clone(), self.quat.clone(), self.omega.clone(), self.cmd.clone()
            ordered_q, ordered_dq = q[REINDEX], dq[REINDEX]
            obs = torch.cat((self.inv_rotate(quat, omega) * .25, self.inv_rotate(quat, torch.tensor([0., 0., -1.])),
                             cmd * torch.tensor([2., 2., .25]), ordered_q - DEFAULT, ordered_dq * .05, self.actions))
            self.history = torch.cat((self.history[1:], obs.to(self.device).unsqueeze(0)))
            with torch.inference_mode(): self.actions = self.model.act_inference(self.history.reshape(1, -1)).squeeze().cpu()
            targets = self.actions[REINDEX] * .25 + DEFAULT[REINDEX]
            for i, pub in enumerate(self.pubs):
                pub.publish(MotorCmd(mode=10, q=float(targets[i]), dq=0., Kp=80., Kd=1., tau=0.))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("a1_rl_policy")
    Policy().run()
