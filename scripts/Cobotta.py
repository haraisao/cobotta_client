#!/usr/bin/python3
#
#
import sys
import copy
import time

import numpy as np
from rclpy.node import Node
from action_msgs.srv import CancelGoal

import moveit_commander
import moveit_msgs.msg

from denso_robot_core_interfaces.srv import ChangeMode
from std_msgs.msg import Int32
import rclpy

import matplotlib.pyplot as plt

#########################
#
def get_time_val(x):
    return x.time_from_start.sec + x.time_from_start.nanosec/1000000000

def get_time_seq(plan):
    return [get_time_val(x) for x in plan.joint_trajectory.points]

def get_pos_seq(plan):
    return [x.positions for x in plan.joint_trajectory.points]

def plot_joint_plan(plan):
    x = get_time_seq(plan)
    y = np.array(get_pos_seq(plan))
    for i in range(6):
        plt.plot(x, y[:, i])
    plt.show()

#########################
#
#
JOINTS_HOME = [0, 20, 120, 0, -50, 0]
JOINTS_PACK = [90, -30, 120, -170, -94, 0]
HOME_POSE = [0.1, 0.1, 0.04, 0, np.pi, 0]
GRIPPER_MAX_WIDTH = 30  # milimeter
BIG_OBJECT_GRIPPER_WIDTH = 24  # milimeter
SMALL_OBJECT_GRIPPER_WIDTH = 5  # milimeter

POS1 = [27, 63, 60, 33, -50, 28]
POS2 = [-23, 29, 122, -25, -75, 12]

class Cobotta(Node):
    #
    #
    def __init__(self, extended=True, use_sim=None):
        try:
            rclpy.init()
        except Exception:
            pass
        super().__init__("arm_controller")
        self.declare_parameter("sim", False)
        if use_sim is None:
            sim_mode = self.get_parameter("sim").get_parameter_value().bool_value
        else:
            sim_mode=use_sim
        moveit_commander.roscpp_initialize(sys.argv, use_sim=sim_mode)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.scene_pub = self.create_publisher(
            moveit_msgs.msg.PlanningScene, "planning_scene", 5
        )

        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.hand = moveit_commander.MoveGroupCommander("hand")

        #
        # for COBOTTA
        self.mode_client = self.create_client(ChangeMode, "/cobotta/ChangeMode")
        self.motion_client = self.create_client(
            CancelGoal,
            "/denso_joint_trajectory_controller/follow_joint_trajectory/_action/cancel_goal",
        )
        self.execution_client = self.create_client(
            CancelGoal,
            "/execute_trajectory/_action/cancel_goal",
        )
        self.move_client = self.create_client(
            CancelGoal,
            "/move_action/_action/cancel_goal",
        )
        self.sub_state = self.create_subscription(Int32, "/cobotta/CurMode",
                                                  self.callback_cur_mode, 10)
        #
        # Extended function adn topics by RT-Coorp
        self.current_joints = None
        self.current_pose = None
        self.velocity = 0.8
        self.accel = 0.6
        self.resample = 0.0
        self.current_mode = 514
        self.esp = 0.01

        self.set_scaling(self.velocity, self.accel)
        self.arm.set_planning_time(8.0)
        self.update()

    #
    #
    def cancel_all_actions(self):
        req = CancelGoal.Request()
        self.move_client.call_async(req)
        self.execution_client.call_async(req)
        self.motion_client.call_async(req)
        return

    #
    #
    def set_scaling(self, v_scale=1.0, a_scale=1.0):
        self.arm.set_max_acceleration_scaling_factor(a_scale)
        self.arm.set_max_velocity_scaling_factor(v_scale)
        self.velocity=v_scale
        self.accel=a_scale
        return

    def callback_cur_mode(self, msg):
        self.current_mode=msg.data
        return

    #
    #
    def update(self):
        self.get_current_joints()
        self.get_current_pose()
        return

    #
    #
    def get_current_joints(self):
        self.current_joints = self.arm.get_current_joint_values()
        return self.current_joints

    #
    #
    def get_current_pos(self):
        pos_ = self.current_pose.pose.position
        return [pos_.x, pos_.y, pos_.z]

    #
    #
    def get_current_pose(self):
        self.current_pose = self.arm.get_current_pose()
        return self.current_pose

    #
    #
    #
    def move_gripper(self, v=0):
        val = v / 2000.0
        self.hand.set_joint_value_target([val, -val])
        self.hand.go()
        return

    #
    #
    def home(self, id=0):
        if id == 1:
            self.move_pose(HOME_POSE)
        else:
            self.move_joint(JOINTS_HOME)
        return

    def reset_target(self):
        self.arm.set_pose_target(self.get_current_pose())
        self.arm.go()
        return

    #
    #
    def close_hand(self, val=5):
        self.move_gripper(val)
        return

    #
    #
    def open_hand(self):
        self.move_gripper(30)
        return

    #
    #
    def reset(self):
        try:
            self.reset_target()
            self.arm.stop()
            self.cancel_all_actions()
            self.set_slave()
        except Exception as e:
            self.get_logger().info(f"COBOTTA RESET WARNING: Not supported {e}")
        return

    def set_normal(self):
        req = ChangeMode.Request()
        req.mode = 0
        self.mode_client.call_async(req)
        return

    def set_slave(self):
        req = ChangeMode.Request()
        req.mode = 0x202
        self.mode_client.call_async(req)
        return

    def retime_plan(self, plan, vel=0, acc=0, resample=-1):
        stat_ = self.arm.get_current_state()
        if vel <= 0 :
            vel=self.velocity
        if acc <= 0 :
            acc=self.accel
        if resample < 0 :
            resample=self.resample
        if resample:
            return self.arm.retime_trajectory(stat_, plan, vel, acc, resample_dt=resample)
        else:
            return self.arm.retime_trajectory(stat_, plan, vel, acc)

    def get_joint_trajectory(self, plan, deg=False, restruct=False):
        trj=[]
        try:
            if plan[0] :
                trj_ = plan[1][0].joint_trajectory
                for p in trj_.points:
                    if deg:
                        trj.append(np.rad2deg(p.positions))
                    else:
                        trj.append(p.positions)
                if restruct:
                    return self.get_waypoints(trj)
                else:
                    return trj
            else:
                print("Fail to get plan")
                return None
        except Exception:
            print("Invalid argument")
            return None

    def get_waypoints(self, trj):
        dt = np.array(trj[1])  - np.array(trj[0])
        res = [trj[0]]
        #res = []
        for i in range(len(trj) - 2):
            dt_tmp = np.array(trj[i+2]) - np.array(trj[i+1])
            if np.linalg.norm(dt - dt_tmp) > self.esp :
                res.append(trj[i+1])
                dt = dt_tmp
        res.append(trj[-1])
        return res

    def resample_plan(self, plan):
        #pp_ = copy.copy(plan.joint_trajectory.points)
        #plan.joint_trajectory.points = pp_[::2]
        return plan
    #
    #
    def planning(self):
        ret, plan, _1, _2 = self.arm.plan()
        if len(plan[0].joint_trajectory.points) == 0:
            return None
        else:
            if self.resample > 0:
                return self.arm.retime_trajectory(self.arm.get_current_state(), plan[0],
                        self.velocity, self.accel, resample_dt=self.resample)
            else:
                return self.resample_plan(plan[0])

    #
    #
    def move(self, x, y, z):
        self.update()
        target_pose = copy.deepcopy(self.current_pose)
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)

        self.arm.set_pose_target(target_pose)
        plan = self.planning()

        if plan is None:
            return False
        if self.current_mode == 0: return False
        res = self.arm.execute(plan, wait=True)

        self.update()
        return res

    def move_orientation(self, ori):
        self.update()
        target_pose = self.get_current_pos() + ori
        self.arm.set_pose_target(target_pose)
        plan = self.planning()

        if plan is None:
            return False

        if self.current_mode == 0:
            return False
        res = self.arm.execute(plan, wait=True)
        self.update()
        return

    def move_pose(self, target_pose):
        self.update()
        self.arm.set_pose_target(target_pose)
        plan = self.planning()

        if plan is None:
            return False

        if self.current_mode == 0: return False
        res = self.arm.execute(plan, wait=True)

        self.update()
        return res

    #
    #
    def move_joint(self, goal):
        if self.current_mode == 0: return False
        joints = [np.deg2rad(x) for x in goal]
        res = self.arm.go(joints, wait=True)
        self.update()
        return res

    def pickup_from_position(self, x, y, z, gripper_width=7, height_margin=0.05):
        # ピッキングの高さまえ移動
        pose = self.current_pose
        pose.pose.position.z = z + height_margin
        res=self.move_pose(pose)
        if not res:
            return False
        self.open_hand()

        # 物体の真上まで移動
        pose.pose.position.x = x
        pose.pose.position.y = y
        res=self.move_pose(pose)
        if not res:
            return False

        time.sleep(0.8)
        # 物体を掴みに行く
        pose.pose.position.z = z
        res=self.move_pose(pose)
        if not res:
            return False
        self.close_hand(val=gripper_width)

        # 物体を持ち上げる
        pose.pose.position.z = z + height_margin
        res=self.move_pose(pose)
        if not res:
            return False
        return True

    def dropoff_to_position(self, x, y, z, height_margin=0.05):
        pose = self.current_pose

        # 置く位置まで移動
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z + height_margin
        res = self.move_pose(pose)
        if not res:
            return False

        time.sleep(1)
        # 物体を置く
        pose.pose.position.z -= height_margin
        res=self.move_pose(pose)
        if not res:
            return False
        self.open_hand()
        return True

    def add_box(self, name, x, y, z, size=(0.05, 0.05, 0.05)):
        pose_ = self.scene.gen_pose(x, y, z)
        self.scene.add_box(name, pose_, size)
        return

    def move_joint_value(self, jpos, deg=True):
        self.arm.set_joint_value_target(jpos, deg=deg)
        self.arm.go()
        return
