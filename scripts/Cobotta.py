#
#
#
import sys
import os
import copy
import time
import traceback

import numpy as np

import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.action import ActionClient
#import actionlib
from action_msgs.srv import CancelGoal

import moveit
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.action
import geometry_msgs.msg
import trajectory_msgs.msg
from denso_robot_core_interfaces.srv import ChangeMode

from std_msgs.msg import String,Int32,UInt32

import cobotta_client

#########################
#
#
joints_home=[0, 20, 120, 0, -50, 0]

class Cobotta(Node):
  #
  #
  def __init__(self, extended=True, sim=False, ip_addr="192.168.0.1"):
    super().__init__("arm_controller")
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.scene_pub = self.create_publisher(moveit_msgs.msg.PlanningScene, 'planning_scene', 5)

    self.move_group = moveit_commander.MoveGroupCommander('arm')

    #
    # for COBOTTA
    self.mode_client =  self.create_client(ChangeMode, '/cobotta/ChangeMode')
    self.motion_client =  self.create_client(CancelGoal, '/denso_joint_trajectory_controller/follow_joint_trajectory/_action/cancel_goal')

    self.execution_client = ActionClient(self, moveit_msgs.action.ExecuteTrajectory, '/execute_trajectory')
    self.arm_controller_command = self.create_publisher(trajectory_msgs.msg.JointTrajectory, '/cobotta/arm_controller/command', 1)
    self.error_sub = self.create_subscription(UInt32, 'ErrorCode', self.error_callback, 10)
    
    self.bcap_client = cobotta_client.Rc8Client(ip_addr)
    #
    # Extended function adn topics by RT-Coorp

    self.update()

  #
  # 
  def cancel_all_actions(self):
    #self.execution_client.cancel_all_goals()
    #self.arm_controller_command.publish(trajectory_msgs.msg.JointTrajectory())
    req=CancelGoal.Request()
    self.motion_client.call_async(req)
    return

  #
  # 
  def set_scaling(self, v_scale=1.0, a_scale=1.0):
    self.move_group.set_max_acceleration_scaling_factor(a_scale)
    self.move_group.set_max_velocity_scaling_factor(v_scale)
    return

  #
  #
  def error_callback(self, msg):
    print("ErrorCode: ", hex(msg.data))
    self.cancel_all_actions()
   
  #
  #
  def update(self):
    self.get_current_joints()
    self.get_current_pose()
    return
  #
  #
  def get_current_joints(self):
    self.current_joints = self.move_group.get_current_joint_values()
    return self.current_joints

  #
  #
  def get_current_pose(self):
    self.current_pose = self.move_group.get_current_pose()
    return self.current_pose

  def move_gripper(self, v=0, sp=100):
    self.bcap_client.controller_execute('HandMoveA', [v, sp])
    return

  def home(self):
    self.move_joint(joints_home)
    return

  #
  #
  def close_hand(self, val=16,timeout=1, sp=100):
    self.bcap_client,controller_execute('HandMoveA', [val, sp])
    return

  #
  #
  def open_hand(self,sp=100):
    self.bcap_client.controller_execute('HandMoveA', [30, sp])
    return

  #
  #
  def reset(self):
    try:
      self.cancel_all_actions()
      self.bcap_client.reset()
      self.set_slave()
    except:
     print("Not supported")
    return

  def set_normal(self):
    req = ChangeMode.Request()
    req.mode=0
    self.mode_client.call_async(req)
    return

  def set_slave(self):
    req = ChangeMode.Request()
    req.mode=0x202
    self.mode_client.call_async(req)
    return

  #
  #
  def set_motor(self, val):
    try:
      self.bcap_client.motor(val)
    except:
     print("Not supported")

    return

  #
  #  
  def planning(self):
    ret, plan, _1, _2 =self.move_group.plan()
    if len(plan[0].joint_trajectory.points) == 0:
      return None
    else:
      return plan[0]

  #
  #  
  def move(self, x, y, z):
    self.update()
    target_pose = copy.deepcopy(self.current_pose)
    target_pose.pose.position.x += x
    target_pose.pose.position.y += y
    target_pose.pose.position.z += z

    self.move_group.set_pose_target(target_pose)
    plan = self.planning()

    if plan is None: return False
    res = self.move_group.execute(plan, wait=True)

    self.update()
    return res

  #
  #
  def move_joint(self, goal):
    joints = [np.deg2rad(x) for x in goal]
    res = self.move_group.go(joints, wait=True)
    self.update()
    return res

  #
  #
  def move_tp_pos(self, pname, tool=True):
    try:
      res=self.bcap_client.get_variable(pname)
      pos=res.value
      target=[pos[0]/1000, pos[1]/1000, pos[2]/1000, np.deg2rad(pos[3]) , np.deg2rad(pos[4]) , np.deg2rad(pos[5])]
      if tool :
        self.move_group_tool.set_pose_target(target)
        self.move_group_tool.go()
      else:
        self.move_group.set_pose_target(target)
        self.move_group.go()
      return target

    except:
      return None

  #
  #
  def set_current_pos_to_tp(self, pname):
    try:
      self.set_normal()
      pos=self.get_current_position()
      res=self.set_variable(pname, pos.value)
      self.set_slave()
      return True
    except:
      return False


if __name__ == '__main__':
  rclpy.init()
  logger = rclpy.logging.get_logger("moveit_py.pose_goal")

  cobotta = MoveItPy(node_name='moveit_py')
  arm = cobotta.get_planning_component()