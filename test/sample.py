#
#
import os
import sys
import yaml
import copy
import numpy as np

import rclpy
import rclpy.logging

from geometry_msgs.msg import PoseStamped, Pose


from moveit.core.robot_state import RobotState
from moveit.planning import  MoveItPy

from config import moveit_config


def plan_and_execute(
  robot, planning_component, logger,
  single_plan_parameters=None,
  multi_plan_parameters=None,
  ):
  """A helper function to plan and execute a motion."""
  # plan to goal
  logger.info("Planning trajectory")
  if multi_plan_parameters is not None:
    plan_result = planning_component.plan(
      multi_plan_parameters=multi_plan_parameters
    )
  elif single_plan_parameters is not None:
    plan_result = planning_component.plan(
      single_plan_parameters=single_plan_parameters
    )
  else:
    plan_result = planning_component.plan()

  # execute the plan
  if plan_result:
    logger.info("Executing plan")
    robot_trajectory = plan_result.trajectory
    robot.execute(robot_trajectory, controllers=[])
  else:
    logger.error("Planning failed")

if __name__ == '__main__':
  dx = float(sys.argv[1])
  dz = float(sys.argv[2])

  rclpy.init()
  logger = rclpy.logging.get_logger("moveit_py.pose_goal")

  cobotta = MoveItPy(node_name='moveit_py', config_dict=moveit_config.to_dict())
  arm = cobotta.get_planning_component("arm")
  logger.info("MoveItPy instance created")

  model = cobotta.get_robot_model()
  print(model.joint_model_groups[0].link_model_names)

  s1=arm.get_start_state()
  print(s1.get_pose('J6'))

  goal_pose = PoseStamped()
  goal_pose.header.frame_id = "world"
  goal_pose.pose = copy.deepcopy(s1.get_pose('J6'))
  goal_pose.pose.position.x += dx
  goal_pose.pose.position.z += dz

  arm.set_start_state_to_current_state()

  arm.set_goal_state(pose_stamped_msg=goal_pose, pose_link='J6')
  plan_result=arm.plan()
  if plan_result:
    trj = plan_result.trajectory
    print(trj)
    cobotta.execute(trj, controllers=[])
  else:
    print("Fail to plan...")

  # set plan start state using predefined state
  #arm.set_start_state(configuration_name="ready")
  # set pose goal using predefined state
  #arm.set_goal_state(configuration_name="extended")
  # plan to goal
  #plan_and_execute(cobotta, arm, logger)

  print("======= END")
  cobotta.shutdown()
