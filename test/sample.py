#
#
import os
import sys
import yaml
import copy
import numpy as numpy

import rclpy
import rclpy.logging

from geometry_msgs.msg import PoseStamped, Pose


from moveit.core.robot_state import RobotState
from moveit.planning import  MoveItPy

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


IP_ADDR="127.0.0.1"
MODEL="cobotta"

moveit_config = (
        MoveItConfigsBuilder("denso_robot", robot_description="robot_description",
                              package_name="denso_robot_moveit_config")
        .robot_description(
          file_path=os.path.join(get_package_share_directory("denso_robot_descriptions")
                        ,"urdf" , "denso_robot.urdf.xacro"),
          mappings={ "ip_address": IP_ADDR,
                      "model": MODEL,
                      "send_format": "0",
                      "recv_format": "2",
                      "namespace": "",
                      "verbose": "false",
                      "sim": "false",
                      "with_tool": "false",
                      "ros2_control_hardware_type": "",
                    },
          )
        .robot_description_semantic(
          file_path="srdf/denso_robot.srdf.xacro",
          mappings={
                      "model": MODEL,
                      "namespace": "",
              }, 

        )
        .trajectory_execution(file_path="robots/"+MODEL+"/config/moveit_controllers.yaml")
        .joint_limits(file_path="robots/"+MODEL+"/config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .moveit_cpp(
          file_path=os.path.join(get_package_share_directory("denso_robot_moveit_config"),
                                    "config", "motion_planning.yaml")
        )
        .to_moveit_configs()
    )

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
  goal_pose.pose.position.x += 0.05

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

  cobotta.shutdown()