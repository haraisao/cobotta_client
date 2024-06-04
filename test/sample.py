#
#
import rclpy
import rclpy.logging

import moveit

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

  cobotta = moveit.MoveItPy(node_name='moveit_py')
  arm = cobotta.get_planning_component()
  logger.info("MoveItPy instance created")

  # set plan start state using predefined state
  arm.set_start_state(configuration_name="ready")

  # set pose goal using predefined state
  arm.set_goal_state(configuration_name="extended")

  # plan to goal
  plan_and_execute(cobotta, arm, logger)