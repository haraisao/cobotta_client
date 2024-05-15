#!/bin/bash
source /opt/ros/humble/setup.bash

ros2 service call /denso_joint_trajectory_controller/follow_joint_trajectory/_action/cancel_goal action_msgs/srv/CancelGoal
