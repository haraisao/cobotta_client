#!/bin/bash
source /opt/ros/humble/setup.bash

ros2 service call /cobotta/ChangeMode denso_robot_core_interfaces/srv/ChangeMode mode:\ 514\
