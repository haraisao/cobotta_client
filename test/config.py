#
#
import os
import sys

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


IP_ADDR="127.0.0.1"
MODEL="cobotta"

def get_pkg_dir(name, sub_dir=""):
    return os.path.join(get_package_share_directory(name), sub_dir)

MOVEIT_CONFIG = (
  MoveItConfigsBuilder("denso_robot", package_name="denso_robot_moveit_config")
    .robot_description(
          file_path=os.path.join(get_pkg_dir("denso_robot_descriptions","urdf"), "denso_robot.urdf.xacro"),
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
          mappings={ "model": MODEL, "namespace": "", }, 
                )
    .trajectory_execution(file_path="robots/"+MODEL+"/config/moveit_controllers.yaml")
    .joint_limits(file_path="robots/"+MODEL+"/config/joint_limits.yaml")
    .robot_description_kinematics(file_path="config/kinematics.yaml")
    .moveit_cpp(
      file_path="config/motion_planning.yaml"
    )
    .to_moveit_configs()
)
