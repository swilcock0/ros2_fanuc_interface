from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ld = LaunchDescription()
    
    config = os.path.join(
        get_package_share_directory('crx_description'),
        'config',
        'kinematics.yaml'
        )
    
    moveit_config = (
        MoveItConfigsBuilder(robot_name="manipulator", 
                        package_name="crx10ia_l_moveit_config",
                        )
                        .moveit_cpp(
                            file_path=get_package_share_directory("crx_description")
                            + "/config/moveitpy.yaml"
                        ).to_moveit_configs()
    )
    
    reach = Node(
        package="crx_description",
        executable="reachability.py",
        # parameters = [config]
        parameters=[moveit_config.to_dict()],
    )
    
    ld.add_action(reach)
    return ld