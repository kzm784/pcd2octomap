import os
import launch
import launch_ros

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    # Set the path to the pcd2octomap config
    pcd2octomap_config_path = launch.substitutions.LaunchConfiguration(
        'pcd2octomap_config',
        default=os.path.join(
            get_package_share_directory('pcd2octomap'),
                'config',
                'config_pcd2octomap.yaml'
        )
    )

    waypoint_navigator_node = Node(
        package='pcd2octomap',
        executable='pcd2octomap_node',
        name='pcd2octomap_node',
        output='screen',
        parameters=[pcd2octomap_config_path]
    )
    
    ld.add_action(waypoint_navigator_node)

    return ld