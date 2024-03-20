import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_name='wall_following_bravo' #<--- CHANGE ME

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    
    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params],
                    
    )

    return LaunchDescription([
        Node(
             package='wall_following_bravo',
             executable='dist_finder_bravo',
             name='dist_finder_bravo',
             output='screen'
         ),
         Node(
             package='wall_following_bravo',
             executable='control_bravo',
             name='control_bravo',
             output='screen'
        ),
        twist_mux_node
        # Node(
        #     package='wall_following_bravo',
        #     executable='Gap_Node',
        #     name='gap_finder_bravo',
        #     output='screen'
        # )
    ])