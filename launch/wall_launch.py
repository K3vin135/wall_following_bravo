from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='wall_following_bravo',
        #     executable='dist_finder_bravo',
        #     name='dist_finder_bravo',
        #     output='screen'
        # ),
        # Node(
        #     package='wall_following_bravo',
        #     executable='control_bravo',
        #     name='control_bravo',
        #     output='screen'
        # )
        Node(
            package='wall_following_bravo',
            executable='Gap_Node',
            name='gap_finder_bravo',
            output='screen'
        )
    ])