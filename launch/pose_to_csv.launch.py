from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='listener_recorder',
            executable='target_recorder',
            name='pose_to_csv_node',
            # output='screen'
        ),
        Node(
            package='listener_recorder',
            executable='trajectory_listener',
            name='csv_writer',
            # output='screen'
        )
    ])
