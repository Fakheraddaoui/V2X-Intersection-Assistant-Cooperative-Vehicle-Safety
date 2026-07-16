from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='v2x_conflict_detector', executable='conflict_node',
             parameters=[{'ttc_threshold': 2.0, 'stale_after': 0.5}]),
    ])
