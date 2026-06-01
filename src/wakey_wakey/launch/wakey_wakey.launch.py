from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='wakey_wakey', executable='state_machine'),
        Node(package='wakey_wakey', executable='flee_behavior'),
        Node(
            package='wakey_wakey',
            executable='detection',
            parameters=[{
                'image_topic': '/oak/right/image_rect',
                'difference_threshold': 35,
                'min_area_fraction': 0.08,
            }],
        ),
        Node(package='wakey_wakey', executable='wakefulness'),
        Node(package='wakey_wakey', executable='game'),
        Node(package='wakey_wakey', executable='audio'),
    ])
