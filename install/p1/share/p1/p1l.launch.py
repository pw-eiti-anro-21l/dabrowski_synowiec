from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            prefix='gnome-terminal --',
            package='p1',
            executable='controller',
            name='custom_controller',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'up': 'w'},
                {'down': 's'},
                {'left': 'a'},
                {'right': 'd'}
            ]
        ),
    ])
