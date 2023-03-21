from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='Patrick_turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='homework',
            executable='problem3',
            name='configurations'
        ),
        Node(
            package='homework',
            executable='problem5',
            name='broadcaster1',
            parameters=[
                {'boatname': 'usv',
                 'obstaclename': 'obstacle',
                 'observername': 'observer'}
            ]),
    ])