from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        Node(
            package='my_second_package',
            executable='my_subscriber',
            name='my_subscriber'
        ),

        Node(
            package='my_second_package',
            executable='my_publisher',
            name='my_publisher'
        ),

        Node(
            package='my_second_package',
            executable='dist_turtle_action_server',
            name='dist_turtle_action_server',
            parameters=[
                {'quatile_time': 0.75},
                {'almost_goal_time': 0.95}
            ]
        ),
    ])