from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            # namespace='turtlesim1',
            executable='turtlesim_node',
            # name='sim'
        ),
        Node(
            package='route_initializer',
            executable='route_initializer_node',
            # name='route_initializer'
        ),
        Node(
            package='tank_controller',
            # namespace='turtle_demo_controller',
            executable='tank_controller_node',
            # name='controller'
        ),
        Node(
            package='interceptor',
            executable='interceptor_node',
            # name='interceptor'
        ),

    ])