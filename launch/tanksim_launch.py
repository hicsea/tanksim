from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tanksim',
            executable='tanksim_node',
        ),        
        Node(
            package='tank_controller',
            executable='obstacle_initializer_node',
        ),        
        Node(
            package='tank_controller',
            executable='route_initializer_node',
        ),
        Node(
            package='tank_controller',
            executable='tank_controller_node',
        ),
        Node(
            package='interceptor',
            executable='interceptor_node',
        ),

    ])