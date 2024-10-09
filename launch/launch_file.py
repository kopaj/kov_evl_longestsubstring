import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node 1: String Generator Node
        Node(
            package='kov_evl_longestsubstring',
            executable='string_generator_node',
            name='string_generator',
            output='screen',
            parameters=[]
        ),
        # Node 2: String Processor Node
        Node(
            package='kov_evl_longestsubstring',
            executable='substring_finder_node',
            name='substring_finder',
            output='screen',
            parameters=[]
        )
    ])
