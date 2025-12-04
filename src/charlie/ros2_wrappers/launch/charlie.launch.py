"""
    Launch file for Charlie Robot
    Starts all necessary nodes for the emergency rescue mission
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with all Charlie nodes"""
    
    return LaunchDescription([
        #Behavior Tree Node (Main AI Controller)
        Node(
            package='charlie',
            executable='behavior_tree_node',
            name='behavior_tree',
            output='screen',
            parameters=[{
                'battery_threshold': 15.0,
            }]
        ),
        
        #Perception Node (Vision & Sensors)
        Node(
            package='charlie',
            executable='perception_node',
            name='perception',
            output='screen',
        ),
        
        #Navigation Node (Path Planning)
        Node(
            package='charlie',
            executable='navigation_node',
            name='navigation',
            output='screen',
        ),
    ])
