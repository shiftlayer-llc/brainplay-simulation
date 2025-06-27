"""
ROS2 Launch file for 4-Player Codenames Game
Launches orchestrator and 4 player nodes with different configurations
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'player1_model',
            default_value='gpt-4',
            description='AI model for Player1'
        ),
        
        DeclareLaunchArgument(
            'player2_model',
            default_value='gpt-4',
            description='AI model for Player2'
        ),
        
        DeclareLaunchArgument(
            'player3_model',
            default_value='claude-3-5-sonnet-20241022',
            description='AI model for Player3'
        ),
        
        DeclareLaunchArgument(
            'player4_model',
            default_value='claude-3-5-sonnet-20241022',
            description='AI model for Player4'
        ),
        
        DeclareLaunchArgument(
            'enable_isaac',
            default_value='false',
            description='Enable Isaac Sim bridge'
        ),
        
        DeclareLaunchArgument(
            'ros_domain_id',
            default_value='42',
            description='ROS Domain ID'
        ),

        # Orchestrator Node (Game Master & Role Manager)
        Node(
            package='codenames_game',
            executable='orchestrator_node',
            name='orchestrator_node',
            output='screen',
            parameters=[{
                'ros_domain_id': LaunchConfiguration('ros_domain_id')
            }],
            environment={
                'ROS_DOMAIN_ID': LaunchConfiguration('ros_domain_id'),
                'NODE_NAME': 'orchestrator'
            }
        ),

        # Player 1
        TimerAction(
            period=2.0,  # Start after orchestrator
            actions=[
                Node(
                    package='codenames_game',
                    executable='player_node',
                    name='player1_node',
                    output='screen',
                    parameters=[{
                        'player_model': LaunchConfiguration('player1_model'),
                        'ros_domain_id': LaunchConfiguration('ros_domain_id')
                    }],
                    environment={
                        'ROS_DOMAIN_ID': LaunchConfiguration('ros_domain_id'),
                        'PLAYER_ID': 'player_1',
                        'PLAYER_NAME': 'Player1',
                        'NODE_NAME': 'player1',
                        'PLAYER_MODEL': LaunchConfiguration('player1_model')
                    }
                )
            ]
        ),

        # Player 2
        TimerAction(
            period=2.5,
            actions=[
                Node(
                    package='codenames_game',
                    executable='player_node',
                    name='player2_node',
                    output='screen',
                    parameters=[{
                        'player_model': LaunchConfiguration('player2_model'),
                        'ros_domain_id': LaunchConfiguration('ros_domain_id')
                    }],
                    environment={
                        'ROS_DOMAIN_ID': LaunchConfiguration('ros_domain_id'),
                        'PLAYER_ID': 'player_2',
                        'PLAYER_NAME': 'Player2',
                        'NODE_NAME': 'player2',
                        'PLAYER_MODEL': LaunchConfiguration('player2_model')
                    }
                )
            ]
        ),

        # Player 3
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='codenames_game',
                    executable='player_node',
                    name='player3_node',
                    output='screen',
                    parameters=[{
                        'player_model': LaunchConfiguration('player3_model'),
                        'ros_domain_id': LaunchConfiguration('ros_domain_id')
                    }],
                    environment={
                        'ROS_DOMAIN_ID': LaunchConfiguration('ros_domain_id'),
                        'PLAYER_ID': 'player_3',
                        'PLAYER_NAME': 'Player3',
                        'NODE_NAME': 'player3',
                        'PLAYER_MODEL': LaunchConfiguration('player3_model')
                    }
                )
            ]
        ),

        # Player 4
        TimerAction(
            period=3.5,
            actions=[
                Node(
                    package='codenames_game',
                    executable='player_node',
                    name='player4_node',
                    output='screen',
                    parameters=[{
                        'player_model': LaunchConfiguration('player4_model'),
                        'ros_domain_id': LaunchConfiguration('ros_domain_id')
                    }],
                    environment={
                        'ROS_DOMAIN_ID': LaunchConfiguration('ros_domain_id'),
                        'PLAYER_ID': 'player_4',
                        'PLAYER_NAME': 'Player4',
                        'NODE_NAME': 'player4',
                        'PLAYER_MODEL': LaunchConfiguration('player4_model')
                    }
                )
            ]
        ),

        # Isaac Bridge Node (conditional)
        Node(
            package='codenames_game',
            executable='isaac_bridge_node',
            name='isaac_bridge_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_isaac')),
            parameters=[{
                'ros_domain_id': LaunchConfiguration('ros_domain_id')
            }],
            environment={
                'ROS_DOMAIN_ID': LaunchConfiguration('ros_domain_id'),
                'NODE_NAME': 'isaac_bridge'
            }
        ),

        # Optional: Display node graph
        # ExecuteProcess(
        #     cmd=['rqt_graph'],
        #     output='screen',
        #     condition=IfCondition('false')  # Set to 'true' to enable
        # ),
        
        # Optional: Monitor topics
        # ExecuteProcess(
        #     cmd=['ros2', 'topic', 'echo', '/game/status'],
        #     output='screen',
        #     condition=IfCondition('false')  # Set to 'true' to enable
        # )
    ])