"""
Complete ROS2 Launch file for 4-Player Codenames Game with Enhanced Web Interface
Launches orchestrator, 4 player nodes, and web server with paragraph clue support
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
            description='AI model for Player1 (Alice)'
        ),
        
        DeclareLaunchArgument(
            'player2_model',
            default_value='gpt-4',
            description='AI model for Player2 (Bob)'
        ),
        
        DeclareLaunchArgument(
            'player3_model',
            default_value='claude-3-5-sonnet-20241022',
            description='AI model for Player3 (Charlie)'
        ),
        
        DeclareLaunchArgument(
            'player4_model',
            default_value='claude-3-5-sonnet-20241022',
            description='AI model for Player4 (Diana)'
        ),
        
        DeclareLaunchArgument(
            'clue_style',
            default_value='paragraph',
            description='Clue style: paragraph or single_word'
        ),
        
        DeclareLaunchArgument(
            'enable_isaac',
            default_value='false',
            description='Enable Isaac Sim bridge'
        ),
        
        DeclareLaunchArgument(
            'enable_web',
            default_value='true',
            description='Enable web interface'
        ),
        
        DeclareLaunchArgument(
            'web_port',
            default_value='8080',
            description='Web server port'
        ),
        
        DeclareLaunchArgument(
            'ros_domain_id',
            default_value='42',
            description='ROS Domain ID'
        ),

        # Enhanced Orchestrator Node (Game Master & Role Manager)
        Node(
            package='codenames_game',
            executable='orchestrator_node',
            name='orchestrator_node',
            output='screen',
            parameters=[{
                'ros_domain_id': LaunchConfiguration('ros_domain_id'),
                'enhanced_features': True,
                'paragraph_clues_enabled': True
            }],
            environment={
                'ROS_DOMAIN_ID': LaunchConfiguration('ros_domain_id'),
                'NODE_NAME': 'enhanced_orchestrator',
                'ENHANCED_MODE': 'true'
            }
        ),

        # Enhanced Player 1 (Alice)
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
                        'ros_domain_id': LaunchConfiguration('ros_domain_id'),
                        'clue_style': LaunchConfiguration('clue_style')
                    }],
                    environment={
                        'ROS_DOMAIN_ID': LaunchConfiguration('ros_domain_id'),
                        'PLAYER_ID': 'player_1',
                        'PLAYER_NAME': 'Alice',
                        'NODE_NAME': 'player1',
                        'PLAYER_MODEL': LaunchConfiguration('player1_model'),
                        'CLUE_STYLE': LaunchConfiguration('clue_style')
                    }
                )
            ]
        ),

        # Enhanced Player 2 (Bob)
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
                        'ros_domain_id': LaunchConfiguration('ros_domain_id'),
                        'clue_style': LaunchConfiguration('clue_style')
                    }],
                    environment={
                        'ROS_DOMAIN_ID': LaunchConfiguration('ros_domain_id'),
                        'PLAYER_ID': 'player_2',
                        'PLAYER_NAME': 'Bob',
                        'NODE_NAME': 'player2',
                        'PLAYER_MODEL': LaunchConfiguration('player2_model'),
                        'CLUE_STYLE': LaunchConfiguration('clue_style')
                    }
                )
            ]
        ),

        # Enhanced Player 3 (Charlie)
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
                        'ros_domain_id': LaunchConfiguration('ros_domain_id'),
                        'clue_style': LaunchConfiguration('clue_style')
                    }],
                    environment={
                        'ROS_DOMAIN_ID': LaunchConfiguration('ros_domain_id'),
                        'PLAYER_ID': 'player_3',
                        'PLAYER_NAME': 'Charlie',
                        'NODE_NAME': 'player3',
                        'PLAYER_MODEL': LaunchConfiguration('player3_model'),
                        'CLUE_STYLE': LaunchConfiguration('clue_style')
                    }
                )
            ]
        ),

        # Enhanced Player 4 (Diana)
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
                        'ros_domain_id': LaunchConfiguration('ros_domain_id'),
                        'clue_style': LaunchConfiguration('clue_style')
                    }],
                    environment={
                        'ROS_DOMAIN_ID': LaunchConfiguration('ros_domain_id'),
                        'PLAYER_ID': 'player_4',
                        'PLAYER_NAME': 'Diana',
                        'NODE_NAME': 'player4',
                        'PLAYER_MODEL': LaunchConfiguration('player4_model'),
                        'CLUE_STYLE': LaunchConfiguration('clue_style')
                    }
                )
            ]
        ),

        # Web Server (Enhanced Interface)
        TimerAction(
            period=4.0,  # Start after all players
            actions=[
                Node(
                    package='codenames_game',
                    executable='web_server',
                    name='web_server_node',
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('enable_web')),
                    parameters=[{
                        'port': LaunchConfiguration('web_port'),
                        'ros_domain_id': LaunchConfiguration('ros_domain_id')
                    }],
                    environment={
                        'ROS_DOMAIN_ID': LaunchConfiguration('ros_domain_id'),
                        'NODE_NAME': 'web_server',
                        'WEB_PORT': LaunchConfiguration('web_port')
                    }
                )
            ]
        ),

        # Enhanced Isaac Bridge Node (conditional)
        Node(
            package='codenames_game',
            executable='isaac_bridge_node',
            name='isaac_bridge_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_isaac')),
            parameters=[{
                'ros_domain_id': LaunchConfiguration('ros_domain_id'),
                'paragraph_clues_enabled': True
            }],
            environment={
                'ROS_DOMAIN_ID': LaunchConfiguration('ros_domain_id'),
                'NODE_NAME': 'enhanced_isaac_bridge',
                'ENHANCED_MODE': 'true'
            }
        ),

        # Information Display
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['echo', '🌐 Web Interface Available at: http://localhost:8080'],
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('enable_web'))
                )
            ]
        ),
        
        TimerAction(
            period=5.5,
            actions=[
                ExecuteProcess(
                    cmd=['echo', '🔧 Admin Panel: http://localhost:8080/admin'],
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('enable_web'))
                )
            ]
        ),
        
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=['echo', '👥 Player Details: http://localhost:8080/players'],
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('enable_web'))
                )
            ]
        ),
        
        # Optional: Monitor game topics
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=['echo', '📡 Monitor topics with: ros2 topic list | grep /game'],
                    output='screen',
                    condition=IfCondition('false')  # Set to 'true' to enable
                )
            ]
        )
    ])