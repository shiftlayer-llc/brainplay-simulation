"""
ROS2 Launch file for Codenames Game with Isaac Sim Integration
Launches all game nodes plus Isaac Sim bridge for robot simulation
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'spymaster_model',
            default_value='gpt-4',
            description='Model to use for spymaster'
        ),
        
        DeclareLaunchArgument(
            'finder_model',
            default_value='gpt-4', 
            description='Model to use for finder'
        ),
        
        DeclareLaunchArgument(
            'isaac_sim_host',
            default_value='localhost',
            description='Isaac Sim host address'
        ),
        
        DeclareLaunchArgument(
            'isaac_sim_port',
            default_value='8211',
            description='Isaac Sim port'
        ),

        # Start game nodes first
        Node(
            package='codenames_game',
            executable='orchestrator_node',
            name='orchestrator_node',
            output='screen',
            environment={
                'ROS_DOMAIN_ID': '42',
                'NODE_NAME': 'orchestrator'
            }
        ),

        Node(
            package='codenames_game', 
            executable='spymaster_node',
            name='spymaster_node',
            output='screen',
            environment={
                'ROS_DOMAIN_ID': '42',
                'NODE_NAME': 'spymaster',
                'SPYMASTER_MODEL': LaunchConfiguration('spymaster_model')
            }
        ),

        Node(
            package='codenames_game',
            executable='finder_node', 
            name='finder_node',
            output='screen',
            environment={
                'ROS_DOMAIN_ID': '42',
                'NODE_NAME': 'finder',
                'FINDER_MODEL': LaunchConfiguration('finder_model')
            }
        ),

        # Start Isaac Bridge after a delay to ensure other nodes are ready
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='codenames_game',
                    executable='isaac_bridge_node',
                    name='isaac_bridge_node',
                    output='screen',
                    parameters=[{
                        'isaac_sim_host': LaunchConfiguration('isaac_sim_host'),
                        'isaac_sim_port': LaunchConfiguration('isaac_sim_port')
                    }],
                    environment={
                        'ROS_DOMAIN_ID': '42',
                        'NODE_NAME': 'isaac_bridge',
                        'ISAAC_SIM_HOST': LaunchConfiguration('isaac_sim_host'),
                        'ISAAC_SIM_PORT': LaunchConfiguration('isaac_sim_port')
                    }
                )
            ]
        ),

        # Optional: Start Isaac Sim if not already running
        # Note: This requires Isaac Sim to be installed and configured
        # ExecuteProcess(
        #     cmd=[
        #         'bash', '-c',
        #         'cd $ISAAC_SIM_PATH && ./isaac-sim.sh --/app/window/dpiScaleOverride=1.0 --/app/file/ignoreUnsavedOnExit=true'
        #     ],
        #     output='screen',
        #     condition=IfCondition('false')  # Set to 'true' to auto-start Isaac Sim
        # ),

        # Start RViz for visualization
        ExecuteProcess(
            cmd=['rviz2', '-d', 'config/codenames_rviz.rviz'],
            output='screen',
            condition=IfCondition('false')  # Set to 'true' to start RViz
        )
    ])