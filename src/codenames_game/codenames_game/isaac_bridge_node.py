#!/usr/bin/env python3
"""
Isaac Sim Bridge Node - Robot Integration for Codenames Robotics Game
Connects ROS2 game to Isaac Sim 4.5.0 for robot visualization and control.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
import json
import logging
from typing import Dict, Any

class IsaacBridgeNode(Node):
    def __init__(self):
        super().__init__('isaac_bridge_node')
        
        # Setup logging
        self.logger = self.get_logger()
        self.logger.info("🤖 Isaac Sim Bridge Node starting...")
        
        # Isaac Sim connection status
        self.isaac_connected = False
        self.simulation_running = False
        
        # Publishers (to Isaac Sim)
        self.robot_command_pub = self.create_publisher(String, '/isaac/robot_commands', 10)
        self.speech_command_pub = self.create_publisher(String, '/isaac/speech_commands', 10)
        self.scene_update_pub = self.create_publisher(String, '/isaac/scene_updates', 10)
        
        # Subscribers (from game)
        self.board_state_sub = self.create_subscription(
            String, '/game/board_state', self.handle_board_state, 10
        )
        self.clue_sub = self.create_subscription(
            String, '/game/clue_response', self.handle_clue_announcement, 10
        )
        self.guess_sub = self.create_subscription(
            String, '/game/guess_response', self.handle_guess_announcement, 10
        )
        
        # Subscribers (from Isaac Sim)
        self.isaac_status_sub = self.create_subscription(
            String, '/isaac/status', self.handle_isaac_status, 10
        )
        self.robot_pose_sub = self.create_subscription(
            PoseStamped, '/isaac/robot_pose', self.handle_robot_pose, 10
        )
        
        # Status timer
        self.status_timer = self.create_timer(5.0, self.publish_status)
        self.connection_timer = self.create_timer(10.0, self.check_isaac_connection)
        
        # Robot states
        self.robot_poses = {
            "spymaster_robot": None,
            "operative_robot": None
        }
        
        self.logger.info("✅ Isaac Sim Bridge Node initialized")
        self.logger.info("🔌 Waiting for Isaac Sim connection...")
    
    def handle_board_state(self, msg: String):
        """Handle board state updates and sync to Isaac Sim"""
        try:
            data = json.loads(msg.data)
            
            if not self.isaac_connected:
                return
            
            # Prepare scene update for Isaac Sim
            scene_update = {
                "type": "board_update",
                "game_id": data.get('game_id', 'unknown'),
                "board_state": data.get('current_state', 'unknown'),
                "current_team": data.get('current_team', 'unknown'),
                "cards": self.format_cards_for_isaac(data.get('cards', [])),
                "red_remaining": data.get('red_remaining', 0),
                "blue_remaining": data.get('blue_remaining', 0)
            }
            
            # Publish to Isaac Sim
            isaac_msg = String()
            isaac_msg.data = json.dumps(scene_update)
            self.scene_update_pub.publish(isaac_msg)
            
            self.logger.info("📤 Sent board state update to Isaac Sim")
            
        except Exception as e:
            self.logger.error(f"❌ Error handling board state: {e}")
    
    def handle_clue_announcement(self, msg: String):
        """Handle clue announcements and trigger robot speech"""
        try:
            data = json.loads(msg.data)
            
            clue = data.get('clue', 'UNKNOWN')
            count = data.get('count', 1)
            team = data.get('team', 'unknown')
            
            # Generate speech command for robot
            speech_text = f"The clue is {clue} for {count} words"
            
            speech_command = {
                "type": "speech",
                "robot": "spymaster_robot",
                "text": speech_text,
                "team": team,
                "action": "announce_clue"
            }
            
            # Send to Isaac Sim
            if self.isaac_connected:
                isaac_msg = String()
                isaac_msg.data = json.dumps(speech_command)
                self.speech_command_pub.publish(isaac_msg)
                
                self.logger.info(f"🎤 Sent speech command to Isaac Sim: '{speech_text}'")
            
            # Also send robot gesture/animation command
            self.send_robot_gesture(team, "give_clue")
            
        except Exception as e:
            self.logger.error(f"❌ Error handling clue announcement: {e}")
    
    def handle_guess_announcement(self, msg: String):
        """Handle guess announcements and trigger robot actions"""
        try:
            data = json.loads(msg.data)
            
            word = data.get('word', 'UNKNOWN')
            team = data.get('team', 'unknown')
            
            # Generate speech command for operative robot
            speech_text = f"I guess {word}"
            
            speech_command = {
                "type": "speech",
                "robot": "operative_robot",
                "text": speech_text,
                "team": team,
                "action": "make_guess"
            }
            
            # Send to Isaac Sim
            if self.isaac_connected:
                isaac_msg = String()
                isaac_msg.data = json.dumps(speech_command)
                self.speech_command_pub.publish(isaac_msg)
                
                self.logger.info(f"🎤 Sent guess speech to Isaac Sim: '{speech_text}'")
            
            # Send robot pointing gesture
            self.send_robot_gesture(team, "point_to_card", {"word": word})
            
        except Exception as e:
            self.logger.error(f"❌ Error handling guess announcement: {e}")
    
    def send_robot_gesture(self, team: str, gesture: str, params: Dict = None):
        """Send robot gesture/animation command to Isaac Sim"""
        if not self.isaac_connected:
            return
        
        robot_name = "spymaster_robot" if "spymaster" in gesture else "operative_robot"
        
        command = {
            "type": "gesture",
            "robot": robot_name,
            "team": team,
            "gesture": gesture,
            "parameters": params or {}
        }
        
        isaac_msg = String()
        isaac_msg.data = json.dumps(command)
        self.robot_command_pub.publish(isaac_msg)
        
        self.logger.info(f"🤖 Sent robot gesture '{gesture}' to {robot_name}")
    
    def handle_isaac_status(self, msg: String):
        """Handle status updates from Isaac Sim"""
        try:
            data = json.loads(msg.data)
            
            status = data.get('status', 'unknown')
            
            if status == "connected":
                if not self.isaac_connected:
                    self.isaac_connected = True
                    self.logger.info("✅ Isaac Sim connected!")
            elif status == "disconnected":
                if self.isaac_connected:
                    self.isaac_connected = False
                    self.logger.warning("⚠️ Isaac Sim disconnected!")
            
            self.simulation_running = data.get('simulation_running', False)
            
        except Exception as e:
            self.logger.error(f"❌ Error handling Isaac status: {e}")
    
    def handle_robot_pose(self, msg: PoseStamped):
        """Handle robot pose updates from Isaac Sim"""
        try:
            # Extract robot name from frame_id or topic
            frame_id = msg.header.frame_id
            
            if "spymaster" in frame_id:
                self.robot_poses["spymaster_robot"] = msg.pose
            elif "operative" in frame_id:
                self.robot_poses["operative_robot"] = msg.pose
            
            # Log pose updates periodically
            if hasattr(self, '_last_pose_log') and (self.get_clock().now().nanoseconds - self._last_pose_log) < 5e9:
                return
            
            self._last_pose_log = self.get_clock().now().nanoseconds
            self.logger.info(f"📍 Updated pose for {frame_id}")
            
        except Exception as e:
            self.logger.error(f"❌ Error handling robot pose: {e}")
    
    def format_cards_for_isaac(self, cards: list) -> list:
        """Format card data for Isaac Sim visualization"""
        isaac_cards = []
        
        for i, card in enumerate(cards):
            isaac_card = {
                "index": i,
                "word": card.get('word', 'UNKNOWN'),
                "revealed": card.get('revealed', False),
                "type": card.get('type', 'unknown'),
                "position": {
                    "x": (i % 5) * 0.2,  # 5x5 grid with 0.2m spacing
                    "y": (i // 5) * 0.2,
                    "z": 0.01
                }
            }
            isaac_cards.append(isaac_card)
        
        return isaac_cards
    
    def check_isaac_connection(self):
        """Periodically check Isaac Sim connection"""
        if not self.isaac_connected:
            self.logger.info("🔍 Checking for Isaac Sim connection...")
            
            # Send ping to Isaac Sim
            ping_command = {
                "type": "ping",
                "timestamp": self.get_clock().now().nanoseconds
            }
            
            isaac_msg = String()
            isaac_msg.data = json.dumps(ping_command)
            self.robot_command_pub.publish(isaac_msg)
    
    def publish_status(self):
        """Publish bridge status"""
        status = {
            "node": "isaac_bridge",
            "isaac_connected": self.isaac_connected,
            "simulation_running": self.simulation_running,
            "robots_tracked": len([p for p in self.robot_poses.values() if p is not None]),
            "status": "active" if self.isaac_connected else "waiting_for_isaac"
        }
        
        # Publish on ROS2 topic for monitoring
        msg = String()
        msg.data = json.dumps(status)
        
        # Create a publisher for bridge status if not exists
        if not hasattr(self, 'bridge_status_pub'):
            self.bridge_status_pub = self.create_publisher(String, '/isaac/bridge_status', 10)
        
        self.bridge_status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    # Setup logging
    logging.basicConfig(level=logging.INFO)
    
    node = IsaacBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Isaac Bridge node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()