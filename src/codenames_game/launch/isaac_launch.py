#!/usr/bin/env python3
"""
Enhanced Isaac Sim Bridge Node - Robot Integration for Codenames Robotics Game
Enhanced to support paragraph-style clues with detailed speech synthesis and gestures.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
import json
import logging
from typing import Dict, Any

class EnhancedIsaacBridgeNode(Node):
    def __init__(self):
        super().__init__('isaac_bridge_node')
        
        # Setup logging
        self.logger = self.get_logger()
        self.logger.info("🤖 Enhanced Isaac Sim Bridge Node starting...")
        
        # Isaac Sim connection status
        self.isaac_connected = False
        self.simulation_running = False
        self.paragraph_clues_enabled = True
        
        # Publishers (to Isaac Sim)
        self.robot_command_pub = self.create_publisher(String, '/isaac/robot_commands', 10)
        self.speech_command_pub = self.create_publisher(String, '/isaac/speech_commands', 10)
        self.scene_update_pub = self.create_publisher(String, '/isaac/scene_updates', 10)
        self.enhanced_display_pub = self.create_publisher(String, '/isaac/enhanced_display', 10)
        
        # Subscribers (from game)
        self.board_state_sub = self.create_subscription(
            String, '/game/board_state', self.handle_board_state, 10
        )
        self.clue_announcement_sub = self.create_subscription(
            String, '/game/clue_announcement', self.handle_clue_announcement, 10
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
        
        # Enhanced features
        self.current_clue_display = None
        self.clue_history = []
        
        self.logger.info("✅ Enhanced Isaac Sim Bridge Node initialized")
        self.logger.info("🔌 Waiting for Isaac Sim connection...")
        self.logger.info("📝 Paragraph clue support enabled")
    
    def handle_board_state(self, msg: String):
        """Handle board state updates and sync to Isaac Sim with enhanced clue data"""
        try:
            data = json.loads(msg.data)
            
            if not self.isaac_connected:
                return
            
            # Prepare enhanced scene update for Isaac Sim
            scene_update = {
                "type": "enhanced_board_update",
                "game_id": data.get('game_id', 'unknown'),
                "board_state": data.get('current_state', 'unknown'),
                "current_team": data.get('current_team', 'unknown'),
                "cards": self.format_cards_for_isaac(data.get('cards', [])),
                "red_remaining": data.get('red_remaining', 0),
                "blue_remaining": data.get('blue_remaining', 0),
                "current_clue_data": data.get('current_clue_data'),
                "clue_history_detailed": data.get('clue_history_detailed', []),
                "enhanced_features": ["paragraph_clues", "detailed_reasoning"]
            }
            
            # Publish to Isaac Sim
            isaac_msg = String()
            isaac_msg.data = json.dumps(scene_update)
            self.scene_update_pub.publish(isaac_msg)
            
            self.logger.info("📤 Sent enhanced board state update to Isaac Sim")
            
        except Exception as e:
            self.logger.error(f"❌ Error handling board state: {e}")
    
    def handle_clue_announcement(self, msg: String):
        """Handle clue announcements with enhanced paragraph support"""
        try:
            data = json.loads(msg.data)
            
            team = data.get('team', 'unknown')
            player_name = data.get('player_name', 'Unknown Player')
            clue_type = data.get('clue_type', 'single_word')
            
            if clue_type == 'paragraph':
                # Handle paragraph clue
                clue_paragraph = data.get('clue_paragraph', '')
                main_theme = data.get('main_theme', 'THEME')
                count = data.get('count', 1)
                reasoning = data.get('reasoning', '')
                warnings = data.get('warnings', '')
                
                self.logger.info(f"🎤 Processing paragraph clue from {player_name} ({team} team)")
                self.logger.info(f"   Theme: {main_theme}")
                
                # Generate enhanced speech for paragraph clue
                speech_text = self.generate_paragraph_speech(
                    clue_paragraph, main_theme, count, player_name
                )
                
                # Create enhanced visual display for Isaac Sim
                visual_display = {
                    "type": "paragraph_clue_display",
                    "team": team,
                    "player_name": player_name,
                    "main_theme": main_theme,
                    "clue_paragraph": clue_paragraph,
                    "count": count,
                    "reasoning": reasoning,
                    "warnings": warnings,
                    "formatted_text": self.format_clue_for_display(clue_paragraph, main_theme, count)
                }
                
                # Send visual display to Isaac Sim
                display_msg = String()
                display_msg.data = json.dumps(visual_display)
                self.enhanced_display_pub.publish(display_msg)
                
            else:
                # Handle traditional single-word clue
                clue = data.get('clue', 'UNKNOWN')
                count = data.get('count', 1)
                
                speech_text = f"The clue is {clue} for {count} words"
                
                visual_display = {
                    "type": "single_word_clue_display",
                    "team": team,
                    "player_name": player_name,
                    "clue": clue,
                    "count": count
                }
                
                display_msg = String()
                display_msg.data = json.dumps(visual_display)
                self.enhanced_display_pub.publish(display_msg)
            
            # Generate speech command for robot
            speech_command = {
                "type": "enhanced_speech",
                "robot": "spymaster_robot",
                "text": speech_text,
                "team": team,
                "player_name": player_name,
                "action": "announce_clue",
                "clue_type": clue_type,
                "speech_style": "explanatory" if clue_type == 'paragraph' else "simple"
            }
            
            # Send to Isaac Sim
            if self.isaac_connected:
                isaac_msg = String()
                isaac_msg.data = json.dumps(speech_command)
                self.speech_command_pub.publish(isaac_msg)
                
                self.logger.info(f"🎤 Sent enhanced speech command to Isaac Sim")
            
            # Send robot gesture/animation command
            self.send_enhanced_robot_gesture(team, "give_clue", {
                "clue_type": clue_type,
                "complexity": "high" if clue_type == 'paragraph' else "normal"
            })
            
            # Store for history
            self.current_clue_display = visual_display
            self.clue_history.append(visual_display)
            
        except Exception as e:
            self.logger.error(f"❌ Error handling clue announcement: {e}")
    
    def generate_paragraph_speech(self, clue_paragraph: str, main_theme: str, count: int, player_name: str) -> str:
        """Generate natural speech from paragraph clue"""
        # Break down the paragraph into digestible speech
        sentences = clue_paragraph.split('. ')
        
        # Create a natural speech flow
        speech_parts = [
            f"{player_name} says:",
            f"The theme is {main_theme}.",
        ]
        
        # Add the main clue content with natural pauses
        for sentence in sentences:
            if sentence.strip():
                speech_parts.append(sentence.strip() + ".")
        
        speech_parts.append(f"I'm looking for {count} words.")
        
        return " ".join(speech_parts)
    
    def format_clue_for_display(self, clue_paragraph: str, main_theme: str, count: int) -> str:
        """Format clue for visual display in Isaac Sim"""
        formatted = f"THEME: {main_theme}\n"
        formatted += f"COUNT: {count}\n\n"
        formatted += f"CLUE:\n{clue_paragraph}"
        return formatted
    
    def handle_guess_announcement(self, msg: String):
        """Handle guess announcements with enhanced context understanding"""
        try:
            data = json.loads(msg.data)
            
            word = data.get('word', 'UNKNOWN')
            team = data.get('team', 'unknown')
            player_name = data.get('player_name', 'Unknown Player')
            confidence = data.get('confidence', 0.5)
            clue_understood = data.get('clue_understood', {})
            
            # Generate contextual speech based on clue type
            if clue_understood.get('clue_type') == 'paragraph':
                main_theme = clue_understood.get('main_theme', 'the theme')
                speech_text = f"{player_name} says: Based on {main_theme}, I guess {word}"
            else:
                speech_text = f"{player_name} says: I guess {word}"
            
            speech_command = {
                "type": "enhanced_speech",
                "robot": "operative_robot",
                "text": speech_text,
                "team": team,
                "player_name": player_name,
                "action": "make_guess",
                "confidence": confidence,
                "word": word
            }
            
            # Send to Isaac Sim
            if self.isaac_connected:
                isaac_msg = String()
                isaac_msg.data = json.dumps(speech_command)
                self.speech_command_pub.publish(isaac_msg)
                
                self.logger.info(f"🎤 Sent enhanced guess speech to Isaac Sim: '{speech_text}'")
            
            # Send robot pointing gesture with enhanced context
            self.send_enhanced_robot_gesture(team, "point_to_card", {
                "word": word,
                "confidence": confidence,
                "context": clue_understood
            })
            
        except Exception as e:
            self.logger.error(f"❌ Error handling guess announcement: {e}")
    
    def send_enhanced_robot_gesture(self, team: str, gesture: str, params: Dict = None):
        """Send enhanced robot gesture/animation command to Isaac Sim"""
        if not self.isaac_connected:
            return
        
        robot_name = "spymaster_robot" if "spymaster" in gesture or gesture == "give_clue" else "operative_robot"
        
        command = {
            "type": "enhanced_gesture",
            "robot": robot_name,
            "team": team,
            "gesture": gesture,
            "parameters": params or {},
            "enhanced_features": ["context_aware", "confidence_based"]
        }
        
        isaac_msg = String()
        isaac_msg.data = json.dumps(command)
        self.robot_command_pub.publish(isaac_msg)
        
        complexity = params.get('complexity', 'normal') if params else 'normal'
        self.logger.info(f"🤖 Sent enhanced robot gesture '{gesture}' ({complexity}) to {robot_name}")
    
    def handle_isaac_status(self, msg: String):
        """Handle status updates from Isaac Sim"""
        try:
            data = json.loads(msg.data)
            
            status = data.get('status', 'unknown')
            
            if status == "connected":
                if not self.isaac_connected:
                    self.isaac_connected = True
                    self.logger.info("✅ Isaac Sim connected!")
                    self.logger.info("📝 Enhanced paragraph clue features active")
            elif status == "disconnected":
                if self.isaac_connected:
                    self.isaac_connected = False
                    self.logger.warning("⚠️ Isaac Sim disconnected!")
            
            self.simulation_running = data.get('simulation_running', False)
            
            # Check for enhanced feature support
            isaac_features = data.get('enhanced_features', [])
            if 'paragraph_clues' in isaac_features:
                self.logger.info("✅ Isaac Sim supports enhanced paragraph clues")
            
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
        """Format card data for Isaac Sim visualization with enhanced display"""
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
                },
                "enhanced_display": True,
                "clue_relevance": self.calculate_clue_relevance(card, self.current_clue_display)
            }
            isaac_cards.append(isaac_card)
        
        return isaac_cards
    
    def calculate_clue_relevance(self, card: dict, current_clue: dict) -> float:
        """Calculate how relevant a card might be to the current clue (for visual hints)"""
        if not current_clue or card.get('revealed', False):
            return 0.0
        
        # This is a simplified relevance calculation
        # In a real implementation, you might use word embeddings or similarity models
        word = card.get('word', '').upper()
        
        if current_clue.get('clue_type') == 'paragraph':
            theme = current_clue.get('main_theme', '').upper()
            clue_text = current_clue.get('clue_paragraph', '').upper()
            
            # Simple keyword matching (could be enhanced with NLP)
            relevance = 0.0
            if theme in word or word in theme:
                relevance += 0.8
            if word in clue_text:
                relevance += 0.6
            
            return min(1.0, relevance)
        
        return 0.0
    
    def check_isaac_connection(self):
        """Periodically check Isaac Sim connection"""
        if not self.isaac_connected:
            self.logger.info("🔍 Checking for Isaac Sim connection...")
            
            # Send enhanced ping to Isaac Sim
            ping_command = {
                "type": "enhanced_ping",
                "timestamp": self.get_clock().now().nanoseconds,
                "features_requested": ["paragraph_clues", "enhanced_display", "context_aware_gestures"]
            }
            
            isaac_msg = String()
            isaac_msg.data = json.dumps(ping_command)
            self.robot_command_pub.publish(isaac_msg)
    
    def publish_status(self):
        """Publish enhanced bridge status"""
        status = {
            "node": "enhanced_isaac_bridge",
            "isaac_connected": self.isaac_connected,
            "simulation_running": self.simulation_running,
            "robots_tracked": len([p for p in self.robot_poses.values() if p is not None]),
            "paragraph_clues_enabled": self.paragraph_clues_enabled,
            "current_clue_type": self.current_clue_display.get('type') if self.current_clue_display else None,
            "clues_processed": len(self.clue_history),
            "enhanced_features": ["paragraph_clues", "context_aware_gestures", "detailed_speech"],
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
    
    node = EnhancedIsaacBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Enhanced Isaac Bridge node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()