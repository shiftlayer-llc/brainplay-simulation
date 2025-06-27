#!/usr/bin/env python3
"""
Player Node - Universal Player for Codenames Robotics Game
Can act as either Spymaster or Operative based on role assignment from orchestrator.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import logging
import time
from typing import List, Optional

from .llm_client import llm_client

class PlayerNode(Node):
    def __init__(self):
        super().__init__('player_node')
        
        # Setup logging
        self.logger = self.get_logger()
        self.logger.info("🎭 Player Node starting...")
        
        # Player configuration
        self.player_id = os.getenv('PLAYER_ID', 'player_1')
        self.player_name = os.getenv('PLAYER_NAME', f'Player {self.player_id[-1]}')
        self.model = os.getenv('PLAYER_MODEL', 'gpt-4')
        
        # Role assignment (will be set by orchestrator)
        self.assigned_role = None  # 'red_spymaster', 'red_operative', 'blue_spymaster', 'blue_operative'
        self.assigned_team = None  # 'red', 'blue'
        self.role_type = None      # 'spymaster', 'operative'
        
        # Publishers
        self.clue_response_pub = self.create_publisher(String, '/game/clue_response', 10)
        self.guess_response_pub = self.create_publisher(String, '/game/guess_response', 10)
        self.player_status_pub = self.create_publisher(String, f'/game/{self.player_id}_status', 10)
        self.player_ready_pub = self.create_publisher(String, '/game/player_ready', 10)
        
        # Subscribers
        self.role_assignment_sub = self.create_subscription(
            String, '/game/role_assignments', self.handle_role_assignment, 10
        )
        self.clue_request_sub = self.create_subscription(
            String, '/game/clue_request', self.handle_clue_request, 10
        )
        self.guess_request_sub = self.create_subscription(
            String, '/game/guess_request', self.handle_guess_request, 10
        )
        
        # Status timer
        self.status_timer = self.create_timer(10.0, self.publish_status)
        
        # State tracking
        self.processing_action = False
        self.actions_completed = 0
        self.current_game_id = None
        
        # Personality traits (could be randomized or configured)
        self.personality = {
            'risk_tolerance': 0.7,  # 0.0 = very conservative, 1.0 = very risky
            'creativity': 0.8,      # 0.0 = basic clues, 1.0 = creative clues
            'confidence': 0.6       # affects guess patterns
        }
        
        self.logger.info(f"✅ Player Node '{self.player_name}' ({self.player_id}) initialized")
        
        # Announce readiness
        self.announce_ready()
    
    def announce_ready(self):
        """Announce player is ready to receive role assignments"""
        ready_msg = {
            "player_id": self.player_id,
            "player_name": self.player_name,
            "model": self.model,
            "personality": self.personality,
            "status": "ready_for_assignment"
        }
        
        msg = String()
        msg.data = json.dumps(ready_msg)
        self.player_ready_pub.publish(msg)
        
        self.logger.info(f"📢 Player {self.player_name} ready for role assignment")
    
    def handle_role_assignment(self, msg: String):
        """Handle role assignment from orchestrator"""
        try:
            data = json.loads(msg.data)
            
            # Check if this assignment is for this player
            if data.get('player_id') != self.player_id:
                return
            
            self.assigned_role = data.get('role')  # e.g., 'red_spymaster'
            self.assigned_team = data.get('team')  # 'red' or 'blue'
            self.role_type = data.get('role_type')  # 'spymaster' or 'operative'
            self.current_game_id = data.get('game_id')
            
            self.logger.info(f"🎭 Role assigned: {self.assigned_role}")
            self.logger.info(f"👥 Team: {self.assigned_team}")
            self.logger.info(f"🎯 Role type: {self.role_type}")
            
            # Acknowledge role assignment
            ack_msg = {
                "player_id": self.player_id,
                "game_id": self.current_game_id,
                "role_accepted": self.assigned_role,
                "status": "role_accepted"
            }
            
            msg = String()
            msg.data = json.dumps(ack_msg)
            self.player_ready_pub.publish(msg)
            
        except Exception as e:
            self.logger.error(f"❌ Error handling role assignment: {e}")
    
    def handle_clue_request(self, msg: String):
        """Handle clue request (only if assigned as spymaster)"""
        if self.role_type != 'spymaster' or self.processing_action:
            return
        
        try:
            data = json.loads(msg.data)
            
            # Check if this request is for this player's team
            if data.get('team') != self.assigned_team:
                return
            
            # Check if this is for the correct game
            if data.get('game_id') != self.current_game_id:
                return
            
            self.processing_action = True
            
            target_words = data.get('target_words', [])
            avoid_words = data.get('avoid_words', [])
            turn = data.get('turn', 0)
            
            self.logger.info(f"🎯 {self.player_name} generating clue for {self.assigned_team} team")
            self.logger.info(f"   Target words: {target_words}")
            self.logger.info(f"   Avoid words: {len(avoid_words)} words")
            
            # Generate clue using personality-influenced strategy
            clue, count = self.generate_clue_with_personality(target_words, avoid_words)
            
            # Prepare response
            response = {
                "game_id": self.current_game_id,
                "player_id": self.player_id,
                "player_name": self.player_name,
                "team": self.assigned_team,
                "clue": clue,
                "count": count,
                "turn": turn,
                "model_used": self.model,
                "personality_influence": self.personality
            }
            
            # Publish clue response
            response_msg = String()
            response_msg.data = json.dumps(response)
            self.clue_response_pub.publish(response_msg)
            
            self.actions_completed += 1
            self.logger.info(f"📤 {self.player_name} sent clue: '{clue}' (count: {count})")
            
        except Exception as e:
            self.logger.error(f"❌ Error handling clue request: {e}")
        finally:
            self.processing_action = False
    
    def handle_guess_request(self, msg: String):
        """Handle guess request (only if assigned as operative)"""
        if self.role_type != 'operative' or self.processing_action:
            return
        
        try:
            data = json.loads(msg.data)
            
            # Check if this request is for this player's team
            if data.get('team') != self.assigned_team:
                return
            
            # Check if this is for the correct game
            if data.get('game_id') != self.current_game_id:
                return
            
            self.processing_action = True
            
            clue = data.get('clue', '')
            clue_count = data.get('clue_count', 1)
            available_words = data.get('available_words', [])
            guesses_made = data.get('guesses_made', 0)
            turn = data.get('turn', 0)
            
            self.logger.info(f"🔍 {self.player_name} making guess for {self.assigned_team} team")
            self.logger.info(f"   Clue: '{clue}' (count: {clue_count})")
            self.logger.info(f"   Guesses made: {guesses_made}")
            
            # Generate guess using personality
            guess = self.generate_guess_with_personality(
                clue, clue_count, available_words, guesses_made
            )
            
            # Simulate thinking time based on personality
            thinking_time = self.calculate_thinking_time(guesses_made)
            self.logger.info(f"🤔 {self.player_name} thinking for {thinking_time:.1f}s...")
            time.sleep(thinking_time)
            
            # Prepare response
            response = {
                "game_id": self.current_game_id,
                "player_id": self.player_id,
                "player_name": self.player_name,
                "team": self.assigned_team,
                "word": guess,
                "clue": clue,
                "guesses_made": guesses_made + 1,
                "turn": turn,
                "model_used": self.model,
                "confidence": self.calculate_guess_confidence(guess, guesses_made)
            }
            
            # Publish guess response
            response_msg = String()
            response_msg.data = json.dumps(response)
            self.guess_response_pub.publish(response_msg)
            
            self.actions_completed += 1
            self.logger.info(f"📤 {self.player_name} sent guess: '{guess}'")
            
        except Exception as e:
            self.logger.error(f"❌ Error handling guess request: {e}")
        finally:
            self.processing_action = False
    
    def generate_clue_with_personality(self, target_words: List[str], avoid_words: List[str]) -> tuple:
        """Generate clue influenced by player personality"""
        try:
            # Adjust prompt based on personality
            creativity_prompt = ""
            if self.personality['creativity'] > 0.7:
                creativity_prompt = "Be creative and think outside the box with your clues."
            elif self.personality['creativity'] < 0.3:
                creativity_prompt = "Use straightforward, obvious connections."
            
            risk_prompt = ""
            if self.personality['risk_tolerance'] > 0.7:
                risk_prompt = "You can take risks and go for ambitious multi-word clues."
            elif self.personality['risk_tolerance'] < 0.3:
                risk_prompt = "Play it safe and avoid risky clues that might confuse."
            
            # Use LLM client with personality-influenced prompt
            clue, count = llm_client.get_spymaster_clue(
                target_words=target_words,
                avoid_words=avoid_words,
                model=self.model
            )
            
            # Apply personality-based adjustments
            if self.personality['risk_tolerance'] < 0.4 and count > 2:
                count = min(count, 2)  # Conservative players limit clue count
                self.logger.info(f"🛡️ {self.player_name} being conservative, limited count to {count}")
            
            return clue, count
            
        except Exception as e:
            self.logger.error(f"❌ Error generating clue: {e}")
            return "FALLBACK", 1
    
    def generate_guess_with_personality(self, clue: str, clue_count: int, 
                                      available_words: List[str], guesses_made: int) -> str:
        """Generate guess influenced by player personality"""
        try:
            # Get potential guesses from LLM
            guesses = llm_client.get_operative_guesses(
                clue=clue,
                count=clue_count,
                available_words=available_words,
                model=self.model
            )
            
            # Apply personality-based selection
            if guesses_made == 0:
                # First guess - all personalities make it
                selected_guess = guesses[0] if guesses else available_words[0]
            else:
                # Subsequent guesses - personality matters
                confidence_threshold = 1.0 - self.personality['confidence']
                
                if self.personality['risk_tolerance'] < 0.3:
                    # Conservative: stop after 1-2 guesses unless very confident
                    if guesses_made >= 1:
                        self.logger.info(f"🛡️ {self.player_name} being conservative, might pass")
                        # Could implement pass functionality here
                
                selected_guess = guesses[min(guesses_made, len(guesses)-1)] if guesses else available_words[0]
            
            # Validate guess
            if selected_guess.upper() in [w.upper() for w in available_words]:
                return selected_guess.upper()
            else:
                return available_words[0].upper() if available_words else "UNKNOWN"
            
        except Exception as e:
            self.logger.error(f"❌ Error generating guess: {e}")
            return available_words[0].upper() if available_words else "UNKNOWN"
    
    def calculate_thinking_time(self, guesses_made: int) -> float:
        """Calculate thinking time based on personality and situation"""
        base_time = 2.0  # Base thinking time
        
        # Confidence affects thinking time
        confidence_factor = 2.0 - self.personality['confidence']  # Less confident = more time
        
        # More guesses = more thinking
        guess_factor = 1.0 + (guesses_made * 0.5)
        
        thinking_time = base_time * confidence_factor * guess_factor
        
        # Cap between 1-8 seconds
        return max(1.0, min(8.0, thinking_time))
    
    def calculate_guess_confidence(self, guess: str, guesses_made: int) -> float:
        """Calculate confidence for a guess"""
        base_confidence = self.personality['confidence']
        
        # First guess is most confident
        guess_penalty = guesses_made * 0.15
        
        confidence = max(0.1, min(1.0, base_confidence - guess_penalty))
        
        return confidence
    
    def publish_status(self):
        """Publish periodic status updates"""
        status = {
            "player_id": self.player_id,
            "player_name": self.player_name,
            "model": self.model,
            "assigned_role": self.assigned_role,
            "assigned_team": self.assigned_team,
            "role_type": self.role_type,
            "processing_action": self.processing_action,
            "actions_completed": self.actions_completed,
            "personality": self.personality,
            "status": "active" if self.assigned_role else "waiting_for_role"
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.player_status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    # Setup logging
    logging.basicConfig(level=logging.INFO)
    
    node = PlayerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"🛑 Player {node.player_name} stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()