#!/usr/bin/env python3
"""
Enhanced Player Node - Universal Player for Codenames Robotics Game
Enhanced to support paragraph-style clues instead of single-word clues.
Can act as either Spymaster or Operative based on role assignment from orchestrator.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import logging
import time
from typing import List, Optional, Dict

# Import the enhanced LLM client
from .llm_client import enhanced_llm_client

class EnhancedPlayerNode(Node):
    def __init__(self):
        super().__init__('player_node')
        
        # Setup logging
        self.logger = self.get_logger()
        self.logger.info("🎭 Enhanced Player Node starting...")
        
        # Player configuration
        self.player_id = os.getenv('PLAYER_ID', 'player_1')
        self.player_name = os.getenv('PLAYER_NAME', f'Player {self.player_id[-1]}')
        self.model = os.getenv('PLAYER_MODEL', 'gpt-4')
        
        # Clue style configuration
        self.clue_style = os.getenv('CLUE_STYLE', 'paragraph')  # 'paragraph' or 'single_word'
        
        # Role assignment (will be set by orchestrator)
        self.assigned_role = None  # 'red_spymaster', 'red_operative', 'blue_spymaster', 'blue_operative'
        self.assigned_team = None  # 'red', 'blue'
        self.role_type = None      # 'spymaster', 'operative'
        
        # Publishers
        self.clue_response_pub = self.create_publisher(String, '/game/clue_response', 10)
        self.guess_response_pub = self.create_publisher(String, '/game/guess_response', 10)
        self.player_status_pub = self.create_publisher(String, f'/game/{self.player_id}_status', 10)
        self.player_ready_pub = self.create_publisher(String, '/game/player_ready', 10)
        self.pass_request_pub = self.create_publisher(String, '/game/pass_request', 10)
        
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
        
        # Enhanced personality traits for paragraph clues
        self.personality = {
            'risk_tolerance': 0.7,      # 0.0 = very conservative, 1.0 = very risky
            'creativity': 0.8,          # 0.0 = basic clues, 1.0 = creative clues
            'confidence': 0.6,          # affects guess patterns
            'verbosity': 0.7,           # affects clue explanation length
            'teaching_style': 0.6       # how much explanation to give
        }
        
        self.logger.info(f"✅ Enhanced Player Node '{self.player_name}' ({self.player_id}) initialized")
        self.logger.info(f"🎯 Clue style: {self.clue_style}")
        
        # Announce readiness
        self.announce_ready()
    
    def announce_ready(self):
        """Announce player is ready to receive role assignments"""
        ready_msg = {
            "player_id": self.player_id,
            "player_name": self.player_name,
            "model": self.model,
            "clue_style": self.clue_style,
            "personality": self.personality,
            "status": "ready_for_assignment",
            "capabilities": ["paragraph_clues", "detailed_reasoning"]
        }
        
        msg = String()
        msg.data = json.dumps(ready_msg)
        self.player_ready_pub.publish(msg)
        
        self.logger.info(f"📢 Enhanced Player {self.player_name} ready for role assignment")
    
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
                "clue_style": self.clue_style,
                "status": "role_accepted"
            }
            
            msg = String()
            msg.data = json.dumps(ack_msg)
            self.player_ready_pub.publish(msg)
            
        except Exception as e:
            self.logger.error(f"❌ Error handling role assignment: {e}")
    
    def handle_clue_request(self, msg: String):
        """Handle clue request (only if assigned as spymaster) - Enhanced with paragraph support"""
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
            
            self.logger.info(f"🎯 {self.player_name} generating {self.clue_style} clue for {self.assigned_team} team")
            self.logger.info(f"   Target words: {target_words}")
            self.logger.info(f"   Avoid words: {len(avoid_words)} words")
            
            # Generate clue using enhanced personality-influenced strategy
            clue_data = self.generate_enhanced_clue_with_personality(target_words, avoid_words)
            
            # Prepare response with enhanced clue data
            response = {
                "game_id": self.current_game_id,
                "player_id": self.player_id,
                "player_name": self.player_name,
                "team": self.assigned_team,
                "turn": turn,
                "model_used": self.model,
                "clue_style": self.clue_style,
                "personality_influence": self.personality
            }
            
            # Add clue data based on style
            if self.clue_style == 'paragraph':
                response.update({
                    "clue_paragraph": clue_data.get('clue_paragraph', ''),
                    "main_theme": clue_data.get('main_theme', ''),
                    "count": clue_data.get('count', 1),
                    "reasoning": clue_data.get('reasoning', ''),
                    "warnings": clue_data.get('warnings', ''),
                    "clue_type": "paragraph"
                })
                self.logger.info(f"📝 Generated paragraph clue:")
                self.logger.info(f"   Theme: {clue_data.get('main_theme', '')}")
                self.logger.info(f"   Paragraph: {clue_data.get('clue_paragraph', '')}")
                self.logger.info(f"   Count: {clue_data.get('count', 1)}")
            else:
                response.update({
                    "clue": clue_data.get('clue', 'FALLBACK'),
                    "count": clue_data.get('count', 1),
                    "clue_type": "single_word"
                })
                self.logger.info(f"📤 Generated single-word clue: '{clue_data.get('clue', '')}' (count: {clue_data.get('count', 1)})")
            
            # Publish clue response
            response_msg = String()
            response_msg.data = json.dumps(response)
            self.clue_response_pub.publish(response_msg)
            
            self.actions_completed += 1
            
        except Exception as e:
            self.logger.error(f"❌ Error handling clue request: {e}")
        finally:
            self.processing_action = False
    
    def handle_guess_request(self, msg: String):
        """Handle guess request (only if assigned as operative) - Enhanced to handle paragraph clues"""
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
            
            # Extract clue data - support both paragraph and single-word formats
            clue_data = self.extract_clue_data(data)
            available_words = data.get('available_words', [])
            guesses_made = data.get('guesses_made', 0)
            turn = data.get('turn', 0)
            
            self.logger.info(f"🔍 {self.player_name} making guess for {self.assigned_team} team")
            
            if clue_data.get('clue_type') == 'paragraph':
                self.logger.info(f"   Paragraph clue received:")
                self.logger.info(f"   Theme: {clue_data.get('main_theme', 'N/A')}")
                self.logger.info(f"   Full clue: {clue_data.get('clue_paragraph', 'N/A')}")
                self.logger.info(f"   Count: {clue_data.get('count', 1)}")
            else:
                self.logger.info(f"   Single-word clue: '{clue_data.get('clue', '')}' (count: {clue_data.get('count', 1)})")
            
            self.logger.info(f"   Guesses made: {guesses_made}")
            
            # Generate guess using enhanced personality
            guess = self.generate_enhanced_guess_with_personality(
                clue_data, available_words, guesses_made
            )
            
            # Handle pass decision
            if guess == "PASS":
                self.logger.info(f"⏭️ {self.player_name} decided to pass")
                return  # Don't send a guess response, pass request already sent
            
            # Simulate thinking time based on personality and clue complexity
            thinking_time = self.calculate_enhanced_thinking_time(clue_data, guesses_made)
            self.logger.info(f"🤔 {self.player_name} thinking for {thinking_time:.1f}s...")
            time.sleep(thinking_time)
            
            # Prepare response
            response = {
                "game_id": self.current_game_id,
                "player_id": self.player_id,
                "player_name": self.player_name,
                "team": self.assigned_team,
                "word": guess,
                "guesses_made": guesses_made + 1,
                "turn": turn,
                "model_used": self.model,
                "confidence": self.calculate_guess_confidence(guess, guesses_made),
                "clue_understood": clue_data
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
    
    def extract_clue_data(self, data: Dict) -> Dict:
        """Extract clue data from request, supporting both paragraph and single-word formats"""
        clue_data = {}
        
        # Check if it's a paragraph clue
        if 'clue_paragraph' in data:
            clue_data = {
                'clue_paragraph': data.get('clue_paragraph', ''),
                'main_theme': data.get('main_theme', ''),
                'count': data.get('count', 1),
                'reasoning': data.get('reasoning', ''),
                'warnings': data.get('warnings', ''),
                'clue_type': 'paragraph'
            }
        else:
            # Traditional single-word clue
            clue_data = {
                'clue': data.get('clue', ''),
                'count': data.get('clue_count', data.get('count', 1)),
                'clue_type': 'single_word'
            }
        
        return clue_data
    
    def generate_enhanced_clue_with_personality(self, target_words: List[str], avoid_words: List[str]) -> Dict:
        """Generate clue influenced by player personality and clue style"""
        try:
            # Use enhanced LLM client with personality considerations
            clue_data = enhanced_llm_client.get_spymaster_clue_paragraph(
                target_words=target_words,
                avoid_words=avoid_words,
                model=self.model,
                clue_style=self.clue_style
            )
            
            # Apply personality-based adjustments
            if self.clue_style == 'paragraph':
                # Adjust verbosity based on personality
                if self.personality['verbosity'] < 0.4:
                    # Make clue more concise
                    original_paragraph = clue_data.get('clue_paragraph', '')
                    sentences = original_paragraph.split('. ')
                    if len(sentences) > 1:
                        clue_data['clue_paragraph'] = '. '.join(sentences[:2]) + '.'
                
                # Adjust risk tolerance
                if self.personality['risk_tolerance'] < 0.4 and clue_data.get('count', 1) > 2:
                    clue_data['count'] = min(clue_data.get('count', 1), 2)
                    self.logger.info(f"🛡️ {self.player_name} being conservative, limited count to {clue_data['count']}")
            
            return clue_data
            
        except Exception as e:
            self.logger.error(f"❌ Error generating enhanced clue: {e}")
            if self.clue_style == 'paragraph':
                return {
                    'clue_paragraph': "Think carefully about the connection. I'm looking for 1 word that fits. Be cautious.",
                    'main_theme': 'FALLBACK',
                    'count': 1,
                    'reasoning': 'Fallback due to error',
                    'warnings': 'Choose carefully',
                    'clue_type': 'paragraph'
                }
            else:
                return {'clue': 'FALLBACK', 'count': 1, 'clue_type': 'single_word'}
    
    def generate_enhanced_guess_with_personality(self, clue_data: Dict, available_words: List[str], guesses_made: int) -> str:
        """Generate guess influenced by player personality and clue type"""
        try:
            # Get potential guesses from enhanced LLM
            guesses = enhanced_llm_client.get_operative_guesses_with_context(
                clue_data=clue_data,
                available_words=available_words,
                model=self.model
            )
            
            # Check if player should pass based on personality and situation
            should_pass = self.should_pass_turn(clue_data, guesses_made, guesses)
            if should_pass:
                self.send_pass_request(clue_data)
                return "PASS"  # Special return value to indicate pass
            
            # Apply personality-based selection
            if guesses_made == 0:
                # First guess - all personalities make it
                selected_guess = guesses[0] if guesses else available_words[0]
            else:
                # Subsequent guesses - personality matters more
                if self.personality['risk_tolerance'] < 0.3:
                    # Conservative: stop after 1-2 guesses unless very confident
                    if guesses_made >= 1 and clue_data.get('clue_type') != 'paragraph':
                        self.logger.info(f"🛡️ {self.player_name} being conservative")
                        self.send_pass_request(clue_data)
                        return "PASS"
                
                selected_guess = guesses[min(guesses_made, len(guesses)-1)] if guesses else available_words[0]
            
            # Validate guess
            if selected_guess.upper() in [w.upper() for w in available_words]:
                return selected_guess.upper()
            else:
                return available_words[0].upper() if available_words else "UNKNOWN"
            
        except Exception as e:
            self.logger.error(f"❌ Error generating enhanced guess: {e}")
            return available_words[0].upper() if available_words else "UNKNOWN"
    
    def should_pass_turn(self, clue_data: Dict, guesses_made: int, potential_guesses: List[str]) -> bool:
        """Determine if player should pass based on personality and situation"""
        # Never pass on first guess
        if guesses_made == 0:
            return False
        
        # Check if we have no good guesses
        if not potential_guesses:
            self.logger.info(f"🤔 {self.player_name} has no good guesses - considering pass")
            return True
        
        clue_count = clue_data.get('count', 1)
        max_safe_guesses = clue_count  # Don't go beyond clue count unless very confident
        
        # Conservative players pass earlier
        if self.personality['risk_tolerance'] < 0.4:
            if guesses_made >= max_safe_guesses:
                self.logger.info(f"🛡️ {self.player_name} playing it safe - passing after {guesses_made} guesses")
                return True
        
        # Very conservative players might pass even earlier
        if self.personality['risk_tolerance'] < 0.2:
            if guesses_made >= min(2, max_safe_guesses):
                self.logger.info(f"🛡️ {self.player_name} being very conservative - early pass")
                return True
        
        # Confidence affects passing decision
        if self.personality['confidence'] < 0.3 and guesses_made >= max_safe_guesses:
            self.logger.info(f"😰 {self.player_name} lacks confidence - passing")
            return True
        
        return False
    
    def send_pass_request(self, clue_data: Dict):
        """Send pass request to orchestrator"""
        pass_msg = {
            "game_id": self.current_game_id,
            "player_id": self.player_id,
            "player_name": self.player_name,
            "team": self.assigned_team,
            "action": "pass",
            "clue_understood": clue_data,
            "reason": "strategic_pass"
        }
        
        msg = String()
        msg.data = json.dumps(pass_msg)
        self.pass_request_pub.publish(msg)
        
        self.logger.info(f"⏭️ {self.player_name} chose to pass turn")
    
    def calculate_enhanced_thinking_time(self, clue_data: Dict, guesses_made: int) -> float:
        """Calculate thinking time based on personality, clue complexity, and situation"""
        base_time = 2.0  # Base thinking time
        
        # Confidence affects thinking time
        confidence_factor = 2.0 - self.personality['confidence']
        
        # Paragraph clues may require more thinking
        if clue_data.get('clue_type') == 'paragraph':
            complexity_factor = 1.5  # More complex clues need more time
        else:
            complexity_factor = 1.0
        
        # More guesses = more thinking
        guess_factor = 1.0 + (guesses_made * 0.5)
        
        thinking_time = base_time * confidence_factor * complexity_factor * guess_factor
        
        # Cap between 1-10 seconds (longer for paragraph clues)
        max_time = 10.0 if clue_data.get('clue_type') == 'paragraph' else 8.0
        return max(1.0, min(max_time, thinking_time))
    
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
            "clue_style": self.clue_style,
            "assigned_role": self.assigned_role,
            "assigned_team": self.assigned_team,
            "role_type": self.role_type,
            "processing_action": self.processing_action,
            "actions_completed": self.actions_completed,
            "personality": self.personality,
            "capabilities": ["paragraph_clues", "detailed_reasoning"],
            "status": "active" if self.assigned_role else "waiting_for_role"
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.player_status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    # Setup logging
    logging.basicConfig(level=logging.INFO)
    
    node = EnhancedPlayerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"🛑 Enhanced Player {node.player_name} stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()