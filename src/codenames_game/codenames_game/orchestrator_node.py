#!/usr/bin/env python3
"""
Enhanced Orchestrator Node - Game Master for 4-Player Codenames Robotics Game
Enhanced to support paragraph-style clues and detailed communication between players.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import logging
import random
from typing import Dict, List, Any

from .game_logic import GameBoard, GameState, CardType, create_default_board

class EnhancedFourPlayerOrchestratorNode(Node):
    def __init__(self):
        super().__init__('orchestrator_node')
        
        # Setup logging
        self.logger = self.get_logger()
        self.logger.info("🎮 Enhanced 4-Player Orchestrator Node starting...")
        
        # Game state
        self.game_board: GameBoard = None
        self.game_id = f"game_{random.randint(1000, 9999)}"
        self.turn_counter = 0
        
        # Player management
        self.players = {}  # player_id -> player_info
        self.ready_players = set()
        self.roles_assigned = False
        self.required_players = 4
        
        # Current role assignments
        self.role_assignments = {
            'red_spymaster': None,
            'red_operative': None,
            'blue_spymaster': None,
            'blue_operative': None
        }
        
        # Enhanced clue tracking for paragraph support
        self.current_clue_data = None  # Store full clue data including paragraphs
        self.clue_history_detailed = []  # Enhanced clue history
        
        # Publishers
        self.board_state_pub = self.create_publisher(String, '/game/board_state', 10)
        self.clue_request_pub = self.create_publisher(String, '/game/clue_request', 10)
        self.guess_request_pub = self.create_publisher(String, '/game/guess_request', 10)
        self.game_status_pub = self.create_publisher(String, '/game/status', 10)
        self.role_assignment_pub = self.create_publisher(String, '/game/role_assignments', 10)
        self.clue_announcement_pub = self.create_publisher(String, '/game/clue_announcement', 10)
        
        # Subscribers
        self.clue_sub = self.create_subscription(String, '/game/clue_response', self.handle_clue_response, 10)
        self.guess_sub = self.create_subscription(String, '/game/guess_response', self.handle_guess_response, 10)
        self.start_game_sub = self.create_subscription(String, '/game/start', self.handle_start_game, 10)
        self.player_ready_sub = self.create_subscription(String, '/game/player_ready', self.handle_player_ready, 10)
        self.pass_request_sub = self.create_subscription(String, '/game/pass_request', self.handle_pass_request, 10)
        
        # Timers
        self.status_timer = self.create_timer(5.0, self.publish_status)
        self.game_timer = self.create_timer(3.0, self.game_loop)
        self.player_check_timer = self.create_timer(2.0, self.check_player_readiness)
        
        # State tracking
        self.waiting_for_clue = False
        self.waiting_for_guess = False
        self.guesses_made = 0
        
        self.logger.info("✅ Enhanced 4-Player Orchestrator Node initialized")
        self.logger.info(f"🎯 Waiting for {self.required_players} players to join...")
    
    def handle_player_ready(self, msg: String):
        """Handle player ready announcements with enhanced capabilities"""
        try:
            data = json.loads(msg.data)
            player_id = data.get('player_id')
            
            if data.get('status') == 'ready_for_assignment':
                # New player joining
                self.players[player_id] = {
                    'player_id': player_id,
                    'player_name': data.get('player_name', player_id),
                    'model': data.get('model', 'unknown'),
                    'clue_style': data.get('clue_style', 'single_word'),
                    'personality': data.get('personality', {}),
                    'capabilities': data.get('capabilities', []),
                    'ready': True,
                    'role': None,
                    'team': None
                }
                self.ready_players.add(player_id)
                
                clue_style = data.get('clue_style', 'single_word')
                capabilities = data.get('capabilities', [])
                self.logger.info(f"✅ Player {data.get('player_name', player_id)} joined ({len(self.ready_players)}/{self.required_players})")
                self.logger.info(f"   Clue style: {clue_style}, Capabilities: {capabilities}")
                
            elif data.get('status') == 'role_accepted':
                # Player accepted role assignment
                player_id = data.get('player_id')
                role = data.get('role_accepted')
                self.logger.info(f"✅ Player {player_id} accepted role: {role}")
                
                if player_id in self.players:
                    self.players[player_id]['role_confirmed'] = True
                
                # Check if all players have confirmed roles
                if all(p.get('role_confirmed', False) for p in self.players.values()):
                    self.logger.info("🎉 All players have accepted their roles!")
                    self.start_game_after_assignments()
                    
        except Exception as e:
            self.logger.error(f"❌ Error handling player ready: {e}")
    
    def check_player_readiness(self):
        """Check if we have enough players to start role assignment"""
        if len(self.ready_players) >= self.required_players and not self.roles_assigned:
            self.logger.info(f"🎯 {self.required_players} players ready! Assigning roles...")
            self.assign_roles()
    
    def assign_roles(self):
        """Randomly assign roles to players"""
        if len(self.ready_players) < self.required_players:
            return
        
        # Get list of available players
        available_players = list(self.ready_players)
        random.shuffle(available_players)
        
        # Define roles
        roles = ['red_spymaster', 'red_operative', 'blue_spymaster', 'blue_operative']
        
        # Assign roles randomly
        for i, role in enumerate(roles):
            player_id = available_players[i]
            team = 'red' if 'red' in role else 'blue'
            role_type = 'spymaster' if 'spymaster' in role else 'operative'
            
            # Update player info
            self.players[player_id]['role'] = role
            self.players[player_id]['team'] = team
            self.players[player_id]['role_type'] = role_type
            
            # Update role assignments
            self.role_assignments[role] = player_id
            
            # Send role assignment to player
            assignment_msg = {
                'player_id': player_id,
                'role': role,
                'team': team,
                'role_type': role_type,
                'game_id': self.game_id
            }
            
            msg = String()
            msg.data = json.dumps(assignment_msg)
            self.role_assignment_pub.publish(msg)
            
            player_name = self.players[player_id]['player_name']
            clue_style = self.players[player_id].get('clue_style', 'single_word')
            self.logger.info(f"🎭 Assigned {role} to {player_name} (clue style: {clue_style})")
        
        self.roles_assigned = True
        self.logger.info("✅ All roles assigned!")
        
        # Publish role assignments for UI
        self.publish_role_assignments()
    
    def publish_role_assignments(self):
        """Publish role assignments for monitoring"""
        assignments = {}
        for role, player_id in self.role_assignments.items():
            if player_id and player_id in self.players:
                assignments[role] = {
                    'player_id': player_id,
                    'player_name': self.players[player_id]['player_name'],
                    'model': self.players[player_id]['model'],
                    'clue_style': self.players[player_id].get('clue_style', 'single_word'),
                    'capabilities': self.players[player_id].get('capabilities', [])
                }
        
        assignment_msg = {
            'type': 'role_assignments',
            'game_id': self.game_id,
            'assignments': assignments
        }
        
        msg = String()
        msg.data = json.dumps(assignment_msg)
        self.game_status_pub.publish(msg)
    
    def handle_start_game(self, msg: String):
        """Handle game start request"""
        self.logger.info("📨 Received start game request")
        if len(self.ready_players) >= self.required_players:
            if not self.roles_assigned:
                self.assign_roles()
            else:
                self.start_new_game()
        else:
            self.logger.warning(f"⚠️ Need {self.required_players} players, only have {len(self.ready_players)}")
    
    def start_game_after_assignments(self):
        """Start game after all roles are assigned and confirmed"""
        self.logger.info("🎯 Starting new enhanced 4-player Codenames game...")
        self.start_new_game()
    
    def start_new_game(self):
        """Initialize a new game with assigned roles"""
        self.game_board = create_default_board()
        self.turn_counter = 0
        self.waiting_for_clue = False
        self.waiting_for_guess = False
        self.current_clue_data = None
        self.guesses_made = 0
        self.clue_history_detailed = []
        
        # Publish initial board state
        self.publish_board_state()
        
        # Start with red team spymaster
        self.game_board.current_state = GameState.RED_SPYMASTER
        self.game_board.current_team = "red"
        
        self.logger.info("✅ New enhanced 4-player game started - Red team spymaster turn")
    
    def game_loop(self):
        """Main game loop - manages turn flow with enhanced clue support"""
        if self.game_board is None or not self.roles_assigned:
            return
        
        if self.game_board.current_state == GameState.GAME_OVER:
            return
        
        current_state = self.game_board.current_state
        
        # Only request new clues when transitioning to spymaster states
        if current_state == GameState.RED_SPYMASTER and not self.waiting_for_clue:
            self.request_clue_from_player("red")
            
        elif current_state == GameState.BLUE_SPYMASTER and not self.waiting_for_clue:
            self.request_clue_from_player("blue")
            
        # For operative states, only request guess if not already waiting and have a current clue
        elif current_state == GameState.RED_OPERATIVES and not self.waiting_for_guess:
            if self.current_clue_data is not None:
                self.request_guess_from_player("red")
            else:
                self.logger.error("❌ Red operatives state but no current clue data!")
                self.game_board.current_state = GameState.RED_SPYMASTER
            
        elif current_state == GameState.BLUE_OPERATIVES and not self.waiting_for_guess:
            if self.current_clue_data is not None:
                self.request_guess_from_player("blue")
            else:
                self.logger.error("❌ Blue operatives state but no current clue data!")
                self.game_board.current_state = GameState.BLUE_SPYMASTER
    
    def request_clue_from_player(self, team: str):
        """Request a clue from the specific team's spymaster"""
        role = f"{team}_spymaster"
        player_id = self.role_assignments.get(role)
        
        if not player_id:
            self.logger.error(f"❌ No player assigned to {role}")
            return
        
        player_info = self.players[player_id]
        player_name = player_info['player_name']
        clue_style = player_info.get('clue_style', 'single_word')
        
        self.logger.info(f"🎯 Requesting {clue_style} clue from {player_name} ({role})")
        
        # Get target words (team's unrevealed words)
        if team == "red":
            target_words = self.game_board.get_unrevealed_words_by_type(CardType.RED)
        else:
            target_words = self.game_board.get_unrevealed_words_by_type(CardType.BLUE)
        
        # Get avoid words (all other unrevealed words)
        avoid_words = []
        for card in self.game_board.cards:
            if not card.revealed and card.word not in target_words:
                avoid_words.append(card.word)
        
        clue_request = {
            "game_id": self.game_id,
            "team": team,
            "player_id": player_id,
            "target_words": target_words,
            "avoid_words": avoid_words,
            "turn": self.turn_counter,
            "clue_style_requested": clue_style
        }
        
        msg = String()
        msg.data = json.dumps(clue_request)
        self.clue_request_pub.publish(msg)
        
        self.waiting_for_clue = True
        self.logger.info(f"📤 {clue_style.title()} clue request sent to {player_name}")
    
    def request_guess_from_player(self, team: str):
        """Request a guess from the specific team's operative - Enhanced to pass full clue data"""
        role = f"{team}_operative"
        player_id = self.role_assignments.get(role)
        
        if not player_id:
            self.logger.error(f"❌ No player assigned to {role}")
            return
        
        player_name = self.players[player_id]['player_name']
        self.logger.info(f"🎯 Requesting guess from {player_name} ({role})")
        
        # Get available words (unrevealed)
        available_words = [card.word for card in self.game_board.cards if not card.revealed]
        
        guess_request = {
            "game_id": self.game_id,
            "team": team,
            "player_id": player_id,
            "available_words": available_words,
            "guesses_made": self.guesses_made,
            "turn": self.turn_counter
        }
        
        # Add full clue data based on type
        if self.current_clue_data:
            guess_request.update(self.current_clue_data)
        else:
            # Fallback for legacy support
            guess_request.update({
                "clue": "UNKNOWN",
                "clue_count": 1
            })
        
        msg = String()
        msg.data = json.dumps(guess_request)
        self.guess_request_pub.publish(msg)
        
        self.waiting_for_guess = True
        self.logger.info(f"📤 Guess request sent to {player_name}")
    
    def handle_clue_response(self, msg: String):
        """Handle clue from spymaster player - Enhanced for paragraph clues"""
        try:
            data = json.loads(msg.data)
            team = data.get('team', 'unknown')
            player_id = data.get('player_id')
            player_name = data.get('player_name', player_id)
            clue_style = data.get('clue_style', 'single_word')
            
            # Extract clue data based on type
            if data.get('clue_type') == 'paragraph' or 'clue_paragraph' in data:
                # Paragraph clue
                clue_paragraph = data.get('clue_paragraph', '')
                main_theme = data.get('main_theme', '')
                count = data.get('count', 1)
                reasoning = data.get('reasoning', '')
                warnings = data.get('warnings', '')
                
                self.logger.info(f"📨 Received paragraph clue from {player_name} ({team} team):")
                self.logger.info(f"   Theme: {main_theme}")
                self.logger.info(f"   Paragraph: {clue_paragraph}")
                self.logger.info(f"   Count: {count}")
                self.logger.info(f"   Reasoning: {reasoning}")
                
                # Store full clue data
                self.current_clue_data = {
                    'clue_paragraph': clue_paragraph,
                    'main_theme': main_theme,
                    'count': count,
                    'reasoning': reasoning,
                    'warnings': warnings,
                    'clue_type': 'paragraph'
                }
                
                # Add to detailed history
                clue_entry = {
                    'team': team,
                    'player_name': player_name,
                    'clue_type': 'paragraph',
                    'main_theme': main_theme,
                    'clue_paragraph': clue_paragraph,
                    'count': count,
                    'reasoning': reasoning,
                    'turn': self.turn_counter
                }
                
            else:
                # Traditional single-word clue
                clue = data.get('clue', 'UNKNOWN')
                count = data.get('count', 1)
                
                self.logger.info(f"📨 Received single-word clue: '{clue}' for {count} words from {player_name} ({team} team)")
                
                # Store clue data
                self.current_clue_data = {
                    'clue': clue,
                    'count': count,
                    'clue_type': 'single_word'
                }
                
                # Add to detailed history
                clue_entry = {
                    'team': team,
                    'player_name': player_name,
                    'clue_type': 'single_word',
                    'clue': clue,
                    'count': count,
                    'turn': self.turn_counter
                }
            
            self.clue_history_detailed.append(clue_entry)
            self.guesses_made = 0
            
            # Add to game board history (legacy support)
            if self.current_clue_data.get('clue_type') == 'paragraph':
                self.game_board.add_clue(self.current_clue_data.get('main_theme', 'THEME'), count, team)
            else:
                self.game_board.add_clue(self.current_clue_data.get('clue', 'UNKNOWN'), count, team)
            
            # Announce clue to all players and observers
            self.announce_clue(self.current_clue_data, team, player_name)
            
            # Transition to operative phase
            if team == "red":
                self.game_board.current_state = GameState.RED_OPERATIVES
            else:
                self.game_board.current_state = GameState.BLUE_OPERATIVES
            
            self.waiting_for_clue = False
            self.publish_board_state()
            
        except Exception as e:
            self.logger.error(f"❌ Error handling clue response: {e}")
            self.waiting_for_clue = False
    
    def announce_clue(self, clue_data: Dict, team: str, player_name: str):
        """Announce clue to all observers and Isaac Sim"""
        announcement = {
            'type': 'clue_announcement',
            'game_id': self.game_id,
            'team': team,
            'player_name': player_name,
            'turn': self.turn_counter
        }
        announcement.update(clue_data)
        
        msg = String()
        msg.data = json.dumps(announcement)
        self.clue_announcement_pub.publish(msg)
        
        # Log the announcement
        if clue_data.get('clue_type') == 'paragraph':
            self.logger.info(f"📢 Announced paragraph clue to all observers")
        else:
            clue = clue_data.get('clue', 'UNKNOWN')
            count = clue_data.get('count', 1)
            self.logger.info(f"📢 Announced clue '{clue}' ({count}) to all observers")
    
    def handle_guess_response(self, msg: String):
        """Handle guess from operative player - Fixed to properly handle multiple guesses per clue"""
        try:
            data = json.loads(msg.data)
            word = data.get('word', '').upper()
            team = data.get('team', 'unknown')
            player_id = data.get('player_id')
            player_name = data.get('player_name', player_id)
            
            self.logger.info(f"📨 Received guess: '{word}' from {player_name} ({team} team)")
            
            # Process the guess
            found, card_type, game_over = self.game_board.make_guess(word)
            
            if found:
                self.logger.info(f"✅ Guess '{word}' was correct! Card type: {card_type.value}")
                self.guesses_made += 1
                
                # Check game over conditions first
                if game_over:
                    self.logger.info("🏁 GAME OVER!")
                    if card_type == CardType.ASSASSIN:
                        winner = "blue" if team == "red" else "red"
                        self.logger.info(f"💀 {team.upper()} team hit the assassin! {winner.upper()} team wins!")
                    else:
                        winner_team = team.upper()
                        self.logger.info(f"🎉 {winner_team} team wins!")
                    
                    self.game_board.current_state = GameState.GAME_OVER
                    self.announce_game_end(winner if game_over else None, card_type == CardType.ASSASSIN)
                    self.waiting_for_guess = False
                    self.publish_board_state()
                    return
                
                # Game continues - check if team should continue guessing
                if (card_type == CardType.RED and team == "red") or (card_type == CardType.BLUE and team == "blue"):
                    # Correct team card - they can guess again if within limits
                    max_guesses = self.current_clue_data.get('count', 1) + 1 if self.current_clue_data else 2
                    if self.guesses_made < max_guesses:
                        self.logger.info(f"👍 Correct! {team} team can guess again ({self.guesses_made}/{max_guesses})")
                        # Stay in current operative state - DON'T change to spymaster state
                        self.waiting_for_guess = False  # Allow next guess
                        self.publish_board_state()
                        return
                    else:
                        self.logger.info(f"📊 {team} team used all their guesses ({self.guesses_made}/{max_guesses})")
                        self.end_turn()
                else:
                    # Wrong card type (neutral, opponent, or assassin) - end turn immediately
                    self.logger.info(f"👎 Wrong card type ({card_type.value}) - ending {team} team turn")
                    self.end_turn()
            else:
                self.logger.info(f"❌ Guess '{word}' not found or already revealed - ending turn")
                self.end_turn()
            
            self.waiting_for_guess = False
            self.publish_board_state()
            
        except Exception as e:
            self.logger.error(f"❌ Error handling guess response: {e}")
            self.waiting_for_guess = False
    
    def end_turn(self):
        """End current team's turn and switch to next team"""
        current_team = self.game_board.current_team
        
        self.logger.info(f"🔄 Ending {current_team} team's turn")
        self.logger.info(f"   Guesses made this turn: {self.guesses_made}")
        if self.current_clue_data:
            clue_count = self.current_clue_data.get('count', 1)
            self.logger.info(f"   Clue was for {clue_count} words")
        
        # Switch teams
        if current_team == "red":
            self.game_board.current_team = "blue"
            self.game_board.current_state = GameState.BLUE_SPYMASTER
        else:
            self.game_board.current_team = "red"
            self.game_board.current_state = GameState.RED_SPYMASTER
        
        self.turn_counter += 1
        self.current_clue_data = None  # Clear clue data for new turn
        self.guesses_made = 0         # Reset guess counter
        
        self.logger.info(f"🔄 Turn {self.turn_counter}: Now {self.game_board.current_team} team's turn (spymaster phase)")
    
    def handle_pass_request(self, msg: String):
        """Handle when operative team decides to pass (end turn voluntarily)"""
        try:
            data = json.loads(msg.data)
            team = data.get('team', 'unknown')
            player_id = data.get('player_id')
            player_name = data.get('player_name', player_id)
            
            # Verify this is the correct team's operative
            current_team = self.game_board.current_team
            if team != current_team:
                self.logger.warning(f"⚠️ Pass request from {team} but it's {current_team}'s turn")
                return
            
            self.logger.info(f"⏭️ {player_name} ({team} team) chose to pass - ending turn")
            self.end_turn()
            self.waiting_for_guess = False
            self.publish_board_state()
            
        except Exception as e:
            self.logger.error(f"❌ Error handling pass request: {e}")
    
    def announce_game_end(self, winner_team: str, assassin_hit: bool):
        """Announce game end to all players"""
        end_msg = {
            'type': 'game_end',
            'game_id': self.game_id,
            'winner': winner_team,
            'reason': 'assassin' if assassin_hit else 'all_cards_found',
            'players': self.get_player_summary(),
            'clue_history': self.clue_history_detailed
        }
        
        msg = String()
        msg.data = json.dumps(end_msg)
        self.game_status_pub.publish(msg)
        
        self.logger.info(f"🏁 Game ended - {winner_team} team wins!")
    
    def get_player_summary(self) -> Dict:
        """Get summary of all players and their performance"""
        return {
            player_id: {
                'name': info['player_name'],
                'role': info.get('role'),
                'team': info.get('team'),
                'model': info['model'],
                'clue_style': info.get('clue_style', 'single_word'),
                'capabilities': info.get('capabilities', [])
            }
            for player_id, info in self.players.items()
        }
    
    def publish_board_state(self):
        """Publish current board state with enhanced clue information"""
        if self.game_board is None:
            return
        
        board_state = self.game_board.to_dict()
        board_state['game_id'] = self.game_id
        board_state['turn'] = self.turn_counter
        board_state['players'] = self.get_player_summary()
        board_state['role_assignments'] = self.role_assignments
        board_state['current_clue_data'] = self.current_clue_data
        board_state['clue_history_detailed'] = self.clue_history_detailed
        
        msg = String()
        msg.data = json.dumps(board_state)
        self.board_state_pub.publish(msg)
    
    def publish_status(self):
        """Publish periodic status updates"""
        if self.game_board is None:
            status = {
                "node": "enhanced_orchestrator",
                "status": "waiting_for_players",
                "game_id": self.game_id,
                "players_ready": len(self.ready_players),
                "players_needed": self.required_players,
                "roles_assigned": self.roles_assigned,
                "enhanced_features": ["paragraph_clues", "detailed_reasoning"]
            }
        else:
            status = {
                "node": "enhanced_orchestrator", 
                "status": "game_active",
                "game_id": self.game_id,
                "current_state": self.game_board.current_state.value,
                "current_team": self.game_board.current_team,
                "turn": self.turn_counter,
                "red_remaining": self.game_board.red_remaining,
                "blue_remaining": self.game_board.blue_remaining,
                "waiting_for_clue": self.waiting_for_clue,
                "waiting_for_guess": self.waiting_for_guess,
                "players": self.get_player_summary(),
                "roles_assigned": self.roles_assigned,
                "current_clue_type": self.current_clue_data.get('clue_type') if self.current_clue_data else None,
                "enhanced_features": ["paragraph_clues", "detailed_reasoning"]
            }
        
        msg = String()
        msg.data = json.dumps(status)
        self.game_status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    # Setup logging
    logging.basicConfig(level=logging.INFO)
    
    node = EnhancedFourPlayerOrchestratorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Enhanced 4-Player Orchestrator stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()