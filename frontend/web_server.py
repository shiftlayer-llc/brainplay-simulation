#!/usr/bin/env python3
"""
Enhanced Web Server for 4-Player Codenames Robotics Game Frontend
Flask + SocketIO server with improved logging and player name handling.
"""

import os
import json
import logging
import threading
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Global variables for ROS2 node
ros_node = None
ros_thread = None

class EnhancedFourPlayerWebBridgeNode(Node):
    def __init__(self, socketio):
        super().__init__('web_bridge_node')
        
        self.socketio = socketio
        self.logger = self.get_logger()
        self.logger.info("🌐 Enhanced 4-Player Web Bridge Node starting...")
        
        # Game state
        self.current_board_state = None
        self.game_status = {}
        self.player_statuses = {}
        self.role_assignments = {}
        
        # Enhanced logging and history
        self.game_log = []
        self.clue_history = []
        self.guess_history = []
        self.current_clue_data = None
        
        # Publishers (to game)
        self.start_game_pub = self.create_publisher(String, '/game/start', 10)
        
        # Subscribers (from game)
        self.board_state_sub = self.create_subscription(
            String, '/game/board_state', self.handle_board_state, 10
        )
        self.game_status_sub = self.create_subscription(
            String, '/game/status', self.handle_game_status, 10
        )
        
        # Enhanced clue and guess tracking
        self.clue_announcement_sub = self.create_subscription(
            String, '/game/clue_announcement', self.handle_clue_announcement, 10
        )
        self.clue_response_sub = self.create_subscription(
            String, '/game/clue_response', self.handle_clue_response, 10
        )
        self.guess_response_sub = self.create_subscription(
            String, '/game/guess_response', self.handle_guess_response, 10
        )
        
        # Subscribe to all player status topics with proper naming
        player_topics = [
            '/game/player_1_status',
            '/game/player_2_status', 
            '/game/player_3_status',
            '/game/player_4_status'
        ]
        
        for topic in player_topics:
            player_id = topic.split('/')[-1].replace('_status', '')
            self.create_subscription(
                String, topic, 
                lambda msg, pid=player_id: self.handle_player_status(msg, pid), 
                10
            )
        
        # Optional: Isaac bridge status
        self.isaac_status_sub = self.create_subscription(
            String, '/isaac/bridge_status', self.handle_isaac_status, 10
        )
        
        self.logger.info("✅ Enhanced 4-Player Web Bridge Node initialized")
        self.add_log_entry("🎮 Web bridge initialized and ready")
    
    def add_log_entry(self, message, log_type="info"):
        """Add entry to game log and emit to web clients"""
        import datetime
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        
        log_entry = {
            "timestamp": timestamp,
            "message": message,
            "type": log_type
        }
        
        self.game_log.insert(0, log_entry)  # Add to beginning
        
        # Keep only last 100 entries
        if len(self.game_log) > 100:
            self.game_log = self.game_log[:100]
        
        # Emit to web clients
        self.socketio.emit('log_entry', log_entry)
        
        self.logger.info(f"📝 {message}")
    
    def handle_board_state(self, msg: String):
        """Handle board state updates from game"""
        try:
            self.current_board_state = json.loads(msg.data)
            
            # Extract role assignments if present
            if 'role_assignments' in self.current_board_state:
                self.role_assignments = self.current_board_state['role_assignments']
            
            # Extract current clue data
            if 'current_clue_data' in self.current_board_state:
                self.current_clue_data = self.current_board_state['current_clue_data']
            
            # Log board state changes
            game_state = self.current_board_state.get('current_state', 'unknown')
            current_team = self.current_board_state.get('current_team', 'unknown')
            turn = self.current_board_state.get('turn', 0)
            
            self.add_log_entry(f"🎯 Turn {turn}: {game_state.replace('_', ' ').title()} ({current_team.upper()} team)")
            
            # Emit to all connected web clients
            self.socketio.emit('board_update', self.current_board_state)
            
        except Exception as e:
            self.logger.error(f"❌ Error handling board state: {e}")
            self.add_log_entry(f"❌ Error updating board state: {e}", "error")
    
    def handle_game_status(self, msg: String):
        """Handle game status updates"""
        try:
            status = json.loads(msg.data)
            self.game_status = status
            
            # Handle role assignments announcement
            if status.get('type') == 'role_assignments':
                self.role_assignments = status.get('assignments', {})
                self.socketio.emit('role_assignments', self.role_assignments)
                
                # Log role assignments
                self.add_log_entry("🎭 Role assignments updated:", "success")
                for role, player_info in self.role_assignments.items():
                    if player_info:
                        team_color = "🔴" if "red" in role else "🔵"
                        role_name = "Spymaster" if "spymaster" in role else "Operative"
                        player_name = player_info.get('player_name', player_info.get('player_id', 'Unknown'))
                        model = player_info.get('model', 'Unknown')
                        self.add_log_entry(f"  {team_color} {player_name} ({model}) → {role_name}")
            
            # Handle game end
            elif status.get('type') == 'game_end':
                winner = status.get('winner', 'Unknown')
                reason = status.get('reason', 'unknown')
                if reason == 'assassin':
                    self.add_log_entry(f"💀 Game Over! {winner.upper()} team wins (opponent hit assassin)!", "success")
                else:
                    self.add_log_entry(f"🎉 Game Over! {winner.upper()} team wins (all cards found)!", "success")
            
            # Log other status changes
            elif status.get('status') == 'waiting_for_players':
                players_ready = status.get('players_ready', 0)
                players_needed = status.get('players_needed', 4)
                self.add_log_entry(f"⏳ Waiting for players ({players_ready}/{players_needed} ready)")
            
            # Emit to web clients
            self.socketio.emit('game_status', status)
            
        except Exception as e:
            self.logger.error(f"❌ Error handling game status: {e}")
            self.add_log_entry(f"❌ Error updating game status: {e}", "error")
    
    def handle_clue_announcement(self, msg: String):
        """Handle clue announcements with enhanced logging"""
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
                
                team_color = "🔴" if team == "red" else "🔵"
                self.add_log_entry(f"{team_color} {player_name} (Spymaster) gives paragraph clue:", "info")
                self.add_log_entry(f"   Theme: {main_theme} (for {count} words)", "info")
                self.add_log_entry(f"   Clue: {clue_paragraph}", "info")
                if reasoning:
                    self.add_log_entry(f"   Reasoning: {reasoning}", "info")
                
                clue_entry = {
                    'team': team,
                    'player_name': player_name,
                    'clue_type': 'paragraph',
                    'main_theme': main_theme,
                    'clue_paragraph': clue_paragraph,
                    'count': count,
                    'reasoning': reasoning,
                    'timestamp': self.get_clock().now().nanoseconds
                }
            else:
                # Handle traditional single-word clue
                clue = data.get('clue', 'UNKNOWN')
                count = data.get('count', 1)
                
                team_color = "🔴" if team == "red" else "🔵"
                self.add_log_entry(f"{team_color} {player_name} (Spymaster): '{clue}' for {count} words", "info")
                
                clue_entry = {
                    'team': team,
                    'player_name': player_name,
                    'clue_type': 'single_word',
                    'clue': clue,
                    'count': count,
                    'timestamp': self.get_clock().now().nanoseconds
                }
            
            self.clue_history.insert(0, clue_entry)
            
            # Keep only last 50 clues
            if len(self.clue_history) > 50:
                self.clue_history = self.clue_history[:50]
            
            # Emit clue to web clients
            self.socketio.emit('clue_announcement', data)
            self.socketio.emit('clue_history_update', self.clue_history)
            
        except Exception as e:
            self.logger.error(f"❌ Error handling clue announcement: {e}")
            self.add_log_entry(f"❌ Error processing clue: {e}", "error")
    
    def handle_clue_response(self, msg: String):
        """Handle clue responses from spymasters"""
        try:
            data = json.loads(msg.data)
            # This is handled by clue_announcement, but we can log additional details
            pass
        except Exception as e:
            self.logger.error(f"❌ Error handling clue response: {e}")
    
    def handle_guess_response(self, msg: String):
        """Handle guess responses from operatives"""
        try:
            data = json.loads(msg.data)
            
            word = data.get('word', 'UNKNOWN')
            team = data.get('team', 'unknown')
            player_name = data.get('player_name', 'Unknown Player')
            confidence = data.get('confidence', 0.5)
            
            team_color = "🔴" if team == "red" else "🔵"
            confidence_str = f" (confidence: {confidence:.1%})" if confidence != 0.5 else ""
            
            self.add_log_entry(f"{team_color} {player_name} (Operative): guesses '{word}'{confidence_str}", "info")
            
            guess_entry = {
                'team': team,
                'player_name': player_name,
                'word': word,
                'confidence': confidence,
                'timestamp': self.get_clock().now().nanoseconds
            }
            
            self.guess_history.insert(0, guess_entry)
            
            # Keep only last 50 guesses
            if len(self.guess_history) > 50:
                self.guess_history = self.guess_history[:50]
            
            # Emit guess to web clients
            self.socketio.emit('guess_announcement', data)
            self.socketio.emit('guess_history_update', self.guess_history)
            
        except Exception as e:
            self.logger.error(f"❌ Error handling guess response: {e}")
            self.add_log_entry(f"❌ Error processing guess: {e}", "error")
    
    def handle_player_status(self, msg: String, player_id: str):
        """Handle individual player status updates with improved player name tracking"""
        try:
            status = json.loads(msg.data)
            
            # Store with proper player ID
            self.player_statuses[player_id] = status
            
            # Log player status changes
            player_name = status.get('player_name', player_id)
            
            if not hasattr(self, '_logged_players'):
                self._logged_players = set()
            
            # Log when player first joins
            if player_id not in self._logged_players:
                model = status.get('model', 'Unknown')
                clue_style = status.get('clue_style', 'single_word')
                self.add_log_entry(f"👤 {player_name} joined (Model: {model}, Clue style: {clue_style})")
                self._logged_players.add(player_id)
            
            # Log role assignments
            if status.get('assigned_role') and not hasattr(status, '_role_logged'):
                role = status.get('assigned_role', '').replace('_', ' ').title()
                team = status.get('assigned_team', '').upper()
                self.add_log_entry(f"🎭 {player_name} assigned as {team} {role}")
                status['_role_logged'] = True
            
            # Log when player is processing actions
            if status.get('processing_action') and not hasattr(status, '_processing_logged'):
                action_type = "giving clue" if "spymaster" in status.get('assigned_role', '') else "making guess"
                self.add_log_entry(f"🤔 {player_name} is {action_type}...")
                status['_processing_logged'] = True
            elif not status.get('processing_action'):
                if hasattr(status, '_processing_logged'):
                    delattr(status, '_processing_logged')
            
            # Emit to web clients
            self.socketio.emit('player_status', {
                'player_id': player_id,
                'status': status
            })
            
        except Exception as e:
            self.logger.error(f"❌ Error handling player status for {player_id}: {e}")
            self.add_log_entry(f"❌ Error updating {player_id} status: {e}", "error")
    
    def handle_isaac_status(self, msg: String):
        """Handle Isaac bridge status"""
        try:
            status = json.loads(msg.data)
            
            # Log Isaac connection changes
            if status.get('isaac_connected') and not hasattr(self, '_isaac_logged'):
                self.add_log_entry("🤖 Isaac Sim connected", "success")
                self._isaac_logged = True
            elif not status.get('isaac_connected') and hasattr(self, '_isaac_logged'):
                self.add_log_entry("🤖 Isaac Sim disconnected", "warning")
                delattr(self, '_isaac_logged')
            
            # Emit to web clients
            self.socketio.emit('isaac_status', status)
            
        except Exception as e:
            self.logger.error(f"❌ Error handling Isaac status: {e}")
    
    def start_new_game(self):
        """Start a new game"""
        msg = String()
        msg.data = json.dumps({"action": "start_game"})
        self.start_game_pub.publish(msg)
        
        self.add_log_entry("🎮 Starting new game...", "info")
        
        # Clear histories for new game
        self.clue_history = []
        self.guess_history = []
        self.socketio.emit('clue_history_update', self.clue_history)
        self.socketio.emit('guess_history_update', self.guess_history)
    
    def get_complete_status(self):
        """Get complete status for new client connections"""
        return {
            'board_state': self.current_board_state,
            'game_status': self.game_status,
            'player_statuses': self.player_statuses,
            'role_assignments': self.role_assignments,
            'game_log': self.game_log,
            'clue_history': self.clue_history,
            'guess_history': self.guess_history,
            'current_clue_data': self.current_clue_data,
            'ros_connected': True
        }

def ros_spin_thread():
    """Run ROS2 spinning in separate thread"""
    global ros_node
    try:
        rclpy.spin(ros_node)
    except Exception as e:
        logging.error(f"❌ ROS2 spinning error: {e}")

# Create Flask app
app = Flask(__name__)
app.config['SECRET_KEY'] = 'codenames_4player_enhanced_secret_key'
CORS(app)

# Create SocketIO
socketio = SocketIO(app, cors_allowed_origins="*")

@app.route('/')
def index():
    """Serve main game interface"""
    return render_template('index.html')

@app.route('/admin')
def admin():
    """Serve admin control panel"""
    return render_template('admin.html')

@app.route('/players')
def players():
    """Serve player status page"""
    return render_template('players.html')

@app.route('/api/status')
def api_status():
    """Get current game status via REST API"""
    global ros_node
    
    if ros_node:
        return jsonify(ros_node.get_complete_status())
    else:
        return jsonify({
            'board_state': None,
            'game_status': {},
            'player_statuses': {},
            'role_assignments': {},
            'game_log': [],
            'clue_history': [],
            'guess_history': [],
            'current_clue_data': None,
            'ros_connected': False
        })

@app.route('/api/players')
def api_players():
    """Get player information"""
    global ros_node
    
    if ros_node:
        return jsonify({
            'players': ros_node.player_statuses,
            'roles': ros_node.role_assignments
        })
    else:
        return jsonify({'players': {}, 'roles': {}})

@app.route('/api/history')
def api_history():
    """Get game history"""
    global ros_node
    
    if ros_node:
        return jsonify({
            'clues': ros_node.clue_history,
            'guesses': ros_node.guess_history,
            'log': ros_node.game_log
        })
    else:
        return jsonify({'clues': [], 'guesses': [], 'log': []})

@app.route('/api/start_game', methods=['POST'])
def api_start_game():
    """Start a new game via REST API"""
    global ros_node
    
    if ros_node:
        ros_node.start_new_game()
        return jsonify({'success': True, 'message': 'Game start command sent'})
    else:
        return jsonify({'success': False, 'message': 'ROS2 not connected'})

# SocketIO event handlers
@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    print(f"🔌 Client connected: {request.sid}")
    
    # Send current state to new client
    global ros_node
    if ros_node:
        complete_status = ros_node.get_complete_status()
        
        if complete_status['board_state']:
            emit('board_update', complete_status['board_state'])
        
        if complete_status['game_status']:
            emit('game_status', complete_status['game_status'])
        
        if complete_status['role_assignments']:
            emit('role_assignments', complete_status['role_assignments'])
        
        # Send all player statuses
        for player_id, status in complete_status['player_statuses'].items():
            emit('player_status', {
                'player_id': player_id,
                'status': status
            })
        
        # Send histories
        emit('game_log_full', complete_status['game_log'])
        emit('clue_history_update', complete_status['clue_history'])
        emit('guess_history_update', complete_status['guess_history'])

@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection"""
    print(f"🔌 Client disconnected: {request.sid}")

@socketio.on('start_game')
def handle_start_game_socket():
    """Handle start game request from web client"""
    global ros_node
    
    if ros_node:
        ros_node.start_new_game()
        emit('message', {'type': 'info', 'text': 'Game start command sent'})
    else:
        emit('message', {'type': 'error', 'text': 'ROS2 not connected'})

@socketio.on('request_status')
def handle_request_status():
    """Handle status request from web client"""
    global ros_node
    
    if ros_node:
        complete_status = ros_node.get_complete_status()
        
        # Send all current status info
        if complete_status['board_state']:
            emit('board_update', complete_status['board_state'])
        
        if complete_status['game_status']:
            emit('game_status', complete_status['game_status'])
        
        if complete_status['role_assignments']:
            emit('role_assignments', complete_status['role_assignments'])
        
        for player_id, status in complete_status['player_statuses'].items():
            emit('player_status', {
                'player_id': player_id,
                'status': status
            })
        
        emit('game_log_full', complete_status['game_log'])
        emit('clue_history_update', complete_status['clue_history'])
        emit('guess_history_update', complete_status['guess_history'])
        
        emit('message', {'type': 'info', 'text': 'Status updated'})
    else:
        emit('message', {'type': 'error', 'text': 'ROS2 not connected'})

@socketio.on('request_new_roles')
def handle_request_new_roles():
    """Handle request to reassign roles"""
    global ros_node
    
    if ros_node:
        # Clear logged players so we log them again
        if hasattr(ros_node, '_logged_players'):
            ros_node._logged_players.clear()
        
        # This will trigger a new role assignment in the orchestrator
        ros_node.start_new_game()
        emit('message', {'type': 'info', 'text': 'New role assignment requested'})
    else:
        emit('message', {'type': 'error', 'text': 'ROS2 not connected'})

def main():
    """Main function to start web server and ROS2 node"""
    global ros_node, ros_thread
    
    # Setup logging
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)
    
    try:
        # Initialize ROS2
        rclpy.init()
        logger.info("🤖 ROS2 initialized")
        
        # Create ROS2 node
        ros_node = EnhancedFourPlayerWebBridgeNode(socketio)
        logger.info("✅ Enhanced 4-Player Web Bridge Node created")
        
        # Start ROS2 spinning in separate thread
        ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
        ros_thread.start()
        logger.info("🔄 ROS2 spinning thread started")
        
        # Start Flask-SocketIO server
        logger.info("🌐 Starting enhanced 4-player web server...")
        logger.info("📱 Frontend: http://localhost:8080")
        logger.info("🔧 Admin: http://localhost:8080/admin")
        logger.info("👥 Players: http://localhost:8080/players")
        logger.info("📊 API: http://localhost:8080/api/status")
        
        socketio.run(app, host='0.0.0.0', port=8080, debug=False)
        
    except KeyboardInterrupt:
        logger.info("🛑 Web server stopped by user")
    except Exception as e:
        logger.error(f"❌ Error starting web server: {e}")
    finally:
        # Cleanup
        if ros_node:
            ros_node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()
        
        logger.info("🧹 Cleanup complete")

if __name__ == '__main__':
    main()