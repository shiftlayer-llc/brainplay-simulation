#!/usr/bin/env python3
"""
Web Server for 4-Player Codenames Robotics Game Frontend
Flask + SocketIO server that bridges ROS2 and web interface.
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

class FourPlayerWebBridgeNode(Node):
    def __init__(self, socketio):
        super().__init__('web_bridge_node')
        
        self.socketio = socketio
        self.logger = self.get_logger()
        self.logger.info("🌐 4-Player Web Bridge Node starting...")
        
        # Game state
        self.current_board_state = None
        self.game_status = {}
        self.player_statuses = {}
        self.role_assignments = {}
        
        # Publishers (to game)
        self.start_game_pub = self.create_publisher(String, '/game/start', 10)
        
        # Subscribers (from game)
        self.board_state_sub = self.create_subscription(
            String, '/game/board_state', self.handle_board_state, 10
        )
        self.game_status_sub = self.create_subscription(
            String, '/game/status', self.handle_game_status, 10
        )
        
        # Subscribe to all player status topics
        for i in range(1, 5):
            player_topic = f'/game/player_{i}_status'
            self.create_subscription(
                String, player_topic, 
                lambda msg, player_id=f'player_{i}': self.handle_player_status(msg, player_id), 
                10
            )
        
        # Optional: Isaac bridge status
        self.isaac_status_sub = self.create_subscription(
            String, '/isaac/bridge_status', self.handle_isaac_status, 10
        )
        
        self.logger.info("✅ 4-Player Web Bridge Node initialized")
    
    def handle_board_state(self, msg: String):
        """Handle board state updates from game"""
        try:
            self.current_board_state = json.loads(msg.data)
            
            # Extract role assignments if present
            if 'role_assignments' in self.current_board_state:
                self.role_assignments = self.current_board_state['role_assignments']
            
            # Emit to all connected web clients
            self.socketio.emit('board_update', self.current_board_state)
            
            self.logger.info("📤 Sent board state to web clients")
            
        except Exception as e:
            self.logger.error(f"❌ Error handling board state: {e}")
    
    def handle_game_status(self, msg: String):
        """Handle game status updates"""
        try:
            status = json.loads(msg.data)
            self.game_status = status
            
            # Handle role assignments announcement
            if status.get('type') == 'role_assignments':
                self.role_assignments = status.get('assignments', {})
                self.socketio.emit('role_assignments', self.role_assignments)
                self.logger.info("📤 Sent role assignments to web clients")
            
            # Emit to web clients
            self.socketio.emit('game_status', status)
            
        except Exception as e:
            self.logger.error(f"❌ Error handling game status: {e}")
    
    def handle_player_status(self, msg: String, player_id: str):
        """Handle individual player status updates"""
        try:
            status = json.loads(msg.data)
            self.player_statuses[player_id] = status
            
            # Emit to web clients
            self.socketio.emit('player_status', {
                'player_id': player_id,
                'status': status
            })
            
        except Exception as e:
            self.logger.error(f"❌ Error handling player status for {player_id}: {e}")
    
    def handle_isaac_status(self, msg: String):
        """Handle Isaac bridge status"""
        try:
            status = json.loads(msg.data)
            
            # Emit to web clients
            self.socketio.emit('isaac_status', status)
            
        except Exception as e:
            self.logger.error(f"❌ Error handling Isaac status: {e}")
    
    def start_new_game(self):
        """Start a new game"""
        msg = String()
        msg.data = json.dumps({"action": "start_game"})
        self.start_game_pub.publish(msg)
        
        self.logger.info("🎮 Sent start game command")

def ros_spin_thread():
    """Run ROS2 spinning in separate thread"""
    global ros_node
    try:
        rclpy.spin(ros_node)
    except Exception as e:
        logging.error(f"❌ ROS2 spinning error: {e}")

# Create Flask app
app = Flask(__name__)
app.config['SECRET_KEY'] = 'codenames_4player_secret_key'
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
        return jsonify({
            'board_state': ros_node.current_board_state,
            'game_status': ros_node.game_status,
            'player_statuses': ros_node.player_statuses,
            'role_assignments': ros_node.role_assignments,
            'ros_connected': True
        })
    else:
        return jsonify({
            'board_state': None,
            'game_status': {},
            'player_statuses': {},
            'role_assignments': {},
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
        if ros_node.current_board_state:
            emit('board_update', ros_node.current_board_state)
        
        if ros_node.game_status:
            emit('game_status', ros_node.game_status)
        
        if ros_node.role_assignments:
            emit('role_assignments', ros_node.role_assignments)
        
        for player_id, status in ros_node.player_statuses.items():
            emit('player_status', {
                'player_id': player_id,
                'status': status
            })

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
        # Send all current status info
        if ros_node.current_board_state:
            emit('board_update', ros_node.current_board_state)
        
        if ros_node.game_status:
            emit('game_status', ros_node.game_status)
        
        if ros_node.role_assignments:
            emit('role_assignments', ros_node.role_assignments)
        
        for player_id, status in ros_node.player_statuses.items():
            emit('player_status', {
                'player_id': player_id,
                'status': status
            })
        
        emit('message', {'type': 'info', 'text': 'Status updated'})
    else:
        emit('message', {'type': 'error', 'text': 'ROS2 not connected'})

@socketio.on('request_new_roles')
def handle_request_new_roles():
    """Handle request to reassign roles"""
    global ros_node
    
    if ros_node:
        # This would trigger a new role assignment in the orchestrator
        ros_node.start_new_game()  # This will reassign roles
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
        ros_node = FourPlayerWebBridgeNode(socketio)
        logger.info("✅ 4-Player Web Bridge Node created")
        
        # Start ROS2 spinning in separate thread
        ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
        ros_thread.start()
        logger.info("🔄 ROS2 spinning thread started")
        
        # Start Flask-SocketIO server
        logger.info("🌐 Starting 4-player web server...")
        logger.info("📱 Frontend: http://localhost:8080")
        logger.info("🔧 Admin: http://localhost:8080/admin")
        logger.info("👥 Players: http://localhost:8080/players")
        
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