# 🤖 Codenames Robotics Game
 
 ### **🏆 Key Innovations**

- **🎲 Dynamic Role Assignment**: Players randomly assigned to Red/Blue Spymaster/Operative each game
- **🤖 Multi-LLM Tournament**: GPT-4 vs Claude Sonnet head-to-head competition  
- **🎭 Personality-Driven AI**: Risk tolerance, creativity, and confidence affect gameplay
- **📊 Real-Time Monitoring**: Comprehensive web interface with live game tracking
- **🤝 ROS2 Distributed**: Scalable, robust multi-node architecture
- **🔬 Research Ready**: Perfect for AI model comparison and behavioral analysis

---

## 🏗️ **System Architecture**

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   🎮 Frontend   │    │  🎭 Players     │    │ 🤖 Isaac Sim   │
│                 │    │                 │    │   (Optional)    │
│ • Game View     │    │ • Alice (GPT-4) │    │                 │
│ • Admin Panel   │    │ • Bob (GPT-4)   │    │ • Robot Viz     │
│ • Player Stats  │    │ • Charlie(Claude│    │ • Speech/TTS    │
│                 │    │ • Diana (Claude)│    │ • Animations    │
└─────────┬───────┘    └─────────┬───────┘    └─────────┬───────┘
          │                      │                      │
          │              ┌───────▼───────┐             │
          │              │               │             │
          └─────────────▶│ 🎯 Orchestrator│◀────────────┘
                         │               │
                         │ • Role Manager│
                         │ • Game Master │
                         │ • Turn Logic  │
                         │ • Win Detection│
                         └───────────────┘
```

### **🔄 Tournament Flow**

1. **🚀 Startup**: 4 AI players announce readiness
2. **🎲 Role Assignment**: Random team/role distribution each game
3. **🎮 Gameplay**: Turn-based Codenames with LLM-powered decisions
4. **📊 Monitoring**: Real-time performance tracking and visualization
5. **🏁 Results**: Comprehensive game analysis and player statistics

---

## 🚀 **Quick Start**

### **Prerequisites**
- Linux Os
- Docker & Docker Compose 
- OpenAI API key
- Anthropic API key (optional, for Claude players)

### **1. Clone & Setup**
```bash
git clone <your-repo-url>
cd codenames_robotics
chmod +x start.sh
```

### **2. Configure API Keys**
```bash
cp .env.template .env
# Edit .env with your API keys
nano .env
```

### **3. Start Tournament**
```bash
# First time (with build)
./start.sh --build

# Subsequent runs
./start.sh

# Include Isaac Sim robots
./start.sh --isaac
```

### **4. Access Interfaces**
- **🎮 Game Interface**: http://localhost:8080
- **⚙️ Admin Panel**: http://localhost:8080/admin  
- **👥 Player Stats**: http://localhost:8080/players
- **📡 API Status**: http://localhost:8080/api/status

---

## 🎮 **Game Modes & Configuration**

### **Player Model Configuration**
Set different AI models for competitive variety:

```bash
# In .env file
PLAYER1_MODEL=gpt-4                    # Alice
PLAYER2_MODEL=gpt-4                    # Bob  
PLAYER3_MODEL=claude-3-5-sonnet-20241022  # Charlie
PLAYER4_MODEL=claude-3-5-sonnet-20241022  # Diana
```

### **AI Personality System**
Each player has unique behavioral traits:

```python
personality = {
    'risk_tolerance': 0.7,  # 0.0=conservative, 1.0=risky
    'creativity': 0.8,      # 0.0=basic, 1.0=creative  
    'confidence': 0.6       # 0.0=hesitant, 1.0=confident
}
```

### **Supported Models**
- **OpenAI**: `gpt-4`, `gpt-3.5-turbo`
- **Anthropic**: `claude-3-5-sonnet-20241022`, `claude-3-haiku-20240307`

---

## 🛠️ **Management Commands**

```bash
# 📊 Check system status
./start.sh --status

# 📋 View real-time logs  
./start.sh --logs

# 🧪 Run connectivity tests
./start.sh --test

# 🛑 Stop tournament
./start.sh --stop

# 🧹 Complete cleanup
./start.sh --clean

# 🔧 Force rebuild
./start.sh --build
```

---

## 📊 **Web Interface Guide**

### **🎮 Game View** (`/`)
- **Live Game Board**: 5x5 Codenames grid with real-time updates
- **Team Panels**: Red vs Blue with assigned players
- **Turn Tracking**: Current team, remaining cards, game state
- **Game Log**: History of clues, guesses, and results

### **⚙️ Admin Panel** (`/admin`)
- **System Metrics**: Container health, ROS2 connectivity
- **Player Monitoring**: Individual AI status and performance
- **Game Control**: Start/stop games, reassign roles
- **Performance Analytics**: Success rates, response times

### **👥 Player Details** (`/players`)
- **Team Assignments**: Current role distribution
- **AI Personalities**: Risk tolerance, creativity, confidence
- **Performance Stats**: Clues given, guesses made, accuracy
- **Model Information**: Which LLM each player is using

---

## 🤖 **Isaac Sim Integration**

### **Enable Robot Visualization**
```bash
# Start with Isaac Sim support
./start.sh --isaac

# Configure in .env
ISAAC_SIM_HOST=localhost
ISAAC_SIM_PORT=8211
ISAAC_ENABLED=true
```

### **Robot Features**
- **🎤 Speech Synthesis**: Robots announce clues and guesses
- **🤖 Gesture Control**: Pointing, thinking, celebration animations
- **🎬 Scene Management**: Dynamic board visualization
- **📊 Pose Tracking**: Real-time robot position monitoring

---

## 🔧 **Development Guide**

### **Project Structure**
```
codenames_robotics/
├── 🐳 Docker Files
│   ├── Dockerfile.base           # ROS2 + AI dependencies
│   ├── Dockerfile.frontend       # Web interface
│   ├── docker-compose.yml        # Complete system
│   └── entrypoint.sh            # Container initialization
│
├── 🤖 ROS2 Package: src/codenames_game/
│   ├── orchestrator_node.py      # 🎯 Game master & role manager
│   ├── player_node.py            # 🎭 Universal AI player
│   ├── isaac_bridge_node.py      # 🤖 Robot integration
│   ├── game_logic.py             # 🎲 Core Codenames rules
│   ├── llm_client.py             # 🧠 Multi-LLM interface
│   └── similarity_model.py       # 📊 Word relationship analysis
│
├── 🌐 Frontend/
│   ├── web_server.py             # Flask + SocketIO server
│   ├── templates/
│   │   ├── index.html            # Game interface
│   │   ├── admin.html            # Admin panel
│   │   └── players.html          # Player monitoring
│
├── 🚀 Launch Files/
│   ├── 4_player_launch.py        # Full tournament system
│   ├── game_launch.py            # Basic game mode
│   └── isaac_launch.py           # With robot simulation
│
└── ⚙️ Configuration/
    ├── .env.template             # Environment template
    ├── game_config.yaml          # Game rules & timing
    ├── llm_config.yaml           # AI model settings
    └── robot_config.yaml         # Isaac Sim configuration
```

### **Adding New AI Models**
1. **Update LLM Client** (`llm_client.py`):
```python
def _call_new_provider(self, prompt: str, model: str) -> str:
    # Implement new API integration
    pass
```

2. **Configure Model** (`.env`):
```bash
PLAYER1_MODEL=new-model-name
NEW_PROVIDER_API_KEY=your_key_here
```

### **Custom Personality Profiles**
Create specialized AI behaviors:
```python
# Conservative player
conservative = {
    'risk_tolerance': 0.2,
    'creativity': 0.4, 
    'confidence': 0.6
}

# Aggressive player  
aggressive = {
    'risk_tolerance': 0.9,
    'creativity': 0.8,
    'confidence': 0.9
}
```

---

## 📡 **API Reference**

### **REST Endpoints**
```http
GET  /api/status          # System status
GET  /api/players         # Player information  
POST /api/start_game      # Start new game
```

### **WebSocket Events**
```javascript
// Real-time game updates
socket.on('board_update', (data) => { ... });
socket.on('role_assignments', (assignments) => { ... });
socket.on('player_status', (status) => { ... });
socket.on('game_status', (status) => { ... });
```

### **ROS2 Topics**
```bash
# Game coordination
/game/board_state         # Game board updates
/game/role_assignments    # Player role distribution
/game/clue_request        # Spymaster clue requests
/game/clue_response       # Clue responses  
/game/guess_request       # Operative guess requests
/game/guess_response      # Guess responses

# Player management
/game/player_1_status     # Individual player status
/game/player_2_status     # (one topic per player)
/game/player_3_status     
/game/player_4_status     

# Isaac Sim integration
/isaac/robot_commands     # Robot control
/isaac/speech_commands    # Text-to-speech
/isaac/scene_updates      # 3D scene management
```


---

## 🔍 **Troubleshooting**

### **Common Issues**

**❌ "API key not found"**
```bash
# Check .env configuration
cat .env | grep API_KEY
# Restart containers after .env changes
./start.sh --stop && ./start.sh
```

**❌ "ROS2 nodes not communicating"**
```bash
# Check ROS2 connectivity
docker-compose exec orchestrator ros2 node list
docker-compose exec orchestrator ros2 topic list | grep game
```

**❌ "Web interface not loading"**
```bash
# Check frontend container
docker-compose logs frontend
# Verify port mapping
docker-compose ps
```

**❌ "Players not joining"**
```bash
# Check player container logs
docker-compose logs player1
# Verify environment variables
docker-compose exec player1 env | grep PLAYER
```

### **Debug Mode**
```bash
# Enable detailed logging
export LOG_LEVEL=DEBUG
./start.sh --build

# Access container shells
docker-compose exec orchestrator bash
docker-compose exec player1 bash
``` 
  