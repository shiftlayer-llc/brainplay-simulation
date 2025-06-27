#!/bin/bash
set -e

echo "🤖 Starting ROS2 container..."

# Source ROS2 setup
source /opt/ros/humble/setup.bash
echo "✅ ROS2 Humble sourced"

# Check if workspace exists and has source code
if [ -d "/ros2_ws/src" ] && [ "$(ls -A /ros2_ws/src)" ]; then
    echo "📁 Source code found, building workspace..."
    
    cd /ros2_ws
    
    # Initialize rosdep if needed
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        rosdep update
    fi
    
    # Install dependencies (skip problematic ones)
    rosdep install --from-paths src --ignore-src -r -y \
        --skip-keys='python3-requests python3-numpy' || echo "⚠️ Some dependencies may be missing"
    
    # Build the workspace
    colcon build --packages-select codenames_game
    
    if [ $? -eq 0 ]; then
        echo "✅ Workspace built successfully"
        # Source the workspace
        source /ros2_ws/install/setup.bash
        echo "✅ Workspace sourced"
        
        # List available executables for debugging
        echo "🔍 Available executables:"
        find /ros2_ws/install -name "*_node" -type f 2>/dev/null || echo "  No *_node executables found"
        find /ros2_ws/install -name "*.py" -type f | head -5 || echo "  No Python files found"
        
        # Check if our package is properly installed
        if [ -d "/ros2_ws/install/codenames_game" ]; then
            echo "✅ Package codenames_game installed"
            ls -la /ros2_ws/install/codenames_game/lib/codenames_game/ 2>/dev/null || echo "  No executables in lib directory"
        else
            echo "❌ Package codenames_game not found in install directory"
        fi
    else
        echo "❌ Workspace build failed"
        exit 1
    fi
else
    echo "⚠️ No source code found in workspace"
fi

# Simple ROS2 checks
echo "🔍 ROS2 Status Check:"
echo "   ROS_DISTRO: ${ROS_DISTRO:-NOT_SET}"
echo "   ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-NOT_SET}"
echo "   NODE_NAME: ${NODE_NAME:-NOT_SET}"

# Check if ROS2 command is available
if command -v ros2 &> /dev/null; then
    echo "✅ ros2 command available"
    
    # Quick daemon check (non-blocking)
    if timeout 2 ros2 daemon status &> /dev/null; then
        echo "✅ ROS2 daemon responsive"
    else
        echo "⚠️ ROS2 daemon starting..."
    fi
    
    # Check if our package executables are available
    echo "🔍 Checking executable availability:"
    for executable in orchestrator_node spymaster_node finder_node isaac_bridge_node; do
        if ros2 pkg executables codenames_game | grep -q "$executable"; then
            echo "  ✅ $executable found"
        else
            echo "  ❌ $executable not found"
        fi
    done
else
    echo "❌ ros2 command not found"
fi

echo "🚀 Starting node..."

# Execute command
exec "$@"