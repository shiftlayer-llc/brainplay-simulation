#!/bin/bash

# Codenames Game Startup Script

set -e

echo "🎮 Starting Codenames Robotics Game..."

# Check if .env file exists
if [ ! -f .env ]; then
    echo "⚠️  No .env file found. Creating from template..."
    if [ -f .env.template ]; then
        cp .env.template .env
        echo "✅ Created .env file from template"
        echo "📝 Please edit .env with your API keys before running again"
        exit 1
    else
        echo "❌ No .env.template found. Please create .env file manually"
        exit 1
    fi
fi

# Check Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker first."
    exit 1
fi

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --build      Force rebuild of containers"
    echo "  --isaac      Include Isaac Sim bridge"
    echo "  --logs       Show container logs"
    echo "  --status     Show container status"
    echo "  --stop       Stop all containers"
    echo "  --clean      Stop and remove all containers"
    echo "  --test       Run basic connectivity tests"
    echo "  --help       Show this help message"
}

# Function to check container health
check_health() {
    echo "🔍 Checking container health..."
    
    # Check if containers are running
    RUNNING=$(docker-compose ps --services --filter "status=running" | wc -l)
    TOTAL=$(docker-compose ps --services | wc -l)
    
    echo "📊 Containers running: $RUNNING/$TOTAL"
    
    if [ "$RUNNING" -eq "$TOTAL" ]; then
        echo "✅ All containers are healthy"
        
        # Test ROS2 connectivity
        echo "🔍 Testing ROS2 connectivity..."
        timeout 10 docker-compose exec -T orchestrator ros2 node list > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "✅ ROS2 nodes are communicating"
        else
            echo "⚠️  ROS2 connectivity test failed"
        fi
        
        # Test web interface
        echo "🔍 Testing web interface..."
        if curl -s http://localhost:8080 > /dev/null 2>&1; then
            echo "✅ Web interface is accessible"
        else
            echo "⚠️  Web interface test failed"
        fi
    else
        echo "❌ Some containers are not running"
        docker-compose ps
    fi
}

# Function to run tests
run_tests() {
    echo "🧪 Running system tests..."
    
    # Wait for services to be ready
    echo "⏳ Waiting for services to start..."
    sleep 10
    
    # Test API endpoints
    echo "🔍 Testing API endpoints..."
    if curl -s http://localhost:8080/api/status | jq . > /dev/null 2>&1; then
        echo "✅ API endpoints responding"
    else
        echo "❌ API test failed"
    fi
    
    # Test ROS2 topics
    echo "🔍 Testing ROS2 topics..."
    TOPICS=$(timeout 5 docker-compose exec -T orchestrator ros2 topic list 2>/dev/null | grep "/game/" | wc -l)
    if [ "$TOPICS" -gt 0 ]; then
        echo "✅ ROS2 topics active ($TOPICS found)"
    else
        echo "❌ No game topics found"
    fi
    
    check_health
}

# Parse command line arguments
BUILD_FLAG=""
ISAAC_PROFILE=""
COMMAND="up -d"

while [[ $# -gt 0 ]]; do
    case $1 in
        --build)
            BUILD_FLAG="--build"
            shift
            ;;
        --isaac)
            ISAAC_PROFILE="--profile isaac"
            echo "🤖 Including Isaac Sim bridge..."
            shift
            ;;
        --logs)
            COMMAND="logs -f"
            shift
            ;;
        --status)
            docker-compose ps
            check_health
            exit 0
            ;;
        --test)
            run_tests
            exit 0
            ;;
        --stop)
            echo "🛑 Stopping Codenames containers..."
            docker-compose down
            exit 0
            ;;
        --clean)
            echo "🧹 Cleaning up containers and volumes..."
            docker-compose down -v --remove-orphans
            docker system prune -f
            exit 0
            ;;
        --help)
            show_usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Create necessary directories
echo "📁 Creating necessary directories..."
mkdir -p models/bert_similarity
mkdir -p frontend/templates
mkdir -p frontend/static/{css,js,images}

# Build and start containers
echo "🚀 Starting containers..."
docker-compose $ISAAC_PROFILE $COMMAND $BUILD_FLAG

if [ "$COMMAND" = "up -d" ]; then
    echo ""
    echo "⏳ Waiting for services to start..."
    sleep 5
    
    # Check if services started successfully
    check_health
    
    echo ""
    echo "✅ Codenames Game is starting!"
    echo ""
    echo "🌐 Frontend:    http://localhost:8080"
    echo "⚙️  Admin Panel: http://localhost:8080/admin"
    echo "📡 API Status:  http://localhost:8080/api/status"
    echo ""
    echo "📊 Check container status:"
    echo "   ./start.sh --status"
    echo ""
    echo "📋 View logs:"
    echo "   ./start.sh --logs"
    echo "   docker-compose logs -f [service_name]"
    echo ""
    echo "🧪 Run tests:"
    echo "   ./start.sh --test"
    echo ""
    echo "🛑 Stop all containers:"
    echo "   ./start.sh --stop"
    echo ""
    echo "🧹 Clean everything:"
    echo "   ./start.sh --clean"
fi