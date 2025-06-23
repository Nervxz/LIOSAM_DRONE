#!/bin/bash

# Complete Auto Calibrate Script
# Starts IMU, waits for it to be ready, then runs calibration

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "🚀 COMPLETE AUTO CALIBRATE"
echo "========================================"
echo "🎯 Starts IMU + runs calibration automatically"
echo "🔧 Fixes 'flying around' IMU issue"
echo

# Check if ROS2 is sourced
if [[ -z "$ROS_DISTRO" ]]; then
    echo -e "${RED}❌ ROS2 is not sourced${NC}"
    echo "Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo -e "${GREEN}✅ ROS2 environment ready${NC}"

# Step 1: Check if IMU is already running
echo -e "${BLUE}📊 Checking IMU status...${NC}"
timeout 2 ros2 topic echo /imu/data --once > /dev/null 2>&1
if [[ $? -eq 0 ]]; then
    echo -e "${GREEN}✅ IMU is already running${NC}"
else
    echo -e "${YELLOW}⚠️  IMU not running. Starting IMU...${NC}"
    
    # Start IMU in background
    cd /home/nerv/lio_ws/src/LIOSAM_DRONE/ros2_imu
    gnome-terminal --title="IMU for Calibration" -- bash -c "
        source /opt/ros/humble/setup.bash
        source /home/nerv/lio_ws/install/setup.bash
        echo '🔄 Starting IMU for calibration...'
        echo '📌 Keep this terminal open during calibration'
        ros2 launch witmotion_ros final_demo.launch.py
        exec bash
    " &
    
    echo -e "${BLUE}⏱️  Waiting for IMU to start...${NC}"
    
    # Wait for IMU to be ready
    for i in {1..20}; do
        sleep 1
        echo -n "."
        timeout 1 ros2 topic echo /imu/data --once > /dev/null 2>&1
        if [[ $? -eq 0 ]]; then
            echo
            echo -e "${GREEN}✅ IMU is now running${NC}"
            break
        fi
        
        if [[ $i -eq 20 ]]; then
            echo
            echo -e "${RED}❌ IMU failed to start after 20 seconds${NC}"
            echo "Please check your IMU connection and try again"
            exit 1
        fi
    done
fi

# Step 2: Run calibration
echo
echo -e "${BLUE}🤖 Starting calibration process...${NC}"
echo -e "${YELLOW}📌 Keep your IMU STATIONARY during analysis!${NC}"
echo

cd /home/nerv/lio_ws

# Ask user which calibration method they prefer
echo "🔽 Choose calibration method:"
echo "  1) Interactive (you approve each step)"
echo "  2) Automatic (fully automated)"
echo
read -p "Choice (1 or 2): " method

case $method in
    1)
        echo -e "${BLUE}🎛️  Starting interactive calibration...${NC}"
        python3 interactive_auto_calibrate.py
        ;;
    2)
        echo -e "${BLUE}🤖 Starting automatic calibration...${NC}"
        python3 auto_calibrate_and_apply.py
        ;;
    *)
        echo -e "${YELLOW}⚠️  Invalid choice, using interactive mode${NC}"
        python3 interactive_auto_calibrate.py
        ;;
esac

echo
echo -e "${GREEN}🎉 Complete calibration process finished!${NC}"
echo "Your IMU should now be properly calibrated." 