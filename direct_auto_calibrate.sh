#!/bin/bash

# Direct Auto Calibrate Script
# Skips IMU detection, assumes IMU is already running

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "🤖 DIRECT AUTO CALIBRATE"
echo "======================================"
echo "🎯 Direct calibration (assumes IMU is running)"
echo "🔧 Fixes 'flying around' IMU issue"
echo "⚙️  Automatically applies calibration"
echo

# Check if ROS2 is sourced
if [[ -z "$ROS_DISTRO" ]]; then
    echo -e "${RED}❌ ROS2 is not sourced${NC}"
    echo "Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo -e "${GREEN}✅ Proceeding with calibration${NC}"
echo -e "${YELLOW}📌 Make sure your IMU is STATIONARY during analysis!${NC}"
echo

# Run auto calibration directly
cd /home/nerv/lio_ws
python3 auto_calibrate_and_apply.py

echo
echo -e "${GREEN}🎉 Auto calibration complete!${NC}"
echo "Your IMU should now be properly calibrated." 