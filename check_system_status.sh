#!/bin/bash

# LIO-SAM System Status Check Script
# This script verifies that all components are working correctly

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[⚠]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

print_status "========================================"
print_status "LIO-SAM System Status Check"
print_status "========================================"

# Check if ROS2 is running
if [[ -z "$ROS_DISTRO" ]]; then
    print_error "ROS2 is not sourced"
    exit 1
else
    print_success "ROS2 environment: $ROS_DISTRO"
fi

# Check LiDAR data
print_status "Checking LiDAR data..."
timeout 5 ros2 topic echo /cloud_all_fields_fullframe --once > /dev/null 2>&1
if [[ $? -eq 0 ]]; then
    print_success "LiDAR publishing data on /cloud_all_fields_fullframe"
    LIDAR_FRAME=$(timeout 3 ros2 topic echo /cloud_all_fields_fullframe --once | grep frame_id | cut -d'"' -f2)
    print_status "  Frame: $LIDAR_FRAME"
else
    print_error "LiDAR data not available"
fi

# Check IMU data
print_status "Checking IMU data..."
timeout 5 ros2 topic echo /imu/data --once > /dev/null 2>&1
if [[ $? -eq 0 ]]; then
    print_success "IMU publishing data on /imu/data"
    IMU_FRAME=$(timeout 3 ros2 topic echo /imu/data --once | grep frame_id | cut -d'"' -f2)
    print_status "  Frame: $IMU_FRAME"
else
    print_error "IMU data not available"
fi

# Check coordinate frames
print_status "Checking coordinate frame connections..."
ros2 run tf2_ros tf2_echo world imu_link > /dev/null 2>&1
if [[ $? -eq 0 ]]; then
    print_success "Static transform world → imu_link is active"
else
    print_error "Missing static transform between world and imu_link"
fi

# Check LIO-SAM topics
print_status "Checking LIO-SAM status..."
LIOSAM_TOPICS=$(ros2 topic list | grep lio_sam | wc -l)
if [[ $LIOSAM_TOPICS -gt 10 ]]; then
    print_success "LIO-SAM is running ($LIOSAM_TOPICS topics active)"
else
    print_error "LIO-SAM may not be running properly ($LIOSAM_TOPICS topics)"
fi

# Check odometry
print_status "Checking LIO-SAM odometry..."
timeout 5 ros2 topic echo /lio_sam/mapping/odometry --once > /dev/null 2>&1
if [[ $? -eq 0 ]]; then
    print_success "LIO-SAM odometry is active"
    # Get current position
    POS=$(timeout 3 ros2 topic echo /lio_sam/mapping/odometry --once | grep -A3 "position:" | tail -3 | tr '\n' ' ')
    print_status "  Current position: $POS"
else
    print_error "LIO-SAM odometry not available"
fi

# Check all expected topics
print_status "Topic summary:"
echo "  LiDAR topics:"
ros2 topic list | grep -E "(cloud|scan)" | sed 's/^/    /'
echo "  IMU topics:"
ros2 topic list | grep imu | sed 's/^/    /'
echo "  LIO-SAM topics:"
ros2 topic list | grep lio_sam | head -5 | sed 's/^/    /'
if [[ $(ros2 topic list | grep lio_sam | wc -l) -gt 5 ]]; then
    echo "    ... and $(( $(ros2 topic list | grep lio_sam | wc -l) - 5 )) more"
fi

print_status "========================================"
print_status "System Status Summary:"

# Overall health check
ERRORS=0
timeout 3 ros2 topic echo /cloud_all_fields_fullframe --once > /dev/null 2>&1 || ERRORS=$((ERRORS+1))
timeout 3 ros2 topic echo /imu/data --once > /dev/null 2>&1 || ERRORS=$((ERRORS+1))
timeout 3 ros2 topic echo /lio_sam/mapping/odometry --once > /dev/null 2>&1 || ERRORS=$((ERRORS+1))
ros2 run tf2_ros tf2_echo world imu_link > /dev/null 2>&1 || ERRORS=$((ERRORS+1))

if [[ $ERRORS -eq 0 ]]; then
    print_success "All systems are working correctly!"
    print_status "Your LIO-SAM system should be mapping properly."
    print_status "Move your sensors to see real-time mapping in RViz."
elif [[ $ERRORS -eq 1 ]]; then
    print_warning "System mostly working with 1 issue detected."
elif [[ $ERRORS -eq 2 ]]; then
    print_warning "System partially working with 2 issues detected."
else
    print_error "Multiple issues detected ($ERRORS/4). System may not work properly."
fi

print_status "========================================" 