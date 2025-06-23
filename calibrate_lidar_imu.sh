#!/bin/bash

# LiDAR-IMU Calibration Data Collection Script
# For SICK MultiScan 136 + WitMotion HWT905

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
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_status "========================================"
print_status "LiDAR-IMU Calibration Data Collection"
print_status "SICK MultiScan 136 + WitMotion HWT905"
print_status "========================================"

# Check if ROS2 is sourced
if [[ -z "$ROS_DISTRO" ]]; then
    print_error "ROS2 is not sourced. Please source your ROS2 installation first."
    exit 1
fi

# Source workspace
cd /home/nerv/lio_ws
source install/setup.bash

# Create calibration directory
CALIB_DIR="/home/nerv/lio_ws/calibration_data"
mkdir -p $CALIB_DIR
cd $CALIB_DIR

print_status "Starting sensors for calibration..."

# Start LiDAR
print_status "Step 1: Starting SICK MultiScan LiDAR..."
cd /home/nerv/lio_ws/src/LIOSAM_DRONE/sick_scan_ws

gnome-terminal --title="CALIB: SICK LiDAR" -- bash -c "
    source /opt/ros/humble/setup.bash
    source /home/nerv/lio_ws/install/setup.bash
    echo '=== CALIBRATION MODE: SICK LiDAR ==='
    echo 'LiDAR IP: 169.254.83.177'
    echo 'UDP Receiver: 169.254.148.106'
    ros2 launch sick_scan_xd sick_multiscan.launch.py hostname:=169.254.83.177 udp_receiver_ip:=169.254.148.106
    exec bash
" &

sleep 8

# Start IMU
print_status "Step 2: Starting WitMotion HWT905 IMU..."
cd /home/nerv/lio_ws/src/LIOSAM_DRONE/ros2_imu

gnome-terminal --title="CALIB: WitMotion IMU" -- bash -c "
    source /opt/ros/humble/setup.bash
    source /home/nerv/lio_ws/install/setup.bash
    echo '=== CALIBRATION MODE: WitMotion IMU ==='
    echo 'Publishing IMU data for calibration'
    ros2 launch witmotion_ros final_demo.launch.py
    exec bash
" &

sleep 5

# Check if sensors are ready
print_status "Step 3: Checking sensor availability..."

timeout 10 ros2 topic echo /cloud_all_fields_fullframe --once > /dev/null 2>&1
if [[ $? -eq 0 ]]; then
    print_success "✓ LiDAR data available"
else
    print_error "✗ LiDAR data not available - check connection"
    exit 1
fi

timeout 10 ros2 topic echo /imu/data --once > /dev/null 2>&1
if [[ $? -eq 0 ]]; then
    print_success "✓ IMU data available"
else
    print_error "✗ IMU data not available - check connection"
    exit 1
fi

print_status "========================================"
print_success "Sensors are ready for calibration!"
print_status "========================================"

print_status "CALIBRATION INSTRUCTIONS:"
print_warning "1. Keep LiDAR and IMU RIGIDLY mounted together"
print_warning "2. You will record data while moving the sensor assembly"
print_warning "3. Move in various orientations: pitch, roll, yaw"
print_warning "4. Include translational movements: forward, sideways, up/down"
print_warning "5. Record for about 60-120 seconds of motion"

print_status ""
print_status "MOVEMENT PATTERN:"
print_status "  • Start stationary for 10 seconds"
print_status "  • Slow rotations around X, Y, Z axes"
print_status "  • Figure-8 movements"
print_status "  • Linear motions in different directions"
print_status "  • End stationary for 10 seconds"

print_status ""
print_warning "IMPORTANT: Ensure the sensors are FIXED relative to each other!"
print_warning "Any relative movement will corrupt the calibration."

print_status ""
read -p "Press ENTER when you're ready to start recording..."

# Create unique filename with timestamp
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_FILE="lidar_imu_calib_${TIMESTAMP}"

print_success "Starting data recording..."
print_status "Recording file: ${BAG_FILE}.db3"
print_status ""
print_warning "Recording in 3..."
sleep 1
print_warning "Recording in 2..."
sleep 1
print_warning "Recording in 1..."
sleep 1
print_success "RECORDING NOW! Start moving the sensors!"

# Record the bag file
cd $CALIB_DIR
timeout 120 ros2 bag record -o $BAG_FILE /cloud_all_fields_fullframe /imu/data /tf /tf_static &
RECORD_PID=$!

print_status ""
print_status "Recording for up to 120 seconds..."
print_status "Press Ctrl+C to stop recording early"
print_status ""

# Monitor recording
while kill -0 $RECORD_PID 2>/dev/null; do
    sleep 5
    print_status "Recording... (Ctrl+C to stop)"
done

print_success "Recording complete!"
print_status "Calibration data saved to: $CALIB_DIR/${BAG_FILE}/"

print_status ""
print_status "========================================"
print_status "Next Steps:"
print_status "1. Check the recorded data quality"
print_status "2. Run calibration analysis"
print_status "3. Apply calibration to LIO-SAM"
print_status "========================================"

print_status "To analyze the data, run:"
print_status "  cd $CALIB_DIR"
print_status "  ros2 bag info ${BAG_FILE}"
print_status "  ros2 bag play ${BAG_FILE}" 