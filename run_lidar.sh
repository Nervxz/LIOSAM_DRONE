#!/bin/bash

# Convenience script for launching SICK MultiScan 136 LiDAR
# This script launches the SICK scanner with the correct network configuration

echo "🚀 Starting SICK MultiScan 136 LiDAR..."
echo "📡 LiDAR IP: 192.168.31.240"
echo "📨 UDP Receiver IP: 192.168.31.146"
echo ""

# Source the workspace
source install/setup.bash

# Launch the SICK scanner
ros2 launch sick_scan_xd sick_multiscan.launch.py \
  hostname:=192.168.31.240 \
  udp_receiver_ip:=192.168.31.146

echo "✅ SICK LiDAR launch completed" 