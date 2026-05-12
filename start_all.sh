#!/bin/bash
set -m
trap 'echo; echo "Stopping..."; kill 0; exit 0' INT TERM
source /opt/ros/humble/setup.bash
source ~/qbot-delivery/install/setup.bash 2>/dev/null
echo "[1/4] QUARC..."
quarc_run -r -t tcpip://localhost:17000 /home/nvidia/ros2/install/qbot_platform/share/qbot_platform/rt_models/qbot_platform_driver_physical.rt-linux_qbot_platform 2>&1 | sed 's/^/[QUARC] /' &
sleep 3
echo "[2/4] LiDAR..."
ros2 run qbot_platform lidar 2>&1 | sed 's/^/[LIDAR] /' &
sleep 1
echo "[3/4] Driver interface..."
ros2 run qbot_platform qbot_platform_driver_interface 2>&1 | sed 's/^/[DRIVER] /' &
sleep 2
echo "[4/4] Command (odom)..."
ros2 run qbot_platform command 2>&1 | sed 's/^/[CMD] /' &
sleep 2
if [ "$1" == "demo" ]; then
    echo "[+] Delivery state machine..."
    ros2 launch delivery_bot_bringup bringup.launch.py use_fake_robot:=false 2>&1 | sed 's/^/[DELIVERY] /' &
fi
echo "✅ Ready. Ctrl+C to stop all."
wait
