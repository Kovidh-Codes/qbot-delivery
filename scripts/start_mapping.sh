#!/bin/bash
cd ~/qbot-delivery
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

QUARC_MODEL=/home/nvidia/ros2/install/qbot_platform/share/qbot_platform/rt_models/qbot_platform_driver_physical.rt-linux_qbot_platform

cleanup() {
    echo ""
    echo "🛑 Stopping all..."
    pkill -P $$ 2>/dev/null
    pkill -9 -f odom_publisher 2>/dev/null
    pkill -9 -f lidar_mapper 2>/dev/null
    quarc_run -q -t tcpip://localhost:17000 qbot_platform_driver_physical 2>/dev/null
    exit 0
}
trap cleanup INT TERM

echo "🧹 Cleanup old processes..."
pkill -9 -f odom_publisher 2>/dev/null
pkill -9 -f lidar_mapper 2>/dev/null
quarc_run -q -t tcpip://localhost:17000 qbot_platform_driver_physical 2>/dev/null
rm ~/qbot-delivery/map.pgm 2>/dev/null
sleep 2

echo "[1/5] Starting QUARC..."
( printf '\n\n\ny\n1\n' | quarc_run -r -t tcpip://localhost:17000 "$QUARC_MODEL" -d /tmp -uri 'tcpip://%m:17001' 2>&1 | sed 's/^/[QUARC]  /' ) &
sleep 4

echo "[2/5] Starting LiDAR..."
ros2 run qbot_platform lidar 2>&1 | sed 's/^/[LIDAR]  /' &
sleep 2

echo "[3/5] Starting driver_interface..."
ros2 run qbot_platform qbot_platform_driver_interface 2>&1 | sed 's/^/[DRIVER] /' &
sleep 3

echo "[4/5] Starting odom_publisher..."
python3 ~/qbot-delivery/scripts/odom_publisher.py 2>&1 | sed 's/^/[ODOM]   /' &
sleep 2

echo "[5/5] Starting mapper..."
python3 ~/qbot-delivery/scripts/lidar_mapper.py 2>&1 | sed 's/^/[MAPPER] /' &

echo ""
echo "============================================="
echo "✅ All running. Open ONE more SSH for teleop:"
echo "  ros2 run delivery_bot_teleop wasd_teleop"
echo "Ctrl+C here to stop everything."
echo "============================================="
wait
