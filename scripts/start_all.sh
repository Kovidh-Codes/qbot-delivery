#!/bin/bash
# QBot Delivery — master startup script.
#
# Usage:
#   ./start_all.sh             # base nodes only (for manual driving + waypoint recording)
#   ./start_all.sh demo        # base + delivery state machine (for autonomous demo)
#
# Press Ctrl+C to stop everything.

set -m
trap 'echo; echo "[stop] killing all child processes..."; kill 0 2>/dev/null; exit 0' INT TERM

# --- Paths -----------------------------------------------------------
WORKSPACE="${WORKSPACE:-$HOME/qbot-delivery}"
QUARC_MODEL="${QUARC_MODEL:-/home/nvidia/ros2/install/qbot_platform/share/qbot_platform/rt_models/qbot_platform_driver_physical.rt-linux_qbot_platform}"

# --- Sources ---------------------------------------------------------
source /opt/ros/humble/setup.bash
if [ -f "$WORKSPACE/install/setup.bash" ]; then
  source "$WORKSPACE/install/setup.bash"
fi

# --- 1. QUARC driver model (low-level hardware) ----------------------
echo "[1/5] Starting QUARC driver model..."
quarc_run -r -t tcpip://localhost:17000 "$QUARC_MODEL" 2>&1 | sed 's/^/[QUARC]  /' &
sleep 3

# --- 2. LiDAR --------------------------------------------------------
echo "[2/5] Starting LiDAR..."
ros2 run qbot_platform lidar 2>&1 | sed 's/^/[LIDAR]  /' &
sleep 1

# --- 3. QUARC ↔ ROS2 bridge -----------------------------------------
echo "[3/5] Starting driver interface..."
ros2 run qbot_platform qbot_platform_driver_interface 2>&1 | sed 's/^/[DRIVER] /' &
sleep 2

# --- 4. Command (joystick + /odom + min enforcer) -------------------
echo "[4/5] Starting command node (odom + joystick)..."
ros2 run qbot_platform command 2>&1 | sed 's/^/[CMD]    /' &
sleep 2

# --- 5. Delivery brain (state machine + intent) ---------------------
if [ "$1" == "demo" ]; then
  echo "[5/5] Starting delivery state machine + intent..."
  ros2 launch delivery_bot_bringup bringup.launch.py use_fake_robot:=false 2>&1 | sed 's/^/[BOT]    /' &
  sleep 2
else
  echo "[5/5] Skipping state machine (manual mode). Run with 'demo' arg to enable."
fi

echo ""
echo "==================================================="
echo "  QBot Delivery is RUNNING."
echo "  Mode: $([ "$1" == "demo" ] && echo 'AUTONOMOUS (state machine on)' || echo 'MANUAL (state machine off)')"
echo "  Press Ctrl+C to stop everything."
echo "==================================================="
echo ""

wait
