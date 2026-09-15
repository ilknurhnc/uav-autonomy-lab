#!/bin/zsh

PX4_DIR="$HOME/PX4-Autopilot"

echo "======================================"
echo " Stopping UAV Autonomy Lab"
echo "======================================"

echo "[1/2] Stopping simulation processes..."

pkill -f "camera_viewer" 2>/dev/null
pkill -f "lidar_viewer" 2>/dev/null
pkill -f "$PX4_DIR/build/px4_sitl_default/bin/px4" 2>/dev/null
pkill -f "gz sim" 2>/dev/null

sleep 2

echo "[2/2] Closing UAV terminal windows..."

osascript <<'EOF'
tell application "Terminal"
    set windowList to every window

    repeat with w in windowList
        try
            set windowName to name of w

            if windowName contains "UAV-PX4" or ¬
               windowName contains "UAV-GUI" or ¬
               windowName contains "UAV-CAMERA" or ¬
               windowName contains "UAV-LIDAR" then

                close w
            end if
        end try
    end repeat
end tell
EOF

echo ""
echo "======================================"
echo " Simulation stopped"
echo "======================================"