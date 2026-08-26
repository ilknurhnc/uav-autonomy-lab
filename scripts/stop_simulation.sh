#!/bin/zsh

PX4_DIR="$HOME/PX4-Autopilot"

echo "======================================"
echo " Stopping UAV Autonomy Lab"
echo "======================================"

pkill -f "camera_viewer" 2>/dev/null
pkill -f "$PX4_DIR/build/px4_sitl_default/bin/px4" 2>/dev/null
pkill -f "gz sim" 2>/dev/null

sleep 2

osascript <<'EOF'
tell application "Terminal"

    repeat with w in windows
        repeat with t in tabs of w
            try
                set tabTitle to custom title of t

                if tabTitle is "UAV-PX4" or ¬
                   tabTitle is "UAV-GUI" or ¬
                   tabTitle is "UAV-CAMERA" then

                    close t
                end if
            end try
        end repeat
    end repeat

end tell
EOF

echo "Simulation stopped."