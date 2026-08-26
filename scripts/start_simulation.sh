#!/bin/zsh

PROJECT_DIR="$HOME/Desktop/uav-autonomy-lab"
PX4_DIR="$HOME/PX4-Autopilot"

SOURCE_WORLD="$PROJECT_DIR/simulation/worlds/vision_test.sdf"
PX4_WORLD="$PX4_DIR/Tools/simulation/gz/worlds/vision_test.sdf"

echo "======================================"
echo " UAV Autonomy Lab"
echo "======================================"

echo "[1/4] Cleaning old simulation processes..."

pkill -f "camera_viewer" 2>/dev/null
pkill -f "$PX4_DIR/build/px4_sitl_default/bin/px4" 2>/dev/null
pkill -f "gz sim" 2>/dev/null

sleep 2

echo "[2/4] Syncing vision_test.sdf..."

cp "$SOURCE_WORLD" "$PX4_WORLD"

if [ $? -ne 0 ]; then
    echo "ERROR: Could not copy vision_test.sdf"
    exit 1
fi

echo "[3/4] Starting PX4 + Gazebo..."

osascript <<EOF
tell application "Terminal"

    set px4Tab to do script "cd '$PX4_DIR'; source .venv/bin/activate; export GZ_IP=127.0.0.1; echo 'Starting PX4 + Gazebo...'; PX4_GZ_WORLD=vision_test make px4_sitl gz_x500_mono_cam"
    set custom title of px4Tab to "UAV-PX4"

    delay 7

    set guiTab to do script "export GZ_IP=127.0.0.1; echo 'Opening Gazebo GUI...'; gz sim -g"
    set custom title of guiTab to "UAV-GUI"

end tell
EOF

if [ $? -ne 0 ]; then
    echo "ERROR: Could not start PX4/Gazebo terminals."
    exit 1
fi

echo "[4/4] Starting camera watcher..."

osascript <<EOF
tell application "Terminal"

    set cameraTab to do script "cd '$PROJECT_DIR'; source .venv/bin/activate; export GZ_IP=127.0.0.1; export PYTHONPATH='/opt/homebrew/Cellar/gz-transport13/13.6.0_2/lib/python3.13/site-packages:/opt/homebrew/Cellar/gz-msgs10/10.4.0_2/lib/python3.13/site-packages:\$PYTHONPATH'; export DYLD_LIBRARY_PATH='/opt/homebrew/opt/gz-transport13/lib:\$DYLD_LIBRARY_PATH'; echo 'Waiting for vision_test camera...'; until GZ_IP=127.0.0.1 gz topic -l 2>/dev/null | grep -q '/world/vision_test/model/x500_mono_cam_0/link/camera_link/sensor/camera/image'; do sleep 1; done; echo 'Camera detected!'; python -m companion.vision.camera_viewer"
    set custom title of cameraTab to "UAV-CAMERA"

end tell
EOF

if [ $? -ne 0 ]; then
    echo "ERROR: Could not start camera terminal."
    exit 1
fi

echo ""
echo "======================================"
echo " Simulation launch started"
echo "======================================"
echo ""
echo "World   : vision_test"
echo "Vehicle : x500_mono_cam"
echo "Camera  : starts automatically when ready"
echo ""
echo "To stop:"
echo "./scripts/stop_simulation.sh"