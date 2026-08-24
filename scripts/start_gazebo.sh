#!/bin/zsh

set -e

PX4_DIR="$HOME/PX4-Autopilot"

cd "$PX4_DIR"

export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PX4_DIR/build/px4_sitl_default/src/modules/simulation/gz_plugins"
export GZ_IP=127.0.0.1

echo "Starting Gazebo server..."
gz sim -r -s Tools/simulation/gz/worlds/default.sdf