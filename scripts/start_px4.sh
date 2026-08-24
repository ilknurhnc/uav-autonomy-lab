#!/bin/zsh

set -e

PX4_DIR="$HOME/PX4-Autopilot"

cd "$PX4_DIR"

source .venv/bin/activate

export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PX4_DIR/build/px4_sitl_default/src/modules/simulation/gz_plugins"
export GZ_IP=127.0.0.1

echo "Starting PX4 SITL with X500..."
PX4_GZ_STANDALONE=1 make px4_sitl gz_x500