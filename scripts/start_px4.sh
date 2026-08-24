#!/bin/zsh

cd ~/PX4-Autopilot || exit 1

source .venv/bin/activate

export GZ_SIM_RESOURCE_PATH="$PWD/Tools/simulation/gz/models:$PWD/Tools/simulation/gz/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build/px4_sitl_default/src/modules/simulation/gz_plugins"
export GZ_IP=127.0.0.1

PX4_GZ_STANDALONE=1 make px4_sitl gz_x500