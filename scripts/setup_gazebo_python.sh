#!/bin/zsh

export GZ_IP=127.0.0.1

export PYTHONPATH="/opt/homebrew/Cellar/gz-transport13/13.6.0_2/lib/python3.13/site-packages:$PYTHONPATH"

export PYTHONPATH="/opt/homebrew/Cellar/gz-msgs10/10.4.0_2/lib/python3.13/site-packages:$PYTHONPATH"

export DYLD_LIBRARY_PATH="/opt/homebrew/opt/gz-transport13/lib:$DYLD_LIBRARY_PATH"

echo "Gazebo Python environment configured."