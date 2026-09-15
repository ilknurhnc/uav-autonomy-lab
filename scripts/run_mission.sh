#!/bin/zsh

PROJECT_DIR="$HOME/Desktop/uav-autonomy-lab"

cd "$PROJECT_DIR" || exit 1

source .venv/bin/activate

export GZ_IP=127.0.0.1
export PYTHONPATH="/opt/homebrew/lib/python3.13/site-packages:$PYTHONPATH"
export DYLD_LIBRARY_PATH="/opt/homebrew/opt/gz-transport13/lib:/opt/homebrew/lib:$DYLD_LIBRARY_PATH"

echo "======================================"
echo " UAV Autonomous Search Mission"
echo "======================================"
echo ""
echo "Environment ready."
echo "Starting autonomous search..."
echo ""

python -m companion.missions.autonomous_search