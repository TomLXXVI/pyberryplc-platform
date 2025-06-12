#!/bin/bash

# === CONFIGURATION ===
PROJECT_DIR="/shared/python-projects/pyberryplc-platform/demos/demo_stepper_02"
VENV_PATH="/shared/python-projects/pyberryplc-platform/.venv"
SCRIPT="motion_plc.py"

# === EXECUTION ===
cd "$PROJECT_DIR" || { echo "Directory not found"; exit 1; }

PYTHON="$VENV_PATH/bin/python"

if [ ! -x "$PYTHON" ]; then
    echo "Python not found in virtual environment: $PYTHON"
    exit 1
fi

echo "Execute script with virtual environment in sudo..."
sudo "$PYTHON" "$SCRIPT"
