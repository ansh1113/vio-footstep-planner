#!/usr/bin/env bash
# Quick setup script for vio-footstep-planner
# Usage: ./setup.sh [--dev]

set -e

echo "==========================================="
echo "VIO Footstep Planner - Setup"
echo "==========================================="
echo ""

# Check Python version
PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
echo "✓ Python version: $PYTHON_VERSION"

# Check if we should install dev dependencies
DEV_MODE=false
if [ "$1" == "--dev" ]; then
    DEV_MODE=true
    echo "✓ Installing with development dependencies"
fi

echo ""
echo "Installing package..."
if [ "$DEV_MODE" = true ]; then
    pip install -e ".[dev]"
else
    pip install -e .
fi

echo ""
echo "==========================================="
echo "Setup complete!"
echo "==========================================="
echo ""
echo "Quick start:"
echo "  - Run tests:         make test"
echo "  - Run all checks:    make check"
echo "  - View all commands: make help"
echo ""
echo "Try it out:"
echo "  python3 -c 'from vio_footstep_planner import VIONavigator; print(\"Success!\")'"
echo ""
