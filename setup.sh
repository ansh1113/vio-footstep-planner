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

# Check if in virtual environment
if [ -z "$VIRTUAL_ENV" ]; then
    echo ""
    echo "⚠️  Not running in a virtual environment."
    echo "   It's recommended to use a virtual environment:"
    echo "   python3 -m venv venv && source venv/bin/activate"
    echo ""
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Setup cancelled."
        exit 1
    fi
fi

echo ""
echo "Installing package..."
if [ "$DEV_MODE" = true ]; then
    pip install --upgrade-strategy eager -e ".[dev]"
else
    pip install --upgrade-strategy eager -e .
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
