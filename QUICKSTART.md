# Quick Start Guide

Get up and running with vio-footstep-planner in minutes!

## Prerequisites

- Python 3.8 or higher
- pip

## One-Command Setup

```bash
./setup.sh --dev
```

This will install the package with all development dependencies.

## Manual Setup

If you prefer to install manually:

```bash
# Install package only
pip install -e .

# Or with dev dependencies for testing and development
pip install -e ".[dev]"
```

## Verify Installation

```bash
# Quick test
python3 -c "from vio_footstep_planner import VIONavigator; print('Success!')"

# Run all tests
make test

# Or use pytest directly
pytest tests/ -v
```

## Development Workflow

Common commands (see `make help` for full list):

```bash
make install-dev   # Install with dev dependencies
make test          # Run tests
make lint          # Check code style
make format        # Auto-format code
make check         # Run all checks
make clean         # Clean build artifacts
```

## Quick Example

```python
from vio_footstep_planner import VIONavigator, FootstepPlanner

# Initialize VIO navigator
navigator = VIONavigator(drift_correction=True)

# Initialize footstep planner
planner = FootstepPlanner(
    robot_model="spot",
    config={'max_step_length': 0.4}
)

# Start VIO estimation
navigator.start()

# Plan a path
import numpy as np
current_pose = np.array([0, 0, 0, 0, 0, 0])
goal = [5.0, 3.0, 0.0]
footsteps = planner.plan(current_pose, goal)

if footsteps:
    print(f"Planned {len(footsteps)} footsteps")
```

## Running Examples

```bash
# Example navigation script
python3 scripts/example_navigation.py

# Quick demo
python3 scripts/demo_quick.py

# Complete demo
python3 scripts/demo_complete.py
```

## Troubleshooting

### Import Errors

If you get import errors, ensure the package is installed:

```bash
pip install -e .
```

### Test Failures

If tests fail, install dev dependencies:

```bash
pip install -e ".[dev]"
```

### Missing Dependencies

Install all required dependencies:

```bash
pip install -e ".[dev]"
```

For production use without dev dependencies:

```bash
pip install -e .
```

## Next Steps

- Read the full [README.md](README.md) for detailed documentation
- Check [DEVELOPER_GUIDE.md](DEVELOPER_GUIDE.md) for development practices
- See [INSTALL.md](INSTALL.md) for advanced installation options
- View [CONTRIBUTING.md](CONTRIBUTING.md) to contribute

## Getting Help

- Open an [issue](https://github.com/ansh1113/vio-footstep-planner/issues)
- Check existing documentation
- Contact: anshbhansali5@gmail.com
