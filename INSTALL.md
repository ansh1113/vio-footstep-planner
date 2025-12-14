# Installation and Verification Guide

## Quick Start

### 1. Install Dependencies

```bash
# Create virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install requirements
pip install -r requirements.txt

# Install package
pip install -e .
```

### 2. Verify Installation

```bash
# Test imports
python -c "from vio_footstep_planner import VIONavigator, FootstepPlanner; print('✓ Installation successful')"

# Run tests
pytest tests/

# Run demo
python scripts/demo_quick.py
```

### 3. Expected Output

**Import Test:**
```
✓ Installation successful
```

**Test Suite:**
```
================================================= test session starts ==================================================
...
================================================== 21 passed in 0.80s ==================================================
```

**Demo:**
```
============================================================
VIO Footstep Planner - Quick Demo
============================================================
...
✓ Demo complete!
```

## Troubleshooting

### Common Issues

**ImportError: No module named 'cv2'**
```bash
pip install opencv-python
```

**ImportError: No module named 'numpy'**
```bash
pip install numpy scipy
```

**Tests fail with import errors**
```bash
# Make sure you installed in development mode
pip install -e .
```

## System Requirements

- Python 3.8 or higher
- pip 20.0 or higher
- 2GB RAM minimum
- Linux, macOS, or Windows

## Dependencies

- numpy >= 1.21.0
- scipy >= 1.7.0
- opencv-python >= 4.5.0
- pyyaml >= 5.4.0

## Next Steps

1. Check out the [README.md](README.md) for project overview
2. Read [DEVELOPER_GUIDE.md](DEVELOPER_GUIDE.md) for architecture details
3. See [CONTRIBUTING.md](CONTRIBUTING.md) for contribution guidelines
4. Try the examples in `scripts/` directory

## Getting Help

- GitHub Issues: https://github.com/ansh1113/vio-footstep-planner/issues
- Email: anshbhansali5@gmail.com
