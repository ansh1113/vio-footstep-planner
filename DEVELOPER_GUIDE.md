# Developer Guide

## Overview

This guide provides information for developers who want to contribute to or extend the VIO Footstep Planner project.

## Project Structure

```
vio-footstep-planner/
├── src/vio_footstep_planner/
│   ├── vio/                    # Visual-Inertial Odometry
│   │   ├── vio_estimator.py   # VIO state estimation
│   │   └── feature_tracker.py # Visual feature tracking
│   ├── drift_correction/       # Drift correction
│   │   ├── corrector.py        # Main drift corrector
│   │   ├── loop_closer.py      # Loop closure detection
│   │   └── pose_graph.py       # Pose graph optimization
│   ├── planning/               # Footstep planning
│   │   ├── footstep_planner.py # Main planner
│   │   ├── a_star.py           # A* search
│   │   └── reachability.py     # Kinematic constraints
│   └── navigation/             # High-level navigation
│       └── navigator.py        # VIO navigator
├── config/                     # Configuration files
├── scripts/                    # Example scripts
├── tests/                      # Unit tests
└── docs/                       # Documentation
```

## Development Setup

### 1. Clone the Repository

```bash
git clone https://github.com/ansh1113/vio-footstep-planner.git
cd vio-footstep-planner
```

### 2. Create Virtual Environment

```bash
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install in Development Mode

```bash
pip install -e .
pip install -r requirements.txt
```

### 4. Install Development Dependencies

```bash
pip install pytest pytest-cov black flake8 mypy
```

## Architecture

### VIO Module

The VIO module implements visual-inertial odometry:

- **FeatureTracker**: Tracks visual features across frames using KLT optical flow
- **VIOEstimator**: Performs IMU pre-integration and visual-inertial fusion

Key algorithms:
- Lucas-Kanade optical flow for feature tracking
- IMU pre-integration between frames
- Sliding window state estimation

### Drift Correction Module

Handles long-term drift correction:

- **LoopClosureDetector**: Detects when robot revisits locations using ORB features
- **PoseGraph**: Maintains graph of poses and constraints
- **PoseGraphOptimizer**: Optimizes poses using least squares

Key algorithms:
- ORB feature extraction and matching
- RANSAC geometric verification
- Pose graph optimization (g2o-style)

### Planning Module

Plans collision-free footstep sequences:

- **AStarPlanner**: Generic A* search implementation
- **ReachabilityChecker**: Validates kinematic feasibility
- **FootstepPlanner**: Main planner integrating search and constraints

Key algorithms:
- A* search with custom heuristics
- Kinematic reachability checking
- Stability verification

## Code Style

### Python Style Guide

Follow PEP 8 with these specifics:

- Line length: 100 characters max
- Indentation: 4 spaces
- Quotes: Double quotes for strings
- Imports: Grouped by standard library, third-party, local

Example:
```python
"""Module docstring."""

import numpy as np
from typing import List, Optional

from .other_module import SomeClass


class MyClass:
    """Class docstring.
    
    Args:
        param1: First parameter
        param2: Second parameter
    """
    
    def __init__(self, param1: int, param2: str):
        self.param1 = param1
        self.param2 = param2
    
    def my_method(self, arg: np.ndarray) -> np.ndarray:
        """Method docstring.
        
        Args:
            arg: Input array
            
        Returns:
            Output array
        """
        return arg * 2
```

### Type Hints

Use type hints for all function signatures:

```python
def plan_path(start: np.ndarray, goal: np.ndarray, 
              obstacles: Optional[np.ndarray] = None) -> List[np.ndarray]:
    """Plan path from start to goal."""
    pass
```

### Documentation

All public APIs must have docstrings:

```python
def complex_function(param1, param2):
    """Brief description.
    
    Longer description explaining behavior, edge cases, etc.
    
    Args:
        param1: Description of param1
        param2: Description of param2
        
    Returns:
        Description of return value
        
    Raises:
        ValueError: When param1 is negative
    """
    pass
```

## Testing

### Running Tests

```bash
# Run all tests
python -m pytest tests/

# Run specific test file
python -m pytest tests/test_footstep_planner.py

# Run with coverage
python -m pytest --cov=vio_footstep_planner tests/
```

### Writing Tests

Each module should have corresponding tests:

```python
import unittest
from vio_footstep_planner.planning import FootstepPlanner


class TestFootstepPlanner(unittest.TestCase):
    
    def setUp(self):
        """Set up test fixtures."""
        self.planner = FootstepPlanner(robot_model='spot')
    
    def test_planning(self):
        """Test basic planning."""
        current = np.zeros(6)
        goal = [1.0, 0.0, 0.0]
        
        result = self.planner.plan(current, goal)
        
        self.assertIsNotNone(result)
```

## Contributing

### Workflow

1. Create a feature branch:
   ```bash
   git checkout -b feature/my-feature
   ```

2. Make changes and commit:
   ```bash
   git add .
   git commit -m "Add feature X"
   ```

3. Run tests:
   ```bash
   pytest tests/
   ```

4. Push and create pull request:
   ```bash
   git push origin feature/my-feature
   ```

### Pull Request Guidelines

- Include tests for new features
- Update documentation
- Follow code style guidelines
- Write clear commit messages
- Reference related issues

## Extending the System

### Adding a New Planner

1. Create new file in `planning/`:
   ```python
   # planning/my_planner.py
   from .a_star import AStarPlanner
   
   class MyPlanner(AStarPlanner):
       def plan(self, start, goal):
           # Your implementation
           pass
   ```

2. Add to `planning/__init__.py`:
   ```python
   from .my_planner import MyPlanner
   ```

3. Add tests in `tests/test_my_planner.py`

### Adding New Sensor Support

1. Extend `VIOEstimator`:
   ```python
   def process_lidar(self, timestamp, points):
       """Process LIDAR data."""
       pass
   ```

2. Update `VIONavigator` to handle new sensor

3. Add configuration support

## Performance Optimization

### Profiling

```python
import cProfile

def profile_planning():
    planner = FootstepPlanner('spot')
    # ... planning code
    
cProfile.run('profile_planning()')
```

### Optimization Tips

1. Use NumPy vectorization
2. Minimize Python loops
3. Profile before optimizing
4. Consider Cython for critical paths

## Debugging

### Enable Verbose Logging

```python
import logging

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger('vio_footstep_planner')
```

### Visualization Tools

```python
# Visualize feature tracks
vis_image = tracker.visualize_tracks(image, points)
cv2.imshow('Features', vis_image)
cv2.waitKey(1)
```

## Resources

- [VINS-Fusion Paper](https://arxiv.org/abs/1804.00890)
- [A* Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [Pose Graph Optimization](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf)

## Getting Help

- GitHub Issues: Report bugs and request features
- Discussions: Ask questions and share ideas
- Email: anshbhansali5@gmail.com
