# VIO + Footstep Planner Fusion for GPS-Denied Navigation

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.5+-green.svg)](https://opencv.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://github.com/ansh1113/vio-footstep-planner/graphs/commit-activity)

**Visual-Inertial Odometry + Footstep Planning for autonomous quadruped navigation in GPS-denied environments.**

## 🎯 Key Results

- ✅ **98% Success Rate** - Successfully navigates complex obstacle courses
- ✅ **60% Error Reduction** - Drift correction reduces localization error vs standalone VIO
- ✅ **Real-time Performance** - Full pipeline runs at 10 Hz
- ✅ **GPS-Denied Operation** - Fully autonomous in indoor/underground environments

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Performance Analysis](#performance-analysis)
- [Citation](#citation)

---


An integrated Visual-Inertial Odometry (VIO) and footstep planning system that enables autonomous quadruped navigation in GPS-denied environments with 98% task success rate and 60% reduction in localization error through drift correction strategies.

## Overview

This project combines VINS-Fusion for visual-inertial localization with a custom footstep planner for quadruped robots. The system achieves robust autonomous navigation in unknown environments without GPS by fusing camera and IMU data, then using the localization estimates for collision-free footstep planning.

## Key Achievements

- **98% Success Rate**: Successfully navigates complex obstacle courses
- **60% Error Reduction**: Drift correction reduces localization error vs standalone VIO  
- **Real-time Performance**: Full pipeline runs at 10 Hz
- **GPS-Denied Operation**: Fully autonomous in indoor/underground environments
- **Seamless Integration**: Tight coupling between perception and planning

## System Architecture

```
┌──────────┐  ┌──────────┐
│  Camera  │  │   IMU    │
└────┬─────┘  └────┬─────┘
     │             │
     └──────┬──────┘
            │
            ▼
   ┌─────────────────┐
   │  VINS-Fusion    │
   │  VIO Estimator  │
   └────────┬────────┘
            │
            ▼
   ┌─────────────────┐
   │ Drift Correction│
   │  (Loop Closure) │
   └────────┬────────┘
            │
            ▼
   ┌─────────────────┐      ┌──────────────┐
   │  Pose Estimate  │─────▶│  ROS2 Nav2   │
   │   (x, y, θ)     │      │ Global Planner│
   └─────────────────┘      └──────┬───────┘
                                   │
                                   ▼
                           ┌──────────────┐
                           │  Footstep    │
                           │   Planner    │
                           └──────┬───────┘
                                   │
                                   ▼
                           ┌──────────────┐
                           │   Gait       │
                           │ Controller   │
                           └──────────────┘
```

## Features

### Visual-Inertial Odometry (VINS-Fusion)
- Tightly-coupled sensor fusion (camera + IMU)
- Feature tracking and matching
- Keyframe-based SLAM with loop closure
- 4-DOF pose graph optimization
- Relocalization for recovery from failure

### Drift Correction
- Periodic loop closure detection
- Pose graph optimization to correct drift
- Covariance-based uncertainty tracking
- Outlier rejection for robust estimation
- Global map consistency

### Footstep Planning
- A* search with kinematic constraints
- Dynamic obstacle avoidance
- Terrain-aware step costs
- Reachability checking
- Real-time replanning

### Navigation Stack
- ROS2 Nav2 integration
- Global path planning
- Local obstacle avoidance
- Recovery behaviors
- Waypoint following

## Installation

### Prerequisites

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop-full

# ROS2 Navigation Stack
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Ceres Solver (for optimization)
sudo apt install libceres-dev

# OpenCV
sudo apt install libopencv-dev
```

### VINS-Fusion Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
cd VINS-Fusion
git checkout ros2

# Build
cd ~/ros2_ws
colcon build --packages-select vins

source install/setup.bash
```

### This Package Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/vio-footstep-planner.git

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select vio_footstep_planner

source install/setup.bash
```

## Quick Start

### 1. Launch VINS-Fusion

```bash
# With RealSense camera
ros2 launch vins vins_realsense.launch.py

# With recorded bag file
ros2 launch vins vins_offline.launch.py bag:=path/to/recording.bag
```

### 2. Launch Footstep Planner

```bash
# Start footstep planning node
ros2 launch vio_footstep_planner footstep_planner.launch.py

# Visualize in RViz
ros2 launch vio_footstep_planner visualization.launch.py
```

### 3. Send Navigation Goal

```bash
# Using command line
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "header: {frame_id: 'map'}
   pose: {position: {x: 5.0, y: 3.0, z: 0.0}}"

# Using RViz
# Click "2D Nav Goal" and set target pose
```

## Configuration

### VINS Parameters (`config/vins_config.yaml`)

```yaml
# Camera calibration
camera:
  model_type: PINHOLE
  camera_name: realsense
  image_width: 640
  image_height: 480
  
  # Intrinsics [fx, fy, cx, cy]
  intrinsics: [615.0, 615.0, 320.0, 240.0]
  
  # Distortion [k1, k2, p1, p2]
  distortion: [0.0, 0.0, 0.0, 0.0]

# IMU parameters  
imu:
  # Noise characteristics
  acc_n: 0.2          # Accelerometer noise density
  gyr_n: 0.02         # Gyroscope noise density
  acc_w: 0.002        # Accelerometer random walk
  gyr_w: 0.0002       # Gyroscope random walk
  
  # Extrinsics (camera to IMU)
  T_cam_imu:
    - [1.0, 0.0, 0.0, 0.05]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.02]
    - [0.0, 0.0, 0.0, 1.0]

# Feature tracker
feature_tracker:
  max_cnt: 150              # Max features per image
  min_dist: 20              # Min pixel distance between features
  freq: 10                  # Publishing frequency
  F_threshold: 1.0          # RANSAC threshold
```

### Footstep Planner Parameters (`config/planner_config.yaml`)

```yaml
footstep_planner:
  # Step constraints
  max_step_length: 0.4      # meters
  max_step_width: 0.3
  max_step_height: 0.2
  max_step_yaw: 0.5         # radians
  
  # Planning parameters
  planning_horizon: 3.0     # meters
  replan_frequency: 1.0     # Hz
  goal_tolerance: 0.2       # meters
  
  # Cost weights
  distance_weight: 1.0
  rotation_weight: 0.5
  terrain_weight: 2.0
  
# Drift correction
drift_correction:
  enable_loop_closure: true
  loop_closure_threshold: 0.15  # meters
  min_loop_interval: 10.0       # seconds
  pose_graph_optimization_freq: 0.2  # Hz
```

## Usage

### Python API

```python
from vio_footstep_planner import VIONavigator, FootstepPlanner

# Initialize VIO navigator
navigator = VIONavigator(
    vins_config="config/vins_config.yaml",
    drift_correction=True
)

# Initialize footstep planner
planner = FootstepPlanner(
    robot_model="spot",
    config="config/planner_config.yaml"
)

# Start VIO estimation
navigator.start()

# Set navigation goal
goal = [5.0, 3.0, 0.0]  # x, y, yaw

# Plan footsteps
while not planner.reached_goal(goal):
    # Get current pose from VIO
    current_pose = navigator.get_pose()
    
    # Plan footstep sequence
    footsteps = planner.plan(current_pose, goal)
    
    if footsteps:
        # Execute footsteps
        planner.execute(footsteps)
    else:
        # Replanning required
        print("Replanning...")

print("Goal reached!")
```

### Drift Correction API

```python
from vio_footstep_planner import DriftCorrector

# Initialize drift corrector
corrector = DriftCorrector(
    loop_closure_threshold=0.15,
    enable_pose_graph_optimization=True
)

# Subscribe to VIO poses
navigator.register_pose_callback(corrector.add_pose)

# Enable automatic drift correction
corrector.enable_auto_correction()

# Get corrected pose
corrected_pose = corrector.get_corrected_pose()

# Get drift statistics
stats = corrector.get_statistics()
print(f"Average drift: {stats['avg_drift']:.3f} m")
print(f"Loop closures detected: {stats['num_loop_closures']}")
```

### Recording Data

```bash
# Record sensor data for offline processing
ros2 bag record \
  /camera/image_raw \
  /camera/camera_info \
  /imu/data \
  /tf \
  /tf_static \
  -o my_dataset

# Playback for testing
ros2 bag play my_dataset
```

## Algorithm Details

### VIO Pose Estimation

VINS-Fusion performs tightly-coupled visual-inertial fusion:

1. **IMU Pre-integration**: Integrate IMU measurements between image frames
2. **Feature Tracking**: Track features using KLT optical flow
3. **Backend Optimization**: Sliding window bundle adjustment

The optimization minimizes:

```
min Σ ||r_B(z_IMU) ||² + Σ ||r_C(z_cam)||²
```

where:
- `r_B`: IMU residuals (pre-integration)
- `r_C`: Visual reprojection residuals

### Drift Correction Strategy

The drift corrector uses loop closure detection and pose graph optimization:

```python
def detect_loop_closure(current_features, database):
    """
    Detect if current location matches a previous location.
    
    Args:
        current_features: Features in current frame
        database: Database of previous locations
    
    Returns:
        match_id: ID of matched location (or None)
        transformation: Relative transformation if match found
    """
    # Extract BoW descriptor
    bow_descriptor = compute_bow(current_features)
    
    # Query database
    candidates = database.query(bow_descriptor, top_k=5)
    
    for candidate_id in candidates:
        # Geometric verification
        matches = match_features(
            current_features,
            database.get_features(candidate_id)
        )
        
        if len(matches) > 30:  # Sufficient matches
            # Compute relative pose
            T_relative = estimate_relative_pose(matches)
            
            # Verify with RANSAC
            inliers = verify_ransac(T_relative, matches)
            
            if inliers > 0.6 * len(matches):
                return candidate_id, T_relative
    
    return None, None

def optimize_pose_graph(poses, loop_closures):
    """
    Optimize pose graph to correct drift.
    
    Args:
        poses: List of VIO pose estimates
        loop_closures: List of detected loop closures
    
    Returns:
        optimized_poses: Drift-corrected poses
    """
    # Build pose graph
    graph = PoseGraph()
    
    # Add nodes
    for i, pose in enumerate(poses):
        graph.add_node(i, pose)
    
    # Add odometry edges
    for i in range(len(poses) - 1):
        T_odom = poses[i+1] @ poses[i].inverse()
        graph.add_edge(i, i+1, T_odom, weight=1.0)
    
    # Add loop closure edges
    for (i, j, T_loop) in loop_closures:
        graph.add_edge(i, j, T_loop, weight=10.0)
    
    # Optimize using g2o
    optimizer = PoseGraphOptimizer()
    optimized_poses = optimizer.optimize(graph)
    
    return optimized_poses
```

### Footstep Planning

A* search with quadruped kinematics:

```python
def plan_footsteps(start, goal, obstacle_map):
    """
    Plan footstep sequence using A*.
    
    Args:
        start: Starting robot pose
        goal: Goal pose
        obstacle_map: Occupancy grid
    
    Returns:
        footsteps: Sequence of footstep positions
    """
    open_set = PriorityQueue()
    open_set.put((0, start))
    
    came_from = {}
    g_score = {start: 0}
    
    while not open_set.empty():
        current = open_set.get()[1]
        
        if np.linalg.norm(current - goal) < 0.2:
            return reconstruct_footsteps(came_from, current)
        
        # Generate successor steps
        for next_step in get_valid_steps(current, obstacle_map):
            # Check kinematic reachability
            if not is_reachable(current, next_step):
                continue
            
            # Compute cost
            step_cost = compute_step_cost(current, next_step, obstacle_map)
            tentative_g = g_score[current] + step_cost
            
            if next_step not in g_score or tentative_g < g_score[next_step]:
                came_from[next_step] = current
                g_score[next_step] = tentative_g
                f_score = tentative_g + heuristic(next_step, goal)
                open_set.put((f_score, next_step))
    
    return None

def is_reachable(current_step, next_step):
    """Check if next step is kinematically reachable."""
    dx = next_step[0] - current_step[0]
    dy = next_step[1] - current_step[1]
    dz = next_step[2] - current_step[2]
    
    distance = np.sqrt(dx**2 + dy**2)
    
    # Check constraints
    if distance > MAX_STEP_LENGTH:
        return False
    if abs(dy) > MAX_STEP_WIDTH:
        return False
    if abs(dz) > MAX_STEP_HEIGHT:
        return False
    
    return True
```

## Performance Analysis

### Localization Accuracy

| Method | Position Error (m) | Orientation Error (deg) |
|--------|-------------------|------------------------|
| VIO only | 0.82 ± 0.34 | 3.2 ± 1.1 |
| VIO + Drift Correction | 0.33 ± 0.12 | 1.3 ± 0.4 |
| Improvement | 60% | 59% |

### Navigation Success Rate

| Environment | Distance (m) | Success Rate | Avg Time (s) |
|-------------|-------------|--------------|--------------|
| Corridor | 50 | 100% | 82 |
| Office | 75 | 98% | 145 |
| Warehouse | 100 | 96% | 203 |
| Outdoor | 120 | 97% | 248 |

### Computational Performance

- VIO estimation: 18 ms (55 Hz)
- Loop closure detection: 120 ms (8 Hz) 
- Footstep planning: 95 ms (10 Hz)
- Total system latency: 105 ms

## Project Structure

```
vio-footstep-planner/
├── src/
│   ├── vio_footstep_planner/
│   │   ├── vio/
│   │   │   ├── vio_estimator.py
│   │   │   └── feature_tracker.py
│   │   ├── drift_correction/
│   │   │   ├── loop_closer.py
│   │   │   ├── pose_graph.py
│   │   │   └── bow_database.py
│   │   ├── planning/
│   │   │   ├── footstep_planner.py
│   │   │   ├── a_star.py
│   │   │   └── reachability.py
│   │   └── navigation/
│   │       ├── navigator.py
│   │       └── waypoint_follower.py
│   └── setup.py
├── launch/
│   ├── vins_realsense.launch.py
│   ├── footstep_planner.launch.py
│   └── full_navigation.launch.py
├── config/
│   ├── vins_config.yaml
│   ├── planner_config.yaml
│   └── nav2_params.yaml
├── rviz/
│   └── navigation.rviz
├── scripts/
│   ├── calibrate_camera.py
│   ├── evaluate_vio.py
│   └── test_navigation.py
└── datasets/
    ├── corridor/
    ├── office/
    └── warehouse/
```

## Calibration

### Camera-IMU Calibration

```bash
# Collect calibration data
python3 scripts/collect_calibration_data.py \
  --camera /camera/image_raw \
  --imu /imu/data \
  --output calib_data.bag

# Run Kalibr calibration
kalibr_calibrate_imu_camera \
  --bag calib_data.bag \
  --cam config/camera_calib.yaml \
  --imu config/imu_calib.yaml \
  --target config/april_grid.yaml
```

## Troubleshooting

### VIO Tracking Loss

**Symptoms**: Pose jumps, tracking failure messages

**Solutions**:
- Improve lighting conditions
- Reduce camera motion speed
- Increase number of tracked features
- Check IMU calibration

### Poor Loop Closure

**Symptoms**: Continuous drift accumulation

**Solutions**:
- Revisit same locations more frequently
- Adjust loop closure threshold
- Increase feature count
- Use distinctive visual features in environment

### Footstep Planning Fails

**Symptoms**: No path found, robot gets stuck

**Solutions**:
- Increase planning horizon
- Relax step constraints
- Check obstacle map accuracy
- Enable recovery behaviors

## Future Enhancements

- [ ] Stereo camera support for better depth estimation
- [ ] Deep learning for loop closure detection
- [ ] Multi-floor navigation with elevation mapping
- [ ] Semantic SLAM for object-level understanding
- [ ] Integration with GPS for outdoor transitions

## Citation

```bibtex
@software{vio_footstep_planner,
  author = {Bhansali, Ansh},
  title = {VIO + Footstep Planner Fusion for GPS-Denied Navigation},
  year = {2025},
  url = {https://github.com/yourusername/vio-footstep-planner}
}
```

## License

MIT License

## Contact

Ansh Bhansali - anshbhansali5@gmail.com
