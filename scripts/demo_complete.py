#!/usr/bin/env python3
"""
Complete demonstration of VIO Footstep Planner.

This script demonstrates the full pipeline:
1. VIO pose estimation with synthetic data
2. Drift correction with loop closure
3. Footstep planning to goal
4. Execution and visualization
"""

import numpy as np
import cv2
import time
from vio_footstep_planner import VIONavigator, FootstepPlanner, DriftCorrector


def generate_synthetic_trajectory(duration=10.0, dt=0.1):
    """Generate synthetic camera images and IMU data for testing.
    
    Args:
        duration: Duration in seconds
        dt: Time step
        
    Returns:
        List of (timestamp, image, imu_data) tuples
    """
    print("Generating synthetic trajectory...")
    
    data = []
    t = 0.0
    
    # Generate circular trajectory
    omega = 0.2  # rad/s
    radius = 2.0  # meters
    
    while t < duration:
        # True position on circle
        x = radius * np.cos(omega * t)
        y = radius * np.sin(omega * t)
        
        # Generate synthetic image (simple pattern)
        image = np.random.randint(0, 255, (480, 640), dtype=np.uint8)
        
        # Add some features (dots) at fixed world positions
        for i in range(20):
            wx = np.random.uniform(-5, 5)
            wy = np.random.uniform(-5, 5)
            
            # Project to image (simplified)
            px = int(320 + (wx - x) * 100)
            py = int(240 + (wy - y) * 100)
            
            if 0 <= px < 640 and 0 <= py < 480:
                cv2.circle(image, (px, py), 5, 255, -1)
        
        # Generate IMU data (simplified)
        # Accelerometer: centripetal acceleration + gravity
        acc_x = -omega**2 * radius * np.cos(omega * t)
        acc_y = -omega**2 * radius * np.sin(omega * t)
        acc_z = -9.81
        
        acc = np.array([acc_x, acc_y, acc_z])
        
        # Gyroscope: constant rotation
        gyro = np.array([0.0, 0.0, omega])
        
        # Add noise
        acc += np.random.randn(3) * 0.1
        gyro += np.random.randn(3) * 0.01
        
        imu_data = {
            'timestamp': t,
            'acc': acc,
            'gyro': gyro
        }
        
        data.append((t, image, imu_data))
        
        t += dt
    
    print(f"Generated {len(data)} frames")
    return data


def main():
    """Main demonstration function."""
    print("=" * 60)
    print("VIO Footstep Planner Demonstration")
    print("=" * 60)
    
    # Initialize components
    print("\n1. Initializing VIO Navigator...")
    navigator = VIONavigator(drift_correction=True)
    navigator.start()
    
    # Reduce optimization frequency for faster demo
    if navigator.drift_corrector is not None:
        navigator.drift_corrector.optimization_frequency = 50  # Optimize less frequently
    
    print("\n2. Initializing Footstep Planner...")
    planner_config = {
        'max_step_length': 0.4,
        'goal_tolerance': 0.2,
        'distance_weight': 1.0,
        'rotation_weight': 0.5
    }
    planner = FootstepPlanner(robot_model='spot', config=planner_config)
    
    # Generate test data
    print("\n3. Generating synthetic sensor data...")
    trajectory_data = generate_synthetic_trajectory(duration=5.0, dt=0.1)
    
    # Process VIO
    print("\n4. Running VIO estimation...")
    for i, (timestamp, image, imu_data) in enumerate(trajectory_data):
        # Process frame
        success = navigator.process_frame(
            image,
            timestamp,
            imu_data['acc'],
            imu_data['gyro']
        )
        
        if success and i % 10 == 0:
            pose = navigator.get_pose()
            print(f"  Frame {i}: Position = ({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f})")
    
    # Get final statistics
    print("\n5. VIO Statistics:")
    stats = navigator.get_statistics()
    print(f"  Total poses: {stats['num_poses']}")
    print(f"  Loop closures: {stats.get('num_loop_closures', 0)}")
    
    # Plan footsteps to goal
    print("\n6. Planning footsteps to goal...")
    current_pose = navigator.get_pose()
    goal = [3.0, 2.0, 0.5]  # x, y, yaw
    
    print(f"  Current pose: ({current_pose[0]:.2f}, {current_pose[1]:.2f})")
    print(f"  Goal: ({goal[0]:.2f}, {goal[1]:.2f})")
    
    footsteps = planner.plan(current_pose, goal)
    
    if footsteps:
        print(f"\n7. Footstep plan generated:")
        plan_stats = planner.get_plan_statistics()
        print(f"  Number of steps: {plan_stats['num_steps']}")
        print(f"  Total distance: {plan_stats['total_distance']:.2f} m")
        print(f"  Average step length: {plan_stats['avg_step_length']:.2f} m")
        
        print("\n  First 5 footsteps:")
        for i, step in enumerate(footsteps[:5]):
            print(f"    Step {i+1}: ({step[0]:.2f}, {step[1]:.2f})")
        
        # Execute (simulation)
        print("\n8. Executing footsteps...")
        planner.execute(footsteps)
        
        print("\n✓ Navigation complete!")
    else:
        print("\n✗ Planning failed!")
    
    print("\n" + "=" * 60)
    print("Demonstration complete")
    print("=" * 60)


if __name__ == "__main__":
    main()
