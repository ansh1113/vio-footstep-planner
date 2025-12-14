#!/usr/bin/env python3
"""Simple demonstration without loop closure for faster execution."""

import numpy as np
from vio_footstep_planner import VIONavigator, FootstepPlanner


def main():
    """Main demonstration function."""
    print("=" * 60)
    print("VIO Footstep Planner - Quick Demo")
    print("=" * 60)
    
    # Initialize components (without drift correction for speed)
    print("\n1. Initializing VIO Navigator...")
    navigator = VIONavigator(drift_correction=False)
    navigator.start()
    
    print("\n2. Initializing Footstep Planner...")
    planner_config = {
        'max_step_length': 0.4,
        'goal_tolerance': 0.2,
        'distance_weight': 1.0,
        'rotation_weight': 0.5
    }
    planner = FootstepPlanner(robot_model='spot', config=planner_config)
    
    # Set current pose
    print("\n3. Setting robot pose...")
    current_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    navigator.current_pose = current_pose
    print(f"   Current: ({current_pose[0]:.2f}, {current_pose[1]:.2f}, {current_pose[2]:.2f})")
    
    # Plan footsteps to goal
    print("\n4. Planning footsteps to goal...")
    goal = [3.0, 2.0, 0.5]  # x, y, yaw
    print(f"   Goal: ({goal[0]:.2f}, {goal[1]:.2f}, yaw={goal[2]:.2f})")
    
    footsteps = planner.plan(current_pose, goal)
    
    if footsteps:
        print(f"\n5. ✓ Footstep plan generated!")
        plan_stats = planner.get_plan_statistics()
        print(f"   - Number of steps: {plan_stats['num_steps']}")
        print(f"   - Total distance: {plan_stats['total_distance']:.2f} m")
        print(f"   - Average step length: {plan_stats['avg_step_length']:.2f} m")
        
        print("\n6. First 5 footsteps:")
        for i, step in enumerate(footsteps[:5]):
            print(f"   Step {i+1}: ({step[0]:.2f}, {step[1]:.2f}, {step[2]:.2f})")
        
        if len(footsteps) > 5:
            print(f"   ... ({len(footsteps) - 5} more steps)")
        
        print("\n7. Executing footsteps...")
        planner.execute(footsteps[:5])  # Execute first 5 for demo
        
        print("\n✓ Demo complete!")
    else:
        print("\n✗ Planning failed!")
    
    print("\n" + "=" * 60)


if __name__ == "__main__":
    main()
