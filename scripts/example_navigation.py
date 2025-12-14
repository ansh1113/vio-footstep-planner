#!/usr/bin/env python3
"""Example navigation script demonstrating basic usage."""

from vio_footstep_planner import VIONavigator, FootstepPlanner
import numpy as np

# Initialize navigator and planner
navigator = VIONavigator(drift_correction=False)  # Disable for simple demo

# Use simpler config for demo
config = {
    'max_step_length': 0.5,
    'goal_tolerance': 0.3,
}
planner = FootstepPlanner(robot_model="spot", config=config)

navigator.start()

# Set a goal
goal = [3.0, 2.0, 0.0]  # x, y, yaw (shorter distance)

print(f"Planning path to goal: ({goal[0]}, {goal[1]})")

# Get current pose and plan
current_pose = navigator.get_pose()
print(f"Current pose: ({current_pose[0]:.2f}, {current_pose[1]:.2f}, {current_pose[2]:.2f})")

# Simple straight-line footstep generation for demo
print("\nGenerating footsteps...")
footsteps = []
current = current_pose[:2]
goal_pos = np.array(goal[:2])
direction = goal_pos - current
distance = np.linalg.norm(direction)

if distance > 0:
    direction = direction / distance
    step_length = 0.4
    num_steps = int(distance / step_length) + 1
    
    for i in range(1, num_steps + 1):
        step_dist = min(i * step_length, distance)
        step_pos = current + direction * step_dist
        footsteps.append(np.array([step_pos[0], step_pos[1], 0.0]))

if footsteps:
    print(f"\n✓ Generated {len(footsteps)} footsteps")
    print(f"First step: ({footsteps[0][0]:.2f}, {footsteps[0][1]:.2f})")
    print(f"Last step: ({footsteps[-1][0]:.2f}, {footsteps[-1][1]:.2f})")
    
    # Execute footsteps
    print("\nExecuting footsteps:")
    for i, step in enumerate(footsteps[:5]):  # Show first 5
        print(f"  Step {i+1}: ({step[0]:.2f}, {step[1]:.2f})")
else:
    print("✗ Failed to generate footsteps")
