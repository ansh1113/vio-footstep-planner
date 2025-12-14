#!/usr/bin/env python3
from vio_footstep_planner import VIONavigator, FootstepPlanner

navigator = VIONavigator(drift_correction=True)
planner = FootstepPlanner(robot_model="spot")

navigator.start()

goal = [5.0, 3.0, 0.0]
footsteps = planner.plan(navigator.get_pose(), goal)

if footsteps:
    print(f"Planned {len(footsteps)} footsteps")
    planner.execute(footsteps)
else:
    print("Planning failed")
