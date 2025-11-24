import numpy as np
from typing import List, Optional

class FootstepPlanner:
    """Footstep planner for quadruped navigation."""
    
    def __init__(self, robot_model, config=None):
        self.robot_model = robot_model
        self.config = config or {}
        self.max_step_length = self.config.get('max_step_length', 0.4)
        self.goal_tolerance = self.config.get('goal_tolerance', 0.2)
        self.current_pose = np.zeros(6)
    
    def plan(self, current_pose, goal):
        """Plan footstep sequence to goal."""
        footsteps = []
        
        current = np.array(current_pose[:2])  # x, y
        goal_pos = np.array(goal[:2])
        
        while np.linalg.norm(current - goal_pos) > self.goal_tolerance:
            direction = goal_pos - current
            direction = direction / np.linalg.norm(direction)
            
            step = current + direction * min(self.max_step_length, 
                                            np.linalg.norm(goal_pos - current))
            footsteps.append(step)
            current = step
            
            if len(footsteps) > 100:
                break
        
        return footsteps if len(footsteps) > 0 else None
    
    def reached_goal(self, goal):
        """Check if goal is reached."""
        current = self.current_pose[:2]
        return np.linalg.norm(current - np.array(goal[:2])) < self.goal_tolerance
    
    def execute(self, footsteps):
        """Execute planned footsteps."""
        print(f"Executing {len(footsteps)} footsteps")
