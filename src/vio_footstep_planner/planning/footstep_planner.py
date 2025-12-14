"""Footstep planner for quadruped navigation."""

import numpy as np
from typing import List, Optional, Dict
from .a_star import AStarPlanner
from .reachability import ReachabilityChecker


class FootstepPlanner:
    """Footstep planner for quadruped navigation using A* search."""
    
    def __init__(self, robot_model: str, config: Optional[Dict] = None):
        """Initialize footstep planner.
        
        Args:
            robot_model: Robot model name (e.g., 'spot', 'anymal')
            config: Configuration dictionary with planning parameters
        """
        self.robot_model = robot_model
        self.config = config or {}
        
        # Planning parameters
        self.max_step_length = self.config.get('max_step_length', 0.4)
        self.max_step_width = self.config.get('max_step_width', 0.3)
        self.max_step_height = self.config.get('max_step_height', 0.2)
        self.max_step_yaw = self.config.get('max_step_yaw', 0.5)
        self.goal_tolerance = self.config.get('goal_tolerance', 0.2)
        self.planning_horizon = self.config.get('planning_horizon', 3.0)
        
        # Cost weights
        self.distance_weight = self.config.get('distance_weight', 1.0)
        self.rotation_weight = self.config.get('rotation_weight', 0.5)
        self.terrain_weight = self.config.get('terrain_weight', 2.0)
        
        # Robot configuration for reachability
        robot_config = {
            'max_step_length': self.max_step_length,
            'max_step_width': self.max_step_width,
            'max_step_height': self.max_step_height,
            'max_step_yaw': self.max_step_yaw,
            'body_width': 0.5,
            'body_length': 0.7
        }
        
        # Initialize components
        self.reachability_checker = ReachabilityChecker(robot_config)
        self.astar_planner = AStarPlanner(
            step_cost_fn=self._compute_step_cost,
            heuristic_fn=self._compute_heuristic
        )
        
        # State
        self.current_pose = np.zeros(6)  # x, y, z, roll, pitch, yaw
        self.obstacle_map = None
        self.last_plan = []
        
    def plan(self, current_pose: np.ndarray, goal: List[float],
             obstacle_map: Optional[np.ndarray] = None) -> Optional[List[np.ndarray]]:
        """Plan footstep sequence from current pose to goal.
        
        Args:
            current_pose: Current robot pose [x, y, z, roll, pitch, yaw]
            goal: Goal pose [x, y, yaw]
            obstacle_map: Optional occupancy grid
            
        Returns:
            List of footstep positions or None if planning failed
        """
        self.current_pose = np.array(current_pose)
        self.obstacle_map = obstacle_map
        
        # Convert to search states
        start_state = (current_pose[0], current_pose[1], current_pose[5])
        goal_state = (goal[0], goal[1], goal[2] if len(goal) > 2 else 0.0)
        
        # Run A* search
        path = self.astar_planner.plan(
            start=start_state,
            goal=goal_state,
            obstacle_map=obstacle_map,
            max_iterations=5000,
            valid_successors_fn=self._generate_successors
        )
        
        if path is None or len(path) == 0:
            return None
        
        # Convert path to footsteps
        footsteps = []
        for state in path:
            footstep = np.array([state[0], state[1], 0.0])  # x, y, z
            footsteps.append(footstep)
        
        self.last_plan = footsteps
        return footsteps
    
    def _generate_successors(self, state: tuple, 
                            obstacle_map: Optional[np.ndarray] = None) -> List[tuple]:
        """Generate valid successor states.
        
        Args:
            state: Current state (x, y, theta)
            obstacle_map: Optional occupancy grid
            
        Returns:
            List of valid successor states
        """
        x, y, theta = state
        current_footstep = np.array([x, y, 0.0, theta])
        
        successors = []
        
        # Generate candidate steps
        step_lengths = [0.2, 0.3, 0.4]
        step_angles = [-0.3, -0.15, 0.0, 0.15, 0.3]
        
        for length in step_lengths:
            for d_angle in step_angles:
                new_theta = theta + d_angle
                
                # Compute new position
                new_x = x + length * np.cos(new_theta)
                new_y = y + length * np.sin(new_theta)
                
                next_footstep = np.array([new_x, new_y, 0.0, new_theta])
                
                # Check reachability
                if not self.reachability_checker.is_reachable(
                    current_footstep, next_footstep
                ):
                    continue
                
                # Check collision with obstacles
                if obstacle_map is not None:
                    if self._is_collision(next_footstep, obstacle_map):
                        continue
                
                successors.append((new_x, new_y, new_theta))
        
        return successors
    
    def _compute_step_cost(self, state1: tuple, state2: tuple,
                          obstacle_map: Optional[np.ndarray] = None) -> float:
        """Compute cost of transitioning between states.
        
        Args:
            state1: Start state (x, y, theta)
            state2: End state (x, y, theta)
            obstacle_map: Optional occupancy grid
            
        Returns:
            Step cost
        """
        # Distance cost
        dx = state2[0] - state1[0]
        dy = state2[1] - state1[1]
        distance = np.sqrt(dx**2 + dy**2)
        distance_cost = self.distance_weight * distance
        
        # Rotation cost
        dtheta = abs(state2[2] - state1[2])
        dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))  # Normalize to [-pi, pi]
        rotation_cost = self.rotation_weight * abs(dtheta)
        
        # Terrain cost (if obstacle map provided)
        terrain_cost = 0.0
        if obstacle_map is not None:
            terrain_cost = self._get_terrain_cost(state2, obstacle_map)
        
        return distance_cost + rotation_cost + terrain_cost
    
    def _compute_heuristic(self, state: tuple, goal: tuple) -> float:
        """Compute heuristic cost to goal.
        
        Args:
            state: Current state (x, y, theta)
            goal: Goal state (x, y, theta)
            
        Returns:
            Heuristic cost
        """
        # Euclidean distance to goal
        distance = np.sqrt((state[0] - goal[0])**2 + (state[1] - goal[1])**2)
        
        # Angular difference to goal
        dtheta = abs(state[2] - goal[2])
        dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))
        
        return distance + 0.5 * abs(dtheta)
    
    def _is_collision(self, footstep: np.ndarray, 
                     obstacle_map: np.ndarray) -> bool:
        """Check if footstep collides with obstacles.
        
        Args:
            footstep: Footstep position [x, y, z, theta]
            obstacle_map: Occupancy grid
            
        Returns:
            True if collision detected
        """
        # Simplified collision check
        # In practice, should check footprint around position
        x, y = int(footstep[0]), int(footstep[1])
        
        if (0 <= x < obstacle_map.shape[0] and 
            0 <= y < obstacle_map.shape[1]):
            return obstacle_map[x, y] > 0.5
        
        return False
    
    def _get_terrain_cost(self, state: tuple, 
                         obstacle_map: np.ndarray) -> float:
        """Get terrain cost at state.
        
        Args:
            state: State (x, y, theta)
            obstacle_map: Occupancy grid
            
        Returns:
            Terrain cost
        """
        x, y = int(state[0]), int(state[1])
        
        if (0 <= x < obstacle_map.shape[0] and 
            0 <= y < obstacle_map.shape[1]):
            return self.terrain_weight * obstacle_map[x, y]
        
        return 0.0
    
    def reached_goal(self, goal: List[float]) -> bool:
        """Check if robot has reached goal.
        
        Args:
            goal: Goal position [x, y, yaw]
            
        Returns:
            True if goal reached
        """
        current = self.current_pose[:2]
        goal_pos = np.array(goal[:2])
        distance = np.linalg.norm(current - goal_pos)
        
        return distance < self.goal_tolerance
    
    def execute(self, footsteps: List[np.ndarray]):
        """Execute planned footsteps.
        
        This is a placeholder for integration with robot controller.
        
        Args:
            footsteps: List of footstep positions
        """
        print(f"Executing {len(footsteps)} footsteps")
        for i, footstep in enumerate(footsteps):
            print(f"  Step {i+1}: ({footstep[0]:.2f}, {footstep[1]:.2f})")
    
    def visualize_plan(self) -> np.ndarray:
        """Get visualization of last plan.
        
        Returns:
            Image with plan visualized
        """
        # Placeholder - would create visualization
        return np.zeros((480, 640, 3), dtype=np.uint8)
    
    def get_plan_statistics(self) -> Dict:
        """Get statistics about last plan.
        
        Returns:
            Dictionary with plan statistics
        """
        if len(self.last_plan) == 0:
            return {
                'num_steps': 0,
                'total_distance': 0.0,
                'avg_step_length': 0.0
            }
        
        # Compute statistics
        total_distance = 0.0
        for i in range(len(self.last_plan) - 1):
            step_dist = np.linalg.norm(
                self.last_plan[i+1][:2] - self.last_plan[i][:2]
            )
            total_distance += step_dist
        
        avg_step = total_distance / max(len(self.last_plan) - 1, 1)
        
        return {
            'num_steps': len(self.last_plan),
            'total_distance': total_distance,
            'avg_step_length': avg_step
        }

