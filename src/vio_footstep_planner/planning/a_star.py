"""A* search algorithm for footstep planning."""

import numpy as np
import heapq
from typing import List, Tuple, Optional, Callable


class Node:
    """A* search node."""
    
    def __init__(self, state: Tuple, g_cost: float, h_cost: float, parent=None):
        """Initialize node.
        
        Args:
            state: State tuple (x, y, theta, foot)
            g_cost: Cost from start
            h_cost: Heuristic cost to goal
            parent: Parent node
        """
        self.state = state
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = g_cost + h_cost
        self.parent = parent
    
    def __lt__(self, other):
        """Comparison for priority queue."""
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        """Equality check."""
        return self.state == other.state
    
    def __hash__(self):
        """Hash for set operations."""
        return hash(self.state)


class AStarPlanner:
    """A* search planner for footstep planning."""
    
    def __init__(self, step_cost_fn: Optional[Callable] = None,
                 heuristic_fn: Optional[Callable] = None):
        """Initialize A* planner.
        
        Args:
            step_cost_fn: Function to compute step cost
            heuristic_fn: Heuristic function for goal distance
        """
        self.step_cost_fn = step_cost_fn or self._default_step_cost
        self.heuristic_fn = heuristic_fn or self._default_heuristic
        
    def plan(self, start: Tuple, goal: Tuple, 
             obstacle_map: Optional[np.ndarray] = None,
             max_iterations: int = 10000,
             valid_successors_fn: Optional[Callable] = None) -> Optional[List]:
        """Plan path using A* search.
        
        Args:
            start: Start state (x, y, theta)
            goal: Goal state (x, y, theta)
            obstacle_map: Optional occupancy grid
            max_iterations: Maximum search iterations
            valid_successors_fn: Function to generate valid successor states
            
        Returns:
            List of states from start to goal, or None if no path found
        """
        if valid_successors_fn is None:
            valid_successors_fn = self._default_successors
        
        # Initialize
        start_node = Node(
            state=start,
            g_cost=0.0,
            h_cost=self.heuristic_fn(start, goal),
            parent=None
        )
        
        open_set = []
        heapq.heappush(open_set, start_node)
        
        closed_set = set()
        g_costs = {start: 0.0}
        
        iterations = 0
        
        while open_set and iterations < max_iterations:
            iterations += 1
            
            # Get node with lowest f_cost
            current = heapq.heappop(open_set)
            
            # Check if goal reached
            if self._is_goal(current.state, goal):
                return self._reconstruct_path(current)
            
            # Add to closed set
            closed_set.add(current.state)
            
            # Generate successors
            successors = valid_successors_fn(current.state, obstacle_map)
            
            for successor_state in successors:
                if successor_state in closed_set:
                    continue
                
                # Compute tentative g_cost
                step_cost = self.step_cost_fn(
                    current.state, successor_state, obstacle_map
                )
                tentative_g = current.g_cost + step_cost
                
                # Check if this path is better
                if (successor_state not in g_costs or 
                    tentative_g < g_costs[successor_state]):
                    
                    g_costs[successor_state] = tentative_g
                    h_cost = self.heuristic_fn(successor_state, goal)
                    
                    successor_node = Node(
                        state=successor_state,
                        g_cost=tentative_g,
                        h_cost=h_cost,
                        parent=current
                    )
                    
                    heapq.heappush(open_set, successor_node)
        
        # No path found
        return None
    
    def _reconstruct_path(self, node: Node) -> List:
        """Reconstruct path from goal to start.
        
        Args:
            node: Goal node
            
        Returns:
            List of states from start to goal
        """
        path = []
        current = node
        
        while current is not None:
            path.append(current.state)
            current = current.parent
        
        return path[::-1]  # Reverse to get start to goal
    
    def _is_goal(self, state: Tuple, goal: Tuple, tolerance: float = 0.2) -> bool:
        """Check if state is at goal.
        
        Args:
            state: Current state
            goal: Goal state
            tolerance: Distance tolerance
            
        Returns:
            True if at goal
        """
        distance = np.sqrt((state[0] - goal[0])**2 + (state[1] - goal[1])**2)
        return distance < tolerance
    
    def _default_heuristic(self, state: Tuple, goal: Tuple) -> float:
        """Default heuristic: Euclidean distance.
        
        Args:
            state: Current state
            goal: Goal state
            
        Returns:
            Heuristic cost
        """
        return np.sqrt((state[0] - goal[0])**2 + (state[1] - goal[1])**2)
    
    def _default_step_cost(self, state1: Tuple, state2: Tuple,
                          obstacle_map: Optional[np.ndarray] = None) -> float:
        """Default step cost: distance + rotation penalty.
        
        Args:
            state1: Start state
            state2: End state
            obstacle_map: Optional occupancy grid
            
        Returns:
            Step cost
        """
        # Distance cost
        dx = state2[0] - state1[0]
        dy = state2[1] - state1[1]
        distance = np.sqrt(dx**2 + dy**2)
        
        # Rotation cost (if theta is in state)
        rotation_cost = 0.0
        if len(state1) > 2 and len(state2) > 2:
            dtheta = abs(state2[2] - state1[2])
            # Normalize to [-pi, pi]
            dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))
            rotation_cost = 0.5 * abs(dtheta)
        
        return distance + rotation_cost
    
    def _default_successors(self, state: Tuple, 
                           obstacle_map: Optional[np.ndarray] = None) -> List[Tuple]:
        """Generate default successor states.
        
        Args:
            state: Current state (x, y, theta)
            obstacle_map: Optional occupancy grid
            
        Returns:
            List of valid successor states
        """
        x, y = state[0], state[1]
        theta = state[2] if len(state) > 2 else 0.0
        
        # Define step actions
        step_length = 0.3
        step_angles = [-0.3, -0.15, 0.0, 0.15, 0.3]  # radians
        
        successors = []
        
        for d_angle in step_angles:
            new_theta = theta + d_angle
            
            # Forward step
            new_x = x + step_length * np.cos(new_theta)
            new_y = y + step_length * np.sin(new_theta)
            
            # Check if valid (simple bounds check)
            if -10 <= new_x <= 10 and -10 <= new_y <= 10:
                successors.append((new_x, new_y, new_theta))
        
        return successors
