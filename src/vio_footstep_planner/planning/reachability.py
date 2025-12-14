"""Kinematic reachability checker for quadruped footstep planning."""

from typing import Optional

import numpy as np


class ReachabilityChecker:
    """Check if footstep transitions are kinematically reachable."""

    def __init__(self, robot_config: dict):
        """Initialize reachability checker.

        Args:
            robot_config: Dictionary with robot parameters
                - max_step_length: Maximum step length (m)
                - max_step_width: Maximum lateral step (m)
                - max_step_height: Maximum vertical step (m)
                - max_step_yaw: Maximum rotation per step (rad)
                - body_width: Robot body width (m)
                - body_length: Robot body length (m)
        """
        self.max_step_length = robot_config.get("max_step_length", 0.4)
        self.max_step_width = robot_config.get("max_step_width", 0.3)
        self.max_step_height = robot_config.get("max_step_height", 0.2)
        self.max_step_yaw = robot_config.get("max_step_yaw", 0.5)
        self.body_width = robot_config.get("body_width", 0.5)
        self.body_length = robot_config.get("body_length", 0.7)

        # Leg workspace (approximate as rectangle)
        self.leg_reach_forward = 0.5
        self.leg_reach_lateral = 0.35
        self.leg_reach_backward = 0.3

    def is_reachable(
        self, current_footstep: np.ndarray, next_footstep: np.ndarray, foot_id: Optional[int] = None
    ) -> bool:
        """Check if next footstep is reachable from current.

        Args:
            current_footstep: Current footstep [x, y, z, yaw]
            next_footstep: Next footstep [x, y, z, yaw]
            foot_id: Optional foot identifier (0-3 for quadruped)

        Returns:
            True if reachable
        """
        # Compute relative step
        dx = next_footstep[0] - current_footstep[0]
        dy = next_footstep[1] - current_footstep[1]
        dz = next_footstep[2] if len(next_footstep) > 2 else 0.0
        dz -= current_footstep[2] if len(current_footstep) > 2 else 0.0

        # Check step length
        distance_xy = np.sqrt(dx**2 + dy**2)
        if distance_xy > self.max_step_length:
            return False

        # Check lateral displacement
        if abs(dy) > self.max_step_width:
            return False

        # Check height change
        if abs(dz) > self.max_step_height:
            return False

        # Check rotation
        if len(current_footstep) > 3 and len(next_footstep) > 3:
            dyaw = abs(next_footstep[3] - current_footstep[3])
            # Normalize to [0, pi]
            dyaw = min(dyaw, 2 * np.pi - dyaw)
            if dyaw > self.max_step_yaw:
                return False

        return True

    def get_reachable_region(
        self, current_footstep: np.ndarray, resolution: float = 0.05
    ) -> np.ndarray:
        """Get grid of reachable positions from current footstep.

        Args:
            current_footstep: Current footstep [x, y, z, yaw]
            resolution: Grid resolution (m)

        Returns:
            Binary occupancy grid (1 = reachable, 0 = not reachable)
        """
        # Create grid around current position
        grid_size = int(2 * self.max_step_length / resolution)
        grid = np.zeros((grid_size, grid_size), dtype=np.uint8)

        center_x, center_y = grid_size // 2, grid_size // 2

        for i in range(grid_size):
            for j in range(grid_size):
                # Convert grid coordinates to world coordinates
                x = (i - center_x) * resolution + current_footstep[0]
                y = (j - center_y) * resolution + current_footstep[1]
                z = current_footstep[2] if len(current_footstep) > 2 else 0.0

                next_footstep = np.array([x, y, z])

                if self.is_reachable(current_footstep, next_footstep):
                    grid[i, j] = 1

        return grid

    def check_stability(self, footsteps: np.ndarray) -> bool:
        """Check if a set of footsteps maintains stability.

        Args:
            footsteps: Array of 4 footsteps [[x, y, z], ...] for quadruped

        Returns:
            True if stable (CoM within support polygon)
        """
        if len(footsteps) < 3:
            return False

        # Compute center of mass (simplified - assume at centroid)
        com = np.mean(footsteps[:, :2], axis=0)

        # Check if CoM is within support polygon (convex hull)
        # Simplified: check if within bounding box with margin
        x_min, x_max = footsteps[:, 0].min(), footsteps[:, 0].max()
        y_min, y_max = footsteps[:, 1].min(), footsteps[:, 1].max()

        margin = 0.05  # 5cm margin

        if (
            x_min + margin <= com[0] <= x_max - margin
            and y_min + margin <= com[1] <= y_max - margin
        ):
            return True

        return False

    def get_valid_footstep_positions(self, body_pose: np.ndarray, foot_id: int) -> np.ndarray:
        """Get valid footstep positions for a specific foot.

        Args:
            body_pose: Robot body pose [x, y, z, roll, pitch, yaw]
            foot_id: Foot identifier (0=FL, 1=FR, 2=RL, 3=RR)

        Returns:
            Array of valid positions [[x, y, z], ...]
        """
        x, y = body_pose[0], body_pose[1]
        yaw = body_pose[5] if len(body_pose) > 5 else 0.0

        # Nominal foot positions relative to body (for Spot-like robot)
        nominal_positions = {
            0: np.array([self.body_length / 2, self.body_width / 2, 0]),  # FL
            1: np.array([self.body_length / 2, -self.body_width / 2, 0]),  # FR
            2: np.array([-self.body_length / 2, self.body_width / 2, 0]),  # RL
            3: np.array([-self.body_length / 2, -self.body_width / 2, 0]),  # RR
        }

        if foot_id not in nominal_positions:
            return np.array([])

        # Transform nominal position to world frame
        R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])

        nominal = nominal_positions[foot_id]
        world_pos = R @ nominal[:2] + np.array([x, y])

        # Generate workspace around nominal position
        positions = []
        for dx in np.linspace(-self.leg_reach_forward, self.leg_reach_backward, 10):
            for dy in np.linspace(-self.leg_reach_lateral, self.leg_reach_lateral, 10):
                pos = world_pos + np.array([dx, dy])
                positions.append(np.concatenate([pos, [0.0]]))

        return np.array(positions)
