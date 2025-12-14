"""Utility functions for the VIO footstep planner."""

import numpy as np


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi].
    
    Args:
        angle: Angle in radians
        
    Returns:
        Normalized angle in [-pi, pi]
    """
    return np.arctan2(np.sin(angle), np.cos(angle))


def compute_angle_difference(angle1: float, angle2: float) -> float:
    """Compute normalized angle difference.
    
    Args:
        angle1: First angle in radians
        angle2: Second angle in radians
        
    Returns:
        Normalized difference in [-pi, pi]
    """
    diff = angle2 - angle1
    return normalize_angle(diff)
