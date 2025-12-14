"""Pose graph optimization for drift correction."""

import numpy as np
from typing import List, Tuple, Dict, Optional
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


class PoseGraph:
    """Pose graph for loop closure optimization."""
    
    def __init__(self):
        """Initialize empty pose graph."""
        self.nodes = {}  # id -> pose (7D: position + quaternion)
        self.edges = []  # List of (from_id, to_id, relative_transform, information_matrix)
    
    def add_node(self, node_id: int, pose: np.ndarray):
        """Add a node to the graph.
        
        Args:
            node_id: Node identifier
            pose: Pose as [x, y, z, qx, qy, qz, qw] or [x, y, z, roll, pitch, yaw]
        """
        if len(pose) == 6:
            # Convert euler to quaternion
            position = pose[:3]
            quat = Rotation.from_euler('xyz', pose[3:], degrees=False).as_quat()
            pose = np.concatenate([position, quat])
        
        self.nodes[node_id] = pose.copy()
    
    def add_edge(self, from_id: int, to_id: int, relative_transform: np.ndarray,
                 information: Optional[np.ndarray] = None):
        """Add an edge (constraint) to the graph.
        
        Args:
            from_id: Source node ID
            to_id: Target node ID
            relative_transform: 4x4 transformation matrix or 7D pose
            information: 6x6 information matrix (inverse covariance)
        """
        if relative_transform.shape == (4, 4):
            # Convert 4x4 matrix to 7D pose
            position = relative_transform[:3, 3]
            quat = Rotation.from_matrix(relative_transform[:3, :3]).as_quat()
            relative_transform = np.concatenate([position, quat])
        
        if information is None:
            # Default information matrix (identity)
            information = np.eye(6)
        
        self.edges.append({
            'from': from_id,
            'to': to_id,
            'transform': relative_transform.copy(),
            'information': information
        })
    
    def optimize(self, max_iterations: int = 100) -> Dict[int, np.ndarray]:
        """Optimize the pose graph using least squares.
        
        Args:
            max_iterations: Maximum optimization iterations
            
        Returns:
            Dictionary of optimized poses (node_id -> pose)
        """
        if len(self.nodes) < 2 or len(self.edges) == 0:
            return self.nodes.copy()
        
        # Convert poses to optimization variables
        node_ids = sorted(self.nodes.keys())
        x0 = []
        for node_id in node_ids:
            x0.extend(self.nodes[node_id])
        x0 = np.array(x0)
        
        # Fix first pose (set as origin)
        fixed_node = node_ids[0]
        
        # Optimize
        result = least_squares(
            self._residual_function,
            x0,
            args=(node_ids, fixed_node),
            max_nfev=max_iterations,
            verbose=0
        )
        
        # Extract optimized poses
        optimized_poses = {}
        x_opt = result.x
        for i, node_id in enumerate(node_ids):
            pose = x_opt[i*7:(i+1)*7]
            optimized_poses[node_id] = pose
        
        return optimized_poses
    
    def _residual_function(self, x: np.ndarray, node_ids: List[int], 
                          fixed_node: int) -> np.ndarray:
        """Compute residuals for pose graph optimization.
        
        Args:
            x: Flattened pose variables
            node_ids: List of node IDs
            fixed_node: ID of fixed node
            
        Returns:
            Residual vector
        """
        residuals = []
        
        # Reconstruct poses from x
        poses = {}
        for i, node_id in enumerate(node_ids):
            poses[node_id] = x[i*7:(i+1)*7]
        
        # Compute residuals for each edge
        for edge in self.edges:
            from_id = edge['from']
            to_id = edge['to']
            
            if from_id not in poses or to_id not in poses:
                continue
            
            # Get poses
            pose_from = poses[from_id]
            pose_to = poses[to_id]
            
            # Compute relative transform
            rel_transform = self._compute_relative_transform(pose_from, pose_to)
            
            # Expected relative transform
            expected_transform = edge['transform']
            
            # Compute error
            error = self._pose_error(rel_transform, expected_transform)
            
            # Weight by information matrix
            info = edge['information']
            weighted_error = np.sqrt(info.diagonal()) * error
            
            residuals.extend(weighted_error)
        
        # Add constraint to fix first node
        if fixed_node in poses:
            fixed_pose_error = poses[fixed_node] - self.nodes[fixed_node]
            residuals.extend(fixed_pose_error * 100.0)  # Strong weight
        
        return np.array(residuals)
    
    def _compute_relative_transform(self, pose_from: np.ndarray, 
                                   pose_to: np.ndarray) -> np.ndarray:
        """Compute relative transform between two poses.
        
        Args:
            pose_from: Source pose [x, y, z, qx, qy, qz, qw]
            pose_to: Target pose [x, y, z, qx, qy, qz, qw]
            
        Returns:
            Relative transform [dx, dy, dz, dqx, dqy, dqz, dqw]
        """
        # Position difference
        p_from = pose_from[:3]
        p_to = pose_to[:3]
        
        # Rotation
        q_from = pose_from[3:]
        q_to = pose_to[3:]
        
        R_from = Rotation.from_quat(q_from)
        R_to = Rotation.from_quat(q_to)
        
        # Relative transform
        R_rel = R_from.inv() * R_to
        p_rel = R_from.inv().apply(p_to - p_from)
        
        return np.concatenate([p_rel, R_rel.as_quat()])
    
    def _pose_error(self, pose1: np.ndarray, pose2: np.ndarray) -> np.ndarray:
        """Compute error between two poses.
        
        Args:
            pose1: First pose
            pose2: Second pose
            
        Returns:
            6D error vector [dx, dy, dz, droll, dpitch, dyaw]
        """
        # Position error
        pos_error = pose1[:3] - pose2[:3]
        
        # Orientation error (convert to euler for interpretability)
        q1 = pose1[3:]
        q2 = pose2[3:]
        
        R1 = Rotation.from_quat(q1)
        R2 = Rotation.from_quat(q2)
        
        R_error = R1.inv() * R2
        euler_error = R_error.as_euler('xyz', degrees=False)
        
        return np.concatenate([pos_error, euler_error])
    
    def get_node_count(self) -> int:
        """Get number of nodes."""
        return len(self.nodes)
    
    def get_edge_count(self) -> int:
        """Get number of edges."""
        return len(self.edges)
    
    def clear(self):
        """Clear the graph."""
        self.nodes.clear()
        self.edges.clear()


class PoseGraphOptimizer:
    """Wrapper for pose graph optimization."""
    
    def __init__(self):
        """Initialize optimizer."""
        self.graph = PoseGraph()
    
    def add_pose(self, pose_id: int, pose: np.ndarray):
        """Add a pose node."""
        self.graph.add_node(pose_id, pose)
    
    def add_odometry_edge(self, from_id: int, to_id: int, 
                         relative_transform: np.ndarray):
        """Add odometry edge (sequential poses)."""
        # Odometry has higher confidence
        information = np.eye(6) * 10.0
        self.graph.add_edge(from_id, to_id, relative_transform, information)
    
    def add_loop_closure_edge(self, from_id: int, to_id: int,
                             relative_transform: np.ndarray, confidence: float = 0.5):
        """Add loop closure edge."""
        # Loop closures have lower confidence
        information = np.eye(6) * confidence
        self.graph.add_edge(from_id, to_id, relative_transform, information)
    
    def optimize(self) -> Dict[int, np.ndarray]:
        """Run optimization and return corrected poses."""
        return self.graph.optimize()
    
    def clear(self):
        """Clear the optimizer."""
        self.graph.clear()
