"""Drift correction using loop closure and pose graph optimization."""

from typing import Dict, Optional

import numpy as np

from .loop_closer import LoopClosureDetector
from .pose_graph import PoseGraphOptimizer


class DriftCorrector:
    """Drift correction using loop closure and pose graph optimization."""

    def __init__(
        self,
        loop_closure_threshold=0.7,
        enable_pose_graph_optimization=True,
        min_loop_interval=30,
        optimization_frequency=10,
    ):
        """Initialize drift corrector.

        Args:
            loop_closure_threshold: Similarity threshold for loop detection
            enable_pose_graph_optimization: Enable pose graph optimization
            min_loop_interval: Minimum frames between loop closures
            optimization_frequency: Optimize every N new poses
        """
        self.threshold = loop_closure_threshold
        self.enable_optimization = enable_pose_graph_optimization
        self.min_loop_interval = min_loop_interval
        self.optimization_frequency = optimization_frequency

        # Loop closure detector
        self.loop_detector = LoopClosureDetector(
            similarity_threshold=loop_closure_threshold, min_loop_interval=min_loop_interval
        )

        # Pose graph optimizer
        self.pose_optimizer = PoseGraphOptimizer()

        # State
        self.pose_database = []
        self.image_database = []
        self.loop_closures = []
        self.corrected_poses = {}
        self.current_id = 0
        self.poses_since_optimization = 0

        # Statistics
        self.total_drift_corrections = 0
        self.cumulative_drift = 0.0

    def add_pose(self, pose: np.ndarray, image: Optional[np.ndarray] = None):
        """Add pose to database and check for loop closures.

        Args:
            pose: Current pose [x, y, z, roll, pitch, yaw]
            image: Optional grayscale image for loop closure detection
        """
        self.pose_database.append(pose.copy())

        if image is not None:
            self.image_database.append(image)

        # Add to pose graph
        self.pose_optimizer.add_pose(self.current_id, pose)

        # Add odometry edge if not first pose
        if self.current_id > 0:
            prev_pose = self.pose_database[-2]
            rel_transform = self._compute_relative_transform(prev_pose, pose)
            self.pose_optimizer.add_odometry_edge(
                self.current_id - 1, self.current_id, rel_transform
            )

        # Detect loop closures (with images)
        if image is not None and self.current_id >= self.min_loop_interval:
            # Add as keyframe
            self.loop_detector.add_keyframe(image, pose)

            # Check for loop closure
            loop_result = self.loop_detector.detect_loop_closure(image, pose, self.current_id)

            if loop_result is not None:
                matched_id, rel_transform, confidence = loop_result
                self.loop_closures.append((matched_id, self.current_id, confidence))

                # Add loop closure edge to pose graph
                if self.enable_optimization:
                    self.pose_optimizer.add_loop_closure_edge(
                        matched_id, self.current_id, rel_transform, confidence
                    )

                print(
                    f"Loop closure detected: {matched_id} <-> {self.current_id} (confidence: {confidence:.2f})"
                )

        self.current_id += 1
        self.poses_since_optimization += 1

        # Trigger optimization periodically
        if (
            self.enable_optimization
            and len(self.loop_closures) > 0
            and self.poses_since_optimization >= self.optimization_frequency
        ):
            self._optimize_poses()
            self.poses_since_optimization = 0

    def _compute_relative_transform(self, pose_from: np.ndarray, pose_to: np.ndarray) -> np.ndarray:
        """Compute relative transform between poses.

        Args:
            pose_from: Source pose [x, y, z, roll, pitch, yaw]
            pose_to: Target pose [x, y, z, roll, pitch, yaw]

        Returns:
            4x4 transformation matrix
        """
        from scipy.spatial.transform import Rotation

        # Convert to homogeneous transforms
        T_from = np.eye(4)
        T_from[:3, 3] = pose_from[:3]
        T_from[:3, :3] = Rotation.from_euler("xyz", pose_from[3:], degrees=False).as_matrix()

        T_to = np.eye(4)
        T_to[:3, 3] = pose_to[:3]
        T_to[:3, :3] = Rotation.from_euler("xyz", pose_to[3:], degrees=False).as_matrix()

        # Relative transform
        T_rel = np.linalg.inv(T_from) @ T_to

        return T_rel

    def _optimize_poses(self):
        """Run pose graph optimization."""
        if not self.enable_optimization or len(self.pose_database) < 2:
            return

        # Optimize
        optimized = self.pose_optimizer.optimize()

        # Update corrected poses
        self.corrected_poses = {}
        for pose_id, opt_pose in optimized.items():
            # Convert from 7D (pos + quat) to 6D (pos + euler)
            from scipy.spatial.transform import Rotation

            position = opt_pose[:3]
            quat = opt_pose[3:]
            euler = Rotation.from_quat(quat).as_euler("xyz", degrees=False)
            self.corrected_poses[pose_id] = np.concatenate([position, euler])

        # Compute drift correction
        if len(self.corrected_poses) > 0:
            for pose_id in self.corrected_poses:
                if pose_id < len(self.pose_database):
                    original = self.pose_database[pose_id]
                    corrected = self.corrected_poses[pose_id]
                    drift = np.linalg.norm(original[:3] - corrected[:3])
                    self.cumulative_drift += drift

            self.total_drift_corrections += 1

        print(f"Pose graph optimized: {len(self.corrected_poses)} poses corrected")

    def get_corrected_pose(self, pose_id: Optional[int] = None) -> np.ndarray:
        """Get drift-corrected pose.

        Args:
            pose_id: Specific pose ID, or None for most recent

        Returns:
            Corrected pose [x, y, z, roll, pitch, yaw]
        """
        if pose_id is None:
            pose_id = self.current_id - 1

        # Return corrected pose if available
        if pose_id in self.corrected_poses:
            return self.corrected_poses[pose_id].copy()

        # Otherwise return original
        if 0 <= pose_id < len(self.pose_database):
            return self.pose_database[pose_id].copy()

        return np.zeros(6)

    def get_statistics(self) -> Dict:
        """Get drift correction statistics.

        Returns:
            Dictionary with statistics
        """
        avg_drift = (
            self.cumulative_drift / self.total_drift_corrections
            if self.total_drift_corrections > 0
            else 0.0
        )

        return {
            "num_loop_closures": len(self.loop_closures),
            "avg_drift": avg_drift,
            "total_poses": len(self.pose_database),
            "corrected_poses": len(self.corrected_poses),
            "optimization_runs": self.total_drift_corrections,
        }

    def enable_auto_correction(self):
        """Enable automatic drift correction."""
        self.enable_optimization = True

    def disable_auto_correction(self):
        """Disable automatic drift correction."""
        self.enable_optimization = False

    def clear(self):
        """Clear all data."""
        self.pose_database = []
        self.image_database = []
        self.loop_closures = []
        self.corrected_poses = {}
        self.current_id = 0
        self.poses_since_optimization = 0
        self.total_drift_corrections = 0
        self.cumulative_drift = 0.0
        self.loop_detector.clear()
        self.pose_optimizer.clear()
