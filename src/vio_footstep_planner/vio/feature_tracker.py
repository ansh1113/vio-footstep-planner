"""Feature tracking for Visual-Inertial Odometry."""

from typing import List, Tuple

import cv2
import numpy as np


class FeatureTracker:
    """Track visual features across frames using optical flow."""

    def __init__(self, max_features=150, min_distance=20, quality_level=0.01):
        """Initialize feature tracker.

        Args:
            max_features: Maximum number of features to track
            min_distance: Minimum distance between features (pixels)
            quality_level: Quality level for corner detection
        """
        self.max_features = max_features
        self.min_distance = min_distance
        self.quality_level = quality_level

        # Lucas-Kanade optical flow parameters
        self.lk_params = {
            "winSize": (21, 21),
            "maxLevel": 3,
            "criteria": (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        }

        # Shi-Tomasi corner detection parameters
        self.feature_params = {
            "maxCorners": max_features,
            "qualityLevel": quality_level,
            "minDistance": min_distance,
            "blockSize": 7,
        }

        self.prev_frame = None
        self.prev_points = None
        self.track_ids = []
        self.next_id = 0

    def detect_features(self, image: np.ndarray) -> np.ndarray:
        """Detect new features in the image.

        Args:
            image: Grayscale image

        Returns:
            Array of detected feature points (N, 2)
        """
        corners = cv2.goodFeaturesToTrack(image, mask=None, **self.feature_params)

        if corners is not None:
            return corners.reshape(-1, 2)
        return np.array([]).reshape(0, 2)

    def track_features(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray, List[int]]:
        """Track features from previous frame to current frame.

        Args:
            image: Current grayscale image

        Returns:
            Tuple of (current_points, previous_points, track_ids)
        """
        if self.prev_frame is None or self.prev_points is None or len(self.prev_points) == 0:
            # First frame or no points to track - detect new features
            points = self.detect_features(image)
            self.prev_frame = image.copy()
            self.prev_points = points
            self.track_ids = list(range(self.next_id, self.next_id + len(points)))
            self.next_id += len(points)
            return points, points, self.track_ids

        # Track existing features
        curr_points, status, err = cv2.calcOpticalFlowPyrLK(
            self.prev_frame, image, self.prev_points.astype(np.float32), None, **self.lk_params
        )

        if curr_points is None:
            # Tracking failed - detect new features
            points = self.detect_features(image)
            self.prev_frame = image.copy()
            self.prev_points = points
            self.track_ids = list(range(self.next_id, self.next_id + len(points)))
            self.next_id += len(points)
            return points, points, self.track_ids

        # Keep only successfully tracked points
        status = status.flatten() == 1

        # Additional check: reverse optical flow for verification
        if np.sum(status) > 0:
            back_points, back_status, _ = cv2.calcOpticalFlowPyrLK(
                image, self.prev_frame, curr_points[status], None, **self.lk_params
            )

            if back_points is not None:
                # Check forward-backward consistency
                d = np.linalg.norm(self.prev_points[status] - back_points.reshape(-1, 2), axis=1)
                fb_status = d < 1.0  # 1 pixel threshold

                # Update status
                valid_indices = np.where(status)[0]
                status[valid_indices] = fb_status

        # Filter points and IDs
        prev_points_tracked = self.prev_points[status]
        curr_points_tracked = curr_points[status].reshape(-1, 2)
        track_ids_tracked = [self.track_ids[i] for i in range(len(status)) if status[i]]

        # Detect new features if we have too few
        if len(curr_points_tracked) < self.max_features // 2:
            # Create mask to avoid existing points
            mask = np.ones_like(image, dtype=np.uint8) * 255
            for pt in curr_points_tracked:
                cv2.circle(mask, tuple(pt.astype(int)), self.min_distance, 0, -1)

            new_corners = cv2.goodFeaturesToTrack(
                image,
                mask=mask,
                maxCorners=self.max_features - len(curr_points_tracked),
                qualityLevel=self.quality_level,
                minDistance=self.min_distance,
                blockSize=7,
            )

            if new_corners is not None and len(new_corners) > 0:
                new_points = new_corners.reshape(-1, 2)
                curr_points_tracked = np.vstack([curr_points_tracked, new_points])
                prev_points_tracked = np.vstack([prev_points_tracked, new_points])
                new_ids = list(range(self.next_id, self.next_id + len(new_points)))
                track_ids_tracked.extend(new_ids)
                self.next_id += len(new_points)

        # Update state
        self.prev_frame = image.copy()
        self.prev_points = curr_points_tracked
        self.track_ids = track_ids_tracked

        return curr_points_tracked, prev_points_tracked, track_ids_tracked

    def visualize_tracks(self, image: np.ndarray, points: np.ndarray) -> np.ndarray:
        """Visualize tracked features on image.

        Args:
            image: Input image
            points: Feature points to visualize

        Returns:
            Image with features drawn
        """
        vis = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if len(image.shape) == 2 else image.copy()

        for pt in points:
            cv2.circle(vis, tuple(pt.astype(int)), 3, (0, 255, 0), -1)

        return vis

    def reset(self):
        """Reset tracker state."""
        self.prev_frame = None
        self.prev_points = None
        self.track_ids = []
        self.next_id = 0
