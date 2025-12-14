"""Loop closure detection for drift correction."""

from typing import List, Optional, Tuple

import cv2
import numpy as np


class LoopClosureDetector:
    """Detect loop closures using visual features and BoW (Bag of Words).

    This is a simplified implementation. For production, use DBoW2/DBoW3.
    """

    def __init__(self, similarity_threshold=0.7, min_loop_interval=30):
        """Initialize loop closure detector.

        Args:
            similarity_threshold: Minimum similarity score for loop detection
            min_loop_interval: Minimum frames between loop closures
        """
        self.similarity_threshold = similarity_threshold
        self.min_loop_interval = min_loop_interval

        # Database of keyframes
        self.keyframes = []  # List of dicts with 'id', 'features', 'descriptors', 'pose'
        self.next_id = 0

        # ORB detector for feature extraction
        self.orb = cv2.ORB_create(nfeatures=500)

        # BFMatcher is more reliable for ORB binary descriptors
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

    def add_keyframe(self, image: np.ndarray, pose: np.ndarray) -> int:
        """Add a new keyframe to the database.

        Args:
            image: Grayscale image
            pose: Current pose [x, y, z, roll, pitch, yaw]

        Returns:
            Keyframe ID
        """
        # Extract features
        keypoints, descriptors = self.orb.detectAndCompute(image, None)

        if descriptors is None or len(keypoints) < 50:
            return -1  # Not enough features

        # Store keyframe
        keyframe = {
            "id": self.next_id,
            "keypoints": keypoints,
            "descriptors": descriptors,
            "pose": pose.copy(),
            "image": image.copy(),
        }

        self.keyframes.append(keyframe)
        self.next_id += 1

        return keyframe["id"]

    def detect_loop_closure(
        self, image: np.ndarray, current_pose: np.ndarray, current_id: int
    ) -> Optional[Tuple[int, np.ndarray, float]]:
        """Detect if current frame matches a previous keyframe.

        Args:
            image: Current grayscale image
            current_pose: Current pose estimate
            current_id: Current keyframe ID

        Returns:
            Tuple of (matched_keyframe_id, relative_transform, confidence) or None
        """
        if len(self.keyframes) < 10:
            return None

        # Extract features from current frame
        keypoints_curr, descriptors_curr = self.orb.detectAndCompute(image, None)

        if descriptors_curr is None or len(keypoints_curr) < 50:
            return None

        best_match_id = None
        best_score = 0
        best_transform = None

        # Search through keyframes (skip recent ones)
        for keyframe in self.keyframes:
            if current_id - keyframe["id"] < self.min_loop_interval:
                continue

            # Match features
            matches = self._match_features(descriptors_curr, keyframe["descriptors"])

            if len(matches) < 30:
                continue

            # Compute similarity score
            score = len(matches) / min(len(keypoints_curr), len(keyframe["keypoints"]))

            if score > self.similarity_threshold and score > best_score:
                # Geometric verification
                valid_matches, transform = self._geometric_verification(
                    keypoints_curr, keyframe["keypoints"], matches
                )

                if len(valid_matches) > 20:  # Sufficient inliers
                    best_match_id = keyframe["id"]
                    best_score = score
                    best_transform = transform

        if best_match_id is not None:
            return (best_match_id, best_transform, best_score)

        return None

    def _match_features(self, desc1: np.ndarray, desc2: np.ndarray) -> List:
        """Match features between two descriptor sets.

        Args:
            desc1: First descriptor set
            desc2: Second descriptor set

        Returns:
            List of good matches
        """
        if desc1 is None or desc2 is None:
            return []

        try:
            # Use BFMatcher with ratio test
            matches = self.matcher.knnMatch(desc1, desc2, k=2)

            # Lowe's ratio test
            good_matches = []
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.75 * n.distance:  # Stricter ratio for ORB
                        good_matches.append(m)

            return good_matches
        except Exception:
            # Fallback to simple matching
            matches = self.matcher.match(desc1, desc2)
            return sorted(matches, key=lambda x: x.distance)[:100]

    def _geometric_verification(
        self, kp1: List, kp2: List, matches: List
    ) -> Tuple[List, Optional[np.ndarray]]:
        """Verify matches using geometric constraints.

        Args:
            kp1: Keypoints from first image
            kp2: Keypoints from second image
            matches: List of matches

        Returns:
            Tuple of (inlier_matches, relative_transform)
        """
        if len(matches) < 8:
            return [], None

        # Extract matched point coordinates
        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])

        # Find essential matrix using RANSAC
        E, mask = cv2.findEssentialMat(
            pts1,
            pts2,
            focal=500.0,  # Approximate focal length
            pp=(320, 240),  # Approximate principal point
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0,
        )

        if E is None or mask is None:
            return [], None

        # Filter inliers
        inlier_matches = [matches[i] for i in range(len(matches)) if mask[i]]

        if len(inlier_matches) < 20:
            return [], None

        # Recover pose from essential matrix
        _, R, t, pose_mask = cv2.recoverPose(E, pts1, pts2)

        # Construct relative transform
        transform = np.eye(4)
        transform[:3, :3] = R
        transform[:3, 3] = t.flatten()

        return inlier_matches, transform

    def get_keyframe_pose(self, keyframe_id: int) -> Optional[np.ndarray]:
        """Get pose of a specific keyframe.

        Args:
            keyframe_id: Keyframe ID

        Returns:
            Pose or None if not found
        """
        for kf in self.keyframes:
            if kf["id"] == keyframe_id:
                return kf["pose"].copy()
        return None

    def clear(self):
        """Clear all keyframes."""
        self.keyframes = []
        self.next_id = 0
