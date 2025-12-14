"""Unit tests for VIO feature tracker."""

import unittest

import cv2
import numpy as np

from vio_footstep_planner.vio.feature_tracker import FeatureTracker


class TestFeatureTracker(unittest.TestCase):
    """Test cases for FeatureTracker."""

    def setUp(self):
        """Set up test fixtures."""
        self.tracker = FeatureTracker(max_features=100, min_distance=20)

    def test_initialization(self):
        """Test tracker initialization."""
        self.assertEqual(self.tracker.max_features, 100)
        self.assertEqual(self.tracker.min_distance, 20)
        self.assertIsNone(self.tracker.prev_frame)

    def test_detect_features(self):
        """Test feature detection."""
        # Create test image with corners
        image = np.zeros((480, 640), dtype=np.uint8)
        cv2.rectangle(image, (100, 100), (200, 200), 255, -1)
        cv2.rectangle(image, (300, 300), (400, 400), 255, -1)

        features = self.tracker.detect_features(image)

        # Should detect some features
        self.assertGreater(len(features), 0)
        self.assertEqual(features.shape[1], 2)  # x, y coordinates

    def test_track_features_first_frame(self):
        """Test tracking on first frame."""
        image = np.random.randint(0, 255, (480, 640), dtype=np.uint8)

        curr_points, prev_points, track_ids = self.tracker.track_features(image)

        # First frame: current and previous should be same
        self.assertEqual(len(curr_points), len(prev_points))
        self.assertEqual(len(curr_points), len(track_ids))

    def test_track_features_multiple_frames(self):
        """Test tracking across multiple frames."""
        # Create sequence of images with moving feature
        for i in range(5):
            image = np.zeros((480, 640), dtype=np.uint8)
            x_pos = 100 + i * 10
            cv2.circle(image, (x_pos, 200), 20, 255, -1)

            curr_points, prev_points, track_ids = self.tracker.track_features(image)

            self.assertGreater(len(curr_points), 0)

    def test_reset(self):
        """Test tracker reset."""
        image = np.random.randint(0, 255, (480, 640), dtype=np.uint8)
        self.tracker.track_features(image)

        self.tracker.reset()

        self.assertIsNone(self.tracker.prev_frame)
        self.assertIsNone(self.tracker.prev_points)
        self.assertEqual(len(self.tracker.track_ids), 0)


if __name__ == "__main__":
    unittest.main()
