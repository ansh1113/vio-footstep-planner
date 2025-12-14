"""Unit tests for VIO estimator."""

import unittest
import numpy as np
from vio_footstep_planner.vio.vio_estimator import VIOEstimator


class TestVIOEstimator(unittest.TestCase):
    """Test cases for VIOEstimator."""
    
    def setUp(self):
        """Set up test fixtures."""
        camera_intrinsics = np.array([
            [615.0, 0.0, 320.0],
            [0.0, 615.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        
        imu_params = {
            'acc_n': 0.2,
            'gyr_n': 0.02,
            'acc_w': 0.002,
            'gyr_w': 0.0002
        }
        
        self.estimator = VIOEstimator(camera_intrinsics, imu_params)
        
    def test_initialization(self):
        """Test estimator initialization."""
        pose = self.estimator.get_pose()
        
        self.assertEqual(len(pose), 6)
        np.testing.assert_array_almost_equal(pose, np.zeros(6))
        
    def test_process_imu(self):
        """Test IMU processing."""
        acc = np.array([0.0, 0.0, -9.81])
        gyro = np.array([0.0, 0.0, 0.0])
        
        self.estimator.process_imu(0.0, acc, gyro)
        
        self.assertEqual(len(self.estimator.imu_buffer), 1)
        
    def test_process_image(self):
        """Test image processing."""
        features = np.random.rand(50, 2) * 100 + 200  # Random features
        feature_ids = list(range(50))
        
        # Add some IMU data first
        self.estimator.process_imu(0.0, np.array([0, 0, -9.81]), np.zeros(3))
        self.estimator.process_imu(0.1, np.array([0, 0, -9.81]), np.zeros(3))
        
        success = self.estimator.process_image(0.1, features, feature_ids)
        
        self.assertTrue(success)
        
    def test_get_pose_matrix(self):
        """Test pose matrix retrieval."""
        T = self.estimator.get_pose_matrix()
        
        self.assertEqual(T.shape, (4, 4))
        np.testing.assert_array_almost_equal(T, np.eye(4))
        
    def test_set_initial_pose(self):
        """Test setting initial pose."""
        position = np.array([1.0, 2.0, 0.5])
        orientation = np.array([0.0, 0.0, 0.5])
        
        self.estimator.set_initial_pose(position, orientation)
        
        pose = self.estimator.get_pose()
        np.testing.assert_array_almost_equal(pose[:3], position)
        
    def test_reset(self):
        """Test estimator reset."""
        # Add some data
        self.estimator.process_imu(0.0, np.array([0, 0, -9.81]), np.zeros(3))
        
        self.estimator.reset()
        
        pose = self.estimator.get_pose()
        np.testing.assert_array_almost_equal(pose, np.zeros(6))
        self.assertEqual(len(self.estimator.imu_buffer), 0)


if __name__ == '__main__':
    unittest.main()
