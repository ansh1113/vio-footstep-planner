"""Unit tests for drift corrector."""

import unittest
import numpy as np
from vio_footstep_planner.drift_correction.corrector import DriftCorrector


class TestDriftCorrector(unittest.TestCase):
    """Test cases for DriftCorrector."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.corrector = DriftCorrector(
            loop_closure_threshold=0.7,
            enable_pose_graph_optimization=True
        )
        
    def test_initialization(self):
        """Test corrector initialization."""
        self.assertEqual(self.corrector.threshold, 0.7)
        self.assertTrue(self.corrector.enable_optimization)
        
    def test_add_pose(self):
        """Test adding poses."""
        pose = np.array([1.0, 2.0, 0.0, 0.0, 0.0, 0.5])
        
        self.corrector.add_pose(pose)
        
        self.assertEqual(len(self.corrector.pose_database), 1)
        
    def test_get_corrected_pose(self):
        """Test getting corrected pose."""
        pose = np.array([1.0, 2.0, 0.0, 0.0, 0.0, 0.5])
        self.corrector.add_pose(pose)
        
        corrected = self.corrector.get_corrected_pose()
        
        self.assertEqual(len(corrected), 6)
        
    def test_get_statistics(self):
        """Test statistics retrieval."""
        # Add some poses
        for i in range(10):
            pose = np.array([float(i), 0.0, 0.0, 0.0, 0.0, 0.0])
            self.corrector.add_pose(pose)
        
        stats = self.corrector.get_statistics()
        
        self.assertIn('num_loop_closures', stats)
        self.assertIn('total_poses', stats)
        self.assertEqual(stats['total_poses'], 10)
        
    def test_clear(self):
        """Test clearing corrector."""
        pose = np.array([1.0, 2.0, 0.0, 0.0, 0.0, 0.5])
        self.corrector.add_pose(pose)
        
        self.corrector.clear()
        
        self.assertEqual(len(self.corrector.pose_database), 0)
        self.assertEqual(self.corrector.current_id, 0)


if __name__ == '__main__':
    unittest.main()
