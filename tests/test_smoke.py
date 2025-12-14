"""Smoke tests for basic package functionality."""

import unittest


class TestImports(unittest.TestCase):
    """Test that package can be imported."""

    def test_import_package(self):
        """Test importing the main package."""
        import vio_footstep_planner
        self.assertTrue(hasattr(vio_footstep_planner, '__version__'))

    def test_import_navigator(self):
        """Test importing VIONavigator."""
        from vio_footstep_planner import VIONavigator
        self.assertIsNotNone(VIONavigator)

    def test_import_footstep_planner(self):
        """Test importing FootstepPlanner."""
        from vio_footstep_planner import FootstepPlanner
        self.assertIsNotNone(FootstepPlanner)

    def test_import_drift_corrector(self):
        """Test importing DriftCorrector."""
        from vio_footstep_planner import DriftCorrector
        self.assertIsNotNone(DriftCorrector)

    def test_import_vio_estimator(self):
        """Test importing VIOEstimator."""
        from vio_footstep_planner import VIOEstimator
        self.assertIsNotNone(VIOEstimator)

    def test_import_feature_tracker(self):
        """Test importing FeatureTracker."""
        from vio_footstep_planner import FeatureTracker
        self.assertIsNotNone(FeatureTracker)


class TestBasicFunctionality(unittest.TestCase):
    """Test basic functionality works."""

    def test_navigator_creation(self):
        """Test VIONavigator can be created."""
        from vio_footstep_planner import VIONavigator
        navigator = VIONavigator(drift_correction=False)
        self.assertIsNotNone(navigator)
        self.assertFalse(navigator.is_running)

    def test_planner_creation(self):
        """Test FootstepPlanner can be created."""
        from vio_footstep_planner import FootstepPlanner
        planner = FootstepPlanner(robot_model='spot', config={'max_step_length': 0.4})
        self.assertIsNotNone(planner)
        self.assertEqual(planner.robot_model, 'spot')

    def test_generate_code_importable(self):
        """Test that generate_code.py can be imported without side effects."""
        # This test ensures generate_code.py has been refactored properly
        import sys
        import os
        
        # Add current directory to path for import
        old_path = sys.path.copy()
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        
        try:
            import generate_code
            self.assertTrue(hasattr(generate_code, 'main'))
            # Check that importing doesn't create directories
            self.assertFalse(os.path.exists('quadruped-ppo'))
            self.assertFalse(os.path.exists('rl-locomotion-cbf'))
        finally:
            sys.path = old_path


if __name__ == '__main__':
    unittest.main()
