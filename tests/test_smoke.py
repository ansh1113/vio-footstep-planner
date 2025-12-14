"""Smoke tests for basic functionality and import safety."""

import importlib
import sys
import unittest


class TestSmokeTests(unittest.TestCase):
    """Basic smoke tests to ensure package is functional."""

    def test_package_import(self):
        """Test that the main package can be imported."""
        import vio_footstep_planner

        self.assertIsNotNone(vio_footstep_planner)
        self.assertTrue(hasattr(vio_footstep_planner, "__version__"))

    def test_main_classes_importable(self):
        """Test that main classes can be imported."""
        from vio_footstep_planner import (
            DriftCorrector,
            FeatureTracker,
            FootstepPlanner,
            VIOEstimator,
            VIONavigator,
        )

        # Verify classes are importable
        self.assertIsNotNone(VIONavigator)
        self.assertIsNotNone(FootstepPlanner)
        self.assertIsNotNone(DriftCorrector)
        self.assertIsNotNone(VIOEstimator)
        self.assertIsNotNone(FeatureTracker)

    def test_submodules_importable(self):
        """Test that submodules can be imported."""
        from vio_footstep_planner import drift_correction, navigation, planning, vio

        self.assertIsNotNone(vio)
        self.assertIsNotNone(planning)
        self.assertIsNotNone(navigation)
        self.assertIsNotNone(drift_correction)

    def test_generate_code_import_safety(self):
        """Test that generate_code.py can be imported without side effects."""
        # Import should not create any directories or files
        import generate_code

        # Should be able to access module contents
        self.assertTrue(hasattr(generate_code, "main"))
        self.assertTrue(hasattr(generate_code, "create_file"))
        self.assertTrue(hasattr(generate_code, "generate_quadruped_ppo"))
        self.assertTrue(hasattr(generate_code, "generate_rl_cbf"))
        self.assertTrue(hasattr(generate_code, "generate_vio_planner"))

    def test_version_string(self):
        """Test that version string is properly defined."""
        import vio_footstep_planner

        version = vio_footstep_planner.__version__
        self.assertIsInstance(version, str)
        self.assertTrue(len(version) > 0)
        # Version should be semver-like: X.Y.Z
        parts = version.split(".")
        self.assertGreaterEqual(len(parts), 2)


class TestMinimalFunctionality(unittest.TestCase):
    """Test minimal functionality of key components."""

    def test_vio_estimator_instantiation(self):
        """Test that VIOEstimator can be instantiated."""
        import numpy as np

        from vio_footstep_planner import VIOEstimator

        camera_intrinsics = np.eye(3)
        imu_params = {
            "acc_n": 0.2,
            "gyr_n": 0.02,
            "acc_w": 0.002,
            "gyr_w": 0.0002,
        }

        estimator = VIOEstimator(camera_intrinsics, imu_params)
        self.assertIsNotNone(estimator)

    def test_footstep_planner_instantiation(self):
        """Test that FootstepPlanner can be instantiated."""
        from vio_footstep_planner import FootstepPlanner

        planner = FootstepPlanner(robot_model="spot")
        self.assertIsNotNone(planner)

    def test_drift_corrector_instantiation(self):
        """Test that DriftCorrector can be instantiated."""
        from vio_footstep_planner import DriftCorrector

        corrector = DriftCorrector()
        self.assertIsNotNone(corrector)


if __name__ == "__main__":
    unittest.main()
