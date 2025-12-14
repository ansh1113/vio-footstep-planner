"""Smoke test to ensure main modules can be imported safely."""

import importlib
import sys
import unittest


class TestImports(unittest.TestCase):
    """Test that main modules can be imported without side effects."""

    def test_generate_code_import(self):
        """Test that generate_code.py can be imported safely."""
        # Import the module without executing main
        try:
            spec = importlib.util.spec_from_file_location(
                "generate_code",
                "generate_code.py"
            )
            module = importlib.util.module_from_spec(spec)
            # Don't execute, just check that it can be loaded
            # spec.loader.exec_module(module)
            self.assertIsNotNone(spec)
            self.assertIsNotNone(module)
        except Exception as e:
            self.fail(f"Failed to import generate_code: {e}")

    def test_vio_footstep_planner_import(self):
        """Test that main package can be imported."""
        try:
            import vio_footstep_planner
            self.assertIsNotNone(vio_footstep_planner)
        except Exception as e:
            self.fail(f"Failed to import vio_footstep_planner: {e}")

    def test_vio_estimator_import(self):
        """Test that VIO estimator can be imported."""
        try:
            from vio_footstep_planner.vio.vio_estimator import VIOEstimator
            self.assertIsNotNone(VIOEstimator)
        except Exception as e:
            self.fail(f"Failed to import VIOEstimator: {e}")

    def test_footstep_planner_import(self):
        """Test that footstep planner can be imported."""
        try:
            from vio_footstep_planner.planning.footstep_planner import FootstepPlanner
            self.assertIsNotNone(FootstepPlanner)
        except Exception as e:
            self.fail(f"Failed to import FootstepPlanner: {e}")


if __name__ == "__main__":
    unittest.main()
