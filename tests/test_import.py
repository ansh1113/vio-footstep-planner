"""Smoke test to ensure main module can be imported safely."""

import importlib
import sys


def test_generate_code_import():
    """Test that generate_code.py can be imported without side effects."""
    # Remove if already imported
    if "generate_code" in sys.modules:
        del sys.modules["generate_code"]

    # Import should not raise exceptions or run main code
    try:
        module = importlib.import_module("generate_code")
        assert module is not None
        assert hasattr(module, "main")
        assert callable(module.main)
    except Exception as e:
        assert False, f"Failed to import generate_code: {e}"


def test_package_import():
    """Test that the main package can be imported."""
    try:
        import vio_footstep_planner

        assert vio_footstep_planner is not None
    except Exception as e:
        assert False, f"Failed to import vio_footstep_planner: {e}"


def test_submodules_import():
    """Test that key submodules can be imported."""
    submodules = [
        "vio_footstep_planner.navigation",
        "vio_footstep_planner.planning",
        "vio_footstep_planner.drift_correction",
        "vio_footstep_planner.vio",
    ]

    for submodule in submodules:
        try:
            module = importlib.import_module(submodule)
            assert module is not None
        except Exception as e:
            assert False, f"Failed to import {submodule}: {e}"
