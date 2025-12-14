"""Unit tests for footstep planner."""

import unittest
import numpy as np
from vio_footstep_planner.planning.footstep_planner import FootstepPlanner


class TestFootstepPlanner(unittest.TestCase):
    """Test cases for FootstepPlanner."""
    
    def setUp(self):
        """Set up test fixtures."""
        config = {
            'max_step_length': 0.4,
            'goal_tolerance': 0.2
        }
        self.planner = FootstepPlanner(robot_model='spot', config=config)
        
    def test_initialization(self):
        """Test planner initialization."""
        self.assertEqual(self.planner.robot_model, 'spot')
        self.assertEqual(self.planner.max_step_length, 0.4)
        
    def test_plan_straight_path(self):
        """Test planning a straight path."""
        current_pose = np.array([0, 0, 0, 0, 0, 0])
        goal = [2.0, 0.0, 0.0]
        
        footsteps = self.planner.plan(current_pose, goal)
        
        self.assertIsNotNone(footsteps)
        self.assertGreater(len(footsteps), 0)
        
        # Check that plan progresses toward goal
        first_step = footsteps[0]
        last_step = footsteps[-1]
        
        self.assertGreater(last_step[0], first_step[0])
        
    def test_plan_with_rotation(self):
        """Test planning with rotation."""
        current_pose = np.array([0, 0, 0, 0, 0, 0])
        goal = [1.0, 1.0, 0.5]  # With rotation
        
        footsteps = self.planner.plan(current_pose, goal)
        
        self.assertIsNotNone(footsteps)
        
    def test_reached_goal(self):
        """Test goal reaching check."""
        self.planner.current_pose = np.array([2.0, 3.0, 0, 0, 0, 0])
        goal = [2.1, 3.1, 0]
        
        reached = self.planner.reached_goal(goal)
        self.assertTrue(reached)
        
        # Test far goal
        far_goal = [10.0, 10.0, 0]
        not_reached = self.planner.reached_goal(far_goal)
        self.assertFalse(not_reached)
        
    def test_get_plan_statistics(self):
        """Test plan statistics."""
        current_pose = np.array([0, 0, 0, 0, 0, 0])
        goal = [2.0, 0.0, 0.0]
        
        footsteps = self.planner.plan(current_pose, goal)
        stats = self.planner.get_plan_statistics()
        
        self.assertIn('num_steps', stats)
        self.assertIn('total_distance', stats)
        self.assertIn('avg_step_length', stats)
        
        self.assertGreater(stats['num_steps'], 0)


if __name__ == '__main__':
    unittest.main()
