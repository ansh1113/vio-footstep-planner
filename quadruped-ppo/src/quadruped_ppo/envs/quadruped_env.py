import gym
from gym import spaces
import pybullet as p
import pybullet_data
import numpy as np

class QuadrupedEnv(gym.Env):
    """Quadruped locomotion environment using PyBullet."""
    
    metadata = {'render.modes': ['human', 'rgb_array']}
    
    def __init__(self, terrain_type='flat', terrain_difficulty=0.5, render=False):
        super(QuadrupedEnv, self).__init__()
        
        self.terrain_type = terrain_type
        self.terrain_difficulty = terrain_difficulty
        self.render_enabled = render
        
        # Simulation parameters
        self.dt = 1/240
        self.control_freq = 50  # Hz
        self.control_steps = int(1/(self.control_freq * self.dt))
        
        # Robot parameters
        self.num_joints = 12
        
        # Action space: joint position targets [-1, 1]
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(self.num_joints,), dtype=np.float32
        )
        
        # Observation space (48-dim as described in README)
        obs_dim = 48
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )
        
        # Connect to PyBullet
        if self.render_enabled:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        self.robot_id = None
        self.plane_id = None
        self.prev_action = np.zeros(self.num_joints)
        self.step_counter = 0
        
    def reset(self):
        """Reset environment to initial state."""
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.dt)
        
        # Load plane with terrain
        self.plane_id = self._create_terrain()
        
        # Load quadruped robot
        start_pos = [0, 0, 0.5]
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        
        # Load actual quadruped URDF (placeholder - use your own)
        # self.robot_id = p.loadURDF("quadruped.urdf", start_pos, start_orientation)
        
        # For demo: create simple body
        self.robot_id = p.loadURDF("sphere2.urdf", start_pos, start_orientation)
        
        self.prev_action = np.zeros(self.num_joints)
        self.step_counter = 0
        
        return self._get_observation()
    
    def step(self, action):
        """Execute one environment step."""
        action = np.clip(action, -1.0, 1.0)
        
        # Apply control multiple times
        for _ in range(self.control_steps):
            # Set joint positions (simplified)
            p.stepSimulation()
        
        self.step_counter += 1
        
        obs = self._get_observation()
        reward = self._compute_reward(action)
        done = self._is_done()
        
        info = {
            'step': self.step_counter,
            'fell': done and self.step_counter < 1000
        }
        
        self.prev_action = action.copy()
        
        return obs, reward, done, info
    
    def _create_terrain(self):
        """Create terrain based on type."""
        if self.terrain_type == 'flat':
            return p.loadURDF("plane.urdf")
        else:
            # Create heightfield for uneven terrain
            return p.loadURDF("plane.urdf")  # Simplified
    
    def _get_observation(self):
        """Get current observation."""
        if self.robot_id is None:
            return np.zeros(48)
        
        # Get robot state
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        vel, ang_vel = p.getBaseVelocity(self.robot_id)
        
        # Build observation (48-dim)
        obs = np.zeros(48)
        obs[0:3] = pos  # Body position
        obs[3:7] = orn  # Body orientation (quaternion)
        obs[7:10] = vel  # Linear velocity
        obs[10:13] = ang_vel  # Angular velocity
        # Joint states, contacts, etc. would go in remaining dims
        
        return obs.astype(np.float32)
    
    def _compute_reward(self, action):
        """Compute reward as described in README."""
        reward = 0.0
        
        if self.robot_id is None:
            return reward
        
        # Forward velocity (primary objective)
        vel, _ = p.getBaseVelocity(self.robot_id)
        reward += 1.5 * vel[0]  # x-direction velocity
        
        # Penalize lateral movement
        reward -= 0.5 * abs(vel[1])
        
        # Check if fallen
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        if pos[2] < 0.2:
            reward -= 10.0
        
        # Penalize extreme orientations
        euler = p.getEulerFromQuaternion(orn)
        reward -= 0.5 * (abs(euler[0]) + abs(euler[1]))
        
        # Energy efficiency
        reward -= 0.01 * np.sum(np.square(action))
        
        # Smooth motion
        reward -= 0.05 * np.sum(np.abs(action - self.prev_action))
        
        # Survival bonus
        reward += 0.1
        
        return reward
    
    def _is_done(self):
        """Check if episode should terminate."""
        if self.step_counter >= 1000:
            return True
        
        if self.robot_id is not None:
            pos, _ = p.getBasePositionAndOrientation(self.robot_id)
            if pos[2] < 0.15:  # Fallen
                return True
        
        return False
    
    def render(self, mode='human'):
        """Render environment."""
        pass
    
    def close(self):
        """Clean up."""
        p.disconnect()
