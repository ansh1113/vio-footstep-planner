"""Visual-Inertial Odometry Estimator."""

import numpy as np
from typing import Dict, List, Tuple, Optional
from scipy.spatial.transform import Rotation


class VIOEstimator:
    """Visual-Inertial Odometry estimator using tightly-coupled fusion.
    
    This is a simplified VIO implementation that demonstrates the key concepts:
    - IMU pre-integration between frames
    - Visual feature tracking and matching
    - Sliding window optimization
    - Pose estimation
    
    For production use, consider using VINS-Fusion or ORB-SLAM3.
    """
    
    def __init__(self, camera_intrinsics: np.ndarray, imu_params: Dict):
        """Initialize VIO estimator.
        
        Args:
            camera_intrinsics: Camera intrinsic matrix (3x3)
            imu_params: Dictionary with IMU noise parameters
                - acc_n: Accelerometer noise density
                - gyr_n: Gyroscope noise density
                - acc_w: Accelerometer random walk
                - gyr_w: Gyroscope random walk
        """
        self.K = camera_intrinsics
        self.imu_params = imu_params
        
        # State: [position, velocity, orientation, ba, bg]
        self.position = np.zeros(3)  # x, y, z
        self.velocity = np.zeros(3)  # vx, vy, vz
        self.orientation = Rotation.identity()  # Rotation object
        self.ba = np.zeros(3)  # Accelerometer bias
        self.bg = np.zeros(3)  # Gyroscope bias
        
        # Gravity vector (z-up convention)
        self.gravity = np.array([0, 0, -9.81])
        
        # History
        self.pose_history = []
        self.timestamp_history = []
        
        # Pre-integration buffer
        self.imu_buffer = []
        self.last_image_timestamp = None
        
    def process_imu(self, timestamp: float, acc: np.ndarray, gyro: np.ndarray):
        """Process IMU measurement.
        
        Args:
            timestamp: Measurement timestamp
            acc: Accelerometer reading (m/s^2)
            gyro: Gyroscope reading (rad/s)
        """
        self.imu_buffer.append({
            'timestamp': timestamp,
            'acc': acc.copy(),
            'gyro': gyro.copy()
        })
        
        # Keep only recent IMU data (last 2 seconds)
        if len(self.imu_buffer) > 1000:
            self.imu_buffer = self.imu_buffer[-1000:]
    
    def process_image(self, timestamp: float, features: np.ndarray, 
                     feature_ids: List[int]) -> bool:
        """Process image frame with tracked features.
        
        Args:
            timestamp: Image timestamp
            features: Feature points in image (N, 2)
            feature_ids: Feature track IDs
            
        Returns:
            True if pose was successfully updated
        """
        if self.last_image_timestamp is None:
            # First frame - initialize
            self.last_image_timestamp = timestamp
            self.pose_history.append(self.get_pose())
            self.timestamp_history.append(timestamp)
            return True
        
        # Get IMU measurements between frames
        imu_measurements = self._get_imu_between_frames(
            self.last_image_timestamp, timestamp
        )
        
        if len(imu_measurements) == 0:
            return False
        
        # IMU pre-integration
        delta_t = timestamp - self.last_image_timestamp
        delta_p, delta_v, delta_q = self._preintegrate_imu(imu_measurements)
        
        # Propagate state using IMU
        R = self.orientation.as_matrix()
        self.position = self.position + self.velocity * delta_t + 0.5 * self.gravity * delta_t**2 + R @ delta_p
        self.velocity = self.velocity + self.gravity * delta_t + R @ delta_v
        self.orientation = self.orientation * Rotation.from_quat(delta_q)
        
        # Store pose
        self.pose_history.append(self.get_pose())
        self.timestamp_history.append(timestamp)
        self.last_image_timestamp = timestamp
        
        # Keep sliding window (last 10 poses)
        if len(self.pose_history) > 10:
            self.pose_history = self.pose_history[-10:]
            self.timestamp_history = self.timestamp_history[-10:]
        
        return True
    
    def _get_imu_between_frames(self, t_start: float, t_end: float) -> List[Dict]:
        """Get IMU measurements between two timestamps."""
        measurements = []
        for imu in self.imu_buffer:
            if t_start <= imu['timestamp'] <= t_end:
                measurements.append(imu)
        return measurements
    
    def _preintegrate_imu(self, measurements: List[Dict]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Pre-integrate IMU measurements.
        
        Args:
            measurements: List of IMU measurements
            
        Returns:
            Tuple of (delta_position, delta_velocity, delta_quaternion)
        """
        delta_p = np.zeros(3)
        delta_v = np.zeros(3)
        delta_q = np.array([0, 0, 0, 1])  # Identity quaternion [x, y, z, w]
        
        if len(measurements) < 2:
            return delta_p, delta_v, delta_q
        
        for i in range(len(measurements) - 1):
            dt = measurements[i+1]['timestamp'] - measurements[i]['timestamp']
            
            if dt <= 0 or dt > 0.1:  # Sanity check
                continue
            
            # Get corrected measurements
            acc = measurements[i]['acc'] - self.ba
            gyro = measurements[i]['gyro'] - self.bg
            
            # Integrate rotation
            gyro_norm = np.linalg.norm(gyro)
            if gyro_norm > 1e-6:
                angle = gyro_norm * dt
                axis = gyro / gyro_norm
                dq = Rotation.from_rotvec(axis * angle).as_quat()
            else:
                dq = np.array([0, 0, 0, 1])
            
            # Compose quaternions
            delta_q = self._quaternion_multiply(delta_q, dq)
            
            # Integrate velocity and position (in local frame)
            R = Rotation.from_quat(delta_q).as_matrix()
            delta_v = delta_v + R @ acc * dt
            delta_p = delta_p + delta_v * dt + 0.5 * R @ acc * dt**2
        
        return delta_p, delta_v, delta_q
    
    def _quaternion_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Multiply two quaternions [x, y, z, w]."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        return np.array([
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        ])
    
    def get_pose(self) -> np.ndarray:
        """Get current pose as [x, y, z, roll, pitch, yaw]."""
        euler = self.orientation.as_euler('xyz', degrees=False)
        return np.concatenate([self.position, euler])
    
    def get_pose_matrix(self) -> np.ndarray:
        """Get current pose as 4x4 transformation matrix."""
        T = np.eye(4)
        T[:3, :3] = self.orientation.as_matrix()
        T[:3, 3] = self.position
        return T
    
    def get_velocity(self) -> np.ndarray:
        """Get current velocity."""
        return self.velocity.copy()
    
    def reset(self):
        """Reset estimator state."""
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = Rotation.identity()
        self.ba = np.zeros(3)
        self.bg = np.zeros(3)
        self.pose_history = []
        self.timestamp_history = []
        self.imu_buffer = []
        self.last_image_timestamp = None
    
    def set_initial_pose(self, position: np.ndarray, orientation: np.ndarray):
        """Set initial pose.
        
        Args:
            position: Initial position [x, y, z]
            orientation: Initial orientation as euler angles [roll, pitch, yaw]
        """
        self.position = position.copy()
        self.orientation = Rotation.from_euler('xyz', orientation, degrees=False)
    
    def get_covariance(self) -> np.ndarray:
        """Get pose covariance (simplified).
        
        Returns:
            6x6 covariance matrix for [x, y, z, roll, pitch, yaw]
        """
        # Simplified covariance based on IMU noise characteristics
        # In practice, this should be computed from the optimization
        cov = np.eye(6)
        cov[0:3, 0:3] *= 0.1  # Position uncertainty
        cov[3:6, 3:6] *= 0.05  # Orientation uncertainty
        return cov
