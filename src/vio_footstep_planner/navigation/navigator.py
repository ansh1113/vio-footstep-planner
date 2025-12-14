"""Visual-Inertial Odometry Navigator."""

import numpy as np
from typing import List, Tuple, Optional, Callable
import cv2


class VIONavigator:
    """Visual-Inertial Odometry Navigator integrating VIO estimation and drift correction."""
    
    def __init__(self, vins_config: Optional[str] = None, drift_correction: bool = True):
        """Initialize VIO Navigator.
        
        Args:
            vins_config: Path to VIO configuration file
            drift_correction: Enable drift correction
        """
        self.vins_config = vins_config
        self.drift_correction_enabled = drift_correction
        
        # State
        self.current_pose = np.zeros(6)  # x, y, z, roll, pitch, yaw
        self.pose_history = []
        self.is_initialized = False
        self.is_running = False
        
        # VIO components (lazy loaded)
        self.vio_estimator = None
        self.feature_tracker = None
        self.drift_corrector = None
        
        # Callbacks
        self.pose_callbacks = []
    
    def start(self):
        """Start VIO estimation."""
        if self.is_running:
            print("VIO Navigator already running")
            return
        
        # Initialize VIO components
        self._initialize_vio()
        
        self.is_running = True
        print("VIO Navigator started")
    
    def stop(self):
        """Stop VIO estimation."""
        self.is_running = False
        print("VIO Navigator stopped")
    
    def _initialize_vio(self):
        """Initialize VIO components."""
        from ..vio.vio_estimator import VIOEstimator
        from ..vio.feature_tracker import FeatureTracker
        
        # Default camera intrinsics (can be loaded from config)
        camera_intrinsics = np.array([
            [615.0, 0.0, 320.0],
            [0.0, 615.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        
        # Default IMU parameters
        imu_params = {
            'acc_n': 0.2,
            'gyr_n': 0.02,
            'acc_w': 0.002,
            'gyr_w': 0.0002
        }
        
        # Initialize components
        self.vio_estimator = VIOEstimator(camera_intrinsics, imu_params)
        self.feature_tracker = FeatureTracker(max_features=150, min_distance=20)
        
        # Initialize drift corrector if enabled
        if self.drift_correction_enabled:
            from ..drift_correction.corrector import DriftCorrector
            self.drift_corrector = DriftCorrector(
                loop_closure_threshold=0.7,
                enable_pose_graph_optimization=True
            )
        
        self.is_initialized = True
    
    def process_frame(self, image: np.ndarray, timestamp: float,
                     imu_acc: Optional[np.ndarray] = None,
                     imu_gyro: Optional[np.ndarray] = None) -> bool:
        """Process camera frame and IMU data.
        
        Args:
            image: Camera image (grayscale or BGR)
            timestamp: Frame timestamp (seconds)
            imu_acc: Accelerometer reading [ax, ay, az] (m/s^2)
            imu_gyro: Gyroscope reading [wx, wy, wz] (rad/s)
            
        Returns:
            True if pose was successfully updated
        """
        if not self.is_initialized:
            self._initialize_vio()
        
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # Track features
        curr_points, prev_points, track_ids = self.feature_tracker.track_features(gray)
        
        # Process IMU if provided
        if imu_acc is not None and imu_gyro is not None:
            self.vio_estimator.process_imu(timestamp, imu_acc, imu_gyro)
        
        # Update VIO estimate
        if len(curr_points) > 0:
            success = self.vio_estimator.process_image(timestamp, curr_points, track_ids)
            
            if success:
                # Get current pose
                self.current_pose = self.vio_estimator.get_pose()
                self.pose_history.append(self.current_pose.copy())
                
                # Apply drift correction
                if self.drift_correction_enabled and self.drift_corrector is not None:
                    self.drift_corrector.add_pose(self.current_pose, gray)
                    
                    # Get corrected pose
                    corrected = self.drift_corrector.get_corrected_pose()
                    if corrected is not None:
                        self.current_pose = corrected
                
                # Notify callbacks
                for callback in self.pose_callbacks:
                    callback(self.current_pose)
                
                return True
        
        return False
    
    def update(self, camera_image: np.ndarray, imu_data: dict):
        """Update pose estimate with new sensor data.
        
        Args:
            camera_image: Camera image
            imu_data: Dictionary with 'timestamp', 'acc', 'gyro'
        """
        timestamp = imu_data.get('timestamp', 0.0)
        acc = imu_data.get('acc', np.zeros(3))
        gyro = imu_data.get('gyro', np.zeros(3))
        
        self.process_frame(camera_image, timestamp, acc, gyro)
    
    def get_pose(self) -> np.ndarray:
        """Get current estimated pose.
        
        Returns:
            Current pose [x, y, z, roll, pitch, yaw]
        """
        return self.current_pose.copy()
    
    def get_pose_matrix(self) -> np.ndarray:
        """Get current pose as 4x4 transformation matrix.
        
        Returns:
            4x4 homogeneous transformation matrix
        """
        if self.vio_estimator is not None:
            return self.vio_estimator.get_pose_matrix()
        
        # Fallback
        from scipy.spatial.transform import Rotation
        T = np.eye(4)
        T[:3, 3] = self.current_pose[:3]
        T[:3, :3] = Rotation.from_euler('xyz', self.current_pose[3:], degrees=False).as_matrix()
        return T
    
    def get_velocity(self) -> np.ndarray:
        """Get current velocity estimate.
        
        Returns:
            Current velocity [vx, vy, vz]
        """
        if self.vio_estimator is not None:
            return self.vio_estimator.get_velocity()
        return np.zeros(3)
    
    def register_pose_callback(self, callback: Callable):
        """Register callback for pose updates.
        
        Args:
            callback: Function to call with new pose
        """
        self.pose_callbacks.append(callback)
    
    def get_statistics(self) -> dict:
        """Get VIO statistics.
        
        Returns:
            Dictionary with statistics
        """
        stats = {
            'num_poses': len(self.pose_history),
            'is_running': self.is_running,
            'is_initialized': self.is_initialized
        }
        
        # Add drift correction stats
        if self.drift_correction_enabled and self.drift_corrector is not None:
            drift_stats = self.drift_corrector.get_statistics()
            stats.update(drift_stats)
        
        return stats
    
    def reset(self):
        """Reset navigator state."""
        self.current_pose = np.zeros(6)
        self.pose_history = []
        
        if self.vio_estimator is not None:
            self.vio_estimator.reset()
        
        if self.feature_tracker is not None:
            self.feature_tracker.reset()
        
        if self.drift_corrector is not None:
            self.drift_corrector.clear()
        
        print("VIO Navigator reset")
    
    def set_initial_pose(self, pose: np.ndarray):
        """Set initial pose.
        
        Args:
            pose: Initial pose [x, y, z, roll, pitch, yaw]
        """
        self.current_pose = pose.copy()
        
        if self.vio_estimator is not None:
            self.vio_estimator.set_initial_pose(pose[:3], pose[3:])

