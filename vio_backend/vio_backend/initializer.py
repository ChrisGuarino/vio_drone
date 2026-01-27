"""
VIO Static Initializer

Accumulates IMU data while the drone is stationary to estimate:
- Initial orientation (gravity direction alignment)
- Gyroscope bias
- Accelerometer bias

Requires ~2 seconds of stationary IMU data at 200 Hz.
"""

import numpy as np

try:
    import gtsam
    HAS_GTSAM = True
except ImportError:
    HAS_GTSAM = False

from collections import deque


class VioInitializer:
    def __init__(self, required_samples=400, gravity=9.81):
        """
        Args:
            required_samples: Number of IMU samples needed (2s at 200Hz).
            gravity: Expected gravity magnitude (m/s^2).
        """
        self.required_samples = required_samples
        self.gravity_magnitude = gravity
        self.accel_buffer = deque(maxlen=required_samples)
        self.gyro_buffer = deque(maxlen=required_samples)
        self.initialized = False

        # Thresholds for stationarity detection
        self.accel_var_threshold = 0.1    # m^2/s^4
        self.gyro_var_threshold = 0.01    # rad^2/s^2

    def add_imu(self, accel, gyro):
        """Buffer an IMU measurement during initialization."""
        self.accel_buffer.append(np.array(accel, dtype=np.float64))
        self.gyro_buffer.append(np.array(gyro, dtype=np.float64))

    def ready(self):
        """Check if enough samples have been collected."""
        return len(self.accel_buffer) >= self.required_samples

    def try_initialize(self):
        """
        Attempt to initialize the VIO system.

        Returns:
            (success, init_rotation_matrix, init_velocity, gyro_bias, accel_bias)
            - init_rotation_matrix: 3x3 rotation from body to world frame
            - init_velocity: 3-vector (always zeros for static init)
            - gyro_bias: 3-vector estimated gyroscope bias
            - accel_bias: 3-vector estimated accelerometer bias
        """
        if not self.ready():
            return False, None, None, None, None

        accels = np.array(list(self.accel_buffer))
        gyros = np.array(list(self.gyro_buffer))

        # Check stationarity
        accel_var = np.var(accels, axis=0)
        gyro_var = np.var(gyros, axis=0)

        if np.max(accel_var) > self.accel_var_threshold:
            return False, None, None, None, None
        if np.max(gyro_var) > self.gyro_var_threshold:
            return False, None, None, None, None

        # Gyro bias = mean angular velocity while stationary
        gyro_bias = np.mean(gyros, axis=0)

        # Mean acceleration = gravity in body frame (with bias)
        mean_accel = np.mean(accels, axis=0)
        accel_norm = np.linalg.norm(mean_accel)

        # Accelerometer bias: difference between measured and expected gravity magnitude
        # Simple model: assume bias is along the gravity direction
        if accel_norm < 1e-6:
            return False, None, None, None, None

        scale_error = accel_norm - self.gravity_magnitude
        accel_bias = (scale_error / accel_norm) * mean_accel

        # Compute rotation: align body gravity with world gravity
        # In ENU convention: gravity = [0, 0, -g] (down = -z)
        # The accelerometer measures reaction force: -gravity in body frame
        # So mean_accel ≈ -R^T * g_world => g_body = mean_accel / |mean_accel|
        g_body = mean_accel / accel_norm
        g_world = np.array([0.0, 0.0, -1.0])  # gravity direction in ENU (downward)

        # We need R such that R * g_body = g_world (body-to-world rotation)
        # Note: accelerometer measures upward (reaction), so g_body points up
        # Actually the accelerometer reads +g when stationary (pointing up = positive z
        # in sensor frame if z-up). Let's just align them:
        R = self._rotation_between_vectors(-g_body, g_world)

        init_velocity = np.zeros(3)
        self.initialized = True

        return True, R, init_velocity, gyro_bias, accel_bias

    def _rotation_between_vectors(self, a, b):
        """
        Compute rotation matrix that rotates unit vector a to unit vector b.
        Uses Rodrigues' formula.
        """
        a = a / np.linalg.norm(a)
        b = b / np.linalg.norm(b)

        v = np.cross(a, b)
        s = np.linalg.norm(v)
        c = np.dot(a, b)

        if s < 1e-8:
            # Vectors are parallel
            if c > 0:
                return np.eye(3)
            else:
                # 180-degree rotation: find any perpendicular axis
                perp = np.array([1, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1, 0])
                axis = np.cross(a, perp)
                axis = axis / np.linalg.norm(axis)
                # 180-degree rotation around axis
                return 2.0 * np.outer(axis, axis) - np.eye(3)

        vx = np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])
        R = np.eye(3) + vx + vx @ vx * (1.0 - c) / (s * s)
        return R
