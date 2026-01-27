"""
IMU Preintegration Module

Buffers IMU measurements and performs preintegration between keyframes
using GTSAM's PreintegratedImuMeasurements.

Falls back to a simple Euler integration if GTSAM is not available.
"""

import numpy as np
from collections import deque

try:
    import gtsam
    HAS_GTSAM = True
except ImportError:
    HAS_GTSAM = False


class ImuPreintegrator:
    """Manages IMU buffering and preintegration between keyframes."""

    def __init__(self, gravity_magnitude=9.81, noise_params=None):
        """
        Args:
            gravity_magnitude: Gravity magnitude in m/s^2.
            noise_params: Dict with keys:
                - gyro_noise: Gyroscope white noise (rad/s/sqrt(Hz))
                - accel_noise: Accelerometer white noise (m/s^2/sqrt(Hz))
                - gyro_walk: Gyroscope random walk (rad/s^2/sqrt(Hz))
                - accel_walk: Accelerometer random walk (m/s^3/sqrt(Hz))
                - integration_noise: Integration error sigma
        """
        if noise_params is None:
            noise_params = {
                'gyro_noise': 0.0003,
                'accel_noise': 0.003,
                'gyro_walk': 0.00001,
                'accel_walk': 0.0001,
                'integration_noise': 1e-5,
            }

        self.gravity_magnitude = gravity_magnitude
        self.noise_params = noise_params

        # IMU measurement buffer: (timestamp, accel[3], gyro[3])
        self.imu_buffer = deque(maxlen=4000)  # ~20s at 200 Hz

        # Current bias estimate (updated by optimizer)
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)

        # GTSAM preintegration setup
        if HAS_GTSAM:
            self.params = gtsam.PreintegrationParams.MakeSharedU(gravity_magnitude)
            self.params.setGyroscopeCovariance(
                noise_params['gyro_noise'] ** 2 * np.eye(3))
            self.params.setAccelerometerCovariance(
                noise_params['accel_noise'] ** 2 * np.eye(3))
            self.params.setIntegrationCovariance(
                noise_params['integration_noise'] ** 2 * np.eye(3))

            self.current_bias = gtsam.imuBias.ConstantBias()
            self.pim = None

    def add_imu_measurement(self, timestamp, accel, gyro):
        """Buffer a single IMU measurement."""
        self.imu_buffer.append((
            timestamp,
            np.array(accel, dtype=np.float64),
            np.array(gyro, dtype=np.float64)
        ))

    def preintegrate_between(self, t_start, t_end):
        """
        Preintegrate all buffered IMU measurements between two timestamps.

        Args:
            t_start: Start timestamp (seconds).
            t_end: End timestamp (seconds).

        Returns:
            GTSAM PreintegratedImuMeasurements object, or
            dict with delta_R, delta_v, delta_p if GTSAM unavailable.
        """
        if HAS_GTSAM:
            return self._preintegrate_gtsam(t_start, t_end)
        else:
            return self._preintegrate_euler(t_start, t_end)

    def _preintegrate_gtsam(self, t_start, t_end):
        """GTSAM-based preintegration."""
        pim = gtsam.PreintegratedImuMeasurements(self.params, self.current_bias)

        prev_t = None
        count = 0
        for (t, accel, gyro) in self.imu_buffer:
            if t < t_start:
                prev_t = t
                continue
            if t > t_end:
                break
            if prev_t is not None:
                dt = t - prev_t
                if 0 < dt < 0.05:  # sanity: reject gaps > 50ms
                    pim.integrateMeasurement(accel, gyro, dt)
                    count += 1
            prev_t = t

        self.pim = pim
        return pim

    def _preintegrate_euler(self, t_start, t_end):
        """
        Simple Euler integration fallback when GTSAM is not available.
        Returns a dict with preintegrated deltas.
        """
        delta_R = np.eye(3)
        delta_v = np.zeros(3)
        delta_p = np.zeros(3)
        delta_t = 0.0

        prev_t = None
        for (t, accel, gyro) in self.imu_buffer:
            if t < t_start:
                prev_t = t
                continue
            if t > t_end:
                break
            if prev_t is not None:
                dt = t - prev_t
                if 0 < dt < 0.05:
                    # Bias-corrected measurements
                    gyro_corrected = gyro - self.gyro_bias
                    accel_corrected = accel - self.accel_bias

                    # Update rotation (first-order Rodrigues)
                    angle = np.linalg.norm(gyro_corrected) * dt
                    if angle > 1e-10:
                        axis = gyro_corrected / np.linalg.norm(gyro_corrected)
                        K = np.array([
                            [0, -axis[2], axis[1]],
                            [axis[2], 0, -axis[0]],
                            [-axis[1], axis[0], 0]
                        ])
                        dR = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
                    else:
                        dR = np.eye(3)

                    # Update velocity and position
                    accel_world = delta_R @ accel_corrected
                    delta_p += delta_v * dt + 0.5 * accel_world * dt * dt
                    delta_v += accel_world * dt
                    delta_R = delta_R @ dR
                    delta_t += dt

            prev_t = t

        return {
            'delta_R': delta_R,
            'delta_v': delta_v,
            'delta_p': delta_p,
            'delta_t': delta_t,
        }

    def update_bias(self, accel_bias, gyro_bias):
        """Update bias estimates after optimization."""
        self.accel_bias = np.array(accel_bias, dtype=np.float64)
        self.gyro_bias = np.array(gyro_bias, dtype=np.float64)
        if HAS_GTSAM:
            self.current_bias = gtsam.imuBias.ConstantBias(
                self.accel_bias, self.gyro_bias)

    def get_predicted_nav_state(self, prev_pose, prev_vel):
        """
        Predict current state from previous state + preintegrated IMU.
        Only works with GTSAM.
        """
        if not HAS_GTSAM or self.pim is None:
            return None
        nav_state = gtsam.NavState(prev_pose, prev_vel)
        predicted = self.pim.predict(nav_state, self.current_bias)
        return predicted

    def clear_old_measurements(self, before_timestamp):
        """Remove IMU measurements older than the given timestamp."""
        while self.imu_buffer and self.imu_buffer[0][0] < before_timestamp:
            self.imu_buffer.popleft()
