"""
Factor Graph State Estimator

Uses GTSAM iSAM2 for incremental optimization of:
- Pose (SE3)
- Velocity (R3)
- IMU biases (accelerometer + gyroscope)

Visual features are handled via SmartProjectionPose3Factor
which marginalizes out 3D landmark positions analytically.
"""

import numpy as np

try:
    import gtsam
    from gtsam import symbol_shorthand
    X = symbol_shorthand.X  # Pose keys
    V = symbol_shorthand.V  # Velocity keys
    B = symbol_shorthand.B  # Bias keys
    HAS_GTSAM = True
except ImportError:
    HAS_GTSAM = False


class StateEstimator:
    """
    GTSAM iSAM2-based VIO state estimator with sliding window.
    """

    def __init__(self, window_size=10, camera_cal=None):
        """
        Args:
            window_size: Number of keyframes in the sliding window.
            camera_cal: Dict with fx, fy, cx, cy for camera calibration.
        """
        if not HAS_GTSAM:
            raise ImportError(
                'GTSAM is required for the state estimator. '
                'Install with: pip install gtsam')

        self.window_size = window_size
        self.keyframe_count = 0
        self.initialized = False

        # iSAM2 setup
        isam_params = gtsam.ISAM2Params()
        isam_params.setRelinearizeThreshold(0.1)
        isam_params.setRelinearizeSkip(1)
        self.isam = gtsam.ISAM2(isam_params)

        # Noise models
        self.prior_pose_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([
            0.01, 0.01, 0.01,  # rotation (rad)
            0.1, 0.1, 0.1      # translation (m)
        ]))
        self.prior_vel_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
        self.prior_bias_noise = gtsam.noiseModel.Isotropic.Sigma(6, 1e-3)

        # Visual measurement noise (reprojection error in pixels, mapped to normalized)
        self.visual_noise = gtsam.noiseModel.Isotropic.Sigma(2, 1.5)

        # Robust kernel for visual factors (Huber norm to handle outliers)
        self.visual_noise_robust = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber.Create(1.345),
            self.visual_noise
        )

        # Camera calibration
        self.camera_cal = None
        if camera_cal is not None:
            self.set_camera_calibration(**camera_cal)

        # Current state estimates
        self.current_pose = gtsam.Pose3()
        self.current_vel = np.zeros(3)
        self.current_bias = gtsam.imuBias.ConstantBias()

        # Smart factor bookkeeping
        # feature_id -> {factor, keys: [X(i), ...], measurements: [...]}
        self.feature_tracks = {}

        # Bias noise parameters for between-factor
        self.bias_between_sigmas = np.array([
            0.0001, 0.0001, 0.0001,   # accel bias drift
            0.00001, 0.00001, 0.00001  # gyro bias drift
        ])

    def set_camera_calibration(self, fx, fy, cx, cy):
        """Set camera intrinsics."""
        self.camera_cal = gtsam.Cal3_S2(fx, fy, 0.0, cx, cy)

    def initialize(self, R_body_to_world, velocity, accel_bias, gyro_bias):
        """
        Initialize the factor graph with priors on the first keyframe.

        Args:
            R_body_to_world: 3x3 rotation matrix (body to world).
            velocity: 3-vector initial velocity.
            accel_bias: 3-vector accelerometer bias.
            gyro_bias: 3-vector gyroscope bias.
        """
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()

        init_pose = gtsam.Pose3(
            gtsam.Rot3(R_body_to_world),
            gtsam.Point3(0.0, 0.0, 0.0)
        )
        init_vel = np.array(velocity, dtype=np.float64)
        init_bias = gtsam.imuBias.ConstantBias(
            np.array(accel_bias, dtype=np.float64),
            np.array(gyro_bias, dtype=np.float64)
        )

        i = 0
        # Add priors
        graph.addPriorFactorPose3(X(i), init_pose, self.prior_pose_noise)
        graph.addPriorFactorVector(V(i), init_vel, self.prior_vel_noise)
        graph.addPriorFactorConstantBias(B(i), init_bias, self.prior_bias_noise)

        # Add initial values
        values.insert(X(i), init_pose)
        values.insert(V(i), init_vel)
        values.insert(B(i), init_bias)

        # Update iSAM2
        self.isam.update(graph, values)

        self.current_pose = init_pose
        self.current_vel = init_vel
        self.current_bias = init_bias
        self.keyframe_count = 1
        self.initialized = True

    def add_keyframe(self, pim, feature_observations):
        """
        Add a new keyframe to the factor graph.

        Args:
            pim: GTSAM PreintegratedImuMeasurements between prev and current keyframe.
            feature_observations: List of (feature_id, u_normalized, v_normalized)
                where u,v are in normalized (calibrated) camera coordinates.

        Returns:
            (pose, velocity, bias) tuple, or None if not initialized.
        """
        if not self.initialized:
            return None
        if self.camera_cal is None:
            return None

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()

        i = self.keyframe_count
        prev = i - 1

        # --- IMU Factor ---
        imu_factor = gtsam.ImuFactor(
            X(prev), V(prev),
            X(i), V(i),
            B(prev),
            pim
        )
        graph.add(imu_factor)

        # --- Bias evolution factor ---
        dt = pim.deltaTij()
        bias_noise = gtsam.noiseModel.Diagonal.Sigmas(
            self.bias_between_sigmas * np.sqrt(max(dt, 1e-4))
        )
        graph.add(gtsam.BetweenFactorConstantBias(
            B(prev), B(i),
            gtsam.imuBias.ConstantBias(),
            bias_noise
        ))

        # --- Visual factors (SmartProjectionPose3Factor) ---
        for (fid, u, v) in feature_observations:
            measurement = gtsam.Point2(u, v)

            if fid not in self.feature_tracks:
                # New feature: create smart factor
                smart_params = gtsam.SmartProjectionParams(
                    gtsam.HESSIAN, gtsam.ZERO_ON_DEGENERACY)
                factor = gtsam.SmartProjectionPose3Factor(
                    self.visual_noise_robust, self.camera_cal, smart_params)
                self.feature_tracks[fid] = {
                    'factor': factor,
                    'first_keyframe': i,
                }

            self.feature_tracks[fid]['factor'].add(measurement, X(i))

        # Add all smart factors with >= 2 observations to the graph
        for fid, track in self.feature_tracks.items():
            if track['factor'].size() >= 2:
                graph.add(track['factor'])

        # --- Initial values from IMU prediction ---
        predicted = pim.predict(
            gtsam.NavState(self.current_pose, self.current_vel),
            self.current_bias
        )
        values.insert(X(i), predicted.pose())
        values.insert(V(i), predicted.velocity())
        values.insert(B(i), self.current_bias)

        # --- Update iSAM2 ---
        try:
            self.isam.update(graph, values)
            # Run a few more iterations for convergence
            self.isam.update()
            result = self.isam.calculateEstimate()

            self.current_pose = result.atPose3(X(i))
            self.current_vel = result.atVector(V(i))
            self.current_bias = result.atConstantBias(B(i))
        except RuntimeError as e:
            # Optimization failure: use IMU prediction as fallback
            self.current_pose = predicted.pose()
            self.current_vel = predicted.velocity()
            # Don't update bias on failure
            return self.current_pose, self.current_vel, self.current_bias

        self.keyframe_count += 1

        # --- Prune old feature tracks ---
        self._prune_old_tracks()

        return self.current_pose, self.current_vel, self.current_bias

    def _prune_old_tracks(self):
        """Remove feature tracks that are no longer being observed."""
        oldest_kept = max(0, self.keyframe_count - self.window_size)
        stale = []
        for fid, track in self.feature_tracks.items():
            # If the feature was first seen before the window and hasn't
            # been updated recently, remove it.
            # SmartFactor doesn't easily expose which keys it references,
            # so we use a heuristic: remove tracks older than 2x window.
            if track['first_keyframe'] < oldest_kept:
                stale.append(fid)

        for fid in stale:
            del self.feature_tracks[fid]

    def get_state(self):
        """Get current estimated state."""
        if not self.initialized:
            return None

        t = self.current_pose.translation()
        q = self.current_pose.rotation().toQuaternion()

        return {
            'position': np.array([t[0], t[1], t[2]]),
            'orientation_quat': np.array([q.x(), q.y(), q.z(), q.w()]),
            'velocity': self.current_vel.copy(),
            'accel_bias': self.current_bias.accelerometer(),
            'gyro_bias': self.current_bias.gyroscope(),
            'keyframe_count': self.keyframe_count,
        }
