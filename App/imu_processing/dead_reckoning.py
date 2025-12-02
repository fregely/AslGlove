# dead_reckoning.py
# pylint: disable=E1101
# mypy: ignore-errors

import numpy as np
import math
from collections import deque

class KalmanDeadReckoning:
    def __init__(self, sample_rate=20, p=0.1, q=0.01, r=0.1, gravity_convention='NED', 
                 a_still=0.02, g_still=0.052, av_still=0.01):
       
        self.dt = 1.0 / sample_rate
        self.state = np.zeros(6)  # [x, y, z, vx, vy, vz]
        self.P = np.eye(6) * p
        self.Q = np.eye(6) * q
        self.R = np.eye(3) * r

        # Gravity and alignment (set during warmup)
        self.gravity = None
        self.alignment_quaternion = None
        
        # BIAS CALIBRATION
        self.world_accel_bias = np.zeros(3)
        self.bias_samples = []
        self.BIAS_CALIBRATION_COUNT = 50
        self.bias_calibrated = False
        
        # ZUPT detection parameters
        self.accel_history = deque(maxlen=20)
        self.gyro_history = deque(maxlen=20)
        
        # Thresholds (will be set during warmup)
        self.ACCEL_STILL_THRESHOLD = a_still
        self.GYRO_STILL_THRESHOLD = g_still
        self.ACCEL_VAR_THRESHOLD = av_still
        
        # ZUPT state tracking
        self.zupt_active = False
        self.stationary_time = 0.0
        self.MIN_ZUPT_TIME = 0.1

    def _detect_stationary(self, accel_magnitude_g, gyro_magnitude_rad):
        """Detect if IMU is stationary using multiple criteria."""
        # Criterion 1: Current magnitude checks
        accel_near_gravity = abs(accel_magnitude_g - 1.0) < self.ACCEL_STILL_THRESHOLD
        gyro_low = gyro_magnitude_rad < self.GYRO_STILL_THRESHOLD
        
        # Criterion 2: Recent variance (more robust)
        if len(self.accel_history) >= 10:
            recent_accels = np.array(list(self.accel_history)[-10:])
            accel_variance = np.var(recent_accels)
            low_variance = accel_variance < self.ACCEL_VAR_THRESHOLD
        else:
            low_variance = False
        
        # Calculate confidence
        confidence = 0.0
        if accel_near_gravity:
            confidence += 0.4
        if gyro_low:
            confidence += 0.4
        if low_variance:
            confidence += 0.2
        
        is_stationary = confidence >= 0.6
        
        return is_stationary, confidence

    @staticmethod
    def calculate_alignment_quaternion(from_vec, to_vec):
        """
        Calculate quaternion that rotates from_vec to align with to_vec.
        
        Args:
            from_vec: Source vector (e.g., measured gravity in IMU frame)
            to_vec: Target vector (e.g., gravity in camera frame)
        
        Returns:
            Quaternion [w, x, y, z] representing the rotation
        """
        from_vec = from_vec / np.linalg.norm(from_vec)
        to_vec = to_vec / np.linalg.norm(to_vec)
        
        # Axis of rotation (cross product)
        axis = np.cross(from_vec, to_vec)
        axis_length = np.linalg.norm(axis)
        
        # Handle edge cases
        if axis_length < 1e-6:
            if np.dot(from_vec, to_vec) > 0:
                # Already aligned
                return np.array([1.0, 0.0, 0.0, 0.0])
            else:
                # Opposite - rotate 180° around perpendicular axis
                perp = np.array([1, 0, 0]) if abs(from_vec[0]) < 0.9 else np.array([0, 1, 0])
                axis = np.cross(from_vec, perp)
                axis = axis / np.linalg.norm(axis)
                return np.array([0.0, axis[0], axis[1], axis[2]])
        
        axis = axis / axis_length
        angle = np.arccos(np.clip(np.dot(from_vec, to_vec), -1.0, 1.0))
        
        # Convert axis-angle to quaternion
        half_angle = angle / 2.0
        s = np.sin(half_angle)
        
        return np.array([
            np.cos(half_angle),
            axis[0] * s,
            axis[1] * s,
            axis[2] * s
        ])

    def _quaternion_multiply(self, q1, q2):
        """
        Multiply two quaternions: q1 * q2
        Both quaternions in [w, x, y, z] format
        """
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    def calibrate_from_samples(self, accel_samples, gyro_samples, madgwick_quaternion, 
                        target_gravity, safety_margin=1.5, logger=None):
        """
        Complete calibration from collected warmup samples.
        
        This performs:
        1. Gravity measurement in IMU's world frame
        2. Alignment quaternion calculation to target frame
        3. ZUPT threshold calculation
        4. Internal parameter configuration
        
        Args:
            accel_samples: numpy array of shape (N, 3) - accelerometer in g
            gyro_samples: numpy array of shape (N, 3) - gyroscope in rad/s
            madgwick_quaternion: Current converged quaternion [w, x, y, z]
            target_gravity: Desired gravity in target frame [x, y, z] m/s²
                        (e.g., [0, +9.81, 0] for camera frame with Y=down)
            safety_margin: Multiplier for ZUPT thresholds (default: 1.5)
            logger: Optional logger for output messages
        
        Returns:
            dict: Calibration results with keys:
                - measured_gravity: Gravity measured in IMU frame
                - gravity_magnitude: Magnitude (should be ~9.81)
                - alignment_quat: Calculated alignment quaternion
                - accel_threshold: ZUPT accel threshold
                - gyro_threshold: ZUPT gyro threshold
                - variance_threshold: ZUPT variance threshold
        """
        import numpy as np
        
        # ============================================================
        # 1. MEASURE GRAVITY IN IMU'S WORLD FRAME
        # ============================================================
        gravity_samples_imu_world = []
        for accel_g in accel_samples:
            accel_ms2 = accel_g * 9.81
            accel_world = self._rotate_by_quaternion(accel_ms2, madgwick_quaternion)
            gravity_samples_imu_world.append(accel_world)
        
        measured_gravity_imu = np.mean(gravity_samples_imu_world, axis=0)
        gravity_magnitude = np.linalg.norm(measured_gravity_imu)
        
        if logger:
            logger.info(f"🌍 Measured gravity in IMU frame:")
            logger.info(f"   [{measured_gravity_imu[0]:+.3f}, {measured_gravity_imu[1]:+.3f}, {measured_gravity_imu[2]:+.3f}] m/s²")
            logger.info(f"   Magnitude: {gravity_magnitude:.2f} m/s² (expected: 9.81)")
        
        # ============================================================
        # 2. SET ALIGNMENT TO TARGET FRAME
        # ============================================================
        self.set_alignment_from_gravity(measured_gravity_imu, target_gravity)
        
        if logger:
            logger.info(f"✅ Aligned to target frame: [{target_gravity[0]:+.3f}, {target_gravity[1]:+.3f}, {target_gravity[2]:+.3f}]")
        
        # ============================================================
        # 3. CALCULATE ZUPT THRESHOLDS
        # ============================================================
        # Acceleration thresholds
        accel_magnitudes = np.linalg.norm(accel_samples, axis=1)
        accel_deviation = np.abs(accel_magnitudes - 1.0)
        accel_std = np.std(accel_deviation)
        accel_mean = np.mean(accel_deviation)
        accel_variance = np.var(accel_magnitudes)
        
        # Gyroscope thresholds
        gyro_magnitudes = np.linalg.norm(gyro_samples, axis=1)
        gyro_std = np.std(gyro_magnitudes)
        gyro_mean = np.mean(gyro_magnitudes)
        
        # Apply safety margin
        accel_threshold = (accel_mean + 3 * accel_std) * safety_margin
        gyro_threshold = (gyro_mean + 3 * gyro_std) * safety_margin
        variance_threshold = accel_variance * safety_margin
        
        # ============================================================
        # 4. UPDATE INTERNAL PARAMETERS
        # ============================================================
        self.ACCEL_STILL_THRESHOLD = accel_threshold
        self.GYRO_STILL_THRESHOLD = gyro_threshold
        self.ACCEL_VAR_THRESHOLD = variance_threshold
        
        if logger:
            logger.info(f"📊 ZUPT Thresholds:")
            logger.info(f"   Accel: {accel_mean*1000:.2f}mg ± {accel_std*1000:.2f}mg → {accel_threshold*1000:.2f}mg")
            logger.info(f"   Gyro:  {np.degrees(gyro_mean):.2f}°/s ± {np.degrees(gyro_std):.2f}°/s → {np.degrees(gyro_threshold):.2f}°/s")
            logger.info(f"   Variance: {variance_threshold*1000:.4f}mg²")
        
        # ============================================================
        # 5. RETURN CALIBRATION RESULTS
        # ============================================================
        return {
            'measured_gravity': measured_gravity_imu,
            'gravity_magnitude': gravity_magnitude,
            'alignment_quat': self.alignment_quaternion.copy(),
            'target_gravity': target_gravity,
            'accel_threshold': accel_threshold,
            'gyro_threshold': gyro_threshold,
            'variance_threshold': variance_threshold,
            'accel_stats': {
                'mean': accel_mean,
                'std': accel_std,
                'variance': accel_variance
            },
            'gyro_stats': {
                'mean': gyro_mean,
                'std': gyro_std
            }
        }
    
    def set_alignment_from_gravity(self, measured_gravity_imu, target_gravity_camera):
        """
        Calculate and store alignment quaternion from gravity measurements.
        Call this once during warmup calibration.
        
        Args:
            measured_gravity_imu: Gravity as measured in IMU's world frame [x, y, z] m/s²
            target_gravity_camera: Desired gravity in camera frame [x, y, z] m/s²
        """
        self.alignment_quaternion = self.calculate_alignment_quaternion(
            measured_gravity_imu,
            target_gravity_camera
        )
        self.gravity = target_gravity_camera
        
        print(f"🔄 Alignment set:")
        print(f"   From: [{measured_gravity_imu[0]:+.3f}, {measured_gravity_imu[1]:+.3f}, {measured_gravity_imu[2]:+.3f}]")
        print(f"   To:   [{target_gravity_camera[0]:+.3f}, {target_gravity_camera[1]:+.3f}, {target_gravity_camera[2]:+.3f}]")
        print(f"   Quat: [{self.alignment_quaternion[0]:.3f}, {self.alignment_quaternion[1]:+.3f}, {self.alignment_quaternion[2]:+.3f}, {self.alignment_quaternion[3]:+.3f}]")
    
    def process(self, packet: dict) -> dict:
        """
        Process IMU packet with alignment, bias calibration and ZUPT.
        Updates packet IN-PLACE and returns it.
        """
        quaternion = np.array(packet['quaternion'])
        accel_g = np.array(packet['accel'])
        dt = packet['dt']
        
        # Get gyro if available
        gyro_rad = packet.get('gyro', None)
        if gyro_rad is not None:
            gyro_rad = np.array(gyro_rad)
        
        # Calculate magnitudes for ZUPT detection
        accel_magnitude_g = np.linalg.norm(accel_g)
        gyro_magnitude_rad = np.linalg.norm(gyro_rad) if gyro_rad is not None else 0.0
        
        # Update history
        self.accel_history.append(accel_magnitude_g)
        if gyro_rad is not None:
            self.gyro_history.append(gyro_magnitude_rad)
        
        # Detect if stationary
        is_stationary, confidence = self._detect_stationary(
            accel_magnitude_g, gyro_magnitude_rad
        )
        
        # ============================================================
        # WAIT FOR ALIGNMENT TO BE SET
        # ============================================================
        if self.gravity is None or self.alignment_quaternion is None:
            packet['position'] = (0.0, 0.0, 0.0)
            packet['velocity'] = (0.0, 0.0, 0.0)
            packet['linear_accel'] = (0.0, 0.0, 0.0)
            packet['calibrating'] = True
            packet['calibration_progress'] = 0
            packet['is_stationary'] = is_stationary
            return packet
        
        # ============================================================
        # APPLY ALIGNMENT QUATERNION
        # ============================================================
        aligned_quaternion = self._quaternion_multiply(
            self.alignment_quaternion,
            quaternion
        )
        
        # ============================================================
        # BIAS CALIBRATION PHASE (using aligned quaternion!)
        # ============================================================
        if not self.bias_calibrated:
            # Calculate world-frame acceleration with ALIGNED quaternion
            accel_sensor_ms2 = accel_g * 9.81
            accel_camera_frame = self._rotate_by_quaternion(accel_sensor_ms2, aligned_quaternion)  # ← ALIGNED!
            linear_accel = accel_camera_frame - self.gravity
            
            # Only collect bias samples when HIGHLY confident it's stationary
            if is_stationary and confidence > 0.8:
                self.bias_samples.append(linear_accel.copy())
            
            # Check if calibration complete
            if len(self.bias_samples) >= self.BIAS_CALIBRATION_COUNT:
                self.world_accel_bias = np.mean(self.bias_samples, axis=0)
                self.bias_calibrated = True
                print(f"✅ Bias calibrated: [{self.world_accel_bias[0]:+.3f}, "
                    f"{self.world_accel_bias[1]:+.3f}, {self.world_accel_bias[2]:+.3f}] m/s²")
                print(f"   (Should be small: < 0.1 m/s²)")
            else:
                # Still calibrating
                packet['position'] = (0.0, 0.0, 0.0)
                packet['velocity'] = (0.0, 0.0, 0.0)
                packet['linear_accel'] = tuple(linear_accel)
                packet['calibrating'] = True
                packet['calibration_progress'] = len(self.bias_samples)
                packet['is_stationary'] = is_stationary
                return packet
        
        # ============================================================
        # NORMAL PROCESSING (after calibration, using aligned quaternion!)
        # ============================================================
        
        # Update ZUPT state
        if is_stationary:
            if not self.zupt_active:
                self.zupt_active = True
                self.stationary_time = 0.0
            else:
                self.stationary_time += dt
        else:
            self.zupt_active = False
            self.stationary_time = 0.0
        
        # Apply ZUPT when stationary
        apply_zupt = is_stationary
        
        # Update state (with bias correction and ALIGNED quaternion)
        result = self.update(aligned_quaternion, accel_g, dt, apply_zupt=apply_zupt)  # ← ALIGNED!
        
        # UPDATE PACKET IN-PLACE
        packet['position'] = tuple(result['position'])
        packet['velocity'] = tuple(result['velocity'])
        packet['linear_accel'] = tuple(result['linear_accel'])
        packet['accel_world'] = tuple(result['accel_world'])
        packet['accel_bias'] = tuple(result['accel_bias'])
        packet['zupt_applied'] = result['zupt_applied']
        packet['calibrating'] = False
        packet['is_stationary'] = is_stationary
        packet['zupt_info'] = {
            'active': self.zupt_active,
            'confidence': confidence,
            'stationary_time': self.stationary_time,
            'applied': apply_zupt
        }
        
        return packet
    
    def _rotate_by_quaternion(self, v, q):
        """Rotate vector v by quaternion q."""
        w, x, y, z = q
        vx, vy, vz = v

        # q ⊗ v
        t0 = -x*vx - y*vy - z*vz
        t1 = w*vx + y*vz - z*vy
        t2 = w*vy - x*vz + z*vx
        t3 = w*vz + x*vy - y*vx

        # q* conjugate
        qw_conj = w
        qx_conj = -x
        qy_conj = -y
        qz_conj = -z
        
        # (q ⊗ v) ⊗ q*
        result_x = t1*qw_conj - t0*qx_conj + t2*qz_conj - t3*qy_conj
        result_y = t2*qw_conj - t0*qy_conj - t1*qz_conj + t3*qx_conj
        result_z = t3*qw_conj - t0*qz_conj + t1*qy_conj - t2*qx_conj

        return np.array([result_x, result_y, result_z])

    def _predict(self, linear_accel, dt):
        """Kalman prediction step."""
        pos = self.state[0:3]
        vel = self.state[3:6]

        # Update velocity and position
        vel_new = vel + linear_accel * dt
        pos_new = pos + vel_new * dt

        self.state[0:3] = pos_new
        self.state[3:6] = vel_new

        # Update covariance
        A = np.eye(6)
        A[0:3, 3:6] = np.eye(3) * dt
        
        self.P = A @ self.P @ A.T + self.Q

    def _update_zupt(self):
        """Kalman update step - Zero velocity update."""
        # Measurement: velocity is zero
        z = np.zeros(3)

        # Observation matrix (we measure velocity)
        H = np.zeros((3, 6))
        H[0:3, 3:6] = np.eye(3)

        # Innovation (difference between measurement and prediction)
        y = z - H @ self.state

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # State correction
        self.state = self.state + K @ y

        # Covariance correction
        I = np.eye(6)
        self.P = (I - K @ H) @ self.P

    def update(self, quaternion, accel_sensor_g, dt, apply_zupt=False):        
        """Main update function."""
        quaternion = np.array(quaternion)
        accel_sensor_g = np.array(accel_sensor_g)
        
        # Convert to m/s²
        accel_sensor_ms2 = accel_sensor_g * 9.81
        
        # Rotate to world frame
        accel_world = self._rotate_by_quaternion(accel_sensor_ms2, quaternion)
        
        # Remove gravity
        linear_accel = accel_world - self.gravity
        
        # REMOVE CALIBRATED BIAS
        linear_accel_corrected = linear_accel - self.world_accel_bias
        
        # If applying ZUPT, force acceleration to zero (trust stationary detection)
        if apply_zupt:
            linear_accel_corrected = np.zeros(3)
        
        # PREDICTION (using corrected acceleration)
        self._predict(linear_accel_corrected, dt)
        
        # CORRECTION (Zero velocity update)
        if apply_zupt:
            self._update_zupt()
        
        return {
            'position': self.state[0:3].copy(),
            'velocity': self.state[3:6].copy(),
            'linear_accel': linear_accel_corrected,
            'accel_world': accel_world,
            'accel_bias': self.world_accel_bias.copy(),
            'zupt_applied': apply_zupt
        }
    
    def reset(self):
        """Reset filter state."""
        self.state = np.zeros(6)
        self.P = np.eye(6) * 0.1
        self.zupt_active = False
        self.stationary_time = 0.0
        self.accel_history.clear()
        self.gyro_history.clear()
        self.world_accel_bias = np.zeros(3)
        self.bias_samples = []
        self.bias_calibrated = False
    
    def get_position(self):
        return self.state[0:3].copy()
    
    def get_velocity(self):
        return self.state[3:6].copy()
    
    def is_calibrated(self):
        return self.bias_calibrated
    
    def get_calibration_progress(self):
        return len(self.bias_samples) / self.BIAS_CALIBRATION_COUNT if self.BIAS_CALIBRATION_COUNT > 0 else 0.0