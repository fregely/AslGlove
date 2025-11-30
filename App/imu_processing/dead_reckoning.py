# dead_reckoning.py
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

        # Set gravity based on convention
        if gravity_convention == 'NED':
            self.gravity = np.array([0, 0, +9.81])  # Z down
        else:
            self.gravity = np.array([0, 0, -9.81])  # Z up (ENU)
        
        # BIAS CALIBRATION - Store WORLD-FRAME bias (what's left after gravity removal)
        self.world_accel_bias = np.zeros(3)
        self.bias_samples = []
        self.BIAS_CALIBRATION_COUNT = 50
        self.bias_calibrated = False
        
        # ZUPT detection parameters
        self.accel_history = deque(maxlen=20)
        self.gyro_history = deque(maxlen=20)
        
        # Thresholds for stationary detection
        self.ACCEL_STILL_THRESHOLD = a_still
        self.GYRO_STILL_THRESHOLD = g_still
        self.ACCEL_VAR_THRESHOLD = av_still
        
        # ZUPT state tracking
        self.zupt_active = False
        self.stationary_time = 0.0
        self.MIN_ZUPT_TIME = 0.1  # Reduced from 0.2s for faster response

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

    def process(self, packet: dict) -> dict:
        """
        Process IMU packet with bias calibration and ZUPT.
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
        # BIAS CALIBRATION PHASE
        # ============================================================
        if not self.bias_calibrated:
            # Calculate world-frame acceleration
            accel_sensor_ms2 = accel_g * 9.81
            accel_world = self._rotate_by_quaternion(accel_sensor_ms2, quaternion)
            linear_accel = accel_world - self.gravity
            
            # Only collect bias samples when HIGHLY confident it's stationary
            if is_stationary and confidence > 0.8:  # Higher threshold during calibration
                self.bias_samples.append(linear_accel.copy())
            
            # Check if calibration complete
            if len(self.bias_samples) >= self.BIAS_CALIBRATION_COUNT:
                self.world_accel_bias = np.mean(self.bias_samples, axis=0)
                self.bias_calibrated = True
                print(f"✅ Bias calibrated: [{self.world_accel_bias[0]:+.3f}, "
                      f"{self.world_accel_bias[1]:+.3f}, {self.world_accel_bias[2]:+.3f}] m/s²")
                print(f"   (This represents residual error after gravity removal)")
            else:
                # Still calibrating
                packet['position'] = (0.0, 0.0, 0.0)
                packet['velocity'] = (0.0, 0.0, 0.0)
                packet['linear_accel'] = tuple(linear_accel)
                packet['calibrating'] = True
                packet['calibration_progress'] = len(self.bias_samples)
                packet['is_stationary'] = is_stationary  # ADD THIS
                return packet
        
        # ============================================================
        # NORMAL PROCESSING (after calibration)
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
        
        # Apply ZUPT when stationary (no minimum time needed with good detection)
        apply_zupt = is_stationary  # Simplified - trust the detection
        
        # Update state (with bias correction)
        result = self.update(quaternion, accel_g, dt, apply_zupt=apply_zupt)
        
        # UPDATE PACKET IN-PLACE
        packet['position'] = tuple(result['position'])
        packet['velocity'] = tuple(result['velocity'])
        packet['linear_accel'] = tuple(result['linear_accel'])
        packet['accel_world'] = tuple(result['accel_world'])
        packet['accel_bias'] = tuple(result['accel_bias'])
        packet['zupt_applied'] = result['zupt_applied']
        packet['calibrating'] = False
        packet['is_stationary'] = is_stationary  # ADD THIS
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
