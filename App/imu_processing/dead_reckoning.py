import numpy as np
import math
from collections import deque
  
# from paper 
# https://ieeexplore.ieee.org/document/7133658/metrics#metrics

class KalmanDeadReckoning:
    def __init__(self, sample_rate=20, p=0.1, q=0.01, r=0.1, gravity_convention='NED'):
       
        self.dt = 1.0 / sample_rate

        self.state = np.zeros(6)  # [x, y, z, vx, vy, vz]

        self.P = np.eye(6) * p  # Initial covariance

        self.Q = np.eye(6) * q  # Process noise

        self.R = np.eye(3) * r   # Measurement noise

        # Set gravity based on convention
        if gravity_convention == 'NED':
            self.gravity = np.array([0, 0, +9.81])  # Z down
        else:
            self.gravity = np.array([0, 0, -9.81])  # Z up (ENU)
        
        # BIAS CALIBRATION
        self.accel_bias = np.zeros(3)
        self.bias_samples = []
        self.BIAS_CALIBRATION_COUNT = 50
        self.bias_calibrated = False
        
        # ZUPT detection parameters
        self.accel_history = deque(maxlen=20)
        self.gyro_history = deque(maxlen=20)
        
        # Thresholds for stationary detection
        self.ACCEL_STILL_THRESHOLD = 0.05  # g
        self.GYRO_STILL_THRESHOLD = 0.052  # rad/s (~3 deg/s)
        self.ACCEL_VAR_THRESHOLD = 0.01    # variance threshold
        
        # ZUPT state tracking
        self.zupt_active = False
        self.stationary_time = 0.0
        self.MIN_ZUPT_TIME = 0.2  # seconds before trusting ZUPT

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
            # Calculate linear acceleration for bias estimation
            accel_sensor_ms2 = accel_g * 9.81
            accel_world = self._rotate_by_quaternion(accel_sensor_ms2, quaternion)
            linear_accel = accel_world - self.gravity
            
            # Only collect bias samples when stationary
            if is_stationary:
                self.bias_samples.append(linear_accel.copy())
            
            # Check if calibration complete
            if len(self.bias_samples) >= self.BIAS_CALIBRATION_COUNT:
                self.accel_bias = np.mean(self.bias_samples, axis=0)
                self.bias_calibrated = True
                print(f"✅ Bias calibrated: [{self.accel_bias[0]:+.3f}, "
                      f"{self.accel_bias[1]:+.3f}, {self.accel_bias[2]:+.3f}] m/s²")
            else:
                # Still calibrating - update packet and return
                packet['position'] = (0.0, 0.0, 0.0)
                packet['velocity'] = (0.0, 0.0, 0.0)
                packet['linear_accel'] = tuple(linear_accel)
                packet['calibrating'] = True
                packet['calibration_progress'] = len(self.bias_samples)
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
        
        # Apply ZUPT only after minimum time
        apply_zupt = self.zupt_active and self.stationary_time >= self.MIN_ZUPT_TIME
        
        # Update state (with bias correction)
        result = self.update(quaternion, accel_g, dt, apply_zupt=apply_zupt)
        
        # UPDATE PACKET IN-PLACE (preserve all original fields!)
        packet['position'] = tuple(result['position'])
        packet['velocity'] = tuple(result['velocity'])
        packet['linear_accel'] = tuple(result['linear_accel'])
        packet['accel_world'] = tuple(result['accel_world'])
        packet['accel_bias'] = tuple(result['accel_bias'])
        packet['zupt_applied'] = result['zupt_applied']
        packet['calibrating'] = False
        packet['zupt_info'] = {
            'active': self.zupt_active,
            'confidence': confidence,
            'stationary_time': self.stationary_time,
            'applied': apply_zupt
        }
        
        # Return THE SAME packet (now enriched)
        return packet
    
    def _rotate_by_quaternion(self, v, q):
        w, x, y, z = q
        vx, vy, vz = v

        # Quaternion multiplication q ⊗ v 
        t0 = w*0   - x*vx - y*vy - z*vz
        t1 = w*vx  + x*0  + y*vz - z*vy
        t2 = w*vy  - x*vz + y*0  + z*vx
        t3 = w*vz  + x*vy - y*vx + z*0

        # Get quaternion conjugate
        qw_conj = w
        qx_conj = -x
        qy_conj = -y
        qz_conj = -z
        
        # Multiply temp ⊗ q*
        result_w = t0*qw_conj - t1*qx_conj - t2*qy_conj - t3*qz_conj
        result_x = t0*qx_conj + t1*qw_conj + t2*qz_conj - t3*qy_conj
        result_y = t0*qy_conj - t1*qz_conj + t2*qw_conj + t3*qx_conj
        result_z = t0*qz_conj + t1*qy_conj - t2*qx_conj + t3*qw_conj

        # Extract vector part
        return np.array([result_x, result_y, result_z])

    def _predict(self, linear_accel, dt):
        # Extract current state
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
        # Measurement: velocity is zero
        z = np.zeros(3)

        # Observation matrix
        H = np.zeros((3, 6))
        H[0:3, 3:6] = np.eye(3)

        # Innovation
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
        # Convert to numpy arrays
        quaternion = np.array(quaternion)
        accel_sensor_g = np.array(accel_sensor_g)
        
        # Convert to m/s²
        accel_sensor_ms2 = accel_sensor_g * 9.81
        
        # Rotate to world frame
        accel_world = self._rotate_by_quaternion(accel_sensor_ms2, quaternion)
        
        # Remove gravity
        linear_accel = accel_world - self.gravity
        
        # REMOVE BIAS
        linear_accel_corrected = linear_accel - self.accel_bias
        
        # PREDICTION (using corrected acceleration)
        self._predict(linear_accel_corrected, dt)
        
        # CORRECTION (if ZUPT)
        if apply_zupt:
            self._update_zupt()
        
        return {
            'position': self.state[0:3].copy(),
            'velocity': self.state[3:6].copy(),
            'linear_accel': linear_accel_corrected,
            'accel_world': accel_world,
            'accel_bias': self.accel_bias.copy(),
            'zupt_applied': apply_zupt
        }
    
    def reset(self):
        self.state = np.zeros(6)
        self.P = np.eye(6) * 0.1
        self.zupt_active = False
        self.stationary_time = 0.0
        self.accel_history.clear()
        self.gyro_history.clear()
        self.accel_bias = np.zeros(3)
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
