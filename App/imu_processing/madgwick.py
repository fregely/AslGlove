import numpy as np
import math

class MadgwickFilter:
    """
    Madgwick AHRS filter implementation based on the original paper.
    
    """
    
    def __init__(self, sample_rate=20, beta=None, zeta=None):
        """
        Args:
            sample_rate: Sampling frequency in Hz
            beta: Algorithm gain (how much to trust accel/mag vs gyro)
            zeta: Gyroscope bias drift compensation gain
        """                
        if beta is None:
            gyro_rate_noise_density = 0.015  # dps/√Hz for 20hz, from datasheet
            # Factor of √2 accounts for bandwidth = sample_rate/2 (Nyquist)
            gyro_rms_noise_dps = gyro_rate_noise_density * math.sqrt(sample_rate / 2)
            gyro_meas_error = math.radians(gyro_rms_noise_dps)

            # Madgwick formula for beta: β = √(3/4) × gyro_error
            beta = math.sqrt(3.0/4.0) * gyro_meas_error
        if zeta is None:
            # Gyro bias drift parameters from datasheet
            initial_zro_tolerance = 5.0  # dps (from datasheet)
            drift_time_constant = 100.0   # seconds
            
            gyro_drift_rate_dps = initial_zro_tolerance / drift_time_constant  # dps/s
            gyro_meas_drift = math.radians(gyro_drift_rate_dps)
            
            # Madgwick formula for zeta: ζ = √(3/4) × gyro_drift
            zeta = math.sqrt(3.0/4.0) * gyro_meas_drift
        self.beta_base = beta
        self.zeta_base = zeta
        self.beta = self.beta_base
        self.zeta = self.zeta_base
        
        self.gyro_bias = np.zeros(3)  # Gyroscope bias estimate [wx, wy, wz]
        # State variables
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion [w, x, y, z]
        # Reference direction of flux in earth frame
        self.b_x = 1.0
        self.b_z = 0.0

        # Timing tracking
        self.last_timestamp_us = None
        self.dt = 1.0 / sample_rate 

    def process(self, packet: dict) -> dict:
        gyro = packet['gyro']
        accel = packet['accel']
        mag = packet['mag']
        timestamp_us = packet['timestamp_us']

        # Calculate dt from timestamps
        dt = None
        if self.last_timestamp_us is not None:
            dt_us = timestamp_us - self.last_timestamp_us
            dt = dt_us / 1e6  # Convert to seconds
            
            # Sanity check: should be 0.01-0.2s (5-100Hz)
            if dt < 0.01 or dt > 0.2:
                print(f"Warning: Unusual dt={dt:.3f}s, using previous dt={self.dt:.3f}s")
                dt = self.dt  # Use last good dt
            else:
                self.dt = dt  # Update stored dt
        else :
            dt = self.dt  # First run, use default
        self.last_timestamp_us = timestamp_us

        self.update(gyro, accel, mag, dt)

        packet['quaternion'] = self.get_quaternion()
        packet['euler'] = self.get_euler_angles()
        packet['dt'] = self.dt
    
        return packet
    
    def update(self, gyro, accel, mag, dt):
        """
        Update the filter with new sensor readings.
        
        Args:
            gyro: Angular velocity [wx, wy, wz] in rad/s 
            accel: Acceleration [ax, ay, az] in g 
            mag: Magnetic field [mx, my, mz] in any unit 
        """
        # Convert to numpy arrays if needed
        gyro = np.array(gyro, dtype=float)
        accel = np.array(accel, dtype=float)
        mag = np.array(mag, dtype=float)
            
        # MOTION DETECTION 
        accel_magnitude = np.linalg.norm(accel)
        gyro_magnitude = np.linalg.norm(gyro)
        
        # Check if experiencing linear acceleration (not just gravity)
        accel_error = abs(accel_magnitude - 1.0)
        
        # Adaptive beta based on motion
    
        
        if accel_error > 0.3 or gyro_magnitude > math.radians(200):
            self.beta = self.beta_base * 0.01
            self.zeta = self.zeta_base * 0.01
        elif accel_error > 0.15 or gyro_magnitude > math.radians(100):
            self.beta = self.beta_base * 0.1
            self.zeta = self.zeta_base * 0.1
        elif accel_error > 0.05 or gyro_magnitude > math.radians(30):
            self.beta = self.beta_base * 0.5
            self.zeta = self.zeta_base * 0.5
        else:
            self.beta = self.beta_base
            self.zeta = self.zeta_base

        # Normalize accelerometer and magnetometer measurements
        accel_norm = np.linalg.norm(accel)
        if accel_norm == 0:
            return
        accel = accel / accel_norm
        
        mag_norm = np.linalg.norm(mag)
        if mag_norm == 0:
            return
        mag = mag / mag_norm
        
        # Unpack quaternion 
        q0, q1, q2, q3 = self.q
        ax, ay, az = accel
        mx, my, mz = mag
        # Pre-compute repeated terms
        _2q0 = 2.0 * q0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2b_x = 2.0 * self.b_x
        _2b_z = 2.0 * self.b_z
        
        
        # Objective function (gradient descent direction) (EQ25 & 29)
        # These represent the error between measured and expected directions
        f_a = np.array([
            # Accelerometer error (gravity direction)
            _2q1*q3 - _2q0*q2 - ax,
            _2q0*q1 + _2q2*q3 - ay,
            1.0 - _2q1*q1 - _2q2*q2 - az
        ])  
        f_m = np.array([
            # Magnetometer error (magnetic field direction)
            _2b_x*(0.5 - q2*q2 - q3*q3) + _2b_z*(q1*q3 - q0*q2) - mx,
            _2b_x*(q1*q2 - q0*q3) + _2b_z*(q0*q1 + q2*q3) - my,
            _2b_x*(q0*q2 + q1*q3) + _2b_z*(0.5 - q1*q1 - q2*q2) - mz
        ])
        
        # Jacobian matrix - represents how quaternion changes affect the objective (EQ26 & 30)
        J_a = np.array([
            [-_2q2,              _2q3,              -_2q0,             _2q1],
            [_2q1,               _2q0,              _2q3,              _2q2],
            [0,                  -4*q1,             -4*q2,             0]
            ])
        J_m = np.array([
            [-_2b_z*q2,          _2b_z*q3,            -4*self.b_x*q2-_2b_z*q0,  -4*self.b_x*q3+_2b_z*q1],
            [-_2b_x*q3+_2b_z*q1, _2b_x*q2+_2b_z*q0,   _2b_x*q1+_2b_z*q3,        -_2b_x*q0+_2b_z*q2],
            [_2b_x*q2,           _2b_x*q3-2*_2b_z*q1, _2b_x*q0-2*_2b_z*q2,   _2b_x*q1]
        ])
        
        # Compute gradient (J^T * f) (EQ 34)
        grad_a = J_a.T @ f_a
        grad_m = J_m.T @ f_m
        gradient = grad_a  #+ grad_m
        gradient = gradient / np.linalg.norm(gradient)
        

        # Compute gyroscope bias correction 
 
        # Convert gradient to angular error 
        q_conj = np.array([q0, -q1, -q2, -q3]) # Conjugate 
        # 2 * q_conjugate ⊗ gradient (EQ 47)
        gyro_error = 2.0 * self._quaternion_multiply(q_conj, gradient)[1:]  # Extract xyz components
        
        # Integrate gyroscope bias (EQ 48)
        self.gyro_bias += gyro_error * dt * self.zeta
        
        # Remove bias from gyroscope measurement (EQ 49)
        gyro_corrected = gyro - self.gyro_bias
        
     
        # Compute quaternion rate from gyroscope
       
        # Pure gyroscope quaternion rate (EQ12)
        q_dot_gyro = 0.5 * self._quaternion_multiply(self.q, np.array([0, *gyro_corrected]))
        
     
        # compute then integrate the estimated quaternion rate
        
        # Combine gyroscope rate with gradient correction (EQ43)
        q_dot = q_dot_gyro - (self.beta * gradient)
        
        # Integrate
        self.q = self.q + q_dot * dt
        
        # Normalize quaternion
        self.q = self.q / np.linalg.norm(self.q)
        
       
        # Update reference direction of magnetic field
    
        
        # Rotate magnetometer measurement into earth frame (EQ 45)
        h = self._rotate_vector_by_quaternion(self.q, mag)
        
        # Reference direction has only x and z components (north, down)
        self.b_x = math.sqrt(h[0]*h[0] + h[1]*h[1])
        self.b_z = h[2]
    
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
    
    def _rotate_vector_by_quaternion(self, q, v):
        """
        Rotate vector v by quaternion q
        Returns: q * [0, v] * q_conjugate
        """
        v_quat = np.array([0, v[0], v[1], v[2]])
        q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
        
        result = self._quaternion_multiply(
            self._quaternion_multiply(q, v_quat),
            q_conj
        )
        
        return result[1:]  # Return only xyz components
    
    def get_quaternion(self):
        """Return current orientation as quaternion [w, x, y, z]"""
        return self.q.copy()
    
    def get_euler_angles(self):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw) in degrees
        Returns: (roll, pitch, yaw) in degrees
        """
        w, x, y, z = self.q
        
        # Roll (rotation about x-axis)
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        
        # Pitch (rotation about y-axis)
        sinp = 2*(w*y - z*x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi/2, sinp)  # Use 90° if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (rotation about z-axis)
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        
        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
    
    def get_gyro_bias(self):
        """Return current gyroscope bias estimate in rad/s"""
        return self.gyro_bias.copy()
