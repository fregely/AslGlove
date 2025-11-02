import numpy as np
import math


class MadgwickFilter:
    def __init__(self, sample_rate=20, beta=1.0):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion
        self.dt = 1.0 / sample_rate  # Sample rate in Hz
        self.beta = beta  # Algorithm gain i.e how much to trust the accelerometer/magnetometer vs the gyroscope


    def update(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        """
        Update orientation using gyroscope, accelerometer, and magnetometer
        
        Args:
            gx, gy, gz: Angular velocity in rad/s
            ax, ay, az: Acceleration in g
            mx, my, mz: Magnetic field in µT
        """
        q0, q1, q2, q3 = self.q
        
        # ========================================
        # Normalize sensors
        # ========================================
        accel_norm = np.sqrt(ax*ax + ay*ay + az*az)
        if accel_norm == 0:
            return
        ax /= accel_norm
        ay /= accel_norm
        az /= accel_norm
        
        mag_norm = np.sqrt(mx*mx + my*my + mz*mz)
        if mag_norm == 0:
            return
        mx /= mag_norm
        my /= mag_norm
        mz /= mag_norm
        # ========================================
        # MOTION DETECTION
        # ========================================
        # Detect if IMU is accelerating (not just gravity)
        accel_magnitude_error = abs(accel_norm - 1.0)  # Should be 1.0g if only gravity
        
        # Detect if IMU is rotating fast
        gyro_magnitude = math.sqrt(gx*gx + gy*gy + gz*gz)
        
        # Reduce beta during high dynamics
        effective_beta = self.beta
        if accel_magnitude_error > 0.2 or gyro_magnitude > math.radians(200):  # >200°/s
            # High dynamics - trust gyro more
            effective_beta = self.beta * 0.05
        elif accel_magnitude_error > 0.08 or gyro_magnitude > math.radians(50):  # >50°/s
            # Moderate dynamics - reduce correction
            effective_beta = self.beta * 0.5

        # ========================================
        # Calculate adaptive magnetometer weight
        # ========================================
        # Reduce mag influence at steep pitch angles to prevent oscillation
        sinp = 2*(q0*q1 + q2*q3)
        sinp = max(-1.0, min(1.0, sinp))  # Clamp to avoid domain errors
        mag_weight = math.sqrt(1.0 - sinp*sinp)  # |cos(pitch)|
        
        # ========================================
        # Step 1: Gyroscope derivative
        # ========================================
        q_dot_gyro = 0.5 * np.array([
            -q1*gx - q2*gy - q3*gz,
            q0*gx + q2*gz - q3*gy,
            q0*gy - q1*gz + q3*gx,
            q0*gz + q1*gy - q2*gx
        ])
        
        # ========================================
        # Step 2: Accelerometer correction
        # ========================================
        # Expected gravity direction in sensor frame
        gx_expected = 2*(q1*q3 - q0*q2)
        gy_expected = 2*(q0*q1 + q2*q3)
        gz_expected = q0*q0 - q1*q1 - q2*q2 + q3*q3
        
        # Cross product error
        error_accel_x = gy_expected*az - gz_expected*ay
        error_accel_y = gz_expected*ax - gx_expected*az
        error_accel_z = gx_expected*ay - gy_expected*ax
        
        # ========================================
        # Step 3: Magnetometer correction
        # ========================================
        # Rotate mag into world frame
        hx = 2*(mx*(0.5 - q2*q2 - q3*q3) + my*(q1*q2 - q0*q3) + mz*(q1*q3 + q0*q2))
        hy = 2*(mx*(q1*q2 + q0*q3) + my*(0.5 - q1*q1 - q3*q3) + mz*(q2*q3 - q0*q1))
        hz = 2*(mx*(q1*q3 - q0*q2) + my*(q2*q3 + q0*q1) + mz*(0.5 - q1*q1 - q2*q2))
        
        # Reference field
        bx = math.sqrt(hx*hx + hy*hy)
        bz = hz
        
        # Expected mag in sensor frame
        mx_expected = 2*bx*(0.5 - q2*q2 - q3*q3) + 2*bz*(q1*q3 - q0*q2)
        my_expected = 2*bx*(q1*q2 - q0*q3) + 2*bz*(q0*q1 + q2*q3)
        mz_expected = 2*bx*(q1*q3 + q0*q2) + 2*bz*(0.5 - q1*q1 - q2*q2)
        
        # Cross product error (scaled by pitch angle)
        error_mag_x = (my_expected*mz - mz_expected*my) * mag_weight
        error_mag_y = (mz_expected*mx - mx_expected*mz) * mag_weight
        error_mag_z = (mx_expected*my - my_expected*mx) * mag_weight
        
        # ========================================
        # Step 4: Combine errors
        # ========================================
        error_x = error_accel_x + error_mag_x
        error_y = error_accel_y + error_mag_y
        error_z = error_accel_z + error_mag_z
        
        # ========================================
        # Step 5: Calculate gradient
        # ========================================
        gradient = np.array([
            2*q2*error_x - 2*q1*error_y,
            2*q3*error_x + 2*q0*error_y - 4*q1*error_z,
            2*q0*error_x + 2*q3*error_y - 4*q2*error_z,
            2*q1*error_x + 2*q2*error_y
        ])
        
        gradient_norm = np.linalg.norm(gradient)
        if gradient_norm > 0:
            gradient /= gradient_norm
        
        q_dot_correction = -effective_beta * gradient
        
        # ========================================
        # Step 6: Combine and integrate
        # ========================================
        q_dot = q_dot_gyro + q_dot_correction
        self.q = self.q + q_dot * self.dt
        
        # ========================================
        # Step 7: Normalize quaternion
        # ========================================
        magnitude = np.linalg.norm(self.q)
        self.q = self.q / magnitude


    def get_quaternion(self):
        return self.q.copy()
    
    def get_euler_angles(self):
        #c Convert quaternion to Euler angles (roll, pitch, yaw)
        w, x, y, z = self.q
        roll = math.atan2(2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)) #use actan2 withc uses both x and y to determine quadrant too

        # like this to stop gimbal lock, wich happens at 90 degrees
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range returns 90 based on sign of sinp
        else:
            pitch = math.asin(sinp)
        
        yaw = math.atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z))

        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
    
