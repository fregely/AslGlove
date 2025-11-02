import numpy as np
import math

class DeadReckoning:
    """
    Estimates position by double integrating accelerometer data.

    pipeline:
    1. get orientation from Madgwick filter
    2. rotate accelerometer data to world frame
    3. subtract gravity
    4. integrate to get velocity
    5. integrate to get position

    will drift very quickly without external corrections.
    """

    def __init__(self, sample_rate):
        self.sample_rate_hz = sample_rate
        self.dt = 1.0 / sample_rate
        self.velocity = np.zeros(3)  # vx, vy, vz
        self.position = np.zeros(3)  # x, y, z
        self.gravity = np.array([0, 0, -9.81])  # m/s²

    def quaternion_rotate_vector(self, q, v):
        """Rotate vector v by quaternion q.
           q : Quaternion as [w, x, y, z]
           v : 3D vector as [vx, vy, vz]
        
           returns rotated vector.
            [x', y', z']
        """

        w, x, y, z = q
        vx, vy, vz = v
        
        # Derived from v' = q ⊗ [0, vx, vy, vz] ⊗ q*
        # where q* is the conjugate and ⊗ is quaternion multiplication

        # First: q ⊗ v (treating v as quaternion [0, vx, vy, vz])
        t0 = -x*vx - y*vy - z*vz
        t1 =  w*vx + y*vz - z*vy
        t2 =  w*vy - x*vz + z*vx
        t3 =  w*vz + x*vy - y*vx

        # Second: result ⊗ q* (conjugate = [w, -x, -y, -z])
        rx = t1*w - t0*(-x) - t2*(-z) + t3*(-y)
        ry = t2*w - t0*(-y) + t1*(-z) - t3*(-x)
        rz = t3*w - t0*(-z) - t1*(-y) + t2*(-x)
        
        return np.array([rx, ry, rz])
    
    def update(self, quaternion, accel_sensor_g, stationary=False):
        """
        Update velocity and position based on new IMU reading.
        
        Args:
            quaternion: Current orientation [w, x, y, z] from Madgwick
            accel_sensor_g: Acceleration in sensor frame [ax, ay, az] in g
            stationary: If True, reset velocity (zero-velocity update)
            
        Returns:
            dict with position, velocity, and linear_accel
        """

        # convert g to m/s²
        accel_sensor_ms2 = accel_sensor_g * 9.81  # m/s
        # Rotate acceleration to world frame
        accel_world = self.quaternion_rotate_vector(quaternion, accel_sensor_ms2)
        # Subtract gravity
        linear_accel = accel_world - self.gravity
        # Zero-velocity update (If stationary, reset velocity)
        if stationary:
            self.velocity = np.array([0.0, 0.0, 0.0])
            # Don't update position when stationary
            return {
                'position': self.position.copy(),
                'velocity': self.velocity.copy(),
                'linear_accel': linear_accel,
                'accel_world': accel_world
            }
        # Integrate acceleration to get velocity
        self.velocity += linear_accel * self.dt
        # Integrate velocity to get position
        self.position += self.velocity * self.dt

        return {
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'linear_accel': linear_accel,
            'accel_world': accel_world
        }
    
    def reset(self):
        """Reset velocity and position to zero."""
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)
    
    def get_postion(self):
        """Get current position estimate."""
        return self.position.copy()
    
    def get_velocity(self):
        """Get current velocity estimate."""
        return self.velocity.copy()
    
