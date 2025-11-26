import numpy as np
import math

class DeadReckoning:
    def __init__(self, sample_rate=20):
        self.sample_rate_hz = sample_rate
        self.dt = 1.0 / sample_rate
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)
        
        # DON'T hardcode gravity direction!
        # We'll compute it from initial accelerometer readings
        self.gravity_world = None  # Will be set during calibration
        
        self.stationary_accel_threshold = 0.25  # g units


    def calibrate_gravity(self, accel_world):
        """
        Calibrate gravity direction from stationary accelerometer reading.
        Call this for first few samples when IMU is stationary.
        """
        if self.gravity_world is None:
            self.gravity_world = accel_world.copy()
        else:
            # Running average for first few samples
            self.gravity_world = 0.9 * self.gravity_world + 0.1 * accel_world


    def process(self, packet: dict) -> dict:

            # Extract data
        quaternion = np.array(packet['quaternion'])
        accel_g = np.array(packet['accel'])
        self.dt = packet['dt']
        
        # Detect if stationary (simple method: check if accel magnitude ≈ 1g)
        accel_magnitude = np.linalg.norm(accel_g)
        is_stationary = bool(abs(accel_magnitude - 1.0) < self.stationary_accel_threshold)
        
        # Update position estimate
        result = self.update(quaternion, accel_g, stationary=is_stationary)

        packet['position'] = tuple(result['position'])
        packet['velocity'] = tuple(result['velocity'])
        packet['linear_accel'] = tuple(result['linear_accel'])
        # Return complete state
        return packet
    
    def quaternion_rotate_vector(self, q, v):
        """Rotate vector v by quaternion q."""
        # Try using the CONJUGATE (inverse rotation)
        w, x, y, z = q
        q_conj = np.array([w, -x, -y, -z])  # Conjugate = inverse for unit quaternions
        
        # Use conjugate instead of q
        w, x, y, z = q_conj
        vx, vy, vz = v
        
        # q_conj ⊗ v ⊗ q_conj*
        t0 = -x*vx - y*vy - z*vz
        t1 =  w*vx + y*vz - z*vy
        t2 =  w*vy - x*vz + z*vx
        t3 =  w*vz + x*vy - y*vx

        rx = t1*w - t0*(-x) - t2*(-z) + t3*(-y)
        ry = t2*w - t0*(-y) + t1*(-z) - t3*(-x)
        rz = t3*w - t0*(-z) - t1*(-y) + t2*(-x)
        
        return np.array([rx, ry, rz])
    
    def update(self, quaternion, accel_sensor_g, stationary=False):
        # Convert g to m/s²
        accel_sensor_ms2 = accel_sensor_g * 9.81
        
        # Rotate acceleration to world frame
        accel_world = self.quaternion_rotate_vector(quaternion, accel_sensor_ms2)
        
        # If gravity not calibrated yet, use this reading
        if self.gravity_world is None and stationary:
            self.gravity_world = accel_world.copy()
            print(f"Gravity calibrated to: {self.gravity_world}")
        
        # Subtract actual gravity direction (not hardcoded [0,0,-9.81])
        if self.gravity_world is not None:
            linear_accel = accel_world - self.gravity_world
        else:
            # Fallback if not calibrated
            linear_accel = accel_world - np.array([0, 0, -9.81])
        
        # Now linear_accel should be near zero when stationary!
        # print(f"Linear accel: {linear_accel}")
        
        # Rest of your code...
        if stationary:
            self.velocity = np.zeros(3)
            return {
                'position': self.position.copy(),
                'velocity': self.velocity.copy(),
                'linear_accel': linear_accel,
                'accel_world': accel_world
            }
        
        self.velocity += linear_accel * self.dt
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
    
