import numpy as np


class IMUConverter:
    """
    Converts raw 16-bit sensor values to physical units.
    
    The ICM-20948 gives us raw integer values. We need to know:
    1. What range the sensor is configured for
    2. The bit depth (16-bit = -32768 to +32767)
    3. The scale factor (LSB per physical unit)
    """
    
    def __init__(self) -> None:
        # These are the DEFAULT settings for ICM-20948
        # You can change these if you configured your sensor differently  // CHECK THESE VALUES
        
        # ACCELEROMETER: ±2g range
        # At ±2g range, the sensor outputs 16384 LSB (counts) per 1g
        # Why? Because 2^15 (32768) / 2g = 16384
        self.accel_scale = 16384.0  # LSB/g
        
        # GYROSCOPE: ±250°/s range  
        # At ±250°/s range, the sensor outputs 131 LSB per 1°/s
        # Why? Because 2^15 / 250 = 131.072
        self.gyro_scale = 131.0  # LSB/(°/s)
        
        # MAGNETOMETER: AK09916 inside ICM-20948
        # The magnetometer has a fixed scale of 0.15 µT per LSB
        self.mag_scale = 0.15  # µT/LSB
    
    def set_gyro_bias(self, bias: np.ndarray) -> None:
        """
        Set gyro bias from calibration.
        
        Parameters:
        -----------
        bias : np.ndarray
            Gyro bias [gx, gy, gz] in °/s
        """
        self.gyro_bias = bias.copy()
        
    def convert(self, packet: dict) -> dict:
        accel_raw = packet['accel_raw']
        gyro_raw = packet['gyro_raw']
        mag_raw = packet['mag_raw']

        accel_g = self.convert_accelerometer(*accel_raw)
        gyro_rad = self.convert_gyroscope(*gyro_raw)
        mag_ut = self.convert_magnetometer(*mag_raw)

        packet['accel'] = accel_g
        packet['gyro'] = gyro_rad
        packet['mag'] = mag_ut

        return packet


    def convert_accelerometer(self, raw_x: int, raw_y: int, raw_z: int) -> tuple:
        """
        Convert raw accelerometer values to g (gravity units).
        
        1g = 9.8 m/s² (Earth's gravity)
        
        When the sensor is flat on a table:
        - X and Y should be near 0g
        - Z should be near +1g or -1g (depending on orientation)
        
        Parameters:
        -----------
        raw_x, raw_y, raw_z : int
            Raw 16-bit signed integers from sensor
            
        Returns:
        --------
        tuple : (ax, ay, az) in g
        """
        ax_g = raw_x / self.accel_scale
        ay_g = raw_y / self.accel_scale
        az_g = raw_z / self.accel_scale
        
        return (ax_g, ay_g, az_g)
    
    def convert_gyroscope(self, raw_x: int, raw_y: int, raw_z: int) -> tuple:
        """
        Convert raw gyroscope values to degrees per second (°/s).
        
        Gyroscope measures ROTATION RATE (angular velocity).
        - When sensor is not rotating: should read ~0°/s
        - When rotating: shows how fast (degrees per second)
        
        We also convert to radians/second for math operations.
        
        Parameters:
        -----------
        raw_x, raw_y, raw_z : int
            Raw 16-bit signed integers from sensor
            
        Returns:
        --------
        tuple : (gx_deg, gy_deg, gz_deg)
        """
        # Convert to degrees per second
        gx_deg = raw_x / self.gyro_scale
        gy_deg = raw_y / self.gyro_scale
        gz_deg = raw_z / self.gyro_scale
        
        # Convert to radians per second (needed for math later)
        gx_rad = np.radians(gx_deg)
        gy_rad = np.radians(gy_deg)
        gz_rad = np.radians(gz_deg)
        
        return (gx_rad, gy_rad, gz_rad)
    
    def convert_magnetometer(self, raw_x: int, raw_y: int, raw_z: int) -> tuple:
        """
        Convert raw magnetometer values to microtesla (µT).
        
        Magnetometer measures Earth's magnetic field.
        - Earth's field is ~25-65 µT depending on location
        - Points toward magnetic north (not true north!)
        
        Parameters:
        -----------
        raw_x, raw_y, raw_z : int
            Raw 16-bit signed integers from sensor
            
        Returns:
        --------
        tuple : (mx, my, mz) in µT
        """
        mx_ut = raw_x * self.mag_scale
        my_ut = raw_y * self.mag_scale
        mz_ut = raw_z * self.mag_scale
        
        return (mx_ut, my_ut, mz_ut)
    