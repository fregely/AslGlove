# imu_processing/packet_parser.py

import struct
import logging

class PacketParser:
    """
    Parses raw IMU packet bytes into structured data.
    Works for both BLE and file playback.
    """
    
    @staticmethod
    def parse(data: bytearray) -> dict:
        """
        Parse 27-byte packet into structured dict.
        
        Format:
        - 1 byte: channel
        - 8 bytes: timestamp (uint64, little-endian)
        - 12 bytes: IMU (6x int16, big-endian)
        - 6 bytes: mag (3x int16, little-endian)
        """
        if len(data) != 27:
            raise ValueError(f"Expected 27 bytes, got {len(data)}")
        elif len(data) == 1:
            logging.info("Received LED packet")
            
        channel = data[0]
        timestamp_us = struct.unpack_from('<Q', data, 1)[0]
        
        raw_imu = struct.unpack_from('>6h', data, 9)
        ax_raw, ay_raw, az_raw = raw_imu[0:3]
        gx_raw, gy_raw, gz_raw = raw_imu[3:6]
        
        raw_mag = struct.unpack_from('<3h', data, 21)
        mx_raw, my_raw, mz_raw = raw_mag
        
        return {
            'channel': channel,
            'timestamp_us': timestamp_us,
            'raw_data': data,
            'accel_raw': (ax_raw, ay_raw, az_raw),
            'gyro_raw': (gx_raw, gy_raw, gz_raw),
            'mag_raw': (mx_raw, my_raw, mz_raw),
        }
