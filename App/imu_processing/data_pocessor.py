# imu_processing/data_processor.py

import logging
import math
import numpy as np
from typing import Optional, Callable, Dict, Set
from collections import defaultdict, deque

from .converter import IMUConverter
from .madgwick import MadgwickFilter
from .dead_reckoning import DeadReckoning

logger = logging.getLogger(__name__)


class ProcessedIMUData:
    """Container for processed IMU data from a single reading."""
    
    def __init__(
        self,
        channel: int,
        timestamp_us: int,
        # Raw values
        accel_raw: tuple,
        gyro_raw: tuple,
        mag_raw: tuple,
        # Converted values
        accel_g: tuple,
        gyro_deg: tuple,
        mag_ut: tuple,
        # Orientation
        quaternion: np.ndarray,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        # Position
        position: np.ndarray,
        velocity: np.ndarray,
        linear_accel: np.ndarray,
        # State
        is_stationary: bool,
    ):
        self.channel = channel
        self.timestamp_us = timestamp_us
        
        # Raw
        self.accel_raw = accel_raw
        self.gyro_raw = gyro_raw
        self.mag_raw = mag_raw
        
        # Converted
        self.accel_g = accel_g
        self.gyro_deg = gyro_deg
        self.mag_ut = mag_ut
        
        # Orientation
        self.quaternion = quaternion
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.yaw_deg = yaw_deg
        
        # Position
        self.position = position
        self.velocity = velocity
        self.linear_accel = linear_accel
        
        # State
        self.is_stationary = is_stationary
    
    def __repr__(self):
        return (f"ProcessedIMUData(ch={self.channel}, "
                f"orientation=({self.roll_deg:.1f}, {self.pitch_deg:.1f}, {self.yaw_deg:.1f}), "
                f"position={self.position})")


class DataProcessor:
    """
    Processes raw IMU packets through the complete pipeline:
    1. Unit conversion (raw -> physical units)
    2. Madgwick filtering (orientation estimation)
    3. Dead reckoning (position estimation)
    
    Manages multiple IMU channels dynamically and tracks active channels.
    """
    
    def __init__(
        self,
        sample_rate: float = 20.0,
        beta: float = 0.25,
        max_imus: int = 6,
        stationary_accel_threshold: float = 0.05,  # g
        stationary_gyro_threshold: float = 5.0,    # deg/s
        on_data_processed: Optional[Callable[[ProcessedIMUData], None]] = None,
    ):
        """
        Initialize the data processor.
        
        Args:
            sample_rate: Expected sampling rate in Hz
            beta: Madgwick filter beta (how much to trust accel/mag vs gyro)
            max_imus: Maximum number of IMU channels to support
            stationary_accel_threshold: Accel deviation from 1g to consider stationary
            stationary_gyro_threshold: Gyro magnitude threshold for stationary detection
            on_data_processed: Callback function called with each ProcessedIMUData
        """
        self.sample_rate = sample_rate
        self.beta = beta
        self.max_imus = max_imus
        
        # Thresholds for stationary detection
        self.stationary_accel_threshold = stationary_accel_threshold
        self.stationary_gyro_threshold = stationary_gyro_threshold
        
        # Callback
        self.on_data_processed = on_data_processed
        
        # Processing components
        self.converter = IMUConverter()
        self.filters: Dict[int, MadgwickFilter] = {}
        self.dead_reckoning: Dict[int, DeadReckoning] = {}
        
        # Tracking
        self.active_channels: Set[int] = set()
        self.packet_counts: Dict[int, int] = defaultdict(int)
        self.total_packets = 0
        
        logger.info(f"DataProcessor initialized: "
                   f"sample_rate={sample_rate}Hz, beta={beta}, max_imus={max_imus}")
    
    def _ensure_channel_initialized(self, channel: int):
        """Lazily initialize filter and dead reckoning for a channel."""
        if channel >= self.max_imus:
            logger.warning(f"Channel {channel} exceeds max_imus={self.max_imus}")
            return False
        
        if channel not in self.filters:
            self.filters[channel] = MadgwickFilter(
                sample_rate=self.sample_rate,
                beta=self.beta
            )
            self.dead_reckoning[channel] = DeadReckoning(
                sample_rate=self.sample_rate
            )
            self.active_channels.add(channel)
            logger.info(f"✅ Initialized processing for IMU channel {channel}")
        
        return True
    
    def process_packet(self, packet: dict) -> Optional[ProcessedIMUData]:
        """
        Process a single raw packet through the complete pipeline.
        
        Args:
            packet: Dictionary with keys:
                - channel: int
                - timestamp_us: int
                - accel_raw: tuple (ax, ay, az)
                - gyro_raw: tuple (gx, gy, gz)
                - mag_raw: tuple (mx, my, mz)
        
        Returns:
            ProcessedIMUData object or None if processing failed
        """
        try:
            channel = packet['channel']
            
            # Initialize channel if needed
            if not self._ensure_channel_initialized(channel):
                return None
            
            # Extract raw data
            ax_raw, ay_raw, az_raw = packet['accel_raw']
            gx_raw, gy_raw, gz_raw = packet['gyro_raw']
            mx_raw, my_raw, mz_raw = packet['mag_raw']
            
            # Step 1: Convert to physical units
            ax_g, ay_g, az_g = self.converter.convert_accelerometer(ax_raw, ay_raw, az_raw)
            gx_deg, gy_deg, gz_deg = self.converter.convert_gyroscope(gx_raw, gy_raw, gz_raw)
            mx_ut, my_ut, mz_ut = self.converter.convert_magnetometer(mx_raw, my_raw, mz_raw)
            
            # Convert gyro to radians for Madgwick
            gx_rad = math.radians(gx_deg)
            gy_rad = math.radians(gy_deg)
            gz_rad = math.radians(gz_deg)
            
            # Step 2: Update Madgwick filter for orientation
            self.filters[channel].update(
                gx_rad, gy_rad, gz_rad,
                ax_g, ay_g, az_g,
                mx_ut, my_ut, mz_ut
            )
            
            # Get orientation
            quaternion = self.filters[channel].get_quaternion()
            roll_deg, pitch_deg, yaw_deg = self.filters[channel].get_euler_angles()
            
            # Step 3: Detect if stationary (for zero-velocity update)
            accel_magnitude = math.sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g)
            gyro_magnitude = math.sqrt(gx_deg*gx_deg + gy_deg*gy_deg + gz_deg*gz_deg)
            
            accel_error = abs(accel_magnitude - 1.0)
            is_stationary = (
                accel_error < self.stationary_accel_threshold and 
                gyro_magnitude < self.stationary_gyro_threshold
            )
            
            # Step 4: Update dead reckoning for position
            dr_result = self.dead_reckoning[channel].update(
                quaternion,
                np.array([ax_g, ay_g, az_g]),
                stationary=is_stationary
            )
            
            # Create processed data object
            processed = ProcessedIMUData(
                channel=channel,
                timestamp_us=packet['timestamp_us'],
                accel_raw=(ax_raw, ay_raw, az_raw),
                gyro_raw=(gx_raw, gy_raw, gz_raw),
                mag_raw=(mx_raw, my_raw, mz_raw),
                accel_g=(ax_g, ay_g, az_g),
                gyro_deg=(gx_deg, gy_deg, gz_deg),
                mag_ut=(mx_ut, my_ut, mz_ut),
                quaternion=quaternion,
                roll_deg=roll_deg,
                pitch_deg=pitch_deg,
                yaw_deg=yaw_deg,
                position=dr_result['position'],
                velocity=dr_result['velocity'],
                linear_accel=dr_result['linear_accel'],
                is_stationary=is_stationary,
            )
            
            # Update counters
            self.packet_counts[channel] += 1
            self.total_packets += 1
            
            # Log periodically
            if self.packet_counts[channel] % 20 == 0:
                logger.debug(
                    f"IMU {channel}: "
                    f"Orient=({roll_deg:6.1f}°, {pitch_deg:6.1f}°, {yaw_deg:6.1f}°) "
                    f"Pos={processed.position} "
                    f"{'[STATIONARY]' if is_stationary else ''}"
                )
            
            # Call user callback if provided
            if self.on_data_processed:
                self.on_data_processed(processed)
            
            return processed
            
        except Exception as e:
            logger.error(f"❌ Failed to process packet: {e}")
            return None
    
    def get_active_channels(self) -> Set[int]:
        """Return set of active IMU channels."""
        return self.active_channels.copy()
    
    def get_packet_count(self, channel: Optional[int] = None) -> int:
        """
        Get packet count.
        
        Args:
            channel: Specific channel, or None for total count
        
        Returns:
            Packet count
        """
        if channel is None:
            return self.total_packets
        return self.packet_counts[channel]
    
    def reset_channel(self, channel: int):
        """Reset position and velocity for a specific channel."""
        if channel in self.dead_reckoning:
            self.dead_reckoning[channel].reset()
            logger.info(f"🔄 Reset dead reckoning for channel {channel}")
    
    def reset_all(self):
        """Reset position and velocity for all channels."""
        for dr in self.dead_reckoning.values():
            dr.reset()
        logger.info("🔄 Reset all channels")
    
    def update_beta(self, beta: float, channel: Optional[int] = None):
        """
        Update Madgwick filter beta parameter.
        
        Args:
            beta: New beta value
            channel: Specific channel, or None to update all
        """
        if channel is not None:
            if channel in self.filters:
                self.filters[channel].beta = beta
                logger.info(f"Updated beta={beta} for channel {channel}")
        else:
            for filter in self.filters.values():
                filter.beta = beta
            self.beta = beta
            logger.info(f"Updated beta={beta} for all channels")
    
    def update_stationary_thresholds(
        self, 
        accel_threshold: Optional[float] = None,
        gyro_threshold: Optional[float] = None
    ):
        """Update stationary detection thresholds."""
        if accel_threshold is not None:
            self.stationary_accel_threshold = accel_threshold
            logger.info(f"Updated accel threshold={accel_threshold}g")
        
        if gyro_threshold is not None:
            self.stationary_gyro_threshold = gyro_threshold
            logger.info(f"Updated gyro threshold={gyro_threshold}°/s")


# ====== Example Usage ======
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    
    def on_processed(data: ProcessedIMUData):
        """Example callback."""
        print(f"Channel {data.channel}: "
              f"Orientation=({data.roll_deg:.1f}°, {data.pitch_deg:.1f}°, {data.yaw_deg:.1f}°)")
    
    processor = DataProcessor(
        sample_rate=20,
        beta=0.25,
        on_data_processed=on_processed
    )
    
    # Simulate a packet
    fake_packet = {
        'channel': 0,
        'timestamp_us': 1234567890,
        'accel_raw': (0, 0, 16384),      # ~1g on Z axis
        'gyro_raw': (0, 0, 0),            # Not rotating
        'mag_raw': (100, 0, 200),         # Some magnetic field
    }
    
    result = processor.process_packet(fake_packet)
    print(result)
