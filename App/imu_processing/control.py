# imu_processing/control.py
"""
Control - Position correction using PID with CV ground truth

Key improvements:
1. Better initial position alignment from finger_map.json
2. Debug logging to track what's happening
3. Coordinate system validation
4. More aggressive PID for faster correction
"""

import time
import json
import os
from typing import Dict, Tuple, Optional
import logging

logger = logging.getLogger(__name__)


class Control:
    """
    Applies continuous position correction to IMU data.
    
    - Initialization: Aligns IMU starting position with CV calibrated position
    - Every packet: Applies current correction offset to IMU position
    - When NEW CV data arrives: Updates correction offset using PID
    """
    def __init__(self, kp: float = 0.5, ki: float = 0.1, kd: float = 0.2) -> None:
        # PID parameters
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # Per-channel correction state
        self.correction_offset: Dict[int, Tuple[float, float]] = {}
        self.integral: Dict[int, Tuple[float, float]] = {}
        self.prev_error: Dict[int, Tuple[float, float]] = {}
        self.last_corrected_pos: Dict[int, Tuple[float, float]] = {}
        self.last_cv_time: Dict[str, float] = {}
        
        # Latest CV measurements
        self.cv_data: Dict[str, Tuple[float, float, float]] = {}
        
        # Time tracking
        self.imu_time: Dict[int, float] = {}
        self.imu_start_time: Dict[int, float] = {}
        self.first_position_received: Dict[int, bool] = {}  # Track if we've seen first position
        
        # Pixel to meter conversion (will be set by main.py)
        self.px_to_m = 0.001
        
        # Initial calibrated positions (from finger_map.json)
        self.initial_led_positions: Dict[str, Tuple[float, float]] = {}
        self.calibration_loaded = False
        
        # Finger to channel mapping
        self.imu_to_finger = {
            2: "thumb",
            1: "index",
            0: "middle",
            7: "ring",
            6: "pinky"
        }
        self.led_to_finger = {
            1: "thumb",
            3: "index",
            20: "middle",
            7: "ring",
            6: "pinky"
        }
        
        # Load initial calibrated positions
        self._load_initial_positions()
    
    def _load_initial_positions(self, filename: str = "finger_map.json") -> None:
        """
        Load initial LED positions from finger_map.json.
        This provides the ground truth starting positions for each finger.
        """
        if not os.path.exists(filename):
            logger.warning(f"⚠️  Calibration file '{filename}' not found!")
            logger.warning("   IMUs will start at (0, 0) - run calibration first!")
            return
        
        try:
            with open(filename, 'r') as f:
                finger_map = json.load(f)
            
            logger.info(f"📁 Loading calibration from {filename}:")
            
            # Extract LED GPIO -> (x, y) pixel positions
            for led_gpio_str, info in finger_map.items():
                led_gpio = int(led_gpio_str)
                finger_name = info.get("finger")
                center_px = info.get("center")  # [x_px, y_px]
                
                if not finger_name or not center_px:
                    logger.warning(f"   ⚠️  Incomplete data for LED {led_gpio_str}")
                    continue
                
                # Store pixel positions (will convert to meters when px_to_m is set)
                self.initial_led_positions[finger_name] = (center_px[0], center_px[1])
                logger.info(f"   ✓ {finger_name}: ({center_px[0]:.1f}, {center_px[1]:.1f}) pixels")
            
            if self.initial_led_positions:
                self.calibration_loaded = True
                logger.info(f"✅ Loaded {len(self.initial_led_positions)} finger positions")
            else:
                logger.warning("⚠️  No valid finger positions found in calibration file")
            
        except Exception as e:
            logger.error(f"❌ Error loading calibration file: {e}")
    
    def initialize_channel_position(self, channel: int, imu_x: float, imu_y: float) -> Tuple[float, float]:
        """
        Initialize a channel's position based on calibrated CV position.
        Called when first position packet arrives for this channel.
        
        Returns: (corrected_x, corrected_y) - the initialized position
        """
        # Get finger name for this channel
        finger_name = self.imu_to_finger.get(channel)
        if not finger_name:
            logger.warning(f"⚠️  Unknown channel {channel} - no finger mapping")
            return (imu_x, imu_y)
        
        # Get calibrated CV position for this finger
        if finger_name not in self.initial_led_positions:
            logger.warning(f"⚠️  No calibrated position for {finger_name} (ch{channel})")
            return (imu_x, imu_y)
        
        # Convert pixel position to meters
        cv_x_px, cv_y_px = self.initial_led_positions[finger_name]
        cv_x_m = cv_x_px * self.px_to_m
        cv_y_m = cv_y_px * self.px_to_m
        
        # Calculate initial offset to align IMU with CV position
        initial_offset_x = cv_x_m - imu_x
        initial_offset_y = cv_y_m - imu_y
        
        # Store the offset
        self.correction_offset[channel] = (initial_offset_x, initial_offset_y)
        
        # Calculate corrected position
        corrected_x = imu_x + initial_offset_x
        corrected_y = imu_y + initial_offset_y
        
        # Store as last corrected position
        self.last_corrected_pos[channel] = (corrected_x, corrected_y)
        
        # Log the initialization
        logger.info(f"🎯 Initialized ch{channel} ({finger_name}):")
        logger.info(f"   IMU raw:  ({imu_x:+.4f}, {imu_y:+.4f})m")
        logger.info(f"   CV target: ({cv_x_m:+.4f}, {cv_y_m:+.4f})m")
        logger.info(f"   Offset:    ({initial_offset_x:+.4f}, {initial_offset_y:+.4f})m")
        logger.info(f"   Corrected: ({corrected_x:+.4f}, {corrected_y:+.4f})m")
        
        return (corrected_x, corrected_y)
    
    def process(self, packet: dict) -> None:
        """
        Process packet and add corrected_position field.
        """
        channel = packet.get('channel')
        if channel is None:
            return
        
        # Initialize correction state for this channel if needed
        if channel not in self.correction_offset:
            self.correction_offset[channel] = (0.0, 0.0)
            self.integral[channel] = (0.0, 0.0)
            self.prev_error[channel] = (0.0, 0.0)
            self.first_position_received[channel] = False
        
        # Update time tracking
        if 'position' in packet:
            dt = packet.get('dt', 0.02)
            
            if channel not in self.imu_time:
                self.imu_start_time[channel] = time.time()
                self.imu_time[channel] = 0.0
            
            self.imu_time[channel] += dt
        
        # Handle NEW CV data arrival (update PID correction)
        if 'led_index' in packet and 'blob_centers' in packet and packet['blob_centers']:
            led_index = packet['led_index']
            blob_centers = packet['blob_centers']
            cv_timestamp = packet.get('blob_timestamp', time.time())
            
            # Map LED index to finger
            cv_finger = self.led_to_finger.get(led_index)
            
            if cv_finger and len(blob_centers) == 1:
                x_px, y_px = blob_centers[0]
                
                # Check if this is NEW CV data
                is_new_cv_data = False
                if cv_finger not in self.cv_data:
                    is_new_cv_data = True
                else:
                    old_x, old_y, old_t = self.cv_data[cv_finger]
                    if (x_px, y_px) != (old_x, old_y) or cv_timestamp != old_t:
                        is_new_cv_data = True
                
                # Store the new CV data
                self.cv_data[cv_finger] = (x_px, y_px, cv_timestamp)
                
                # If NEW CV data, update correction for that finger's channel
                if is_new_cv_data:
                    cv_channel = None
                    for ch, finger in self.imu_to_finger.items():
                        if finger == cv_finger:
                            cv_channel = ch
                            break
                    
                    if cv_channel is not None and cv_channel in self.last_corrected_pos:
                        # Calculate dt since last CV update
                        if cv_finger in self.last_cv_time:
                            cv_dt = cv_timestamp - self.last_cv_time[cv_finger]
                        else:
                            cv_dt = 0.1  # Default for first update (~10Hz)
                        
                        # Use last known corrected position
                        last_x, last_y = self.last_corrected_pos[cv_channel]
                        self._update_correction(cv_channel, cv_finger, last_x, last_y, cv_dt)
                        
                        # Store timestamp for next dt calculation
                        self.last_cv_time[cv_finger] = cv_timestamp
        
        # Apply correction to IMU position
        if 'position' in packet:
            imu_x, imu_y, imu_z = packet['position']
            
            # ============================================================
            # INITIAL POSITION ALIGNMENT (first packet for this channel)
            # ============================================================
            if not self.first_position_received[channel]:
                self.first_position_received[channel] = True
                
                if self.calibration_loaded:
                    # Initialize position using CV calibration
                    corrected_x, corrected_y = self.initialize_channel_position(channel, imu_x, imu_y)
                else:
                    # No calibration - just use raw IMU position
                    corrected_x, corrected_y = imu_x, imu_y
                    logger.warning(f"⚠️  Ch{channel}: Starting at raw IMU position (no calibration)")
                
                corrected_z = imu_z
            else:
                # Normal operation - apply current correction offset
                offset_x, offset_y = self.correction_offset[channel]
                corrected_x = imu_x + offset_x
                corrected_y = imu_y + offset_y
                corrected_z = imu_z
            
            # Store corrected position in packet
            packet['corrected_position'] = (corrected_x, corrected_y, corrected_z)
            
            # Update last corrected position
            self.last_corrected_pos[channel] = (corrected_x, corrected_y)
    
    def _update_correction(self, channel: int, finger: str, current_x: float, current_y: float, dt: float) -> None:
        """
        Update correction offset using PID control.
        """
        # Get CV measurement
        cv_x_px, cv_y_px, cv_timestamp = self.cv_data[finger]
        
        # Convert CV to meters
        cv_x_m = cv_x_px * self.px_to_m
        cv_y_m = cv_y_px * self.px_to_m
        
        # Calculate error (CV is ground truth)
        error_x = cv_x_m - current_x
        error_y = cv_y_m - current_y
        
        # Get previous state
        integral_x, integral_y = self.integral[channel]
        prev_error_x, prev_error_y = self.prev_error[channel]
        
        # PID calculation for X
        p_x = self.kp * error_x
        integral_x += error_x * dt
        i_x = self.ki * integral_x
        d_x = self.kd * (error_x - prev_error_x) / dt if dt > 0 else 0.0
        correction_x = p_x + i_x + d_x
        
        # PID calculation for Y
        p_y = self.kp * error_y
        integral_y += error_y * dt
        i_y = self.ki * integral_y
        d_y = self.kd * (error_y - prev_error_y) / dt if dt > 0 else 0.0
        correction_y = p_y + i_y + d_y
        
        # Update correction offset
        offset_x, offset_y = self.correction_offset[channel]
        self.correction_offset[channel] = (offset_x + correction_x, offset_y + correction_y)
        
        # Update state
        self.integral[channel] = (integral_x, integral_y)
        self.prev_error[channel] = (error_x, error_y)
        
        # Log significant corrections
        error_magnitude = (error_x**2 + error_y**2)**0.5
        if error_magnitude > 0.01:  # More than 10mm error
            logger.debug(f"[PID ch{channel}] Error: {error_magnitude*1000:.1f}mm, "
                        f"Correction: ({correction_x*1000:.1f}, {correction_y*1000:.1f})mm")
    
    def get_correction_offset(self, channel: int) -> Tuple[float, float]:
        """Get current correction offset for a channel."""
        return self.correction_offset.get(channel, (0.0, 0.0))
    
    def reset_correction(self, channel: int) -> None:
        """Reset correction state for a channel."""
        if channel in self.correction_offset:
            self.correction_offset[channel] = (0.0, 0.0)
            self.integral[channel] = (0.0, 0.0)
            self.prev_error[channel] = (0.0, 0.0)
            self.first_position_received[channel] = False
