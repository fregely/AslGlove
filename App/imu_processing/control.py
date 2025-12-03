# pylint: disable=E1101
# mypy: ignore-errors
"""
Control - Position correction using PID with CV ground truth

Continuously applies a correction offset to IMU positions.
Updates the offset when CV measurements arrive using PID control.
"""

import time
from typing import Dict, Tuple


class Control:
    """
    Applies continuous position correction to IMU data.
    
    - Every packet: Applies current correction offset to IMU position
    - When NEW CV data arrives: Immediately updates correction offset using PID
    - Result: packet gets 'corrected_position' field added
    """
    def __init__(self, kp: float = 0.5, ki: float = 0.1, kd: float = 0.2) -> None:
        # PID parameters
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # Per-channel correction state
        self.correction_offset: Dict[int, Tuple[float, float]] = {}  # channel -> (offset_x, offset_y)
        self.integral: Dict[int, Tuple[float, float]] = {}           # channel -> (integral_x, integral_y)
        self.prev_error: Dict[int, Tuple[float, float]] = {}         # channel -> (error_x, error_y)
        self.last_corrected_pos: Dict[int, Tuple[float, float]] = {} # channel -> (x, y) - last corrected position
        self.last_cv_time: Dict[str, float] = {}       # finger -> timestamp of last CV update
        
        # Latest CV measurements
        self.cv_data: Dict[str, Tuple[float, float, float]] = {}  # finger -> (x_px, y_px, timestamp)
        
        # Time tracking for IMU
        self.imu_time: Dict[int, float] = {}
        self.imu_start_time: Dict[int, float] = {}
        self.initial_offset_applied: Dict[int, bool] = {}  # Track if initial offset was set
        
        # Pixel to meter conversion (calibrate this!)
        self.px_to_m = 0.001  # 1000 pixels = 1 meter
        self.initial_led_positions: Dict[str, Tuple[float, float]] = {}
        
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
        
        # Load initial calibrated positions from finger_map.json
        self._load_initial_positions()
    
    def _load_initial_positions(self, filename: str = "finger_map.json") -> None:
        """
        Load initial LED positions from finger_map.json and initialize IMU corrected positions.
        This ensures IMUs start at the same position as their calibrated CV positions.
        """
        import json
        import os
        
        try:
            with open(filename, 'r') as f:
                finger_map = json.load(f)
            
            # Extract LED GPIO -> (x, y) pixel positions
            for led_gpio_str, info in finger_map.items():
                led_gpio = int(led_gpio_str)
                finger_name = info.get("finger")
                center_px = info.get("center")  # [x_px, y_px]
                
                if finger_name and center_px:
                    # Convert pixels to meters
                    x_m = center_px[0] * self.px_to_m
                    y_m = center_px[1] * self.px_to_m
                    self.initial_led_positions[finger_name] = (x_m, y_m)
                    
                    # Initialize last_corrected_pos for the corresponding IMU channel
                    # Map finger -> channel using reverse lookup
                    for channel, finger in self.imu_to_finger.items():
                        if finger == finger_name:
                            self.last_corrected_pos[channel] = (x_m, y_m)
                            print(f"[CONTROL] Initialized ch{channel} ({finger_name}) at CV position: ({x_m:.3f}, {y_m:.3f})m")
                            break
            
            print(f"[CONTROL] Loaded {len(self.initial_led_positions)} initial positions from {filename}")
            
        except FileNotFoundError:
            print(f"[CONTROL] Warning: {filename} not found. IMUs will start at (0, 0)")
        except Exception as e:
            print(f"[CONTROL] Error loading initial positions: {e}")
    
    def process(self, packet: dict) -> None:
        """
        Process packet and add corrected_position field.
        
        Uses these fields from packet:
        - packet['led_index'] - which LED was on
        - packet['blob_centers'] - list of (x, y) tuples
        - packet['blob_timestamp'] - timestamp of frame
        - packet['channel'] - IMU channel
        - packet['position'] - IMU position
        - packet['dt'] - time delta
        
        Modifies packet in place by adding:
            packet['corrected_position'] = (x, y, z)
        """
        channel = packet.get('channel')
        
        # Initialize correction offset for this channel
        if channel not in self.correction_offset:
            self.correction_offset[channel] = (0.0, 0.0)
            self.integral[channel] = (0.0, 0.0)
            self.prev_error[channel] = (0.0, 0.0)
            
            # If we have an initial CV position for this channel's finger, use it
            # This happens if finger_map.json was loaded successfully
            if channel not in self.last_corrected_pos and channel in self.imu_to_finger:
                finger_name = self.imu_to_finger[channel]
                if finger_name in self.initial_led_positions:
                    self.last_corrected_pos[channel] = self.initial_led_positions[finger_name]
        
        # Update time tracking
        if 'position' in packet and channel is not None:
            dt = packet.get('dt', 0.02)
            
            if channel not in self.imu_time:
                self.imu_start_time[channel] = time.time()
                self.imu_time[channel] = 0.0
            
            self.imu_time[channel] += dt
        
        # Store CV data if present AND update correction when NEW CV data arrives
        if 'led_index' in packet and 'blob_centers' in packet and packet['blob_centers']:
            led_index = packet['led_index']
            blob_centers = packet['blob_centers']
            cv_timestamp = packet.get('blob_timestamp', time.time())
            
            # Map LED index to finger
            cv_finger = self.led_to_finger.get(led_index)
            
            # If we found a finger and have exactly one blob
            if cv_finger and len(blob_centers) == 1:
                x_px, y_px = blob_centers[0]
                
                # Check if this is NEW CV data (different from what we have stored)
                is_new_cv_data = False
                if cv_finger not in self.cv_data:
                    is_new_cv_data = True
                else:
                    old_x, old_y, old_t = self.cv_data[cv_finger]
                    if (x_px, y_px) != (old_x, old_y) or cv_timestamp != old_t:
                        is_new_cv_data = True
                
                # Store the new CV data
                self.cv_data[cv_finger] = (x_px, y_px, cv_timestamp)
                # print(f"[CONTROL] Stored CV data for {cv_finger} (LED {led_index}): ({x_px:.1f}, {y_px:.1f})px")
                
                # If this is NEW CV data, update correction for that finger's channel
                if is_new_cv_data:
                    # Find which channel this finger belongs to
                    cv_channel = None
                    for ch, finger in self.imu_to_finger.items():
                        if finger == cv_finger:
                            cv_channel = ch
                            break
                    
                    if cv_channel is not None and cv_channel in self.last_corrected_pos:
                        # Calculate dt = time since last CV update for this finger
                        if cv_finger in self.last_cv_time:
                            cv_dt = cv_timestamp - self.last_cv_time[cv_finger]
                        else:
                            cv_dt = 0.1  # Default for first update (~10Hz)
                        
                        # Use the last known corrected position for this channel
                        last_x, last_y = self.last_corrected_pos[cv_channel]
                        # print(f"[CONTROL] NEW CV data for {cv_finger} (ch{cv_channel}) - updating correction (dt={cv_dt:.3f}s)")
                        self._update_correction(cv_channel, cv_finger, last_x, last_y, cv_dt)
                        
                        # Store this CV timestamp for next dt calculation
                        self.last_cv_time[cv_finger] = cv_timestamp
        
        # Apply correction to IMU position
        if 'position' in packet and channel is not None:
            imu_x, imu_y, imu_z = packet['position']
            
            # On first position packet for this channel, calculate initial offset
            # to align IMU starting position with calibrated CV position
            if channel in self.imu_to_finger and not self.initial_offset_applied.get(channel, False):
                finger_name = self.imu_to_finger[channel]
                if finger_name in self.initial_led_positions:
                    # Calculate offset needed to place IMU at CV calibrated position
                    cv_x, cv_y = self.initial_led_positions[finger_name]
                    initial_offset_x = cv_x - imu_x
                    initial_offset_y = cv_y - imu_y
                    self.correction_offset[channel] = (initial_offset_x, initial_offset_y)
                    self.initial_offset_applied[channel] = True
                    print(f"[CONTROL] Set initial offset for ch{channel} ({finger_name}): IMU({imu_x:.3f},{imu_y:.3f}) -> CV({cv_x:.3f},{cv_y:.3f}) offset=({initial_offset_x:.3f},{initial_offset_y:.3f})m")
            
            offset_x, offset_y = self.correction_offset[channel]
            
            # Apply current correction offset
            corrected_x = imu_x + offset_x
            corrected_y = imu_y + offset_y
            corrected_z = imu_z  # Z unchanged (no CV data for Z)
            
            packet['corrected_position'] = (corrected_x, corrected_y, corrected_z)
            
            # Store this as the last corrected position for this channel
            self.last_corrected_pos[channel] = (corrected_x, corrected_y)
            
    
    def _update_correction(self, channel: int, finger: str, current_x: float, current_y: float, dt: float) -> None:
        """
        Update correction offset using PID control.
        
        Args:
            channel: IMU channel
            finger: Finger name
            current_x, current_y: Current corrected position
            dt: Time step
        """
        # Initialize PID state for this channel if not present
        if channel not in self.integral:
            self.integral[channel] = (0.0, 0.0)
        if channel not in self.prev_error:
            self.prev_error[channel] = (0.0, 0.0)
        
        # Get CV measurement
        cv_x_px, cv_y_px, cv_timestamp = self.cv_data[finger]
        
        # Convert CV to meters
        cv_x_m = cv_x_px * self.px_to_m
        cv_y_m = cv_y_px * self.px_to_m
        
        # Calculate error (CV is ground truth)
        error_x = cv_x_m - current_x
        error_y = cv_y_m - current_y
        
        print(f"[PID] {finger}: CV=({cv_x_m:.3f}, {cv_y_m:.3f})m, IMU=({current_x:.3f}, {current_y:.3f})m, Error=({error_x:.3f}, {error_y:.3f})m")
        
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
        
        print(f"[PID] Correction: ({correction_x*1000:.1f}, {correction_y*1000:.1f})mm")
        
        # Update correction offset
        offset_x, offset_y = self.correction_offset[channel]
        self.correction_offset[channel] = (offset_x + correction_x, offset_y + correction_y)
        
        # Update state
        self.integral[channel] = (integral_x, integral_y)
        self.prev_error[channel] = (error_x, error_y)
    
    def get_correction_offset(self, channel: int) -> Tuple[float, float]:
        """Get current correction offset for a channel."""
        return self.correction_offset.get(channel, (0.0, 0.0))
    
    def reset_correction(self, channel: int) -> None:
        """Reset correction state for a channel."""
        if channel in self.correction_offset:
            self.correction_offset[channel] = (0.0, 0.0)
            self.integral[channel] = (0.0, 0.0)
            self.prev_error[channel] = (0.0, 0.0)