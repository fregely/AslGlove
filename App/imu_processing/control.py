"""
Control - Position correction using PID with CV ground truth

Continuously applies a correction offset to IMU positions.
Updates the offset when CV measurements arrive using PID control.
"""

import time


class Control:
    """
    Applies continuous position correction to IMU data.
    
    - Every packet: Applies current correction offset to IMU position
    - When CV data arrives: Updates correction offset using PID control
    - Result: packet gets 'corrected_position' field added
    """
    
    def __init__(self, kp=0.5, ki=0.1, kd=0.2):
        # PID parameters
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # Per-channel correction state
        self.correction_offset = {}  # channel -> (offset_x, offset_y)
        self.integral = {}           # channel -> (integral_x, integral_y)
        self.prev_error = {}         # channel -> (error_x, error_y)
        
        # Latest CV measurements
        self.cv_data = {}  # finger -> (x_px, y_px, timestamp)
        
        # Time tracking for IMU
        self.imu_time = {}
        self.imu_start_time = {}
        
        # Pixel to meter conversion (calibrate this!)
        self.px_to_m = 0.001  # 1000 pixels = 1 meter
        
        # Finger to channel mapping
        self.finger_to_channel = {
            "thumb": 2,
            "index": 1,
            "middle": 0,
            "ring": 7,
            "pinky": 6
        }
    
    def process(self, packet):
        """
        Process packet and add corrected_position field.
        
        Steps:
        1. Update CV data if present in packet
        2. Apply current correction offset to IMU position
        3. If new CV data, update correction offset using PID
        
        Modifies packet in place by adding:
            packet['corrected_position'] = (x, y, z)
        """
        channel = packet.get('channel')
        
        # Initialize correction offset for this channel
        if channel not in self.correction_offset:
            self.correction_offset[channel] = (0.0, 0.0)
            self.integral[channel] = (0.0, 0.0)
            self.prev_error[channel] = (0.0, 0.0)
        
        # Update time tracking
        if 'position' in packet and channel is not None:
            dt = packet.get('dt', 0.02)
            
            if channel not in self.imu_time:
                self.imu_start_time[channel] = time.time()
                self.imu_time[channel] = 0.0
            
            self.imu_time[channel] += dt
        
        # Check for CV data in packet and update if present
        new_cv = False
        if 'cv' in packet and packet['cv']:
            cv_packet = packet['cv']
            if 'finger_positions' in cv_packet:
                cv_timestamp = cv_packet.get('cv_timestamp', time.time())
                
                for finger, pixel_pos in cv_packet['finger_positions'].items():
                    if pixel_pos is not None:
                        x_px, y_px = pixel_pos
                        self.cv_data[finger] = (x_px, y_px, cv_timestamp)
                        
                        # Check if this CV data is for the current channel
                        if self.finger_to_channel.get(finger) == channel:
                            new_cv = True
        
        # Apply correction to IMU position
        if 'position' in packet and channel is not None:
            imu_x, imu_y, imu_z = packet['position']
            offset_x, offset_y = self.correction_offset[channel]
            
            # Apply current correction offset
            corrected_x = imu_x + offset_x
            corrected_y = imu_y + offset_y
            corrected_z = imu_z  # Z unchanged (no CV data for Z)
            
            packet['corrected_position'] = (corrected_x, corrected_y, corrected_z)
            
            # If new CV data arrived, update the correction offset
            if new_cv:
                # Find which finger corresponds to this channel
                finger = None
                for f, ch in self.finger_to_channel.items():
                    if ch == channel:
                        finger = f
                        break
                
                if finger and finger in self.cv_data:
                    self._update_correction(channel, finger, 
                                          corrected_x, corrected_y,
                                          packet.get('dt', 0.02))
    
    def _update_correction(self, channel, finger, current_x, current_y, dt):
        """
        Update correction offset using PID control.
        
        Args:
            channel: IMU channel
            finger: Finger name
            current_x, current_y: Current corrected position
            dt: Time step
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
    
    def get_correction_offset(self, channel):
        """Get current correction offset for a channel."""
        return self.correction_offset.get(channel, (0.0, 0.0))
    
    def reset_correction(self, channel):
        """Reset correction state for a channel."""
        if channel in self.correction_offset:
            self.correction_offset[channel] = (0.0, 0.0)
            self.integral[channel] = (0.0, 0.0)
            self.prev_error[channel] = (0.0, 0.0)


