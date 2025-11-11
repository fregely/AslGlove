# imu_processing/orientation_position_visualizer.py

import matplotlib.pyplot as plt
from collections import deque, defaultdict
from typing import Dict, Set


class OrientationPositionVisualizer:
    """
    Real-time visualizer for IMU quaternion and position data.
    
    Creates a figure with 2 rows:
    - Top row: Quaternion components (w, x, y, z)
    - Bottom row: Position (x, y, z)
    """
    
    def __init__(self, max_points: int = 200, update_interval: int = 5):
        """
        Initialize the visualizer.
        
        Parameters:
        -----------
        max_points : int
            Maximum number of data points to display (scrolling window)
        update_interval : int
            Update plot every N packets
        """
        self.max_points = max_points
        self.update_interval = update_interval
        self.packet_count = 0
        
        # Storage for each channel
        self.quaternion_data = defaultdict(lambda: {
            'w': deque(maxlen=max_points),
            'x': deque(maxlen=max_points),
            'y': deque(maxlen=max_points),
            'z': deque(maxlen=max_points),
        })
        
        self.position_data = defaultdict(lambda: {
            'x': deque(maxlen=max_points),
            'y': deque(maxlen=max_points),
            'z': deque(maxlen=max_points),
        })
        
        self.active_channels: Set[int] = set()
        
        # Setup matplotlib
        plt.ion()  # Interactive mode
        self.fig, self.axes = plt.subplots(2, 1, figsize=(12, 8))
        self.fig.suptitle('IMU Orientation (Quaternion) & Position', 
                         fontsize=14, fontweight='bold')
        
        # Configure quaternion subplot (top)
        self.axes[0].set_ylabel('Quaternion Components', fontweight='bold')
        self.axes[0].set_ylim(-1.5, 1.5)
        self.axes[0].grid(True, alpha=0.3)
        self.axes[0].axhline(y=0, color='black', linestyle='--', alpha=0.3)
        self.axes[0].axhline(y=1, color='gray', linestyle=':', alpha=0.2)
        self.axes[0].axhline(y=-1, color='gray', linestyle=':', alpha=0.2)
        self.axes[0].set_title('Quaternion (w, x, y, z)', fontsize=11)
        
        # Configure position subplot (bottom)
        self.axes[1].set_ylabel('Position (m)', fontweight='bold')
        self.axes[1].set_xlabel('Sample', fontweight='bold')
        self.axes[1].grid(True, alpha=0.3)
        self.axes[1].axhline(y=0, color='black', linestyle='--', alpha=0.3)
        self.axes[1].set_title('Position (Dead Reckoning)', fontsize=11)
        
        # Line objects for each channel (created dynamically)
        self.lines: Dict[int, Dict[str, plt.Line2D]] = {}
        
        # Color cycle
        self.colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
        
        plt.tight_layout()
    
    def update(self, state: dict):
        """
        Update the plot with new data.
        
        Parameters:
        -----------
        state : dict
            Complete state from DeadReckoning.process() with keys:
            - 'channel': int
            - 'quaternion': (w, x, y, z)
            - 'position': (x, y, z)
        """
        channel = state['channel']
        w, x, y, z = state['quaternion']
        px, py, pz = state['position']
        
        # Store quaternion data
        self.quaternion_data[channel]['w'].append(w)
        self.quaternion_data[channel]['x'].append(x)
        self.quaternion_data[channel]['y'].append(y)
        self.quaternion_data[channel]['z'].append(z)
        
        # Store position data
        self.position_data[channel]['x'].append(px)
        self.position_data[channel]['y'].append(py)
        self.position_data[channel]['z'].append(pz)
        
        # Track active channels
        if channel not in self.active_channels:
            self.active_channels.add(channel)
            self._create_lines_for_channel(channel)
        
        # Update plot every N packets
        self.packet_count += 1
        if self.packet_count % self.update_interval == 0:
            self._redraw()
    
    def _create_lines_for_channel(self, channel: int):
        """Create line objects for a new channel."""
        color = self.colors[channel % len(self.colors)]
        label = f'IMU {channel}'
        
        self.lines[channel] = {
            # Quaternion lines (different styles for each component)
            'w': self.axes[0].plot([], [], color=color, label=f'{label} w', 
                                   linewidth=2, linestyle='-')[0],
            'x': self.axes[0].plot([], [], color=color, label=f'{label} x', 
                                   linewidth=1.5, linestyle='--', alpha=0.7)[0],
            'y': self.axes[0].plot([], [], color=color, label=f'{label} y', 
                                   linewidth=1.5, linestyle='-.', alpha=0.7)[0],
            'z': self.axes[0].plot([], [], color=color, label=f'{label} z', 
                                   linewidth=1.5, linestyle=':', alpha=0.7)[0],
            
            # Position lines
            'px': self.axes[1].plot([], [], color=color, label=f'{label} X', 
                                    linewidth=2, linestyle='-')[0],
            'py': self.axes[1].plot([], [], color=color, label=f'{label} Y', 
                                    linewidth=2, linestyle='--')[0],
            'pz': self.axes[1].plot([], [], color=color, label=f'{label} Z', 
                                    linewidth=2, linestyle='-.')[0],
        }
        
        # Add legends
        self.axes[0].legend(loc='upper right', fontsize=8)
        self.axes[1].legend(loc='upper right', fontsize=8)
    
    def _redraw(self):
        """Redraw all plots with current data."""
        for channel in self.active_channels:
            quat_data = self.quaternion_data[channel]
            pos_data = self.position_data[channel]
            x_range = range(len(quat_data['w']))
            
            # Update quaternion lines
            self.lines[channel]['w'].set_data(x_range, list(quat_data['w']))
            self.lines[channel]['x'].set_data(x_range, list(quat_data['x']))
            self.lines[channel]['y'].set_data(x_range, list(quat_data['y']))
            self.lines[channel]['z'].set_data(x_range, list(quat_data['z']))
            
            # Update position lines
            self.lines[channel]['px'].set_data(x_range, list(pos_data['x']))
            self.lines[channel]['py'].set_data(x_range, list(pos_data['y']))
            self.lines[channel]['pz'].set_data(x_range, list(pos_data['z']))
        
        # Rescale axes
        self.axes[0].relim()
        self.axes[0].autoscale_view(scalex=True, scaley=False)  # Keep y fixed for quaternion
        
        self.axes[1].relim()
        self.axes[1].autoscale_view(scalex=True, scaley=True)
        
        # Redraw
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)
    
    def reset_channel(self, channel: int):
        """Clear data for a specific channel."""
        if channel in self.quaternion_data:
            self.quaternion_data[channel]['w'].clear()
            self.quaternion_data[channel]['x'].clear()
            self.quaternion_data[channel]['y'].clear()
            self.quaternion_data[channel]['z'].clear()
            self.position_data[channel]['x'].clear()
            self.position_data[channel]['y'].clear()
            self.position_data[channel]['z'].clear()
    
    def reset_all(self):
        """Clear all data."""
        for channel in self.active_channels:
            self.reset_channel(channel)
    
    def close(self):
        """Close the plot window."""
        plt.close(self.fig)


class ComprehensiveVisualizer:
    """
    Comprehensive visualizer for a SINGLE IMU showing:
    - Sensor data (accel, gyro, mag) - 3 graphs
    - Orientation (roll, pitch, yaw) - 3 graphs  
    - Position (x, y, z) - 3 graphs
    
    Total: 9 subplots for one IMU
    """
    
    def __init__(self, imu_number: int = 0, max_points: int = 200, update_interval: int = 5):
        """
        Initialize visualizer for a specific IMU.
        
        Parameters:
        -----------
        imu_number : int
            Which IMU this visualizer is for (for labeling)
        max_points : int
            Maximum number of data points to display
        update_interval : int
            Update plot every N packets
        """
        self.imu_number = imu_number
        self.max_points = max_points
        self.update_interval = update_interval
        self.packet_count = 0
        
        # Storage for sensor data
        self.accel_data = {
            'x': deque(maxlen=max_points),
            'y': deque(maxlen=max_points),
            'z': deque(maxlen=max_points),
        }
        
        self.gyro_data = {
            'x': deque(maxlen=max_points),
            'y': deque(maxlen=max_points),
            'z': deque(maxlen=max_points),
        }
        
        self.mag_data = {
            'x': deque(maxlen=max_points),
            'y': deque(maxlen=max_points),
            'z': deque(maxlen=max_points),
        }
        
        # Storage for orientation
        self.roll_data = deque(maxlen=max_points)
        self.pitch_data = deque(maxlen=max_points)
        self.yaw_data = deque(maxlen=max_points)
        
        # Storage for position
        self.position_data = {
            'x': deque(maxlen=max_points),
            'y': deque(maxlen=max_points),
            'z': deque(maxlen=max_points),
        }
        
        # Setup matplotlib - 3x3 grid
        plt.ion()
        self.fig, self.axes = plt.subplots(3, 3, figsize=(16, 10))
        self.fig.canvas.manager.set_window_title(f'IMU {imu_number}')  # Set window title
        self.fig.suptitle(f'IMU {imu_number} - Complete Data', fontsize=16, fontweight='bold')
        
        # ==================== ROW 0: SENSOR DATA ====================
        
        # [0,0] Accelerometer
        ax_accel = self.axes[0, 0]
        ax_accel.set_title('Accelerometer', fontweight='bold', fontsize=12)
        ax_accel.set_ylabel('Acceleration (g)', fontweight='bold')
        ax_accel.grid(True, alpha=0.3)
        ax_accel.axhline(y=0, color='black', linestyle='--', alpha=0.3)
        ax_accel.axhline(y=1, color='gray', linestyle=':', alpha=0.2)
        ax_accel.axhline(y=-1, color='gray', linestyle=':', alpha=0.2)
        
        self.line_ax = ax_accel.plot([], [], color='#e74c3c', label='X', linewidth=2)[0]
        self.line_ay = ax_accel.plot([], [], color='#3498db', label='Y', linewidth=2)[0]
        self.line_az = ax_accel.plot([], [], color='#2ecc71', label='Z', linewidth=2)[0]
        ax_accel.legend(loc='upper right')
        
        # [0,1] Gyroscope
        ax_gyro = self.axes[0, 1]
        ax_gyro.set_title('Gyroscope', fontweight='bold', fontsize=12)
        ax_gyro.set_ylabel('Angular Velocity (°/s)', fontweight='bold')
        ax_gyro.grid(True, alpha=0.3)
        ax_gyro.axhline(y=0, color='black', linestyle='--', alpha=0.3)
        
        self.line_gx = ax_gyro.plot([], [], color='#e74c3c', label='X', linewidth=2)[0]
        self.line_gy = ax_gyro.plot([], [], color='#3498db', label='Y', linewidth=2)[0]
        self.line_gz = ax_gyro.plot([], [], color='#2ecc71', label='Z', linewidth=2)[0]
        ax_gyro.legend(loc='upper right')
        
        # [0,2] Magnetometer
        ax_mag = self.axes[0, 2]
        ax_mag.set_title('Magnetometer', fontweight='bold', fontsize=12)
        ax_mag.set_ylabel('Magnetic Field (µT)', fontweight='bold')
        ax_mag.grid(True, alpha=0.3)
        ax_mag.axhline(y=0, color='black', linestyle='--', alpha=0.3)
        
        self.line_mx = ax_mag.plot([], [], color='#e74c3c', label='X', linewidth=2)[0]
        self.line_my = ax_mag.plot([], [], color='#3498db', label='Y', linewidth=2)[0]
        self.line_mz = ax_mag.plot([], [], color='#2ecc71', label='Z', linewidth=2)[0]
        ax_mag.legend(loc='upper right')
        
        # ==================== ROW 1: ORIENTATION (EULER ANGLES) ====================
        
        # [1,0] Roll
        ax_roll = self.axes[1, 0]
        ax_roll.set_title('Roll', fontweight='bold', fontsize=12, color='#e74c3c')
        ax_roll.set_ylabel('Angle (degrees)', fontweight='bold')
        ax_roll.set_ylim(-180, 180)
        ax_roll.grid(True, alpha=0.3)
        ax_roll.axhline(y=0, color='black', linestyle='--', alpha=0.5, linewidth=1.5)
        ax_roll.axhline(y=90, color='gray', linestyle=':', alpha=0.3)
        ax_roll.axhline(y=-90, color='gray', linestyle=':', alpha=0.3)
        
        self.line_roll = ax_roll.plot([], [], color='#e74c3c', linewidth=3)[0]
        
        # [1,1] Pitch
        ax_pitch = self.axes[1, 1]
        ax_pitch.set_title('Pitch', fontweight='bold', fontsize=12, color='#3498db')
        ax_pitch.set_ylabel('Angle (degrees)', fontweight='bold')
        ax_pitch.set_ylim(-180, 180)
        ax_pitch.grid(True, alpha=0.3)
        ax_pitch.axhline(y=0, color='black', linestyle='--', alpha=0.5, linewidth=1.5)
        ax_pitch.axhline(y=90, color='gray', linestyle=':', alpha=0.3)
        ax_pitch.axhline(y=-90, color='gray', linestyle=':', alpha=0.3)
        
        self.line_pitch = ax_pitch.plot([], [], color='#3498db', linewidth=3)[0]
        
        # [1,2] Yaw
        ax_yaw = self.axes[1, 2]
        ax_yaw.set_title('Yaw', fontweight='bold', fontsize=12, color='#2ecc71')
        ax_yaw.set_ylabel('Angle (degrees)', fontweight='bold')
        ax_yaw.set_ylim(-180, 180)
        ax_yaw.grid(True, alpha=0.3)
        ax_yaw.axhline(y=0, color='black', linestyle='--', alpha=0.5, linewidth=1.5)
        ax_yaw.axhline(y=90, color='gray', linestyle=':', alpha=0.3)
        ax_yaw.axhline(y=-90, color='gray', linestyle=':', alpha=0.3)
        
        self.line_yaw = ax_yaw.plot([], [], color='#2ecc71', linewidth=3)[0]
        
        # ==================== ROW 2: POSITION ====================
        
        # [2,0] Position X
        ax_px = self.axes[2, 0]
        ax_px.set_title('Position X', fontweight='bold', fontsize=12, color='#e74c3c')
        ax_px.set_ylabel('Position (m)', fontweight='bold')
        ax_px.set_xlabel('Sample', fontweight='bold')
        ax_px.grid(True, alpha=0.3)
        ax_px.axhline(y=0, color='black', linestyle='--', alpha=0.3)
        
        self.line_px = ax_px.plot([], [], color='#e74c3c', linewidth=3)[0]
        
        # [2,1] Position Y
        ax_py = self.axes[2, 1]
        ax_py.set_title('Position Y', fontweight='bold', fontsize=12, color='#3498db')
        ax_py.set_ylabel('Position (m)', fontweight='bold')
        ax_py.set_xlabel('Sample', fontweight='bold')
        ax_py.grid(True, alpha=0.3)
        ax_py.axhline(y=0, color='black', linestyle='--', alpha=0.3)
        
        self.line_py = ax_py.plot([], [], color='#3498db', linewidth=3)[0]
        
        # [2,2] Position Z
        ax_pz = self.axes[2, 2]
        ax_pz.set_title('Position Z', fontweight='bold', fontsize=12, color='#2ecc71')
        ax_pz.set_ylabel('Position (m)', fontweight='bold')
        ax_pz.set_xlabel('Sample', fontweight='bold')
        ax_pz.grid(True, alpha=0.3)
        ax_pz.axhline(y=0, color='black', linestyle='--', alpha=0.3)
        
        self.line_pz = ax_pz.plot([], [], color='#2ecc71', linewidth=3)[0]
        
        plt.tight_layout()
    
    def update(self, converted: dict, complete_state: dict):
        """
        Update all 9 plots with new data.
        
        Parameters:
        -----------
        converted : dict
            From IMUConverter with 'accel_g', 'gyro_deg', 'mag_ut'
        complete_state : dict
            From DeadReckoning with 'euler', 'position'
        """
        # Extract sensor data
        ax, ay, az = converted['accel']
        gx, gy, gz = converted['gyro']
        mx, my, mz = converted['mag']
        
        # Extract orientation
        roll, pitch, yaw = complete_state['euler']
        
        # Extract position
        px, py, pz = complete_state['position']
        
        # Store sensor data
        self.accel_data['x'].append(ax)
        self.accel_data['y'].append(ay)
        self.accel_data['z'].append(az)
        
        self.gyro_data['x'].append(gx)
        self.gyro_data['y'].append(gy)
        self.gyro_data['z'].append(gz)
        
        self.mag_data['x'].append(mx)
        self.mag_data['y'].append(my)
        self.mag_data['z'].append(mz)
        
        # Store orientation
        self.roll_data.append(roll)
        self.pitch_data.append(pitch)
        self.yaw_data.append(yaw)
        
        # Store position
        self.position_data['x'].append(px)
        self.position_data['y'].append(py)
        self.position_data['z'].append(pz)
        
        # Update plot
        self.packet_count += 1
        if self.packet_count % self.update_interval == 0:
            self._redraw()
    
    def _redraw(self):
        """Redraw all 9 plots."""
        x = range(len(self.accel_data['x']))
        
        # Update accelerometer
        self.line_ax.set_data(x, list(self.accel_data['x']))
        self.line_ay.set_data(x, list(self.accel_data['y']))
        self.line_az.set_data(x, list(self.accel_data['z']))
        
        # Update gyroscope
        self.line_gx.set_data(x, list(self.gyro_data['x']))
        self.line_gy.set_data(x, list(self.gyro_data['y']))
        self.line_gz.set_data(x, list(self.gyro_data['z']))
        
        # Update magnetometer
        self.line_mx.set_data(x, list(self.mag_data['x']))
        self.line_my.set_data(x, list(self.mag_data['y']))
        self.line_mz.set_data(x, list(self.mag_data['z']))
        
        # Update orientation
        self.line_roll.set_data(x, list(self.roll_data))
        self.line_pitch.set_data(x, list(self.pitch_data))
        self.line_yaw.set_data(x, list(self.yaw_data))
        
        # Update position
        self.line_px.set_data(x, list(self.position_data['x']))
        self.line_py.set_data(x, list(self.position_data['y']))
        self.line_pz.set_data(x, list(self.position_data['z']))
        
        # Rescale all axes
        for i in range(3):
            for j in range(3):
                self.axes[i, j].relim()
                # Row 1 (orientation) keeps y-axis fixed at -180 to 180
                if i == 1:
                    self.axes[i, j].autoscale_view(scalex=True, scaley=False)
                else:
                    self.axes[i, j].autoscale_view(scalex=True, scaley=True)
        
        # Redraw
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)
    
    def reset(self):
        """Clear all data."""
        for key in ['x', 'y', 'z']:
            self.accel_data[key].clear()
            self.gyro_data[key].clear()
            self.mag_data[key].clear()
            self.position_data[key].clear()
        
        self.roll_data.clear()
        self.pitch_data.clear()
        self.yaw_data.clear()
    
    def close(self):
        """Close the plot window."""
        plt.close(self.fig)
