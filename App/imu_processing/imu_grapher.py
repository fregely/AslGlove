"""
imu_grapher.py - Detailed pipeline-compatible IMU grapher

Separates each sensor type and axis into individual graphs for easier debugging.
"""

import matplotlib.pyplot as plt
from collections import deque, defaultdict
from typing import Dict, Set
from enum import Flag, auto


class GraphMode(Flag):
    """Flags for which graphs to display."""
    UNCONVERTED = auto()  # 3 graphs: Accel, Gyro, Mag (raw)
    CONVERTED = auto()    # 3 graphs: Accel, Gyro, Mag (physical units)
    MADGWICK = auto()     # 3 graphs: Roll, Pitch, Yaw
    DEAD_RECKONING = auto()  # 3 graphs: Position X, Y, Z
    
    # Convenience combinations
    ALL = UNCONVERTED | CONVERTED | MADGWICK | DEAD_RECKONING
    FULL_PIPELINE = CONVERTED | MADGWICK | DEAD_RECKONING
    SENSORS_ONLY = UNCONVERTED | CONVERTED
    ORIENTATION_POSITION = MADGWICK | DEAD_RECKONING


class IMUGrapher:
    """
    Detailed IMU grapher with separate graphs for each sensor and axis.
    
    Graph layout per mode:
    - UNCONVERTED: 3 graphs (Accel raw, Gyro raw, Mag raw)
    - CONVERTED: 3 graphs (Accel g, Gyro deg/s, Mag uT)
    - MADGWICK: 3 graphs (Roll, Pitch, Yaw)
    - DEAD_RECKONING: 3 graphs (Position X, Y, Z)
    
    Each graph shows all active IMU channels with different colors.
    """
    
    def __init__(
        self,
        mode: GraphMode = GraphMode.CONVERTED | GraphMode.MADGWICK,
        max_points: int = 200,
        update_interval: int = 5
    ):
        """
        Initialize grapher.
        
        Parameters:
        -----------
        mode : GraphMode
            Which graphs to display (each mode = 3 graphs)
        max_points : int
            Maximum points in scrolling window
        update_interval : int
            Update plot every N packets
        """
        self.mode = mode
        self.max_points = max_points
        self.update_interval = update_interval
        self.packet_count = 0
        
        # Active channels
        self.active_channels: Set[int] = set()
        
        # Data storage per channel
        self.raw_data = defaultdict(lambda: {
            'ax': deque(maxlen=max_points), 'ay': deque(maxlen=max_points), 'az': deque(maxlen=max_points),
            'gx': deque(maxlen=max_points), 'gy': deque(maxlen=max_points), 'gz': deque(maxlen=max_points),
            'mx': deque(maxlen=max_points), 'my': deque(maxlen=max_points), 'mz': deque(maxlen=max_points),
        })
        
        self.converted_data = defaultdict(lambda: {
            'ax': deque(maxlen=max_points), 'ay': deque(maxlen=max_points), 'az': deque(maxlen=max_points),
            'gx': deque(maxlen=max_points), 'gy': deque(maxlen=max_points), 'gz': deque(maxlen=max_points),
            'mx': deque(maxlen=max_points), 'my': deque(maxlen=max_points), 'mz': deque(maxlen=max_points),
        })
        
        self.orientation_data = defaultdict(lambda: {
            'roll': deque(maxlen=max_points),
            'pitch': deque(maxlen=max_points),
            'yaw': deque(maxlen=max_points),
        })
        
        self.position_data = defaultdict(lambda: {
            'x': deque(maxlen=max_points),
            'y': deque(maxlen=max_points),
            'z': deque(maxlen=max_points),
        })
        
        # Setup plots
        self._setup_plots()
    
    def _setup_plots(self):
        """Create subplot layout with 3 graphs per mode."""
        plt.ion()
        
        # Count total number of graphs (3 per mode)
        num_graphs = 0
        if self.mode & GraphMode.UNCONVERTED:
            num_graphs += 3
        if self.mode & GraphMode.CONVERTED:
            num_graphs += 3
        if self.mode & GraphMode.MADGWICK:
            num_graphs += 3
        if self.mode & GraphMode.DEAD_RECKONING:
            num_graphs += 3
        
        if num_graphs == 0:
            raise ValueError("At least one GraphMode must be selected")
        
        # Create figure with 3 columns for better layout
        if num_graphs <= 3:
            # Single row
            rows, cols = 1, num_graphs
            figsize = (6 * cols, 5)
        elif num_graphs <= 6:
            # Two rows
            rows, cols = 2, 3
            figsize = (18, 10)
        else:
            # Multiple rows, 3 columns
            rows = (num_graphs + 2) // 3
            cols = 3
            figsize = (18, 5 * rows)
        
        self.fig, axes = plt.subplots(rows, cols, figsize=figsize)
        axes = axes.flatten() if num_graphs > 1 else [axes]
        
        self.axes = {}
        self.lines = defaultdict(dict)
        
        # Assign axes to each graph
        ax_idx = 0
        
        # UNCONVERTED mode: 3 graphs
        if self.mode & GraphMode.UNCONVERTED:
            self.axes['raw_accel'] = axes[ax_idx]
            self._setup_sensor_axis(axes[ax_idx], 'Raw Accelerometer', 'Raw Value', color='#e74c3c')
            ax_idx += 1
            
            self.axes['raw_gyro'] = axes[ax_idx]
            self._setup_sensor_axis(axes[ax_idx], 'Raw Gyroscope', 'Raw Value', color='#3498db')
            ax_idx += 1
            
            self.axes['raw_mag'] = axes[ax_idx]
            self._setup_sensor_axis(axes[ax_idx], 'Raw Magnetometer', 'Raw Value', color='#2ecc71')
            ax_idx += 1
        
        # CONVERTED mode: 3 graphs
        if self.mode & GraphMode.CONVERTED:
            self.axes['conv_accel'] = axes[ax_idx]
            self._setup_sensor_axis(axes[ax_idx], 'Accelerometer', 'Acceleration (g)', color='#e74c3c')
            ax_idx += 1
            
            self.axes['conv_gyro'] = axes[ax_idx]
            self._setup_sensor_axis(axes[ax_idx], 'Gyroscope', 'Angular Velocity (°/s)', color='#3498db')
            ax_idx += 1
            
            self.axes['conv_mag'] = axes[ax_idx]
            self._setup_sensor_axis(axes[ax_idx], 'Magnetometer', 'Magnetic Field (µT)', color='#2ecc71')
            ax_idx += 1
        
        # MADGWICK mode: 3 graphs
        if self.mode & GraphMode.MADGWICK:
            self.axes['roll'] = axes[ax_idx]
            self._setup_orientation_axis(axes[ax_idx], 'Roll', color='#e74c3c')
            ax_idx += 1
            
            self.axes['pitch'] = axes[ax_idx]
            self._setup_orientation_axis(axes[ax_idx], 'Pitch', color='#3498db')
            ax_idx += 1
            
            self.axes['yaw'] = axes[ax_idx]
            self._setup_orientation_axis(axes[ax_idx], 'Yaw', color='#2ecc71')
            ax_idx += 1
        
        # DEAD_RECKONING mode: 3 graphs
        if self.mode & GraphMode.DEAD_RECKONING:
            self.axes['pos_x'] = axes[ax_idx]
            self._setup_position_axis(axes[ax_idx], 'Position X', color='#e74c3c')
            ax_idx += 1
            
            self.axes['pos_y'] = axes[ax_idx]
            self._setup_position_axis(axes[ax_idx], 'Position Y', color='#3498db')
            ax_idx += 1
            
            self.axes['pos_z'] = axes[ax_idx]
            self._setup_position_axis(axes[ax_idx], 'Position Z', color='#2ecc71')
            ax_idx += 1
        
        # Hide unused subplots
        for i in range(ax_idx, len(axes)):
            axes[i].set_visible(False)
        
        plt.tight_layout()
    
    def _setup_sensor_axis(self, ax, title, ylabel, color='black'):
        """Setup axis for sensor data (accel/gyro/mag)."""
        ax.set_title(title, fontsize=11, fontweight='bold', color=color)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='black', linestyle='--', alpha=0.3)
        # Legend will be added when first channel creates lines
    
    def _setup_orientation_axis(self, ax, title, color='black'):
        """Setup axis for orientation (roll/pitch/yaw)."""
        ax.set_title(title, fontsize=11, fontweight='bold', color=color)
        ax.set_ylabel('Angle (degrees)', fontsize=9)
        ax.set_ylim(-180, 180)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='black', linestyle='--', alpha=0.3, linewidth=1.5)
        ax.axhline(y=90, color='gray', linestyle=':', alpha=0.2)
        ax.axhline(y=-90, color='gray', linestyle=':', alpha=0.2)
        # Legend will be added when first channel creates lines
    
    def _setup_position_axis(self, ax, title, color='black'):
        """Setup axis for position."""
        ax.set_title(title, fontsize=11, fontweight='bold', color=color)
        ax.set_ylabel('Position (meters)', fontsize=9)
        ax.set_xlabel('Sample', fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='black', linestyle='--', alpha=0.3)
        # Legend will be added when first channel creates lines
    
    def _create_lines_for_channel(self, channel: int):
        """Create line objects for a new channel."""
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
        color = colors[channel % len(colors)]
        label = f'IMU{channel}'
        
        # UNCONVERTED: 3 graphs (accel, gyro, mag)
        if self.mode & GraphMode.UNCONVERTED:
            # Raw Accelerometer
            ax = self.axes['raw_accel']
            self.lines[channel]['raw_ax'] = ax.plot([], [], color=color, label=f'{label} X', linestyle='-', linewidth=2)[0]
            self.lines[channel]['raw_ay'] = ax.plot([], [], color=color, label=f'{label} Y', linestyle='--', linewidth=2, alpha=0.7)[0]
            self.lines[channel]['raw_az'] = ax.plot([], [], color=color, label=f'{label} Z', linestyle='-.', linewidth=2, alpha=0.7)[0]
            ax.legend(loc='upper right', fontsize=7)
            
            # Raw Gyroscope
            ax = self.axes['raw_gyro']
            self.lines[channel]['raw_gx'] = ax.plot([], [], color=color, label=f'{label} X', linestyle='-', linewidth=2)[0]
            self.lines[channel]['raw_gy'] = ax.plot([], [], color=color, label=f'{label} Y', linestyle='--', linewidth=2, alpha=0.7)[0]
            self.lines[channel]['raw_gz'] = ax.plot([], [], color=color, label=f'{label} Z', linestyle='-.', linewidth=2, alpha=0.7)[0]
            ax.legend(loc='upper right', fontsize=7)
            
            # Raw Magnetometer
            ax = self.axes['raw_mag']
            self.lines[channel]['raw_mx'] = ax.plot([], [], color=color, label=f'{label} X', linestyle='-', linewidth=2)[0]
            self.lines[channel]['raw_my'] = ax.plot([], [], color=color, label=f'{label} Y', linestyle='--', linewidth=2, alpha=0.7)[0]
            self.lines[channel]['raw_mz'] = ax.plot([], [], color=color, label=f'{label} Z', linestyle='-.', linewidth=2, alpha=0.7)[0]
            ax.legend(loc='upper right', fontsize=7)
        
        # CONVERTED: 3 graphs (accel, gyro, mag)
        if self.mode & GraphMode.CONVERTED:
            # Accelerometer
            ax = self.axes['conv_accel']
            self.lines[channel]['conv_ax'] = ax.plot([], [], color=color, label=f'{label} X', linestyle='-', linewidth=2)[0]
            self.lines[channel]['conv_ay'] = ax.plot([], [], color=color, label=f'{label} Y', linestyle='--', linewidth=2, alpha=0.7)[0]
            self.lines[channel]['conv_az'] = ax.plot([], [], color=color, label=f'{label} Z', linestyle='-.', linewidth=2, alpha=0.7)[0]
            ax.legend(loc='upper right', fontsize=7)
            
            # Gyroscope
            ax = self.axes['conv_gyro']
            self.lines[channel]['conv_gx'] = ax.plot([], [], color=color, label=f'{label} X', linestyle='-', linewidth=2)[0]
            self.lines[channel]['conv_gy'] = ax.plot([], [], color=color, label=f'{label} Y', linestyle='--', linewidth=2, alpha=0.7)[0]
            self.lines[channel]['conv_gz'] = ax.plot([], [], color=color, label=f'{label} Z', linestyle='-.', linewidth=2, alpha=0.7)[0]
            ax.legend(loc='upper right', fontsize=7)
            
            # Magnetometer
            ax = self.axes['conv_mag']
            self.lines[channel]['conv_mx'] = ax.plot([], [], color=color, label=f'{label} X', linestyle='-', linewidth=2)[0]
            self.lines[channel]['conv_my'] = ax.plot([], [], color=color, label=f'{label} Y', linestyle='--', linewidth=2, alpha=0.7)[0]
            self.lines[channel]['conv_mz'] = ax.plot([], [], color=color, label=f'{label} Z', linestyle='-.', linewidth=2, alpha=0.7)[0]
            ax.legend(loc='upper right', fontsize=7)
        
        # MADGWICK: 3 graphs (roll, pitch, yaw)
        if self.mode & GraphMode.MADGWICK:
            # Roll
            ax = self.axes['roll']
            self.lines[channel]['roll'] = ax.plot([], [], color=color, label=label, linestyle='-', linewidth=2.5)[0]
            ax.legend(loc='upper right', fontsize=7)
            
            # Pitch
            ax = self.axes['pitch']
            self.lines[channel]['pitch'] = ax.plot([], [], color=color, label=label, linestyle='-', linewidth=2.5)[0]
            ax.legend(loc='upper right', fontsize=7)
            
            # Yaw
            ax = self.axes['yaw']
            self.lines[channel]['yaw'] = ax.plot([], [], color=color, label=label, linestyle='-', linewidth=2.5)[0]
            ax.legend(loc='upper right', fontsize=7)
        
        # DEAD_RECKONING: 3 graphs (pos_x, pos_y, pos_z)
        if self.mode & GraphMode.DEAD_RECKONING:
            # Position X
            ax = self.axes['pos_x']
            self.lines[channel]['pos_x'] = ax.plot([], [], color=color, label=label, linestyle='-', linewidth=2.5)[0]
            ax.legend(loc='upper right', fontsize=7)
            
            # Position Y
            ax = self.axes['pos_y']
            self.lines[channel]['pos_y'] = ax.plot([], [], color=color, label=label, linestyle='-', linewidth=2.5)[0]
            ax.legend(loc='upper right', fontsize=7)
            
            # Position Z
            ax = self.axes['pos_z']
            self.lines[channel]['pos_z'] = ax.plot([], [], color=color, label=label, linestyle='-', linewidth=2.5)[0]
            ax.legend(loc='upper right', fontsize=7)
    
    def _ensure_channel_exists(self, channel: int):
        """Ensure line objects exist for this channel."""
        if channel not in self.active_channels:
            self.active_channels.add(channel)
            self._create_lines_for_channel(channel)
    
    def update_unconverted(self, packet: dict):
        """Update with raw packet data."""
        if not (self.mode & GraphMode.UNCONVERTED):
            return
        
        channel = packet['channel']
        self._ensure_channel_exists(channel)
        
        ax, ay, az = packet['accel_raw']
        gx, gy, gz = packet['gyro_raw']
        mx, my, mz = packet['mag_raw']
        
        self.raw_data[channel]['ax'].append(ax)
        self.raw_data[channel]['ay'].append(ay)
        self.raw_data[channel]['az'].append(az)
        self.raw_data[channel]['gx'].append(gx)
        self.raw_data[channel]['gy'].append(gy)
        self.raw_data[channel]['gz'].append(gz)
        self.raw_data[channel]['mx'].append(mx)
        self.raw_data[channel]['my'].append(my)
        self.raw_data[channel]['mz'].append(mz)
    
    def update_converted(self, converted: dict):
        """Update with converted sensor data."""
        if not (self.mode & GraphMode.CONVERTED):
            return
        
        channel = converted['channel']
        self._ensure_channel_exists(channel)
        
        ax, ay, az = converted['accel']
        gx, gy, gz = converted['gyro']
        mx, my, mz = converted['mag']
        
        self.converted_data[channel]['ax'].append(ax)
        self.converted_data[channel]['ay'].append(ay)
        self.converted_data[channel]['az'].append(az)
        self.converted_data[channel]['gx'].append(gx)
        self.converted_data[channel]['gy'].append(gy)
        self.converted_data[channel]['gz'].append(gz)
        self.converted_data[channel]['mx'].append(mx)
        self.converted_data[channel]['my'].append(my)
        self.converted_data[channel]['mz'].append(mz)
    
    def update_madgwick(self, madgwick: dict):
        """Update with Madgwick filter output."""
        if not (self.mode & GraphMode.MADGWICK):
            return
        
        channel = madgwick['channel']
        self._ensure_channel_exists(channel)
        
        if 'euler' in madgwick:
            roll, pitch, yaw = madgwick['euler']
        else:
            roll = madgwick['roll']
            pitch = madgwick['pitch']
            yaw = madgwick['yaw']
        
        self.orientation_data[channel]['roll'].append(roll)
        self.orientation_data[channel]['pitch'].append(pitch)
        self.orientation_data[channel]['yaw'].append(yaw)
    
    def update_dead_reckoning(self, dead_reckoning: dict):
        """Update with dead reckoning output."""
        if not (self.mode & GraphMode.DEAD_RECKONING):
            return
        
        channel = dead_reckoning['channel']
        self._ensure_channel_exists(channel)
        
        x, y, z = dead_reckoning['position']
        
        self.position_data[channel]['x'].append(x)
        self.position_data[channel]['y'].append(y)
        self.position_data[channel]['z'].append(z)
    
    def update(self, **kwargs):
        """Convenience method to update all at once."""
        if 'unconverted' in kwargs:
            self.update_unconverted(kwargs['unconverted'])
        if 'converted' in kwargs:
            self.update_converted(kwargs['converted'])
        if 'madgwick' in kwargs:
            self.update_madgwick(kwargs['madgwick'])
        if 'dead_reckoning' in kwargs:
            self.update_dead_reckoning(kwargs['dead_reckoning'])
        
        self.packet_count += 1
        if self.packet_count % self.update_interval == 0:
            self._redraw()
    
    def _redraw(self):
        """Redraw all plots."""
        for channel in self.active_channels:
            # Get x-axis range
            if self.mode & GraphMode.UNCONVERTED:
                x = range(len(self.raw_data[channel]['ax']))
            elif self.mode & GraphMode.CONVERTED:
                x = range(len(self.converted_data[channel]['ax']))
            elif self.mode & GraphMode.MADGWICK:
                x = range(len(self.orientation_data[channel]['roll']))
            else:
                x = range(len(self.position_data[channel]['x']))
            
            # Update unconverted
            if self.mode & GraphMode.UNCONVERTED:
                raw = self.raw_data[channel]
                self.lines[channel]['raw_ax'].set_data(x, list(raw['ax']))
                self.lines[channel]['raw_ay'].set_data(x, list(raw['ay']))
                self.lines[channel]['raw_az'].set_data(x, list(raw['az']))
                self.lines[channel]['raw_gx'].set_data(x, list(raw['gx']))
                self.lines[channel]['raw_gy'].set_data(x, list(raw['gy']))
                self.lines[channel]['raw_gz'].set_data(x, list(raw['gz']))
                self.lines[channel]['raw_mx'].set_data(x, list(raw['mx']))
                self.lines[channel]['raw_my'].set_data(x, list(raw['my']))
                self.lines[channel]['raw_mz'].set_data(x, list(raw['mz']))
            
            # Update converted
            if self.mode & GraphMode.CONVERTED:
                conv = self.converted_data[channel]
                self.lines[channel]['conv_ax'].set_data(x, list(conv['ax']))
                self.lines[channel]['conv_ay'].set_data(x, list(conv['ay']))
                self.lines[channel]['conv_az'].set_data(x, list(conv['az']))
                self.lines[channel]['conv_gx'].set_data(x, list(conv['gx']))
                self.lines[channel]['conv_gy'].set_data(x, list(conv['gy']))
                self.lines[channel]['conv_gz'].set_data(x, list(conv['gz']))
                self.lines[channel]['conv_mx'].set_data(x, list(conv['mx']))
                self.lines[channel]['conv_my'].set_data(x, list(conv['my']))
                self.lines[channel]['conv_mz'].set_data(x, list(conv['mz']))
            
            # Update madgwick
            if self.mode & GraphMode.MADGWICK:
                ori = self.orientation_data[channel]
                self.lines[channel]['roll'].set_data(x, list(ori['roll']))
                self.lines[channel]['pitch'].set_data(x, list(ori['pitch']))
                self.lines[channel]['yaw'].set_data(x, list(ori['yaw']))
            
            # Update dead reckoning
            if self.mode & GraphMode.DEAD_RECKONING:
                pos = self.position_data[channel]
                self.lines[channel]['pos_x'].set_data(x, list(pos['x']))
                self.lines[channel]['pos_y'].set_data(x, list(pos['y']))
                self.lines[channel]['pos_z'].set_data(x, list(pos['z']))
        
        # Rescale axes
        for key, ax in self.axes.items():
            ax.relim()
            # Keep y-axis fixed for orientation
            if key in ['roll', 'pitch', 'yaw']:
                ax.autoscale_view(scalex=True, scaley=False)
            else:
                ax.autoscale_view(scalex=True, scaley=True)
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)
    
    def reset_channel(self, channel: int):
        """Clear data for a specific channel."""
        if channel not in self.active_channels:
            return
        
        if self.mode & GraphMode.UNCONVERTED:
            for key in self.raw_data[channel]:
                self.raw_data[channel][key].clear()
        
        if self.mode & GraphMode.CONVERTED:
            for key in self.converted_data[channel]:
                self.converted_data[channel][key].clear()
        
        if self.mode & GraphMode.MADGWICK:
            for key in self.orientation_data[channel]:
                self.orientation_data[channel][key].clear()
        
        if self.mode & GraphMode.DEAD_RECKONING:
            for key in self.position_data[channel]:
                self.position_data[channel][key].clear()
    
    def reset_all(self):
        """Clear all data."""
        for channel in list(self.active_channels):
            self.reset_channel(channel)
    
    def close(self):
        """Close the plot window."""
        plt.close(self.fig)
