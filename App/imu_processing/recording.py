# imu_processing/recording.py
"""
Recording and playback utilities for IMU data.
"""

import pickle
import json
import struct
from pathlib import Path
from datetime import datetime
from typing import List, Dict


class Recording:
    """
    Handles saving and loading IMU recordings.
    
    Supports two formats:
    - .pkl (pickle): Fast, compact, includes all data types
    - .json: Human-readable, slightly larger
    """
    
    @staticmethod
    def save(packets: List[Dict], filename: str = None) -> str:
        """
        Save recorded packets to file.
        
        Parameters:
        -----------
        packets : list of dict
            Recorded state dictionaries
        filename : str, optional
            Output filename. If None, auto-generates with timestamp.
        
        Returns:
        --------
        str : Path to saved file
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"recording_{timestamp}.pkl"
        
        filepath = Path(filename)
        
        # Determine format from extension
        if filepath.suffix == '.json':
            Recording._save_json(packets, filepath)
        else:
            # Default to pickle
            if filepath.suffix != '.pkl':
                filepath = filepath.with_suffix('.pkl')
            Recording._save_pickle(packets, filepath)
        
        return str(filepath)
    
    @staticmethod
    def _save_pickle(packets: List[Dict], filepath: Path):
        """Save as pickle file."""
        with open(filepath, 'wb') as f:
            pickle.dump({
                'version': '1.0',
                'packet_count': len(packets),
                'channels': list(set(p['channel'] for p in packets)),
                'timestamp': datetime.now().isoformat(),
                'packets': packets
            }, f)
    
    @staticmethod
    def _save_json(packets: List[Dict], filepath: Path):
        """Save as JSON file (converts tuples to lists)."""
        # Convert tuples to lists for JSON serialization
        json_packets = []
        for p in packets:
            json_p = {}
            for key, value in p.items():
                if isinstance(value, tuple):
                    json_p[key] = list(value)
                else:
                    json_p[key] = value
            json_packets.append(json_p)
        
        with open(filepath, 'w') as f:
            json.dump({
                'version': '1.0',
                'packet_count': len(packets),
                'channels': list(set(p['channel'] for p in packets)),
                'timestamp': datetime.now().isoformat(),
                'packets': json_packets
            }, f, indent=2)
    
    @staticmethod
    def load(filename: str) -> List[Dict]:
        """
        Load recorded packets from file.
        
        Parameters:
        -----------
        filename : str
            Input filename (.pkl or .json)
        
        Returns:
        --------
        list of dict : Loaded packets
        """
        filepath = Path(filename)
        
        if not filepath.exists():
            raise FileNotFoundError(f"Recording not found: {filename}")
        
        if filepath.suffix == '.json':
            return Recording._load_json(filepath)
        else:
            return Recording._load_pickle(filepath)
    
    @staticmethod
    def _load_pickle(filepath: Path) -> List[Dict]:
        """Load from pickle file."""
        with open(filepath, 'rb') as f:
            data = pickle.load(f)
        
        print(f"📼 Loaded recording:")
        print(f"   Version: {data.get('version', 'unknown')}")
        print(f"   Packets: {data['packet_count']}")
        print(f"   Channels: {data['channels']}")
        print(f"   Recorded: {data.get('timestamp', 'unknown')}")
        
        return data['packets']
    
    @staticmethod
    def _load_json(filepath: Path) -> List[Dict]:
        """Load from JSON file (converts lists back to tuples)."""
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        print(f"📼 Loaded recording:")
        print(f"   Version: {data.get('version', 'unknown')}")
        print(f"   Packets: {data['packet_count']}")
        print(f"   Channels: {data['channels']}")
        print(f"   Recorded: {data.get('timestamp', 'unknown')}")
        
        # Convert lists back to tuples for consistency
        packets = []
        for p in data['packets']:
            converted_p = {}
            for key, value in p.items():
                if isinstance(value, list) and len(value) in [2, 3, 4]:
                    converted_p[key] = tuple(value)
                else:
                    converted_p[key] = value
            packets.append(converted_p)
        
        return packets
    
    @staticmethod
    def info(filename: str):
        """Print information about a recording without loading all packets."""
        filepath = Path(filename)
        
        if not filepath.exists():
            raise FileNotFoundError(f"Recording not found: {filename}")
        
        if filepath.suffix == '.json':
            with open(filepath, 'r') as f:
                data = json.load(f)
        else:
            with open(filepath, 'rb') as f:
                data = pickle.load(f)
        
        print(f"\n📼 Recording Info: {filename}")
        print(f"{'='*50}")
        print(f"Format:       {filepath.suffix}")
        print(f"Version:      {data.get('version', 'unknown')}")
        print(f"Packets:      {data['packet_count']}")
        print(f"Channels:     {data['channels']}")
        print(f"Recorded:     {data.get('timestamp', 'unknown')}")
        print(f"File size:    {filepath.stat().st_size / 1024:.1f} KB")
        
        if data['packets']:
            first = data['packets'][0]
            last = data['packets'][-1]
            duration_us = last['timestamp_us'] - first['timestamp_us']
            duration_s = duration_us / 1_000_000
            
            print(f"Duration:     {duration_s:.2f} seconds")
            print(f"Sample rate:  ~{data['packet_count'] / duration_s:.1f} Hz")
        
        print(f"{'='*50}\n")


class PlaybackClient:
    """
    Simulates BLE client by playing back recorded packets.
    
    Usage:
        client = PlaybackClient("recording.pkl")
        await client.connect()
        await client.start_streaming()
        
        while True:
            packet_bytes = await client.get_packet()
            # ... process normally
    """
    
    def __init__(self, recording_file: str, realtime: bool = True, speed: float = 1.0):
        """
        Initialize playback client.
        
        Parameters:
        -----------
        recording_file : str
            Path to recording file
        realtime : bool
            If True, replay at original timing. If False, as fast as possible.
        speed : float
            Playback speed multiplier (2.0 = 2x speed, 0.5 = half speed)
        """
        self.recording_file = recording_file
        self.realtime = realtime
        self.speed = speed
        self.packets = None
        self.current_index = 0
        self.connected = False
    
    async def connect(self):
        """Load recording file."""
        import asyncio
        
        print(f"📂 Loading recording: {self.recording_file}")
        self.packets = Recording.load(self.recording_file)
        self.current_index = 0
        self.connected = True
        
        await asyncio.sleep(0.1)  # Simulate connection delay
        print(f"✅ Playback ready: {len(self.packets)} packets")
    
    async def start_streaming(self):
        """Start streaming (no-op for playback)."""
        import asyncio
        await asyncio.sleep(0.01)
    
    async def get_packet(self) -> bytes:
        """
        Get next packet from recording.
        
        Returns bytes in original packet format for compatibility.
        """
        import asyncio
        
        if not self.connected:
            raise RuntimeError("Not connected - call connect() first")
        
        if self.current_index >= len(self.packets):
            raise StopIteration("End of recording")
        
        # Get current packet
        packet = self.packets[self.current_index]
        
        # Sleep to maintain original timing (if realtime mode)
        if self.realtime and self.current_index > 0:
            prev_packet = self.packets[self.current_index - 1]
            time_diff_us = packet['timestamp_us'] - prev_packet['timestamp_us']
            time_diff_s = (time_diff_us / 1_000_000) / self.speed
            
            if time_diff_s > 0:
                await asyncio.sleep(time_diff_s)
        
        # Convert state dict back to raw bytes
        packet_bytes = self._pack_packet(packet)
        
        self.current_index += 1
        return packet_bytes
    
    def _pack_packet(self, packet: Dict) -> bytes:
        """Pack state dict back into raw bytes format."""
        # Pack: channel (1) + timestamp (8) + accel (6) + gyro (6) + mag (6) = 27 bytes
        
        channel = packet['channel']
        timestamp = packet['timestamp_us']
        
        # Get raw values
        ax, ay, az = packet['accel_raw']
        gx, gy, gz = packet['gyro_raw']
        mx, my, mz = packet['mag_raw']
        
        # Pack into bytes
        packet_bytes = struct.pack('<B', channel)  # 1 byte channel
        packet_bytes += struct.pack('<Q', timestamp)  # 8 bytes timestamp
        packet_bytes += struct.pack('>6h', ax, ay, az, gx, gy, gz)  # 12 bytes accel+gyro
        packet_bytes += struct.pack('<3h', mx, my, mz)  # 6 bytes mag
        
        return packet_bytes
    
    async def stop_streaming(self):
        """Stop streaming (no-op for playback)."""
        import asyncio
        await asyncio.sleep(0.01)
    
    async def disconnect(self):
        """Disconnect (no-op for playback)."""
        import asyncio
        self.connected = False
        await asyncio.sleep(0.01)
        print("📼 Playback finished")
