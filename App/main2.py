import argparse
import asyncio
import logging
import struct
import math
import numpy as np
from typing import Optional
from collections import deque, defaultdict
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
import matplotlib.pyplot as plt

from App.imu_processing.converter import IMUConverter
from App.imu_processing.madgwick import MadgwickFilter
from App.imu_processing.dead_reckoning import DeadReckoning

# ====== Configuration ======
DEFAULT_NAME = "ASL_Glove"
DEFAULT_CHAR_UUID = "c4e7a180-7b2f-4c95-bfc5-1d5c62123456"
MAX_POINTS = 200
NUM_IMUS = 2

# ====== Setup ======
converter = IMUConverter()
filters = [MadgwickFilter(sample_rate=20, beta=0.25) for _ in range(NUM_IMUS)]
dead_reckoning = [DeadReckoning(sample_rate=20) for _ in range(NUM_IMUS)]

# Data storage - Orientation and Position
orientation_data = defaultdict(lambda: {
    'roll': deque(maxlen=MAX_POINTS),
    'pitch': deque(maxlen=MAX_POINTS),
    'yaw': deque(maxlen=MAX_POINTS),
})

position_data = defaultdict(lambda: {
    'x': deque(maxlen=MAX_POINTS),
    'y': deque(maxlen=MAX_POINTS),
    'z': deque(maxlen=MAX_POINTS),
})

seen_channels = set()
packet_count = 0
channel_packet_count = defaultdict(int)

# ====== Plot Setup ======
plt.ion()
# Create 2 columns: Orientation (left) and Position (right)
fig, axes = plt.subplots(NUM_IMUS, 2, figsize=(16, 4*NUM_IMUS))
if NUM_IMUS == 1:
    axes = axes.reshape(1, -1)

# Create line objects for each IMU
lines = {}
for ch in range(NUM_IMUS):
    lines[ch] = {
        'orientation': {
            'roll': axes[ch, 0].plot([], [], label='Roll', color='red', linewidth=2)[0],
            'pitch': axes[ch, 0].plot([], [], label='Pitch', color='green', linewidth=2)[0],
            'yaw': axes[ch, 0].plot([], [], label='Yaw', color='blue', linewidth=2)[0],
        },
        'position': {
            'x': axes[ch, 1].plot([], [], label='X', color='red', linewidth=2)[0],
            'y': axes[ch, 1].plot([], [], label='Y', color='green', linewidth=2)[0],
            'z': axes[ch, 1].plot([], [], label='Z', color='blue', linewidth=2)[0],
        }
    }
    
    # Configure orientation subplot
    axes[ch, 0].set_title(f"IMU {ch} - Orientation", fontsize=12, fontweight='bold')
    axes[ch, 0].set_ylabel("Angle (degrees)")
    axes[ch, 0].set_ylim(-180, 180)
    axes[ch, 0].legend(loc='upper right')
    axes[ch, 0].grid(True, alpha=0.3)
    axes[ch, 0].axhline(y=0, color='black', linestyle='--', alpha=0.3)
    
    # Configure position subplot
    axes[ch, 1].set_title(f"IMU {ch} - Position (Dead Reckoning)", fontsize=12, fontweight='bold')
    axes[ch, 1].set_ylabel("Position (meters)")
    axes[ch, 1].set_ylim(-1, 1)
    axes[ch, 1].legend(loc='upper right')
    axes[ch, 1].grid(True, alpha=0.3)
    axes[ch, 1].axhline(y=0, color='black', linestyle='--', alpha=0.3)

plt.tight_layout()

logger = logging.getLogger(__name__)


def update_plot():
    """Update the live plot with orientation and position data."""
    for ch in seen_channels:
        if ch >= NUM_IMUS:
            continue
        
        ori_data = orientation_data[ch]
        pos_data = position_data[ch]
        
        x = range(len(ori_data['roll']))
        
        # Update orientation plot
        lines[ch]['orientation']['roll'].set_data(x, list(ori_data['roll']))
        lines[ch]['orientation']['pitch'].set_data(x, list(ori_data['pitch']))
        lines[ch]['orientation']['yaw'].set_data(x, list(ori_data['yaw']))
        
        # Update position plot
        lines[ch]['position']['x'].set_data(x, list(pos_data['x']))
        lines[ch]['position']['y'].set_data(x, list(pos_data['y']))
        lines[ch]['position']['z'].set_data(x, list(pos_data['z']))
        
        # Rescale axes
        axes[ch, 0].relim()
        axes[ch, 0].autoscale_view(scalex=True, scaley=False)
        axes[ch, 1].relim()
        axes[ch, 1].autoscale_view(scalex=True, scaley=True)
    
    plt.pause(0.01)


def process_packet(data: bytearray):
    """
    Process IMU packet, update Madgwick filter and dead reckoning.
    """
    global packet_count, seen_channels
    
    try:
        if len(data) != 27:
            logger.warning(f"⚠️ Expected 27 bytes, got {len(data)}")
            return
        
        # Unpack header
        channel = data[0]
        timestamp_us = struct.unpack_from('<Q', data, 1)[0]
        
        if channel >= NUM_IMUS:
            logger.warning(f"⚠️ Channel {channel} exceeds NUM_IMUS={NUM_IMUS}")
            return
        
        seen_channels.add(channel)
        
        # Unpack IMU data (big-endian for accel/gyro)
        raw_values = struct.unpack_from('>6h', data, 9)
        ax, ay, az = raw_values[0], raw_values[1], raw_values[2]
        gx, gy, gz = raw_values[3], raw_values[4], raw_values[5]
        
        # Unpack magnetometer (little-endian)
        mag_values = struct.unpack_from('<3h', data, 21)
        mx, my, mz = mag_values[0], mag_values[1], mag_values[2]
        
        # Convert to physical units
        ax_g, ay_g, az_g = converter.convert_accelerometer(ax, ay, az)
        gx_deg, gy_deg, gz_deg = converter.convert_gyroscope(gx, gy, gz)
        mx_ut, my_ut, mz_ut = converter.convert_magnetometer(mx, my, mz)
        
        
        # Update Madgwick filter
        filters[channel].update(
            gx_rad, gy_rad, gz_rad,
            ax_g, ay_g, az_g,
            mx_ut, my_ut, mz_ut
        )
        
        # Get orientation
        roll, pitch, yaw = filters[channel].get_euler_angles()
        quaternion = filters[channel].get_quaternion()

        if len(quaternion) != 4:
            logger.error(f"❌ Quaternion has {len(quaternion)} elements, expected 4!")
            logger.error(f"   Values: {quaternion}")
            return
        # Detect if stationary (for zero-velocity update)
        accel_magnitude = math.sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g)
        gyro_magnitude_deg = math.sqrt(gx_deg*gx_deg + gy_deg*gy_deg + gz_deg*gz_deg)
        is_stationary = (abs(accel_magnitude - 1.0) < 0.05 and gyro_magnitude_deg < 5.0)
        
        # Update dead reckoning
        dr_result = dead_reckoning[channel].update(
            quaternion,
            np.array([ax_g, ay_g, az_g]),
            stationary=is_stationary
        )
        
        # Store orientation data
        orientation_data[channel]['roll'].append(roll)
        orientation_data[channel]['pitch'].append(pitch)
        orientation_data[channel]['yaw'].append(yaw)
        
        # Store position data
        pos = dr_result['position']
        position_data[channel]['x'].append(pos[0])
        position_data[channel]['y'].append(pos[1])
        position_data[channel]['z'].append(pos[2])
        
        # Increment per-channel packet count
        channel_packet_count[channel] += 1
        
        # Log every 10th packet per channel
        if channel_packet_count[channel] % 10 == 0:
            vel = dr_result['velocity']
            logger.info(f"IMU {channel}: Roll={roll:6.1f}° Pitch={pitch:6.1f}° Yaw={yaw:6.1f}° | "
                       f"Pos=[{pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}]m | "
                       f"Vel=[{vel[0]:5.2f}, {vel[1]:5.2f}, {vel[2]:5.2f}]m/s")
        
        packet_count += 1
        
        # Update plot every 5 packets
        if packet_count % 5 == 0:
            update_plot()
        
    except Exception as e:
        logger.error(f"⚠️ Failed to parse packet: {e}")


def notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
    """Handle BLE notifications."""
    process_packet(data)


async def main(args):
    """Main BLE connection and streaming loop."""
    logger.info("🔍 Starting Bluetooth scan...")
    
    if args.address:
        device = await BleakScanner.find_device_by_address(
            args.address, cb={"use_bdaddr": args.macos_use_bdaddr}
        )
    elif args.name:
        device = await BleakScanner.find_device_by_name(
            args.name, cb={"use_bdaddr": args.macos_use_bdaddr}
        )
    else:
        raise ValueError("Either --name or --address must be provided")
    
    if device is None:
        logger.error("❌ Could not find device")
        return
    
    logger.info(f"✅ Found device: {device.name} [{device.address}]")
    
    async with BleakClient(device) as client:
        if not client.is_connected:
            logger.error("❌ Failed to connect")
            return
        
        logger.info("✅ Connected!")
        await client.start_notify(args.characteristic, notification_handler)
        logger.info("📡 Streaming data... (Ctrl+C to stop)")
        logger.info(f"🎯 Madgwick Filter: Full 9-DOF")
        logger.info(f"📍 Dead Reckoning: Enabled")
        logger.info(f"⚠️  Position will drift - use as relative movement only")
        
        try:
            while client.is_connected:
                await asyncio.sleep(0.1)
        except KeyboardInterrupt:
            logger.info("🛑 Stopping...")
        finally:
            await client.stop_notify(args.characteristic)
            logger.info("🔌 Disconnected")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="IMU Orientation and Position Tracker"
    )
    
    parser.add_argument(
        "--name",
        default=DEFAULT_NAME,
        help=f"Device name (default: {DEFAULT_NAME})",
    )
    parser.add_argument(
        "--address",
        default=None,
        help="Device MAC address",
    )
    parser.add_argument(
        "--macos-use-bdaddr",
        action="store_true",
        help="Use BT address on macOS",
    )
    parser.add_argument(
        "--characteristic",
        default=DEFAULT_CHAR_UUID,
        help=f"Characteristic UUID (default: {DEFAULT_CHAR_UUID})",
    )
    parser.add_argument(
        "-d", "--debug",
        action="store_true",
        help="Enable debug logging",
    )
    
    args = parser.parse_args()
    
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=log_level,
        format="%(asctime)-15s %(levelname)s: %(message)s",
    )
    
    asyncio.run(main(args))
