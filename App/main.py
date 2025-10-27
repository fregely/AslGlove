import argparse
import asyncio
import logging
import struct
import time
from typing import Optional, Dict
from pathlib import Path
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
import matplotlib.pyplot as plt
from collections import deque, defaultdict
from converter import IMUConverter

# ====== Default Configuration ======
DEFAULT_NAME = "ASL_Glove"
DEFAULT_CHAR_UUID = "c4e7a180-7b2f-4c95-bfc5-1d5c62123456"

# For smoother real-time plotting
MAX_POINTS = 200
NUM_CHANNELS = 2  # Number of IMU channels (0 and 1)
converter = IMUConverter() 

# Data storage per channel
channel_data = defaultdict(lambda: {
    'ax': deque(maxlen=MAX_POINTS),
    'ay': deque(maxlen=MAX_POINTS),
    'az': deque(maxlen=MAX_POINTS),
    'gx': deque(maxlen=MAX_POINTS),
    'gy': deque(maxlen=MAX_POINTS),
    'gz': deque(maxlen=MAX_POINTS),
    'mx': deque(maxlen=MAX_POINTS),
    'my': deque(maxlen=MAX_POINTS),
    'mz': deque(maxlen=MAX_POINTS),
})

# Recording state
recording_file = None
packet_count = 0
seen_channels = set()

# --- Plot setup - Create subplots for each channel ---
plt.ion()
fig, axes = plt.subplots(NUM_CHANNELS, 3, figsize=(15, 4*NUM_CHANNELS))

# If only one channel, axes needs to be 2D
if NUM_CHANNELS == 1:
    axes = axes.reshape(1, -1)

# Store line objects for each channel
lines = {}
for ch in range(NUM_CHANNELS):
    lines[ch] = {
        'accel': {
            'ax': axes[ch, 0].plot([], [], label='Ax', color='red')[0],
            'ay': axes[ch, 0].plot([], [], label='Ay', color='green')[0],
            'az': axes[ch, 0].plot([], [], label='Az', color='blue')[0],
        },
        'gyro': {
            'gx': axes[ch, 1].plot([], [], label='Gx', color='red')[0],
            'gy': axes[ch, 1].plot([], [], label='Gy', color='green')[0],
            'gz': axes[ch, 1].plot([], [], label='Gz', color='blue')[0],
        },
        'mag': {
            'mx': axes[ch, 2].plot([], [], label='Mx', color='red')[0],
            'my': axes[ch, 2].plot([], [], label='My', color='green')[0],
            'mz': axes[ch, 2].plot([], [], label='Mz', color='blue')[0],
        }
    }
    
    # Configure axes
    axes[ch, 0].set_title(f"Channel {ch} - Accelerometer (g)")
    axes[ch, 1].set_title(f"Channel {ch} - Gyroscope (°/s)")
    axes[ch, 2].set_title(f"Channel {ch} - Magnetometer (µT)")
    
    axes[ch, 0].legend(loc='upper right')
    axes[ch, 1].legend(loc='upper right')
    axes[ch, 2].legend(loc='upper right')
    
    axes[ch, 0].set_ylim(-4, 4)
    axes[ch, 1].set_ylim(-400, 400)
    axes[ch, 2].set_ylim(-100, 100)
    
    axes[ch, 0].grid(True, alpha=0.3)
    axes[ch, 1].grid(True, alpha=0.3)
    axes[ch, 2].grid(True, alpha=0.3)

plt.tight_layout()

def update_plot():
    """Update the live plot with new data for all channels."""
    for ch in seen_channels:
        if ch >= NUM_CHANNELS:
            continue
            
        data = channel_data[ch]
        
        # Update accelerometer
        x = range(len(data['ax']))
        lines[ch]['accel']['ax'].set_data(x, list(data['ax']))
        lines[ch]['accel']['ay'].set_data(x, list(data['ay']))
        lines[ch]['accel']['az'].set_data(x, list(data['az']))
        
        # Update gyroscope
        lines[ch]['gyro']['gx'].set_data(x, list(data['gx']))
        lines[ch]['gyro']['gy'].set_data(x, list(data['gy']))
        lines[ch]['gyro']['gz'].set_data(x, list(data['gz']))
        
        # Update magnetometer
        lines[ch]['mag']['mx'].set_data(x, list(data['mx']))
        lines[ch]['mag']['my'].set_data(x, list(data['my']))
        lines[ch]['mag']['mz'].set_data(x, list(data['mz']))
        
        # Rescale axes
        for ax in axes[ch, :]:
            ax.relim()
            ax.autoscale_view()
    
    plt.pause(0.01)

logger = logging.getLogger(__name__)


class Args(argparse.Namespace):
    name: Optional[str]
    address: Optional[str]
    macos_use_bdaddr: bool
    characteristic: str
    debug: bool
    record: Optional[str]
    playback: Optional[str]
    playback_speed: float


def process_packet(data: bytearray, source: str = "BLE"):
    """
    Process a single IMU packet and update the plot.
    
    Packet structure (27 bytes total):
    - 1 byte: channel (uint8_t)
    - 8 bytes: timestamp_us (uint64_t, little-endian)
    - 12 bytes: raw_data (accel XYZ + gyro XYZ, NO temperature)
        - 6 bytes: accel (big-endian int16)
        - 6 bytes: gyro (big-endian int16)
    - 6 bytes: mag_data (magnetometer XYZ, little-endian int16)
    """
    global packet_count, seen_channels
    
    logger.debug(f"[{source}] Received {len(data)} bytes: {data.hex()}")
    
    try:
        # Check packet length
        if len(data) != 27:
            logger.warning(f"⚠️ Expected 27 bytes, got {len(data)}")
            logger.warning(f"Raw packet: {data.hex()}")
            return
        
        # Unpack the packet header
        channel = data[0]
        timestamp_us = struct.unpack_from('<Q', data, 1)[0]  # uint64_t little-endian
        
        # Track which channels we've seen
        seen_channels.add(channel)
        
        # Unpack the 12 bytes of raw IMU data (big-endian int16)
        # ICM-20948 stores accel/gyro data in big-endian format (high byte first)
        # Register order: Accel X,Y,Z → Gyro X,Y,Z (no temperature)
        raw_values = struct.unpack_from('>6h', data, 9)  # 6 signed 16-bit big-endian
        
        ax, ay, az = raw_values[0], raw_values[1], raw_values[2]
        gx, gy, gz = raw_values[3], raw_values[4], raw_values[5]
        
        # Unpack magnetometer data (6 bytes at offset 21)
        # AK09916 magnetometer uses LITTLE-ENDIAN format (different from ICM-20948!)
        mag_values = struct.unpack_from('<3h', data, 21)  # 3 signed 16-bit little-endian
        mx, my, mz = mag_values[0], mag_values[1], mag_values[2]
        
        ax_g, ay_g, az_g = converter.convert_accelerometer(ax, ay, az)
        gx_deg, gy_deg, gz_deg = converter.convert_gyroscope(gx, gy, gz)
        mx_ut, my_ut, mz_ut = converter.convert_magnetometer(mx, my, mz)

        # Check if magnetometer is all zeros
        if mx == 0 and my == 0 and mz == 0:
            logger.warning(f"[{source}] Ch{channel} @{timestamp_us}µs → Accel({ax},{ay},{az}) Gyro({gx},{gy},{gz}) Mag(0,0,0) ❌ MAG NOT WORKING")
        else:
            logger.info(f"[{source}] Ch{channel} @{timestamp_us}µs → Accel({ax},{ay},{az}) Gyro({gx},{gy},{gz}) Mag({mx},{my},{mz})")
        
        # Append data to the appropriate channel's queues
        data_dict = channel_data[channel]
        data_dict['ax'].append(ax_g)    # NEW - in g
        data_dict['ay'].append(ay_g)    # NEW - in g
        data_dict['az'].append(az_g)    # NEW - in g
        data_dict['gx'].append(gx_deg)  # NEW - in °/s
        data_dict['gy'].append(gy_deg)  # NEW - in °/s
        data_dict['gz'].append(gz_deg)  # NEW - in °/s
        data_dict['mx'].append(mx_ut)   # NEW - in µT
        data_dict['my'].append(my_ut)   # NEW - in µT
        data_dict['mz'].append(mz_ut)   # NEW - in µT
        
        packet_count += 1
        
        # Update plot every 20 packets for better performance
        if packet_count % 20 == 0:
            update_plot()
        
    except Exception as e:
        logger.error(f"⚠️ Failed to parse IMU packet: {e}")


# ====== Notification Handler ======
def notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
    """Handles BLE notifications and optionally records them."""
    global recording_file
    
    # Record packet if recording is enabled
    if recording_file is not None:
        try:
            # Write packet length (2 bytes) + packet data
            recording_file.write(struct.pack('<H', len(data)))
            recording_file.write(data)
            recording_file.flush()  # Ensure data is written immediately
        except Exception as e:
            logger.error(f"Failed to write packet to file: {e}")
    
    # Process the packet normally
    process_packet(data, source="BLE")


# ====== Recording Functions ======
def start_recording(filename: str):
    """Start recording packets to a file."""
    global recording_file, packet_count
    
    filepath = Path(filename)
    recording_file = open(filepath, 'wb')
    packet_count = 0
    
    # Write file header
    header = b'IMU_REC\x01'  # Magic string + version
    recording_file.write(header)
    recording_file.flush()
    
    logger.info(f"📹 Recording started: {filepath.absolute()}")


def stop_recording():
    """Stop recording and close the file."""
    global recording_file, packet_count
    
    if recording_file is not None:
        recording_file.close()
        logger.info(f"⏹️  Recording stopped: {packet_count} packets saved")
        recording_file = None


# ====== Playback Functions ======
async def playback_from_file(filename: str, speed: float = 1.0):
    """Play back recorded packets from a file."""
    global packet_count
    
    filepath = Path(filename)
    if not filepath.exists():
        logger.error(f"❌ Playback file not found: {filepath}")
        return
    
    logger.info(f"▶️  Starting playback from: {filepath.absolute()}")
    packet_count = 0
    
    with open(filepath, 'rb') as f:
        # Read and verify header
        header = f.read(8)
        if header != b'IMU_REC\x01':
            logger.error("❌ Invalid file format (missing or wrong header)")
            return
        
        logger.info(f"✅ Valid recording file, playing at {speed}x speed")
        
        last_timestamp = None
        
        try:
            while True:
                # Read packet length
                length_bytes = f.read(2)
                if len(length_bytes) < 2:
                    break  # End of file
                
                packet_length = struct.unpack('<H', length_bytes)[0]
                
                # Read packet data
                packet_data = f.read(packet_length)
                if len(packet_data) < packet_length:
                    logger.warning("⚠️ Incomplete packet at end of file")
                    break
                
                # Extract timestamp for timing
                if len(packet_data) >= 9:
                    timestamp_us = struct.unpack_from('<Q', packet_data, 1)[0]
                    
                    # Calculate delay based on timestamps
                    if last_timestamp is not None and speed > 0:
                        delay = (timestamp_us - last_timestamp) / 1_000_000.0  # Convert to seconds
                        delay = delay / speed  # Adjust for playback speed
                        if delay > 0:
                            await asyncio.sleep(delay)
                    
                    last_timestamp = timestamp_us
                
                # Process the packet
                process_packet(bytearray(packet_data), source="PLAYBACK")
                
        except KeyboardInterrupt:
            logger.info("🛑 Playback stopped by user")
        
        logger.info(f"✅ Playback complete: {packet_count} packets played")


# ====== Main BLE Routine ======
async def main(args: Args):
    # Handle playback mode
    if args.playback:
        await playback_from_file(args.playback, args.playback_speed)
        return
    
    # Normal BLE mode
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

    # Start recording if requested
    if args.record:
        start_recording(args.record)

    async with BleakClient(device) as client:
        if not client.is_connected:
            logger.error("❌ Failed to connect.")
            return

        logger.info("✅ Connected to device!")
        
        # List all services and characteristics for debugging
        if args.debug:
            for service in client.services:
                logger.debug(f"Service: {service.uuid}")
                for char in service.characteristics:
                    logger.debug(f"  Characteristic: {char.uuid} - {char.properties}")
        
        await client.start_notify(args.characteristic, notification_handler)
        logger.info("📡 Streaming data... (Press Ctrl+C to stop)")

        try:
            # Run until user stops it
            while client.is_connected:
                await asyncio.sleep(0.1)
        except KeyboardInterrupt:
            logger.info("🛑 Stopping notifications...")
        finally:
            await client.stop_notify(args.characteristic)
            stop_recording()  # Stop recording if active
            logger.info("🔌 Disconnected cleanly.")


# ====== CLI Argument Setup ======
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="BLE IMU Data Receiver with Recording/Playback - Separate plots per channel",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Live streaming (no recording)
  python main.py
  
  # Live streaming + record to file
  python main.py --record test_data.imu
  
  # Playback from file
  python main.py --playback test_data.imu
  
  # Playback at 2x speed
  python main.py --playback test_data.imu --playback-speed 2.0
        """
    )

    parser.add_argument(
        "--name",
        metavar="<n>",
        default=DEFAULT_NAME,
        help=f"Bluetooth device name (default: {DEFAULT_NAME})",
    )
    parser.add_argument(
        "--address",
        metavar="<address>",
        default=None,
        help="Bluetooth MAC address",
    )
    parser.add_argument(
        "--macos-use-bdaddr",
        action="store_true",
        help="Use Bluetooth address instead of UUID on macOS",
    )
    parser.add_argument(
        "--characteristic",
        metavar="<uuid>",
        default=DEFAULT_CHAR_UUID,
        help=f"UUID of characteristic to subscribe to (default: {DEFAULT_CHAR_UUID})",
    )
    parser.add_argument(
        "-d",
        "--debug",
        action="store_true",
        help="Enable debug logging output",
    )
    parser.add_argument(
        "--record",
        metavar="<filename>",
        default=None,
        help="Record packets to file while streaming (e.g., test_data.imu)",
    )
    parser.add_argument(
        "--playback",
        metavar="<filename>",
        default=None,
        help="Play back recorded packets from file (instead of live BLE)",
    )
    parser.add_argument(
        "--playback-speed",
        metavar="<speed>",
        type=float,
        default=1.0,
        help="Playback speed multiplier (default: 1.0, use 0 for no delay)",
    )

    args = parser.parse_args(namespace=Args())

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=log_level,
        format="%(asctime)-15s %(name)-8s %(levelname)s: %(message)s",
    )

    asyncio.run(main(args))
