import argparse
import asyncio
import logging
import struct
import time
from typing import Optional
from pathlib import Path
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
import matplotlib.pyplot as plt
from collections import deque

# ====== Default Configuration ======
DEFAULT_NAME = "ASL_Glove"
DEFAULT_CHAR_UUID = "c4e7a180-7b2f-4c95-bfc5-1d5c62123456"

# For smoother real-time plotting
MAX_POINTS = 200
ax_data = deque(maxlen=MAX_POINTS)
ay_data = deque(maxlen=MAX_POINTS)
az_data = deque(maxlen=MAX_POINTS)
gx_data = deque(maxlen=MAX_POINTS)
gy_data = deque(maxlen=MAX_POINTS)
gz_data = deque(maxlen=MAX_POINTS)

# Recording state
recording_file = None
packet_count = 0

# --- Plot setup ---
plt.ion()
fig, (ax_accel, ax_gyro) = plt.subplots(2, 1, figsize=(8, 6))
line_ax, = ax_accel.plot([], [], label='Ax')
line_ay, = ax_accel.plot([], [], label='Ay')
line_az, = ax_accel.plot([], [], label='Az')
line_gx, = ax_gyro.plot([], [], label='Gx')
line_gy, = ax_gyro.plot([], [], label='Gy')
line_gz, = ax_gyro.plot([], [], label='Gz')

ax_accel.set_title("Accelerometer")
ax_gyro.set_title("Gyroscope")
for ax in (ax_accel, ax_gyro):
    ax.legend()
    ax.set_ylim(-40000, 40000)

def update_plot():
    """Update the live plot with new data."""
    line_ax.set_ydata(list(ax_data))
    line_ay.set_ydata(list(ay_data))
    line_az.set_ydata(list(az_data))
    line_gx.set_ydata(list(gx_data))
    line_gy.set_ydata(list(gy_data))
    line_gz.set_ydata(list(gz_data))

    x = range(len(ax_data))
    for line in (line_ax, line_ay, line_az, line_gx, line_gy, line_gz):
        line.set_xdata(x)

    for ax in (ax_accel, ax_gyro):
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
    
    Expected packet structure (23 bytes total):
    - 1 byte: channel (uint8_t)
    - 8 bytes: timestamp_us (uint64_t, little-endian)
    - 14 bytes: raw_data
        - 2 bytes: accel X (high, low) - big-endian int16
        - 2 bytes: accel Y
        - 2 bytes: accel Z
        - 2 bytes: gyro X
        - 2 bytes: gyro Y
        - 2 bytes: gyro Z
    """
    global packet_count
    
    logger.debug(f"[{source}] Received {len(data)} bytes: {data.hex()}")
    
    try:
        # Check packet length
        if len(data) != 21:
            logger.warning(f"⚠️ Expected 21 bytes, got {len(data)}")
            return
        
        # Unpack the packet header
        channel = data[0]
        timestamp_us = struct.unpack_from('<Q', data, 1)[0]  # uint64_t little-endian
        
        # Unpack the 14 bytes of raw IMU data (big-endian int16)
        # ICM-20948 stores data in big-endian format (high byte first)
        raw_values = struct.unpack_from('>6h', data, 9)  # 7 signed 16-bit big-endian
        
        ax, ay, az = raw_values[0], raw_values[1], raw_values[2]  # Accel
        gx, gy, gz = raw_values[3], raw_values[4], raw_values[5]  # Gyro  
        
        logger.info(f"[{source}] Ch{channel} @{timestamp_us}µs → Accel({ax},{ay},{az}) Gyro({gx},{gy},{gz})")
        
        # Append data to queues for live plotting
        ax_data.append(ax)
        ay_data.append(ay)
        az_data.append(az)
        gx_data.append(gx)
        gy_data.append(gy)
        gz_data.append(gz)
        
        update_plot()
        packet_count += 1
        
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
        description="BLE IMU Data Receiver with Recording/Playback",
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
        metavar="<name>",
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
