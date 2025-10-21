import argparse
import asyncio
import logging
import struct
from typing import Optional
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


# ====== Notification Handler ======
def notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
    """
    Handles BLE notifications from the ESP32 and updates live plot.
    
    Expected packet structure (23 bytes total):
    - 1 byte: channel (uint8_t)
    - 8 bytes: timestamp_us (uint64_t, little-endian)
    - 14 bytes: raw_data
        - 2 bytes: accel X (high, low) - big-endian int16
        - 2 bytes: accel Y
        - 2 bytes: accel Z
        - 2 bytes: temperature
        - 2 bytes: gyro X
        - 2 bytes: gyro Y
        - 2 bytes: gyro Z
    """
    logger.debug(f"Received {len(data)} bytes: {data.hex()}")
    
    try:
        # Check packet length
        if len(data) != 23:
            logger.warning(f"⚠️ Expected 23 bytes, got {len(data)}")
            return
        
        # Unpack the packet header
        channel = data[0]
        timestamp_us = struct.unpack_from('<Q', data, 1)[0]  # uint64_t little-endian
        
        # Unpack the 14 bytes of raw IMU data (big-endian int16)
        # ICM-20948 stores data in big-endian format (high byte first)
        raw_values = struct.unpack_from('>7h', data, 9)  # 7 signed 16-bit big-endian
        
        ax, ay, az = raw_values[0], raw_values[1], raw_values[2]
        temp_raw = raw_values[3]
        gx, gy, gz = raw_values[4], raw_values[5], raw_values[6]
        
        logger.info(f"Ch{channel} @{timestamp_us}µs → Accel({ax},{ay},{az}) Gyro({gx},{gy},{gz})")
        
        # Append data to queues for live plotting
        ax_data.append(ax)
        ay_data.append(ay)
        az_data.append(az)
        gx_data.append(gx)
        gy_data.append(gy)
        gz_data.append(gz)
        
        update_plot()
        
    except Exception as e:
        logger.error(f"⚠️ Failed to parse IMU packet: {e}")


# ====== Main BLE Routine ======
async def main(args: Args):
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
            logger.info("🔌 Disconnected cleanly.")


# ====== CLI Argument Setup ======
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="BLE Notification Receiver for ASL Glove")

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
        metavar="<notify uuid>",
        default=DEFAULT_CHAR_UUID,
        help=f"UUID of characteristic to subscribe to (default: {DEFAULT_CHAR_UUID})",
    )
    parser.add_argument(
        "-d",
        "--debug",
        action="store_true",
        help="Enable debug logging output",
    )

    args = parser.parse_args(namespace=Args())

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=log_level,
        format="%(asctime)-15s %(name)-8s %(levelname)s: %(message)s",
    )

    asyncio.run(main(args))
