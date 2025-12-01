# imu_processing/ble_client.py
import asyncio
import logging
from typing import Optional
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from imu_processing.packet_parser import PacketParser


logger = logging.getLogger(__name__)

CMD_LED_SELECT = bytes([3])

class BLEClient:
    """
    Handles BLE connection and receives raw packet bytes.
    Returns raw bytes without any parsing.
    """
    
    def __init__(
        self, 
        device_name: str = "ASL_Glove", 
        characteristic_uuid: str = "c4e7a180-7b2f-4c95-bfc5-1d5c62123456",
        led_characteristic_uuid: str = "01234567-89ab-cdef-0123-456789abcdef"  # ADD THIS
    ):
        self.device_name = device_name
        self.characteristic_uuid = characteristic_uuid  # IMU data
        self.led_characteristic_uuid = led_characteristic_uuid  # LED control
        self.packet_queue = asyncio.Queue()  # Stores raw bytearray (27 bytes)
        self.client: Optional[BleakClient] = None
        self.is_connected = False
        self.led_queue = asyncio.Queue()        # LED index (1 byte)
        
    async def connect(self, address: Optional[str] = None, macos_use_bdaddr: bool = False) -> None:
        """Connect to the BLE device."""
        logger.info(f"🔍 Scanning for {self.device_name}...")
        
        if address:
            device = await BleakScanner.find_device_by_address(
                address, cb={"use_bdaddr": macos_use_bdaddr}
            )
        else:
            device = await BleakScanner.find_device_by_name(
                self.device_name, cb={"use_bdaddr": macos_use_bdaddr}
            )
        
        if device is None:
            raise ConnectionError(f"Could not find device: {self.device_name}")
        
        logger.info(f"✅ Found device: {device.name} [{device.address}]")
        
        self.client = BleakClient(device)
        await self.client.connect()
        
        if not self.client.is_connected:
            raise ConnectionError("Failed to connect to device")
        
        self.is_connected = True
        logger.info("✅ Connected!")
        
    async def start_streaming(self) -> None:
        """Start receiving IMU data notifications."""
        if not self.client or not self.is_connected:
            raise RuntimeError("Not connected to device")
        
        # Subscribe to IMU data
        await self.client.start_notify(
            self.characteristic_uuid,
            self._notification_handler
        )
        
        # Subscribe to LED notifications (for calibration sync)
        await self.client.start_notify(
            self.led_characteristic_uuid,
            self._notification_handler
        )
        
        logger.info("📡 Streaming data... (Ctrl+C to stop)")
        
    async def stop_streaming(self) -> None:
        """Stop receiving notifications."""
        if self.client and self.is_connected:
            await self.client.stop_notify(self.characteristic_uuid)
            await self.client.stop_notify(self.led_characteristic_uuid)
            logger.info("Stopped streaming")
    
    async def disconnect(self) -> None:
        """Disconnect from device."""
        if self.client and self.is_connected:
            await self.client.disconnect()
            self.is_connected = False
            logger.info("Disconnected")
    
    def _notification_handler(
        self, 
        characteristic: BleakGATTCharacteristic, 
        data: bytearray
    ) -> None:
        # LED packet (1 byte)
        if len(data) == 1:
            self.led_queue.put_nowait(data)

            # Forward LED packet to external handler (VisionProcessor)
            if hasattr(self, "external_handler") and self.external_handler:
                try:
                    self.external_handler(characteristic, data)
                except Exception as e:
                    logger.error(f"External handler error: {e}")
            return

        # IMU packet (27 bytes)
        if len(data) == 27:
            self.packet_queue.put_nowait(data)
            return

        # Unexpected size
        logger.warning(f"Unexpected BLE packet size: {len(data)}")
    
    async def get_packet(self) -> bytearray:
        """
        Get the next raw packet bytes from BLE.
        Returns the exact bytearray received from the device.
        """
        return await self.packet_queue.get()
    
    async def run_until_disconnected(self, poll_interval: float = 0.1) -> None:
        """Keep connection alive until stopped."""
        try:
            while self.client and self.client.is_connected:
                await asyncio.sleep(poll_interval)
        except KeyboardInterrupt:
            logger.info("🛑 User interrupted")


    def set_external_handler(self, handler: callable) -> None:
        """ Set an external BLE notification handler. """
        self.external_handler = handler
        
        
    async def select_led(self, gpio: int) -> None:
        if not self.client:
            raise RuntimeError("BLE client not connected")

        await self.client.write_gatt_char(
            self.characteristic_uuid,              # (or the command/write UUID — see next section)
            CMD_LED_SELECT + bytes([gpio]),
            response=False                         # ✅ important on macOS
        )
        logger.debug(f"Sent CMD_LED_SELECT: GPIO={gpio}")