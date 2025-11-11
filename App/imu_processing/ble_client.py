# imu_processing/ble_client.py

import asyncio
import logging
from typing import Optional
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from imu_processing.packet_parser import PacketParser


logger = logging.getLogger(__name__)


class BLEClient:
    """
    Handles BLE connection and receives raw packet bytes.
    Returns raw bytes without any parsing.
    """
    
    def __init__(
        self, 
        device_name: str = "ASL_Glove", 
        characteristic_uuid: str = "c4e7a180-7b2f-4c95-bfc5-1d5c62123456"
    ):
        self.device_name = device_name
        self.characteristic_uuid = characteristic_uuid
        self.packet_queue = asyncio.Queue()  # Stores raw bytearray
        self.client: Optional[BleakClient] = None
        self.is_connected = False
        
    async def connect(self, address: Optional[str] = None, macos_use_bdaddr: bool = False):
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
        
    async def start_streaming(self):
        """Start receiving IMU data notifications."""
        if not self.client or not self.is_connected:
            raise RuntimeError("Not connected to device")
        
        await self.client.start_notify(
            self.characteristic_uuid,
            self._notification_handler
        )
        logger.info("📡 Streaming data... (Ctrl+C to stop)")
        
    async def stop_streaming(self):
        """Stop receiving notifications."""
        if self.client and self.is_connected:
            await self.client.stop_notify(self.characteristic_uuid)
            logger.info("Stopped streaming")
    
    async def disconnect(self):
        """Disconnect from device."""
        if self.client and self.is_connected:
            await self.client.disconnect()
            self.is_connected = False
            logger.info("Disconnected")
    
    def _notification_handler(
        self, 
        characteristic: BleakGATTCharacteristic, 
        data: bytearray
    ):
        """Internal: called when BLE data arrives."""
        # Just queue the raw bytes - no parsing!
        self.packet_queue.put_nowait(data)
        logger.debug(f"Received {len(data)} bytes")
    
    async def get_packet(self) -> bytearray:
        """
        Get the next raw packet bytes from BLE.
        Returns the exact bytearray received from the device.
        """
        return await self.packet_queue.get()
    
    async def run_until_disconnected(self, poll_interval: float = 0.1):
        """Keep connection alive until stopped."""
        try:
            while self.client and self.client.is_connected:
                await asyncio.sleep(poll_interval)
        except KeyboardInterrupt:
            logger.info("🛑 User interrupted")

