#!/usr/bin/env python3
"""
Quick test to verify IMU data is flowing through the system
"""
import asyncio
import logging
from imu_processing import BLEClient, PacketParser

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)

async def test_data_flow():
    """Test if we're receiving IMU data at all."""
    client = BLEClient()
    parser = PacketParser()
    
    try:
        logger.info("🔍 Connecting to BLE device...")
        await client.connect()
        logger.info("✅ Connected!")
        
        logger.info("📡 Starting streaming...")
        await client.start_streaming()
        logger.info("✅ Streaming started!")
        
        logger.info("📦 Waiting for packets...")
        packet_count = 0
        channel_counts = {}
        
        for _ in range(100):  # Collect 100 packets
            raw_bytes = await client.get_packet()
            packet = parser.parse(raw_bytes)
            
            channel = packet['channel']
            channel_counts[channel] = channel_counts.get(channel, 0) + 1
            packet_count += 1
            
            if packet_count % 20 == 0:
                logger.info(f"📦 Received {packet_count} packets: {channel_counts}")
        
        logger.info(f"✅ SUCCESS! Received {packet_count} packets from channels: {list(channel_counts.keys())}")
        logger.info(f"   Channel distribution: {channel_counts}")
        
    except Exception as e:
        logger.error(f"❌ Error: {e}", exc_info=True)
    finally:
        await client.stop_streaming()
        await client.disconnect()
        logger.info("🛑 Disconnected")

if __name__ == "__main__":
    asyncio.run(test_data_flow())
