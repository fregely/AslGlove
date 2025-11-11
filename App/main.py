# main.py

import asyncio
import logging
from imu_processing.ble_client import BLEClient
from imu_processing.packet_parser import PacketParser
from imu_processing.converter import IMUConverter
from imu_processing.madgwick import MadgwickFilter
from imu_processing.dead_reckoning import DeadReckoning
from imu_processing.imu_graphing import ComprehensiveVisualizer
async def main():
    # Setup
    client = BLEClient()
    parser = PacketParser() 
    converter = IMUConverter()
    madgwick_filter = MadgwickFilter(sample_rate=20, beta=.5)
    dead_filter = DeadReckoning(sample_rate=20)
    graph = ComprehensiveVisualizer()
    
    # Optional: Recording
    recording = True
    recorded_packets = []
    
    await client.connect()
    await client.start_streaming()
    
    while True:
        # 1. Get RAW bytes from BLE
        raw_bytes = await client.get_packet()  # ← Just raw bytes!
        
        # 3. Parse the bytes
        packet = parser.parse(raw_bytes)
        
        # 4. Process
        converted = converter.converter(packet)
        
        madgwick = madgwick_filter.process(converted)

        dead_reckoned = dead_filter.process(madgwick)
        
        graph.update(converted, dead_reckoned)
        

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    asyncio.run(main())
