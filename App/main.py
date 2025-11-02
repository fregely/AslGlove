# main.py or wherever you orchestrate everything

from imu_processing import IMUConverter, MadgwickFilter, DeadReckoning
from imu_processing.ble_client import BLEClient
from imu_processing.visualization import RealtimePlotter  # You'll make this

# Setup
converter = IMUConverter()
filters = [MadgwickFilter(sample_rate=20, beta=0.25) for _ in range(6)]
dead_reckoning = [DeadReckoning(sample_rate=20) for _ in range(6)]
plotter = RealtimePlotter(num_imus=6)

# ====== Example Usage ======
async def main():
    def handle_packet(packet: dict):
        """Callback for processing received packets."""
        channel = packet['channel']
        timestamp = packet['timestamp_us']
        print(f"IMU {channel} @ {timestamp}: Accel={packet['accel_raw']}")
    
    client = BLEClient(
        device_name="ASL_Glove",
        on_packet_received=handle_packet
    )
    
    try:
        await client.connect()
        await client.start_streaming()
        await client.run_until_disconnected()
    finally:
        await client.stop_streaming()
        await client.disconnect()



