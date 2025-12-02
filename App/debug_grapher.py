#!/usr/bin/env python3
"""
Debug script to identify why IMU graphs aren't updating
Run this to diagnose the issue step by step
"""

print("="*60)
print("IMU GRAPH DEBUG SCRIPT")
print("="*60)
print("\nThis script will help identify why graphs aren't updating.\n")

# Test 1: Check if grapher can be created
print("Test 1: Creating grapher...")
try:
    from imu_processing import IMUGrapher, GraphMode
    grapher = IMUGrapher(mode=GraphMode.CORRECTED, max_points=50, update_interval=5)
    print("✅ Grapher created successfully")
    print(f"   Mode: {grapher.mode}")
    print(f"   Update interval: {grapher.update_interval}")
except Exception as e:
    print(f"❌ Failed to create grapher: {e}")
    exit(1)

# Test 2: Check channel registration
print("\nTest 2: Testing channel registration...")
test_packet = {
    'channel': 0,
    'position': (0, 0, 0),
    'corrected_position': (0, 0, 0)
}
grapher.update(corrected=test_packet)
if 0 in grapher.active_channels:
    print("✅ Channel 0 registered successfully")
    print(f"   Active channels: {grapher.active_channels}")
else:
    print("❌ Channel 0 NOT registered!")
    print(f"   Active channels: {grapher.active_channels}")

# Test 3: Add some data and verify storage
print("\nTest 3: Adding test data...")
for i in range(10):
    packet = {
        'channel': 0,
        'position': (i*0.1, i*0.1, i*0.1),
        'corrected_position': (i*0.1, i*0.1, i*0.1)
    }
    grapher.update(corrected=packet)

data_len = len(grapher.corrected_data[0]['x'])
print(f"✅ Added 10 packets, stored data points: {data_len}")
if data_len == 10:
    print("   ✓ Data storage working correctly")
else:
    print(f"   ⚠️  Expected 10 points, got {data_len}")

# Test 4: Check if lines exist
print("\nTest 4: Checking plot lines...")
if 0 in grapher.lines:
    print("✅ Plot lines created for channel 0")
    print(f"   Available line keys: {list(grapher.lines[0].keys())}")
else:
    print("❌ No plot lines for channel 0!")

# Test 5: Check redraw logic
print("\nTest 5: Testing redraw logic...")
print(f"   Packet count: {grapher.packet_count}")
print(f"   Update interval: {grapher.update_interval}")
print(f"   Next redraw at packet: {(grapher.packet_count // grapher.update_interval + 1) * grapher.update_interval}")

grapher.close()

print("\n" + "="*60)
print("NEXT STEPS:")
print("="*60)
print("""
If all tests passed:
  → The grapher works fine in isolation
  → The issue is likely in the data pipeline

To debug the actual pipeline:
  1. Run: python main.py --mode corrected --debug
  2. Wait for calibration to complete
  3. Move your fingers
  4. Look for these messages:
     - "[GRAPHER] 📊 Registered new IMU channel: X"
     - "[GRAPHER] 🔄 Redrawing at packet N"
  
If you DON'T see "Registered new IMU channel":
  → Packets aren't reaching the grapher
  → Check if calibration is completing

If you see registration but no redraws:
  → Not enough packets arriving
  → Check BLE connection

If you see redraws but no movement on graph:
  → Check that 'corrected_position' is in packets
  → Try a different graph mode: --mode madgwick
""")
