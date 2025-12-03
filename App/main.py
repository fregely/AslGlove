# main.py
# pylint: disable=E1101
# mypy: ignore-errors

# ------------------------------
# VISION PROCESSING (OpenCV)
# ------------------------------
# Force X11 backend for OpenCV on Linux (must be before cv2 import)
import os
import platform
import asyncio
import argparse
import multiprocessing as mp
# NOW set environment variables BEFORE imports
if platform.system() == 'Linux':
    os.environ['QT_QPA_PLATFORM'] = 'xcb'
    os.environ['QT_LOGGING_RULES'] = '*.debug=false;qt.qpa.*=false'
# PID values
PID_KP = 0.5 # Porportional gain
PID_KI = 0.1 # Integral gain
PID_KD = 0.2 # Derivative gain

# OpenCV parameters
THRESH = 225 # Binary threshold for LED detection
MIN_AREA = 50 # Minimum area of detected blob
MAX_AREA = 10000 # Maximum area of detected blob
MIN_CIRC = 0.6 # Minimum circularity of detected blob
PIXEL_PER_MM = 10.0 # Pixels per millimeter conversion

# Parse args FIRST (before any imports that use Qt)
parser = argparse.ArgumentParser(
    description="ASL Glove IMU Data Pipeline",
    formatter_class=argparse.RawDescriptionHelpFormatter
)

# Add all your arguments here
parser.add_argument('--mode', '-m', type=str, default='position',
                   choices=['all', 'debug', 'sensor', 'madgwick', 'position', 'graph'])
parser.add_argument('graph_options', nargs='*')
parser.add_argument('--no-plot', action='store_true', default=None)
parser.add_argument('--force-plot', action='store_true')
parser.add_argument('--record', '-r', action='store_true', default=None)
parser.add_argument('--no-record', action='store_true')

# Vision (default enabled, can disable)
parser.add_argument('--no-vision', action='store_true', 
                    help='Disable vision processing (default: enabled)')
parser.add_argument('--vision', action='store_true', 
                    help='Enable vision processing (default: enabled)')  # For backwards compatibility
    
parser.add_argument('--calibrate', action='store_true')
parser.add_argument('--output', '-o', type=str)
parser.add_argument('--update', '-u', type=int, default=20)
parser.add_argument('--max-points', type=int, default=200)
parser.add_argument('--playback', '-p', type=str)
parser.add_argument('--speed', '-s', type=float, default=1.0)
parser.add_argument('--fast', '-f', action='store_true')
parser.add_argument('--debug', '-d', action='store_true')
parser.add_argument('--quiet', '-q', action='store_true')

 # PID parameters
parser.add_argument('--pid-kp', type=float, default=PID_KP, help='PID proportional gain')
parser.add_argument('--pid-ki', type=float, default=PID_KI, help='PID integral gain')
parser.add_argument('--pid-kd', type=float, default=PID_KD, help='PID derivative gain')
parser.add_argument('--px-per-mm', type=float, default=PIXEL_PER_MM, help='Pixels per MM')
    

args = parser.parse_args()


import logging
import platform
import math
import numpy as np  # ADD THIS
from collections import defaultdict
from letter_recognizer2 import LetterRecognizer2
from imu_processing.control import Control

control = Control()
recognizer = LetterRecognizer2()
recognizer.debug_mode = True

# Try to load existing calibration data
recognizer.load_calibration()

# for cv functionality
from computer_vision.cv import VisionProcessor

from imu_processing import (
    IMUConverter, 
    MadgwickFilter, 
    KalmanDeadReckoning,
    BLEClient,
    PacketParser,
    Recording,
    PlaybackClient,
    Control
)

# Dead reckoning Kalman parameters
GRAVITY_CONVENTION = 'NED' # 'NED' or 'ENU' This should be correct 
DEAD_Q = 0.1 # Process noise
DEAD_R = 0.01 # Measurement noise
DEAD_P = 0.1 # Estimate error covariance

# Dead reckoning safety margin for thresholds CALIBRATION
SAFETY_MARGIN = 1.5

logger = logging.getLogger(__name__)
MADGWICK_WARMUP_PACKETS = 200

def parse_graph_mode(args):
    """Parse command line arguments to determine GraphMode."""
    if not args.no_plot:
        from imu_processing import GraphMode
        
        mode = args.mode
        
        if mode == 'all':
            return GraphMode.ALL
        elif mode == 'debug':
            return GraphMode.CONVERTED | GraphMode.MADGWICK | GraphMode.DEAD_RECKONING | GraphMode.CORRECTED
        elif mode == 'sensor':
            return GraphMode.UNCONVERTED | GraphMode.CONVERTED
        elif mode == 'madgwick':
            return GraphMode.MADGWICK
        elif mode == 'position':
            return GraphMode.DEAD_RECKONING
        elif mode == 'corrected':
            return GraphMode.CORRECTED
        elif mode == 'compare':
            return GraphMode.POSITION_COMPARISON  # Raw vs Corrected
        elif mode == 'graph':
            if not args.graph_options:
                logger.error("--mode graph requires additional arguments")
                raise ValueError("No graph options provided for 'graph' mode")
            
            result = GraphMode(0)
            for option in args.graph_options:
                option_lower = option.lower()
                if option_lower == 'unconverted':
                    result |= GraphMode.UNCONVERTED
                elif option_lower == 'converted':
                    result |= GraphMode.CONVERTED
                elif option_lower == 'madgwick':
                    result |= GraphMode.MADGWICK
                elif option_lower in ['dead_reckoning', 'position', 'dead']:
                    result |= GraphMode.DEAD_RECKONING
                elif option_lower == 'corrected':
                    result |= GraphMode.CORRECTED
                else:
                    logger.warning(f"Unknown graph option: {option}")
            
            if result == 0:
                raise ValueError("No valid graph options provided")
            return result
        else:
            logger.error(f"Unknown mode: {mode}")
            raise ValueError(f"Invalid mode: {mode}")
    return None

async def main(args):
    """Main data processing pipeline."""
    
    # ---------------------------
    # CALIBRATION MODE EARLY EXIT
    # ---------------------------
    if args.calibrate:
        from computer_vision.calibration import Calibrator

        print("🔧 Entering calibration mode...")

        client = BLEClient()
        await client.connect()
        await client.start_streaming()

        vp = VisionProcessor(client.client, record=False, thresh=THRESH, 
                    min_area=MIN_AREA, 
                    max_area=MAX_AREA, 
                    min_circ=MIN_CIRC, 
                    pixel_per_mm=PIXEL_PER_MM)
        client.set_external_handler(vp.handler)
        # vision_task = asyncio.create_task(vp.start())

        # GPIOs for each finger in correct order
        led_gpio_order = [1, 3, 20, 7, 6]

        calibrator = Calibrator(vp, client, led_gpio_order)

        try:
            vision_task = await calibrator.run()
            
            # Stop the first vision task
            print("\n🛑 Stopping LED calibration vision task...")
            if vision_task:
                vision_task.cancel()
                try:
                    await vision_task
                except asyncio.CancelledError:
                    pass
            
            # Turn off LED override mode and restart normal cycling
            await client.select_led(255)  # Turn all LEDs OFF
            await asyncio.sleep(0.5)
            
            # Now perform open-hand calibration for letter recognition
            print("\n" + "="*50)
            print("📋 OPEN HAND CALIBRATION FOR LETTER RECOGNITION")
            print("="*50)
            print("\nInstructions:")
            print("1. Hold your hand completely open and flat")
            print("2. Keep all fingers extended and spread apart")
            print("3. Hold still for about 3 seconds")
            print("\nPress Enter when ready...")
            input()
            
            # Create a fresh vision processor for open-hand calibration
            vp2 = VisionProcessor(client.client, record=False, thresh=THRESH, 
                        min_area=MIN_AREA, 
                        max_area=MAX_AREA, 
                        min_circ=MIN_CIRC, 
                        pixel_per_mm=PIXEL_PER_MM)
            # Ensure the finger map is loaded
            vp2.load_finger_map("finger_map.json")
            client.set_external_handler(vp2.handler)
            
            # Start vision processing for calibration
            print("📹 Starting vision processor for open-hand calibration...")
            vision_task = asyncio.create_task(vp2.start())
            await asyncio.sleep(2.0)  # Give vision time to start
            
            # Wait for all finger positions to be populated
            print("⏳ Waiting for all 5 LEDs to cycle through...")
            timeout = 30.0  # Increased timeout
            start_time = asyncio.get_event_loop().time()
            last_count = 0
            
            while len(vp2.finger_positions) < 5:
                current_count = len(vp2.finger_positions)
                if current_count != last_count:
                    detected = list(vp2.finger_positions.keys())
                    print(f"   Detected {current_count}/5 fingers: {detected}")
                    last_count = current_count
                
                if asyncio.get_event_loop().time() - start_time > timeout:
                    print("⚠️ Warning: Not all fingers detected after 30 seconds")
                    print(f"   Detected fingers: {list(vp2.finger_positions.keys())}")
                    print(f"   Missing fingers: {[f for f in ['thumb','index','middle','ring','pinky'] if f not in vp2.finger_positions]}")
                    print("   Check that all LEDs are working and visible to camera")
                    break
                await asyncio.sleep(0.2)
            
            if len(vp2.finger_positions) == 5:
                print(f"✅ All 5 fingers detected: {list(vp2.finger_positions.keys())}")
            else:
                print("\n⚠️  Continuing with partial finger detection - calibration may be incomplete")
            
            try:
                # Run open-hand calibration
                await recognizer.calibrate_open_hand(vp2, samples=50)
            except (KeyboardInterrupt, asyncio.CancelledError):
                print("\n⚠️ Open-hand calibration skipped")
            finally:
                # Stop vision
                vision_task.cancel()
                try:
                    await vision_task
                except asyncio.CancelledError:
                    pass
                
        except KeyboardInterrupt:
            print("\n⚠️ Calibration interrupted by user")
        finally:
            print("🛑 Stopping calibration...")
            try:
                await client.stop_streaming()
                await client.disconnect()
            except:
                pass

        print("✅ Calibration complete.")
        return
    
    # Position tracking for logging (always active)
    position_tracker = {
        'packet_counts': defaultdict(int),
        'position': {},
        'start_position': {},
        'last_position': {},
        'last_orientation': {},
        'update_interval': args.update,
        'warmup': defaultdict(int),
        'calibration_data': defaultdict(lambda: {
            'accel_samples': [],
            'gyro_samples': []
        }),
        # ZUPT stationary tracking for letter recognition
        'stationary_states': {},  # {channel: is_stationary}
        'stationary_confidence': {},  # {channel: confidence}
        # IMU-based finger positions (corrected by PID)
        'imu_finger_positions': {}  # {finger_name: (x, y, z)}
    }   
    
    def log_position_data(state):
        """Log position data for any IMU channel."""
        channel = state['channel']
        position_tracker['packet_counts'][channel] += 1
        
        if position_tracker['packet_counts'][channel] % (4*position_tracker['update_interval']) == 0:
            if 'corrected_position' in state:
                x, y, z = state['corrected_position']
                logger.info(f"IMU{channel} Corrected: ({x:+.3f}, {y:+.3f}, {z:+.3f})m")
            elif 'position' in state:
                x, y, z = state['position']
                logger.info(f"IMU{channel} Raw: ({x:+.3f}, {y:+.3f}, {z:+.3f})m")
    
    # Check for playback mode
    if args.playback:
        logger.info(f"📼 Playback mode: {args.playback}")
        client = PlaybackClient(
            args.playback,
            realtime=not args.fast,
            speed=args.speed
        )
    else:
        client = BLEClient()
    
    # Setup Components
    parser = PacketParser()
    converter = IMUConverter()
    
    # Initialize Control for position correction
    control = Control(kp=args.pid_kp, ki=args.pid_ki, kd=args.pid_kd)
    control.px_to_m = 1.0 / (args.px_per_mm * 1000)
    logger.info(f"🎛️  PID Control initialized: Kp={args.pid_kp}, Ki={args.pid_ki}, Kd={args.pid_kd}")
    logger.info(f"📏 Pixel-to-meter conversion: {control.px_to_m:.6f} m/px ({args.px_per_mm} px/mm)")
    
    # Per-IMU filters
    madgwick_filters = {}
    dead_reckoning_filters = {}
    
    # Create grapher if plotting
    grapher = None
    if not args.no_plot:
        from imu_processing import IMUGrapher, GraphMode
        
        try:
            graph_mode = parse_graph_mode(args)
        except ValueError as e:
            logger.error(f"Failed to parse graph mode: {e}")
            return
        
        logger.info("📊 Creating plot window...")
        grapher = IMUGrapher(
            mode=graph_mode,
            max_points=args.max_points,
            update_interval=args.update
        )
    else:
        logger.info("📊 No-plot mode: Position logging enabled")
    
    # Recording storage
    recorded_packets = [] if args.record else None
    
    # Connect to BLE Device / Load Recording
    try:
        if args.playback:
            logger.info("📂 Loading recording...")
        else:
            logger.info("📡 Connecting to BLE device...")
            await client.connect()
            
            # Start vision processor BEFORE starting IMU streaming
            vision_task = None
            vp = None
            if args.vision:
                logger.info("📹 Setting up vision processor...")
                vp = VisionProcessor(
                    client.client, 
                    record=args.record, 
                    thresh=THRESH, 
                    min_area=MIN_AREA, 
                    max_area=MAX_AREA, 
                    min_circ=MIN_CIRC, 
                    pixel_per_mm=PIXEL_PER_MM
                )
                
                # Register vision handler with BLE client
                client.set_external_handler(vp.handler)
                logger.info("✅ Vision handler registered with BLE client")
            else:
                logger.info("📹 Vision processing disabled (--no-vision specified)")
            
            # Start streaming (subscribes to IMU and LED if vision enabled)
            await client.start_streaming()
            logger.info("📡 BLE streaming started")
            
            # Start vision task AFTER streaming is set up
            if args.vision and vp is not None:
                vision_task = asyncio.create_task(vp.start())
                logger.info("📹 Vision processing task started")
                
        logger.info("✅ Connected and streaming")
        logger.info(f"📍 Position logging every {args.update} packets per IMU")
        
        if args.playback:
            if args.fast:
                logger.info("⚡ Fast playback mode (no timing)")
            else:
                logger.info(f"⏱️  Real-time playback (speed: {args.speed}x)")
    
    except Exception as e:
        logger.error(f"❌ Failed to connect: {e}")
        return
    
    # Main Processing Loop
    try:
        packet_count = 0
        while True:
            try:
                # Get raw bytes
                raw_bytes = await client.get_packet()
                
                # Parse
                imu_packet = parser.parse(raw_bytes)
                
                # Add CV data if vision is enabled
                if vp:
                    cv_packet = vp.get_packet()
                    imu_packet["cv"] = cv_packet
                    imu_packet["led_index"] = vp.current_led
                    imu_packet["blob_centers"] = vp.last_blob_centers
                    imu_packet["blob_timestamp"] = vp.last_timestamp
                
                channel = imu_packet['channel']
                
                # Convert
                imu_packet = converter.convert(imu_packet)
                
                # Ensure filters exist for this channel (ONLY ONCE!)
                if channel not in madgwick_filters:
                    madgwick_filters[channel] = MadgwickFilter(sample_rate=20, beta=0.1)
                    dead_reckoning_filters[channel] = KalmanDeadReckoning(
                        sample_rate=20,
                        gravity_convention=GRAVITY_CONVENTION,
                        q=DEAD_Q,
                        r=DEAD_R,
                        p=DEAD_P
                    )
                    
                    position_tracker['warmup'][channel] = 0
                    logger.info(f"🎯 Initialized processing for IMU channel {channel}")
                    
                    # Prompt user on first IMU
                    if len(madgwick_filters) == 1:
                        logger.info("")
                        logger.info("="*70)
                        logger.info("🎯 CALIBRATION: Please hold the glove completely still!")
                        logger.info(f"   Collecting data for {MADGWICK_WARMUP_PACKETS} packets...")
                        logger.info("="*70)
                        logger.info("")

                # Process through filters
                imu_packet = madgwick_filters[channel].process(imu_packet)
                position_tracker['warmup'][channel] += 1

                if position_tracker['warmup'][channel] <= MADGWICK_WARMUP_PACKETS:
                    # Collect calibration data during warmup
                    accel = imu_packet['accel']
                    gyro = imu_packet['gyro']
                    
                    position_tracker['calibration_data'][channel]['accel_samples'].append(accel)
                    position_tracker['calibration_data'][channel]['gyro_samples'].append(gyro)
                    
                    if position_tracker['warmup'][channel] % 20 == 0:
                        logger.info(f"⏳ IMU{channel} warming up: {position_tracker['warmup'][channel]}/{MADGWICK_WARMUP_PACKETS}")
                    
                    # Calculate and apply calibration when warmup complete
                    if position_tracker['warmup'][channel] == MADGWICK_WARMUP_PACKETS:
                        logger.info(f"✅ IMU{channel} warmup complete - running calibration...")
                        
                        accel_samples = np.array(position_tracker['calibration_data'][channel]['accel_samples'])
                        gyro_samples = np.array(position_tracker['calibration_data'][channel]['gyro_samples'])
                        
                        # ============================================================
                        # ONE METHOD CALL DOES EVERYTHING!
                        # ============================================================
                        camera_gravity = np.array([0.0, +9.81, 0.0])  # Camera frame: Y=down
                        
                        calib_results = dead_reckoning_filters[channel].calibrate_from_samples(
                            accel_samples=accel_samples,
                            gyro_samples=gyro_samples,
                            madgwick_quaternion=madgwick_filters[channel].get_quaternion(),
                            target_gravity=camera_gravity,
                            safety_margin=SAFETY_MARGIN,
                            logger=logger
                        )
                        
                        logger.info(f"✅ IMU{channel} calibration complete")
                        logger.info("")
                        
                        # Clean up
                        del position_tracker['calibration_data'][channel]
                    
                    continue

                # Process dead reckoning (only after warmup)
                imu_packet = dead_reckoning_filters[channel].process(imu_packet)
                if imu_packet.get('calibrating', False):
                    progress = imu_packet.get('calibration_progress', 0)
                    if progress % 10 == 0:
                        logger.info(f"⏳ IMU{channel} calibrating bias: {progress}/50 samples")
                    continue
                
                # Apply position correction using Control
                control.process(imu_packet)
                
                # Track stationary state from ZUPT for letter recognition
                if 'zupt_info' in imu_packet:
                    zupt_info = imu_packet['zupt_info']
                    position_tracker['stationary_states'][channel] = zupt_info.get('active', False)
                    position_tracker['stationary_confidence'][channel] = zupt_info.get('confidence', 0.0)
                
                # Store corrected IMU positions mapped to finger names
                # Note: Only X and Y are PID-corrected; Z has accumulated drift, so we ignore it
                if 'corrected_position' in imu_packet and channel in control.imu_to_finger:
                    finger_name = control.imu_to_finger[channel]
                    corrected_x, corrected_y, _ = imu_packet['corrected_position']
                    # Store as 2D position (ignore uncorrected Z-axis)
                    position_tracker['imu_finger_positions'][finger_name] = (corrected_x, corrected_y, 0.0)
                
                # Assign blobs to fingers → update finger map
                if vp and vp.last_blob_centers:
                    vp._assign_fingers(vp.last_blob_centers)
                    vp.update_finger_positions(vp.finger_positions)

                # LETTER RECOGNITION - Hybrid approach with unit conversion
                # Calibration is in PIXELS, so convert IMU meters→pixels (multiply by 1000)
                # Priority 1: IMU corrected positions (converted to pixel scale)
                # Priority 2: Vision positions as fallback (already in pixels)
                finger_positions_for_recognition = {}
                
                # Priority 1: Use IMU corrected positions (meters → pixels via 1000x scale)
                if position_tracker['imu_finger_positions']:
                    for finger_name, pos_3d in position_tracker['imu_finger_positions'].items():
                        # Convert meters to pixel scale (inverse of px_to_m=0.001)
                        finger_positions_for_recognition[finger_name] = (
                            pos_3d[0] * 1000,  # x: meters → pixels
                            pos_3d[1] * 1000,  # y: meters → pixels
                            0.0                # z: ignore for now
                        )
                
                # Priority 2: Fallback to vision positions for any missing fingers
                if vp and vp.finger_positions:
                    for finger_name, pos_2d in vp.finger_positions.items():
                        if finger_name not in finger_positions_for_recognition:
                            # Vision positions already in pixels, just add z=0
                            finger_positions_for_recognition[finger_name] = (pos_2d[0], pos_2d[1], 0.0)
                
                # Only attempt recognition if we have finger position data
                if finger_positions_for_recognition:
                    # Determine overall hand stability from all IMUs
                    # Hand is stable only if MAJORITY of IMUs report stationary
                    stationary_states = list(position_tracker['stationary_states'].values())
                    confidence_scores = list(position_tracker['stationary_confidence'].values())
                    
                    if stationary_states:
                        # Count how many IMUs are stationary
                        num_stationary = sum(stationary_states)
                        total_imus = len(stationary_states)
                        
                        # Hand is stable if >50% of IMUs are stationary
                        is_hand_stable = num_stationary > (total_imus / 2)
                        
                        # Average confidence across all IMUs
                        avg_confidence = sum(confidence_scores) / len(confidence_scores) if confidence_scores else 0.0
                    else:
                        # No ZUPT data yet - allow recognition (backward compatible)
                        is_hand_stable = None
                        avg_confidence = 0.0
                    
                    # Count how many fingers are from IMU vs vision
                    imu_fingers = len(position_tracker['imu_finger_positions'])
                    total_fingers = len(finger_positions_for_recognition)
                    
                    # Check if any finger has large PID correction (indicates drift/movement)
                    max_correction = 0.0
                    for ch in control.imu_to_finger.keys():
                        offset = control.get_correction_offset(ch)
                        if offset:
                            correction_magnitude = math.sqrt(offset[0]**2 + offset[1]**2)
                            max_correction = max(max_correction, correction_magnitude)
                    
                    # Skip recognition if corrections are too large (> 2000mm = 2m)
                    correction_stable = max_correction < 2000.0
                    
                    letter = recognizer.recognize(
                        finger_positions_for_recognition,
                        is_stationary=is_hand_stable and correction_stable,
                        stationary_confidence=avg_confidence if correction_stable else 0.0
                    )
                    if letter:
                        if vp:
                            vp.current_letter = letter
                        stability_indicator = "🟢" if is_hand_stable and correction_stable and avg_confidence > 0.8 else "🟡"
                        source_indicator = "📡" if imu_fingers == total_fingers else "🔀"  # 📡=all IMU, 🔀=hybrid
                        correction_indicator = f"⚠️ drift:{max_correction:.0f}mm" if not correction_stable else ""
                        print(f"\n{stability_indicator}{source_indicator} LETTER: {letter} (stability: {avg_confidence:.1%}, IMU: {imu_fingers}/{total_fingers}) {correction_indicator}\n")
                                
                # Update position tracker (use corrected positions if available)
                if channel not in position_tracker['start_position']:
                    if 'corrected_position' in imu_packet:
                        position_tracker['start_position'][channel] = imu_packet['corrected_position']
                    elif 'position' in imu_packet:
                        position_tracker['start_position'][channel] = imu_packet['position']

                if 'corrected_position' in imu_packet:
                    position_tracker['last_position'][channel] = imu_packet['corrected_position']
                elif 'position' in imu_packet:
                    position_tracker['last_position'][channel] = imu_packet['position']

                if 'orientation' in imu_packet:
                    position_tracker['last_orientation'][channel] = imu_packet['orientation']
                
                # Log position data
                log_position_data(imu_packet)
                
                # Update graphs if plotting
                if grapher:
                    grapher.update(
                        unconverted=imu_packet,
                        converted=imu_packet,
                        madgwick=imu_packet,
                        dead_reckoning=imu_packet,
                        corrected=imu_packet
                    )
                
                # Record if enabled
                if recorded_packets is not None:
                    recorded_packets.append(imu_packet)
                
                packet_count += 1
                if packet_count % 500 == 0:
                    logger.debug(f"📦 Total packets processed: {packet_count}")
                
            except StopIteration:
                logger.info("📼 Reached end of recording")
                break
            except ValueError as e:
                logger.warning(f"⚠️ Packet parsing error: {e}")
                continue
            except Exception as e:
                logger.error(f"❌ Processing error: {e}", exc_info=True)
                continue
    
    except KeyboardInterrupt:
        logger.info("🛑 Stopping pipeline (Ctrl+C)...")
    
    finally:
        # Cancel vision task first
        if 'vision_task' in locals() and vision_task:
            logger.info("🛑 Cancelling vision task...")
            vision_task.cancel()
            try:
                await vision_task
            except asyncio.CancelledError:
                pass
            except Exception as e:
                logger.warning(f"Vision task cleanup: {e}")
        
        # Print final summary
        logger.info("")
        logger.info("="*70)
        logger.info("📊 FINAL POSITION SUMMARY")
        logger.info("="*70)
        
        for ch in sorted(position_tracker['packet_counts'].keys()):
            count = position_tracker['packet_counts'][ch]
            logger.info(f"IMU Channel {ch}: {count} packets")
            
            if ch in position_tracker['last_position'] and ch in position_tracker['start_position']:
                x, y, z = position_tracker['last_position'][ch]
                sx, sy, sz = position_tracker['start_position'][ch]
                dx, dy, dz = x-sx, y-sy, z-sz
                total_disp = math.sqrt(dx**2 + dy**2 + dz**2)
                
                logger.info(f"  Start: ({sx:.3f}, {sy:.3f}, {sz:.3f})m")
                logger.info(f"  Final: ({x:.3f}, {y:.3f}, {z:.3f})m")
                logger.info(f"  Total displacement: {total_disp:.3f}m")
            
            if ch in position_tracker['last_orientation']:
                roll, pitch, yaw = position_tracker['last_orientation'][ch]
                logger.info(f"  Final orientation: Roll={roll:.1f}° Pitch={pitch:.1f}° Yaw={yaw:.1f}°")
            
            # Show correction offset
            offset = control.get_correction_offset(ch)
            if offset != (0.0, 0.0):
                logger.info(f"  Final correction offset: ({offset[0]*1000:.1f}, {offset[1]*1000:.1f})mm")
            
            logger.info("")
        
        logger.info("="*70)
        
        # Cleanup BLE
        try:
            logger.info("🛑 Stopping BLE streaming...")
            await client.stop_streaming()
        except Exception as e:
            logger.debug(f"Stop streaming: {e}")
        
        try:
            logger.info("🛑 Disconnecting BLE...")
            await client.disconnect()
        except Exception as e:
            logger.debug(f"Disconnect: {e}")
        
        # Close grapher
        if grapher:
            logger.info("🛑 Closing grapher...")
            grapher.close()
        
        # Save recording if enabled
        if recorded_packets:
            try:
                filename = args.output if args.output else None
                saved_path = Recording.save(recorded_packets, filename)
                logger.info(f"💾 Saved {len(recorded_packets)} packets to: {saved_path}")
            except Exception as e:
                logger.error(f"❌ Failed to save recording: {e}")
        
        logger.info("✅ Pipeline stopped cleanly")

if __name__ == "__main__":
    
    mp.set_start_method('spawn', force=True)  # Cross-platform compatibility
    
    # BLE MODE: Platform-specific defaults
    if platform.system() == 'Windows':
        if args.no_plot is None:
            args.no_plot = True  # No plot on Windows BLE
        if args.record is None:
            args.record = True  # Auto-record on Windows BLE
    else:
        # Linux/Mac BLE
        if args.no_plot is None:
            args.no_plot = False  # Plot on Linux/Mac
        if args.record is None:
            args.record = False  # Don't auto-record on Linux/Mac

    # Handle override flags
    if args.force_plot:
        args.no_plot = False
    if args.no_record:
        args.record = False
    
    # Setup logging
    log_level = logging.ERROR if args.quiet else (logging.DEBUG if args.debug else logging.INFO)
    logging.basicConfig(
        level=log_level,
        format="%(asctime)-15s %(levelname)s: %(message)s"
    )
    
    # Vision defaults
    if args.no_vision:
        args.vision = False
    else:
        args.vision = True  # Default to enabled
    
    # Smart defaults
    if args.playback:
        if args.no_plot is None:
            args.no_plot = False
        if args.record is None:
            args.record = False
    else:
        if platform.system() == 'Windows':
            if args.no_plot is None:
                args.no_plot = True
            if args.record is None:
                args.record = True
        else:
            if args.no_plot is None:
                args.no_plot = False
            if args.record is None:
                args.record = False
    
    if args.force_plot:
        args.no_plot = False
    if args.no_record:
        args.record = False
    
    # Inform user
    logger.info(f"🎛️  PID enabled: Kp={args.pid_kp}, Ki={args.pid_ki}, Kd={args.pid_kd}, px_to_m={args.px_per_mm}")
    logger.info(f"📹 Vision: {'Enabled' if args.vision else 'Disabled (--no-vision)'}")
    
    if args.playback:
        logger.info(f"📼 Playback mode: {args.playback}")
    else:
        if platform.system() == 'Windows':
            logger.info("🪟 Windows BLE mode:")
            if args.no_plot:
                logger.info("  • No plotting (use --force-plot to override)")
            else:
                logger.warning("  • Plotting enabled (may cause BLE issues!)")
    
    # Run main
    asyncio.run(main(args))