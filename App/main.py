# main.py
# pylint: disable=E1101
# mypy: ignore-errors

# ------------------------------
# VISION PROCESSING (OpenCV)
# ------------------------------
import asyncio
import argparse
import logging
import platform
import math
from collections import defaultdict

# for cv functionality
from computer_vision.cv import VisionProcessor

# Import base modules
from imu_processing import (
    IMUConverter, 
    MadgwickFilter, 
    DeadReckoning,
    BLEClient,
    PacketParser,
    Recording,
    PlaybackClient
)

logger = logging.getLogger(__name__)

def parse_graph_mode(args):
    """Parse command line arguments to determine GraphMode."""
    # Only import GraphMode if we're actually plotting
    if not args.no_plot:
        from imu_processing import GraphMode
        
        mode = args.mode
        
        if mode == 'all':
            return GraphMode.ALL
        elif mode == 'debug':
            return GraphMode.CONVERTED | GraphMode.MADGWICK | GraphMode.DEAD_RECKONING
        elif mode == 'sensor':
            return GraphMode.UNCONVERTED | GraphMode.CONVERTED
        elif mode == 'madgwick':
            return GraphMode.MADGWICK
        elif mode == 'position':
            return GraphMode.DEAD_RECKONING
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

        # Create BLE client here
        client = BLEClient()

        # Connect to glove
        await client.connect()
        await client.start_streaming()

        # Start CV processor (camera + blob detection)
        vp = VisionProcessor(client.client, record=False)
        client.set_external_handler(vp.handler)
        vision_task = asyncio.create_task(vp.start())

        # NEW: IMU-based Calibrator ONLY needs vp
        calibrator = Calibrator(vp, known_leds=list(range(5)))

        try:
            await calibrator.run()
        finally:
            print("🛑 Stopping calibration...")
            vision_task.cancel()
            await client.stop_streaming()
            await client.disconnect()

        print("✅ Calibration complete.")
        return
    
    # Position tracking for logging (always active)
    position_tracker = {
        'packet_counts': defaultdict(int),
        'position': {},             # not used but okay
        'start_position': {},       # first reading per IMU channel
        'last_position': {},        # most recent IMU position
        'last_orientation': {},     # most recent IMU roll/pitch/yaw
        'update_interval': args.update
    }   
    
    def log_position_data(state):
        """Log position data for any IMU channel."""
        channel = state['channel']
        position_tracker['packet_counts'][channel] += 1
        
        # Log every update_interval packets per channel
        if position_tracker['packet_counts'][channel] % position_tracker['update_interval'] == 0:
            if 'position' in state:
                x, y, z = state['position']
                logger.info(f"IMU{channel}: ({x:+.3f}, {y:+.3f}, {z:+.3f})m")
    
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
    
    # Per-IMU filters
    madgwick_filters = {}
    dead_reckoning_filters = {}
    
    # Create grapher if plotting
    grapher = None
    if not args.no_plot:
        # Only import IMUGrapher if we're actually using it
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
        await client.start_streaming()
        
        # Start vision processor if requested
        vision_task = None
        if getattr(args, "vision", False):
            vp = VisionProcessor(client.client, record=args.record)
            client.set_external_handler(vp.handler)
            vision_task = asyncio.create_task(vp.start())
                
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
                cv_packet = vp.get_packet()
                imu_packet["cv"] = cv_packet
                channel = imu_packet['channel']
                
                # Convert
                imu_packet = converter.convert(imu_packet)
                
                # Ensure filters exist for this channel
                if channel not in madgwick_filters:
                    madgwick_filters[channel] = MadgwickFilter(sample_rate=20, beta=0.5)
                    dead_reckoning_filters[channel] = DeadReckoning(sample_rate=20)
                    logger.info(f"🎯 Initialized processing for IMU channel {channel}")
                
                # Process through filters
                imu_packet = madgwick_filters[channel].process(imu_packet)
                imu_packet = dead_reckoning_filters[channel].process(imu_packet)
                
                # ------------------------------
                # ADD CV / BLOB DATA
                # ------------------------------
                if args.vision:
                    imu_packet["led_index"] = vp.current_led
                    imu_packet["blob_centers"] = vp.last_blob_centers
                    imu_packet["blob_timestamp"] = vp.last_timestamp
                
                # ------------------------------
                # UPDATE POSITION TRACKER
                # ------------------------------
                if channel not in position_tracker['start_position']:
                    if 'position' in imu_packet:
                        position_tracker['start_position'][channel] = imu_packet['position']

                if 'position' in imu_packet:
                    position_tracker['last_position'][channel] = imu_packet['position']

                if 'orientation' in imu_packet:
                    position_tracker['last_orientation'][channel] = imu_packet['orientation']
                # ------------------------------
                
                # ALWAYS log position data (regardless of plotting)
                log_position_data(imu_packet)
                
                # Update graphs if plotting
                if grapher:
                    grapher.update(
                        unconverted=imu_packet,
                        converted=imu_packet,
                        madgwick=imu_packet,
                        dead_reckoning=imu_packet
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
        # Print final summary
        logger.info("")
        logger.info("="*70)
        logger.info("📊 FINAL POSITION SUMMARY")
        logger.info("="*70)
        if vision_task:
            vision_task.cancel()
        
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
            
            logger.info("")
        
        logger.info("="*70)
        
        # Cleanup
        try:
            await client.stop_streaming()
            await client.disconnect()
        except:
            pass
        
        if grapher:
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
    parser = argparse.ArgumentParser(
        description="ASL Glove IMU Data Pipeline",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    # Mode Selection
    parser.add_argument(
        '--mode', '-m',
        type=str,
        default='position',
        choices=['all', 'debug', 'sensor', 'madgwick', 'position', 'graph']
    )
    
    parser.add_argument('graph_options', nargs='*')
    
    # No-plot option (default True on Windows ONLY for BLE mode)
    parser.add_argument(
        '--no-plot',
        action='store_true',
        default=None,  # Will be set based on platform AND mode
        help='Disable plotting (default on Windows for BLE only)'
    )
    
    # Force plot on Windows
    parser.add_argument(
        '--force-plot',
        action='store_true',
        help='Force plotting even on Windows with BLE'
    )
    
    # Recording - default on Windows for BLE
    parser.add_argument(
        '--record', '-r',
        action='store_true',
        default=None,  # Will be set based on platform AND mode
        help='Record all packets (default on Windows for BLE) & Record vision blob data to CSV',
    )
    
    # Option to disable recording
    parser.add_argument(
        '--no-record',
        action='store_true',
        help='Disable recording'
    )
    
    parser.add_argument('--vision', action='store_true',
                    help='Enable OpenCV blob detection synchronized with LED flashing')
    
    parser.add_argument("--calibrate", action="store_true",
                    help="Run LED → finger calibration and exit")
    
    parser.add_argument('--output', '-o', type=str)
    parser.add_argument('--update', '-u', type=int, default=5)
    parser.add_argument('--max-points', type=int, default=200)
    parser.add_argument('--playback', '-p', type=str)
    parser.add_argument('--speed', '-s', type=float, default=1.0)
    parser.add_argument('--fast', '-f', action='store_true')
    parser.add_argument('--debug', '-d', action='store_true')
    parser.add_argument('--quiet', '-q', action='store_true')
    
    args = parser.parse_args()
    
    # Smart defaults based on mode and platform
    if args.playback:
        # PLAYBACK MODE: Always enable plotting, never record
        if args.no_plot is None:
            args.no_plot = False  # Always plot in playback mode
        if args.record is None:
            args.record = False  # Don't record playback by default
    else:
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
    
    # Inform user about settings
    if args.playback:
        logger.info(f"📼 Playback mode: {args.playback}")
        if not args.no_plot:
            logger.info("  • Plotting enabled (default for playback)")
    else:
        # BLE mode
        if platform.system() == 'Windows':
            logger.info("🪟 Windows BLE mode:")
            if args.no_plot:
                logger.info("  • No plotting (default, use --force-plot to override)")
            else:
                logger.warning("  • Plotting enabled (may cause BLE issues!)")
            
            if args.record:
                logger.info("  • Recording enabled (default)")
                if args.output:
                    logger.info(f"  • Will save to: {args.output}")
                else:
                    logger.info("  • Will save to: recording_[timestamp].pkl")
    
    # Run main
    asyncio.run(main(args))