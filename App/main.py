# main.py
# pylint: disable=E1101
# mypy: ignore-errors

import asyncio
import argparse
import logging
import platform
import math
from collections import defaultdict

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

logger = logging.getLogger(__name__)
MADGWICK_WARMUP_PACKETS = 100

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
    
    # Position tracking for logging
    position_tracker = {
        'packet_counts': defaultdict(int),
        'position': {},
        'start_position': {},
        'last_position': {},
        'last_orientation': {},
        'update_interval': args.update,
        'warmup': defaultdict(int)
    }   
    
    def log_position_data(state):
        """Log position data for any IMU channel."""
        channel = state['channel']
        position_tracker['packet_counts'][channel] += 1
        
        if position_tracker['packet_counts'][channel] % position_tracker['update_interval'] == 0:
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
    control.px_to_m = args.px_to_m
    logger.info(f"🎛️  PID Control initialized: Kp={args.pid_kp}, Ki={args.pid_ki}, Kd={args.pid_kd}")
    logger.info(f"📏 Pixel-to-meter conversion: {args.px_to_m} m/px")
    
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
        await client.start_streaming()
        
        # Start vision processor (default enabled)
        vision_task = None
        vp = None
        if args.vision:
            vp = VisionProcessor(client.client, record=args.record)
            client.set_external_handler(vp.handler)
            vision_task = asyncio.create_task(vp.start())
            logger.info("📹 Vision processing enabled")
        else:
            logger.info("📹 Vision processing disabled (use --no-vision was specified)")
                
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
                
                channel = imu_packet['channel']
                
                # Convert
                imu_packet = converter.convert(imu_packet)
                
                # Ensure filters exist for this channel
                if channel not in madgwick_filters:
                    madgwick_filters[channel] = MadgwickFilter(sample_rate=20, beta=0.1)
                    dead_reckoning_filters[channel] = KalmanDeadReckoning(sample_rate=20, gravity_convention='NED')
                    position_tracker['warmup'][channel] = 0
                    logger.info(f"🎯 Initialized processing for IMU channel {channel}")
                                
                # Process through filters
                imu_packet = madgwick_filters[channel].process(imu_packet)
                position_tracker['warmup'][channel] += 1
                if position_tracker['warmup'][channel] <= MADGWICK_WARMUP_PACKETS:
                    if position_tracker['warmup'][channel] % 20 == 0:
                        logger.info(f"⏳ IMU{channel} warming up: {position_tracker['warmup'][channel]}/{MADGWICK_WARMUP_PACKETS}")
                    continue

                # Process dead reckoning (only after warmup)
                imu_packet = dead_reckoning_filters[channel].process(imu_packet)
                if imu_packet.get('calibrating', False):
                    progress = imu_packet.get('calibration_progress', 0)
                    if progress % 10 == 0:
                        logger.info(f"⏳ IMU{channel} calibrating bias: {progress}/{50} samples")
                    continue
                
                # Apply position correction using Control
                control.process(imu_packet)
                
                # Update position tracker
                if channel not in position_tracker['start_position']:
                    if 'position' in imu_packet:
                        position_tracker['start_position'][channel] = imu_packet['position']

                if 'position' in imu_packet:
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
            
            # Show correction offset
            offset = control.get_correction_offset(ch)
            if offset != (0.0, 0.0):
                logger.info(f"  Final correction offset: ({offset[0]*1000:.1f}, {offset[1]*1000:.1f})mm")
            
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
        description="ASL Glove IMU Data Pipeline with Position Correction",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    # Mode Selection
    parser.add_argument(
        '--mode', '-m',
        type=str,
        default='corrected',  # Default to showing corrected position
        choices=['all', 'debug', 'sensor', 'madgwick', 'position', 'corrected', 'compare', 'graph']
    )
    
    parser.add_argument('graph_options', nargs='*')
    
    # PID parameters
    parser.add_argument('--pid-kp', type=float, default=0.5, help='PID proportional gain (default: 0.5)')
    parser.add_argument('--pid-ki', type=float, default=0.1, help='PID integral gain (default: 0.1)')
    parser.add_argument('--pid-kd', type=float, default=0.2, help='PID derivative gain (default: 0.2)')
    parser.add_argument('--px-to-m', type=float, default=0.001, help='Pixel to meter conversion (default: 0.001)')
    
    # Plot options
    parser.add_argument('--no-plot', action='store_true', default=None)
    parser.add_argument('--force-plot', action='store_true')
    
    # Recording
    parser.add_argument('--record', '-r', action='store_true', default=None)
    parser.add_argument('--no-record', action='store_true')
    
    # Vision (default enabled, can disable)
    parser.add_argument('--no-vision', action='store_true', help='Disable vision processing (default: enabled)')
    parser.add_argument('--vision', action='store_true', help='Enable vision processing (default: enabled)')  # For backwards compatibility
    
    # Other options
    parser.add_argument('--output', '-o', type=str)
    parser.add_argument('--update', '-u', type=int, default=20)
    parser.add_argument('--max-points', type=int, default=200)
    parser.add_argument('--playback', '-p', type=str)
    parser.add_argument('--speed', '-s', type=float, default=1.0)
    parser.add_argument('--fast', '-f', action='store_true')
    parser.add_argument('--debug', '-d', action='store_true')
    parser.add_argument('--quiet', '-q', action='store_true')
    
    args = parser.parse_args()
    
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
    
    # Setup logging
    log_level = logging.ERROR if args.quiet else (logging.DEBUG if args.debug else logging.INFO)
    logging.basicConfig(
        level=log_level,
        format="%(asctime)-15s %(levelname)s: %(message)s"
    )
    
    # Inform user
    logger.info(f"🎛️  PID enabled: Kp={args.pid_kp}, Ki={args.pid_ki}, Kd={args.pid_kd}, px_to_m={args.px_to_m}")
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
    
    asyncio.run(main(args))
