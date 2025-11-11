# main.py
import asyncio
import argparse
import logging
from imu_processing.ble_client import BLEClient
from imu_processing.packet_parser import PacketParser
from imu_processing.converter import IMUConverter
from imu_processing.madgwick import MadgwickFilter
from imu_processing.dead_reckoning import DeadReckoning
from imu_processing.imu_grapher import IMUGrapher, GraphMode
from imu_processing.recording import Recording, PlaybackClient

logger = logging.getLogger(__name__)


def parse_graph_mode(args):
    """
    Parse command line arguments to determine GraphMode.
    
    Returns appropriate GraphMode based on --mode argument.
    """
    mode = args.mode
    
    # Preset modes
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
        # Custom mode - parse graph_options
        if not args.graph_options:
            logger.error("--mode graph requires additional arguments")
            logger.error("Example: --mode graph converted madgwick")
            raise ValueError("No graph options provided for 'graph' mode")
        
        result = GraphMode(0)  # Start with no flags
        
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


async def main(args):
    """Main data processing pipeline."""
    
    # ========================================
    # Check for playback mode
    # ========================================
    if args.playback:
        logger.info(f"📼 Playback mode: {args.playback}")
        client = PlaybackClient(
            args.playback,
            realtime=not args.fast,
            speed=args.speed
        )
    else:
        client = BLEClient()
    
    # ========================================
    # Setup Components
    # ========================================
    parser = PacketParser()
    converter = IMUConverter()
    
    # Per-IMU filters (created dynamically as channels appear)
    madgwick_filters = {}
    dead_reckoning_filters = {}
    
    # Parse graph mode from arguments
    try:
        graph_mode = parse_graph_mode(args)
    except ValueError as e:
        logger.error(f"Failed to parse graph mode: {e}")
        return
    
    # Create grapher with parsed mode
    grapher = IMUGrapher(
        mode=graph_mode,
        max_points=args.max_points,
        update_interval=args.update
    )
    
    # Recording storage
    recorded_packets = [] if args.record else None
    
    # ========================================
    # Connect to BLE Device / Load Recording
    # ========================================
    try:
        if args.playback:
            logger.info("📂 Loading recording...")
        else:
            logger.info("📡 Connecting to BLE device...")
        
        await client.connect()
        await client.start_streaming()
        
        logger.info("✅ Connected and streaming")
        logger.info(f"📊 Graph mode: {graph_mode}")
        logger.info(f"🔄 Update interval: every {args.update} packets")
        
        if args.playback:
            if args.fast:
                logger.info("⚡ Fast playback mode (no timing)")
            else:
                logger.info(f"⏱️  Real-time playback (speed: {args.speed}x)")
    
    except Exception as e:
        logger.error(f"❌ Failed to connect: {e}")
        return
    
    # ========================================
    # Main Processing Loop
    # ========================================
    try:
        packet_count = 0
        while True:
            try:
                # 1. Get raw bytes
                raw_bytes = await client.get_packet()
                
                # 2. Parse → state has channel, timestamp, accel_raw, gyro_raw, mag_raw
                state = parser.parse(raw_bytes)
                
                channel = state['channel']
                
                # 3. Convert → state gains accel, gyro, mag (keeps *_raw)
                state = converter.convert(state)
                
                # 4. Ensure filters exist for this IMU channel
                if channel not in madgwick_filters:
                    madgwick_filters[channel] = MadgwickFilter(sample_rate=20, beta=0.5)
                    dead_reckoning_filters[channel] = DeadReckoning(sample_rate=20)
                    logger.info(f"📍 Initialized processing for IMU channel {channel}")
                
                # 5. Madgwick → state gains quaternion, euler
                state = madgwick_filters[channel].process(state)
                
                # 6. Dead reckoning → state gains position, velocity, linear_accel
                state = dead_reckoning_filters[channel].process(state)
                
                # 7. Update graphs
                grapher.update(
                    unconverted=state,
                    converted=state,
                    madgwick=state,
                    dead_reckoning=state
                )
                
                # 8. Record complete state
                if recorded_packets is not None:
                    recorded_packets.append(state)
                
                # 9. Log progress
                packet_count += 1
                if packet_count % 100 == 0:
                    logger.debug(f"📦 Processed {packet_count} packets")
                
            except StopIteration:
                # End of playback
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
        # ========================================
        # Cleanup and Save
        # ========================================
        try:
            await client.stop_streaming()
            await client.disconnect()
        except:
            pass
        
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
    # ========================================
    # Argument Parser
    # ========================================
    parser = argparse.ArgumentParser(
        description="ASL Glove IMU Data Pipeline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Mode Examples:
  # Default - just position tracking
  python main.py
  
  # All graphs (12 graphs total!)
  python main.py --mode all
  
  # Debug mode (converted sensors + orientation + position)
  python main.py --mode debug
  
  # Just orientation
  python main.py --mode madgwick
  
  # Sensor comparison (raw + converted)
  python main.py --mode sensor
  
  # Custom selection
  python main.py --mode graph converted madgwick
  python main.py --mode graph unconverted converted madgwick dead_reckoning

Recording & Playback:
  # Record a session
  python main.py --record
  
  # Record with custom filename
  python main.py --record --output my_recording.pkl
  
  # Play back a recording (real-time)
  python main.py --playback recording_20241111_120000.pkl
  
  # Play back at 2x speed
  python main.py --playback recording.pkl --speed 2.0
  
  # Play back as fast as possible (no timing)
  python main.py --playback recording.pkl --fast
  
  # Record while analyzing a playback
  python main.py --playback old.pkl --record --output analyzed.pkl
  
Update Interval:
  # Update plot every 10 packets (better performance)
  python main.py --update 10
  
  # Update plot every packet (real-time but slower)
  python main.py --update 1
  
Debug Logging:
  # Enable verbose logging
  python main.py --debug
        """
    )
    
    # ========================================
    # Mode Selection
    # ========================================
    parser.add_argument(
        '--mode', '-m',
        type=str,
        default='position',
        choices=['all', 'debug', 'sensor', 'madgwick', 'position', 'graph'],
        help="""Graph mode preset:
  all      - All 12 graphs (raw, converted, orientation, position)
  debug    - Debug mode: converted + madgwick + position (9 graphs)
  sensor   - Sensor comparison: raw + converted (6 graphs)
  madgwick - Just orientation: roll, pitch, yaw (3 graphs)
  position - Just position: X, Y, Z (3 graphs) [DEFAULT]
  graph    - Custom selection (requires additional arguments)"""
    )
    
    parser.add_argument(
        'graph_options',
        nargs='*',
        help="""Graph options for '--mode graph':
  unconverted, converted, madgwick, dead_reckoning (or position/dead)
  Example: --mode graph converted madgwick"""
    )
    
    # ========================================
    # Graph Configuration
    # ========================================
    parser.add_argument(
        '--update', '-u',
        type=int,
        default=5,
        metavar='N',
        help='Update plot every N packets (default: 5, lower = smoother but slower)'
    )
    
    parser.add_argument(
        '--max-points',
        type=int,
        default=200,
        metavar='N',
        help='Maximum points in scrolling window (default: 200)'
    )
    
    # ========================================
    # Recording
    # ========================================
    parser.add_argument(
        '--record', '-r',
        action='store_true',
        help='Record all packets (saves to file on exit)'
    )
    
    parser.add_argument(
        '--output', '-o',
        type=str,
        metavar='FILE',
        help='Output filename for recording (default: recording_TIMESTAMP.pkl)'
    )
    
    # ========================================
    # Playback
    # ========================================
    parser.add_argument(
        '--playback', '-p',
        type=str,
        metavar='FILE',
        help='Play back recorded session from file (.pkl or .json)'
    )
    
    parser.add_argument(
        '--speed', '-s',
        type=float,
        default=1.0,
        metavar='X',
        help='Playback speed multiplier (2.0 = 2x, 0.5 = half speed)'
    )
    
    parser.add_argument(
        '--fast', '-f',
        action='store_true',
        help='Fast playback mode (ignore original timing, as fast as possible)'
    )
    
    # ========================================
    # Logging
    # ========================================
    parser.add_argument(
        '--debug', '-d',
        action='store_true',
        help='Enable debug logging'
    )
    
    parser.add_argument(
        '--quiet', '-q',
        action='store_true',
        help='Minimal logging (errors only)'
    )
    
    # ========================================
    # Parse and Run
    # ========================================
    args = parser.parse_args()
    
    # Setup logging
    if args.quiet:
        log_level = logging.ERROR
    elif args.debug:
        log_level = logging.DEBUG
    else:
        log_level = logging.INFO
    
    logging.basicConfig(
        level=log_level,
        format="%(asctime)-15s %(levelname)s: %(message)s"
    )
    
    # Run main
    asyncio.run(main(args))
