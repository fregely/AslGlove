#!/usr/bin/env python
"""
Windows launcher with proper COM initialization
"""
import sys
import os

# ABSOLUTELY FIRST - Set COM mode
sys.coinit_flags = 0

# Prevent any automatic COM initialization
os.environ['BLEAK_WINRT_USE_STA'] = '1'

# Now we can safely import
import asyncio

# Force selector event loop
asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

# NOW import and run your main
if __name__ == "__main__":
    # Import main AFTER fixing COM
    from main import main
    import argparse
    
    # Your argument parsing
    parser = argparse.ArgumentParser(description="ASL Glove IMU Data Pipeline")
    parser.add_argument('--mode', '-m', type=str, default='position')
    parser.add_argument('graph_options', nargs='*')
    parser.add_argument('--update', '-u', type=int, default=5)
    parser.add_argument('--max-points', type=int, default=200)
    parser.add_argument('--record', '-r', action='store_true')
    parser.add_argument('--output', '-o', type=str)
    parser.add_argument('--playback', '-p', type=str)
    parser.add_argument('--speed', '-s', type=float, default=1.0)
    parser.add_argument('--fast', '-f', action='store_true')
    parser.add_argument('--debug', '-d', action='store_true')
    parser.add_argument('--quiet', '-q', action='store_true')
    
    args = parser.parse_args()
    
    # Setup logging
    import logging
    log_level = logging.ERROR if args.quiet else (logging.DEBUG if args.debug else logging.INFO)
    logging.basicConfig(level=log_level, format="%(asctime)-15s %(levelname)s: %(message)s")
    
    # Run
    asyncio.run(main(args))
