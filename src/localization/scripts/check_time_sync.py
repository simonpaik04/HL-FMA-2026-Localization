#!/usr/bin/python3
"""Read-only PC clock preflight. May exec a driver only after a successful check."""
import argparse
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from timing_core import check_host_clock, load_config


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--gps-config')
    parser.add_argument('--use-sim-time', action='store_true', help='Explicit replay exemption; physical sync stays unverified')
    parser.add_argument('--exec', dest='command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
        result = check_host_clock(config['clock_preflight'], args.use_sim_time)
        if args.gps_config:
            gps = load_config(args.gps_config)
            result['configured_gps_stamp_source'] = ('ros_receipt_explicit' if gps.get('use_ros_time', False) else 'gnss_utc')
            result['gps_config_is_not_live_device_proof'] = True
    except (KeyError, ValueError, OSError) as error:
        print(json.dumps({'ready': False, 'status': 'INVALID_CLOCK_CONFIG', 'reason': str(error)}), flush=True)
        return 2
    print(json.dumps(result, sort_keys=True, ensure_ascii=False, allow_nan=False), flush=True)
    if not result['ready']:
        return 1
    if args.command:
        os.execvp(args.command[0], args.command)
    return 0


if __name__ == '__main__':
    sys.exit(main())
