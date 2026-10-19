#!/usr/bin/env python3
"""Scatter live simulated blocks, or restore their poses before the first scatter."""
import argparse
import json
import os
import sys
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('scatter', 'reset'))
    parser.add_argument('--url', default='http://localhost:'+os.environ.get('HTTP_PORT', '23517'),
                        help='Simulator URL (default: localhost:$HTTP_PORT or port 23517)')
    parser.add_argument('--seed', type=int, help='Reproduce a random layout')
    parser.add_argument('--table', default='all', choices=('all', '0_0', '0_1', '1_0', '1_1'),
                        help='Table row_column; 0_1 is the original lower-right table')
    args = parser.parse_args()
    if args.action == 'reset' and (args.seed is not None or args.table != 'all'):
        parser.error('--seed and --table apply only to scatter')
    if args.seed is not None and args.seed < 0:
        parser.error('--seed must be nonnegative')
    request = Request(args.url.rstrip('/')+'/simulation/blocks/'+args.action,
        data=json.dumps({'seed': args.seed, 'table': args.table}).encode(),
        headers={'Content-Type': 'application/json'}, method='POST')
    try:
        with urlopen(request, timeout=30) as response:
            result = json.load(response)
    except HTTPError as error:
        print(f'Simulator rejected {args.action}: {error.read().decode()}', file=sys.stderr)
        return 1
    except (URLError, TimeoutError, OSError) as error:
        print(f'Could not reach {args.url}: {error}. Start apps/junh/run_sim.sh first.', file=sys.stderr)
        return 1
    print(result['message'])
    print(f"Scan restarted (restart_id={result['restart_id']}).")
    for index, pose in enumerate(result['blocks'], 1):
        print(f'block_{index}: position={pose[:3]} quaternion(wxyz)={pose[3:]}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
