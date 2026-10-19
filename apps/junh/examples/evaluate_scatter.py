#!/usr/bin/env python3
"""Score a finished MuJoCo discovery snapshot against simulator block truth.

This evaluator is separate from discovery: ground-truth poses never enter the
detector, workspace, or search controller.
"""
import argparse
import json
import math
from pathlib import Path
import sys
from urllib.request import urlopen

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'sim/piper_mujoco_sim'))
from piper_mujoco_sim.scatter import BlockScatter  # noqa: E402
from piper_mujoco_sim.scene import build_scene, reset_scene  # noqa: E402


def load_snapshot(path, url):
    if path:
        lines = path.read_text().splitlines()
        raw = json.loads(lines[-1])
    else:
        with urlopen(url.rstrip('/') + '/api/workspace', timeout=5) as response:
            raw = json.load(response)
    if 'snapshot' in raw:
        raw = raw['snapshot']
    if not raw or 'workspace' not in raw or 'search' not in raw:
        raise ValueError('Expected a discovery snapshot or simulator workspace API response.')
    return raw


def yaw(quaternion):
    w, x, y, z = quaternion
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def symmetric_yaw_error(a, b):
    difference = yaw(a)-yaw(b)
    return abs(math.atan2(math.sin(2*difference), math.cos(2*difference)))/2


def evaluate(snapshot, poses, config):
    workspace = snapshot['workspace']
    candidates = workspace['tracks'] + workspace['hypotheses']
    pairings = sorted((float(np.linalg.norm(np.asarray(pose[:2])-candidate['position'][:2])), i, j)
                      for i, pose in enumerate(poses)
                      for j, candidate in enumerate(candidates))
    matched_truth, matched_candidates = {}, set()
    for distance, i, j in pairings:
        if distance > .05 or i in matched_truth or j in matched_candidates:
            continue
        matched_truth[i] = j
        matched_candidates.add(j)
    blocks = []
    for i, pose in enumerate(poses):
        result = {'truth': f'block_{i+1}', 'truth_position': [float(v) for v in pose[:3]]}
        if i in matched_truth:
            candidate = candidates[matched_truth[i]]
            confirmed = candidate['id'] is not None
            result.update(match=candidate['id'] if confirmed else candidate['hypothesis_id'],
                          confirmed=confirmed,
                          position_error_mm=round(1000*float(np.linalg.norm(
                              np.asarray(pose[:3])-candidate['position'])), 2),
                          resolved_views=len(candidate.get('supporting_views', [])))
            if confirmed and candidate['resolved']:
                result['yaw_error_deg'] = round(math.degrees(
                    symmetric_yaw_error(pose[3:], candidate['quaternion'])), 2)
        else:
            result.update(match=None, confirmed=False)
        blocks.append(result)

    coverage = workspace['coverage']
    cells = {tuple(cell['index']): cell for cell in coverage['cells']}
    width, depth = config['table_size']
    gap, cell_size = config['table_gap'], coverage['cell_size']
    tables = {}
    for row in range(2):
        for column in range(2):
            x = (column-.5)*(width+gap)
            y = (row-.5)*(depth+gap)
            counts = {'cells': 0, 'observed': 0, 'explored': 0}
            for ix in range(math.floor((x-width/2)/cell_size),
                            math.ceil((x+width/2)/cell_size)):
                for iy in range(math.floor((y-depth/2)/cell_size),
                                math.ceil((y+depth/2)/cell_size)):
                    cx, cy = (ix+.5)*cell_size, (iy+.5)*cell_size
                    if abs(cx-x) > width/2 or abs(cy-y) > depth/2:
                        continue
                    counts['cells'] += 1
                    entry = cells.get((ix, iy), {})
                    state = entry.get('state')
                    if state == 'observed':
                        counts['observed'] += 1
                        counts['explored'] += 1
                    elif state == 'uncertain' and entry.get('explored', True):
                        counts['explored'] += 1
            tables[f'{row}_{column}'] = {
                'observed_fraction': round(counts['observed']/counts['cells'], 4),
                'explored_fraction': round(counts['explored']/counts['cells'], 4)}
    return {'captures': snapshot['search']['accepted'],
            'result': snapshot['search']['result'],
            'confirmed': len(workspace['tracks']),
            'hypotheses': len(workspace['hypotheses']),
            'false_candidates': [c.get('id') or c.get('hypothesis_id')
                                 for j, c in enumerate(candidates) if j not in matched_candidates],
            'blocks': blocks, 'tables': tables,
            'scope_coverage': {key: coverage[key]
                               for key in ('observed_fraction', 'explored_fraction')}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--seed', type=int, help='Seed used by scatter_blocks.py')
    source.add_argument('--aligned', action='store_true', help='Evaluate the reset layout')
    parser.add_argument('--table', default='all', choices=('all', '0_0', '0_1', '1_0', '1_1'))
    parser.add_argument('--snapshot', type=Path, help='Saved JSON/JSONL discovery snapshot')
    parser.add_argument('--url', default='http://localhost:23517', help='Running simulator URL')
    args = parser.parse_args()
    if args.aligned and args.table != 'all':
        parser.error('--table applies only to --seed')
    if args.seed is not None and args.seed < 0:
        parser.error('--seed must be nonnegative')
    scene_path = ROOT/'sim/piper_mujoco_sim/config/lab_scene.json'
    config = json.loads(scene_path.read_text())
    model = mujoco.MjModel.from_xml_string(build_scene(
        Path('/opt/mujoco_menagerie/agilex_piper/piper.xml'), scene_path))
    data = mujoco.MjData(model)
    reset_scene(model, data, config)
    scatter = BlockScatter(model, data, config)
    if args.aligned:
        for _ in range(math.ceil(1/model.opt.timestep)):
            mujoco.mj_step(model, data)
        poses = scatter.poses(data)
    else:
        poses = scatter.plan(config, seed=args.seed, table=args.table)
    print(json.dumps(evaluate(load_snapshot(args.snapshot, args.url), poses, config), indent=2))


if __name__ == '__main__':
    main()
