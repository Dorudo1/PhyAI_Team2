"""Exercise startup using synthetic observations from a lab config, without ROS."""
import argparse
import json
from pathlib import Path

from .sim_adapter import Master


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-config', required=True, type=Path)
    args = parser.parse_args()
    config = json.loads(args.scene_config.read_text())
    scene = {'restart_id': 0, 'simulation_time': 0., 'paused': False,
             'block_size': config['block_size'],
             'blocks': [{'name': f'block_{i+1}',
                         'position': [*xy, config['block_size'][2]/2],
                         'quaternion': [1., 0., 0., 0.]}
                        for i, xy in enumerate(config['block_positions'])]}
    master = Master()
    for stamp in (0., .2, .4, .6, .8):
        scene['simulation_time'] = stamp
        master.consume(scene)
        print(json.dumps(master.status()))


if __name__ == '__main__':
    main()
