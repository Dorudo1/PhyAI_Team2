"""Replay a manifest of measured RGB-D captures through the real detector."""
import argparse
import json
from pathlib import Path

from .observations import load_capture, load_manifest
from . import Discovery, Detector, DetectorConfig, SearchState


def replay(path, config=None):
    manifest, base = load_manifest(path)
    discovery = Discovery({'map_frame': manifest['map_frame'],
        'scope': manifest.get('scope'), 'viewpoints': manifest['viewpoints']},
        detector=Detector(config or DetectorConfig()))
    search = discovery.search
    snapshots = []
    for entry in manifest['captures']:
        if search.state == SearchState.FINISHED:
            break
        rejected = search.rejected
        try:
            capture = load_capture(entry, base)
            if entry.get('stationary_verified') is not True:
                raise ValueError('Recorded capture must include stationary_verified=true.')
            if search.state == SearchState.WRIST:
                discovery.request_capture(capture.stamp)
                discovery.tick(capture.stamp, stationary=True)
            if not search.accepts(capture):
                raise ValueError(f'Unexpected camera/viewpoint; expected {search.target}.')
            discovery.process(capture, stationary=True)
        except (ValueError, KeyError, OSError) as error:
            if search.rejected == rejected:
                search.reject(str(error))
        snapshots.append(discovery.snapshot())
    if search.state != SearchState.FINISHED:
        discovery.finish('Recorded observations exhausted.')
    snapshots.append(discovery.snapshot())
    return snapshots


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--output', type=Path, help='Write each map/progress snapshot as JSON Lines.')
    args = parser.parse_args()
    snapshots = replay(args.manifest)
    content = ''.join(json.dumps(snapshot, allow_nan=False)+'\n' for snapshot in snapshots)
    if args.output:
        args.output.write_text(content)
    print(json.dumps(snapshots[-1], allow_nan=False))


if __name__ == '__main__':
    main()
