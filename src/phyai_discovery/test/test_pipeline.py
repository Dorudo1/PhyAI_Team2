"""Exercise the public feature contract without importing the junh test app."""
import json
from dataclasses import replace

import numpy as np
import pytest

from phyai_discovery import Capture, Discovery, Intrinsics, SearchState


def capture(camera='overhead', stamp=1., viewpoint='overhead'):
    depth = np.ones((60, 80), dtype=np.float32)
    depth[25:35, 30:50] = .98
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = 1.
    return Capture(camera, stamp, camera+'_optical', 'base_link', 'calibration_1',
        Intrinsics(80, 60, 200., 200., 39.5, 29.5), depth, transform, viewpoint)


def test_scan_handoff_and_independent_sessions():
    feature = Discovery({'viewpoints': ['right']})
    other = Discovery({'viewpoints': ['right']})
    feature.process(capture(), stationary=True)
    assert feature.search.state == SearchState.WRIST
    assert len(feature.workspace.hypotheses) == 1
    feature.request_capture(1.)
    feature.tick(1.1, stationary=False)
    assert feature.search.state == SearchState.STATIONARY
    feature.tick(1.2, stationary=True)
    feature.process(capture('wrist', 2., 'right'), stationary=True)
    result = feature.snapshot()
    assert result['search']['state'] == 'search_finished'
    assert result['search']['result']['status'] == 'partial'
    assert result['search']['result']['pick_ready'] is False
    assert len(result['workspace']['tracks']) == 1
    json.dumps(result, allow_nan=False)
    assert other.workspace.revision == 0
    session = feature.workspace.session
    detector = feature.detector
    feature.reset()
    assert feature.workspace.session != session
    assert feature.detector is detector
    assert not feature.workspace.tracks
    assert feature.search.state == SearchState.OVERHEAD


@pytest.mark.parametrize('invalidate', ['reset', 'timeout', 'finish'])
def test_stale_worker_cannot_modify_current_scan(invalidate):
    feature = Discovery({'capture_timeout': 1.})
    feature.tick(0., stationary=True)
    old = feature.begin(capture(), stationary=True)
    result = feature.detector.detect(old.capture)
    if invalidate == 'reset':
        feature.reset()
    elif invalidate == 'timeout':
        feature.tick(2., stationary=True)
    else:
        feature.finish()
        feature.reset()
    current = feature.begin(capture(stamp=2.), stationary=True)
    assert feature.complete(old, result) is False
    assert feature.fail(old, 'late error') is False
    assert feature.workspace.revision == 0
    assert feature.complete(current, feature.detector.detect(current.capture)) is True
    assert feature.complete(current, result) is False
    assert feature.workspace.revision == 1


def test_wrong_view_motion_and_frame_do_not_start_processing():
    feature = Discovery()
    for invalid, stationary in [(capture(), False),
            (capture('wrist', 2., 'wrist_1'), True),
            (replace(capture(), map_frame='different_frame'), True)]:
        with pytest.raises(ValueError):
            feature.process(invalid, stationary=stationary)
        assert feature.search.state == SearchState.OVERHEAD
        assert feature.workspace.revision == 0


def test_processing_failure_uses_retry_budget_and_recovers():
    feature = Discovery()
    bad = capture()
    bad.depth[:] = np.nan
    with pytest.raises(ValueError, match='Insufficient valid depth'):
        feature.process(bad, stationary=True)
    assert feature.search.rejected == 1
    assert feature.search.state == SearchState.OVERHEAD
    feature.process(capture(stamp=2.), stationary=True)
    assert feature.search.accepted == 1
    assert feature.workspace.revision == 1
