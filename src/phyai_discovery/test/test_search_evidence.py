"""Regression cases where measured evidence must prevent a false successful scan."""
from copy import deepcopy
import math

import numpy as np
import pytest

from phyai_discovery import Discovery
from phyai_discovery.criteria import SearchCriteria
from phyai_discovery.perception import Detector, DetectorConfig
from phyai_discovery.workspace import Workspace
from test_discovery import image_capture


SIZE = (.075, .025, .015)
BLOCKS = [(-.1, 0., *SIZE, 0.), (.1, 0., *SIZE, 0.)]


class SelectiveSegmenter:
    def __init__(self, count=1):
        self.count = count
        self.calls = 0

    def segment(self, capture, pixels, groups):
        self.calls += 1
        masks = []
        for group in groups[:self.count]:
            mask = np.zeros(capture.depth.shape, dtype=bool)
            uv = pixels[group]
            mask[uv[:, 1], uv[:, 0]] = True
            masks.append(mask)
        return masks


def feature(detector):
    return Discovery({'scope': [-.2, .2, -.15, .15], 'viewpoints': ['left'],
                      'search_criteria': {}}, detector=detector)


@pytest.mark.parametrize('count', [0, 1])
def test_resolved_depth_blocks_finish_baseline_without_segmenter_inference(count):
    segmenter = SelectiveSegmenter(count)
    detector = Detector(DetectorConfig(known_block_size=SIZE), segmenter)
    scan = feature(detector)
    scan.process(image_capture(BLOCKS), stationary=True)
    scan.request_capture(2.)
    scan.tick(2., stationary=True)
    scan.process(image_capture(BLOCKS, 'wrist', 2., 'left'), stationary=True)
    snap = scan.snapshot()
    assert len(snap['workspace']['tracks']) == 2
    assert not snap['workspace']['hypotheses']
    assert segmenter.calls == 0
    assert snap['search']['result']['criteria_met']
    assert scan.search.inspection_round == 0


def test_unresolved_depth_group_survives_missing_segmentation():
    segmenter = SelectiveSegmenter(0)
    detector = Detector(DetectorConfig(known_block_size=SIZE), segmenter)
    workspace = Workspace(scope=(-.2, .2, -.15, .15))
    touching = [(0., 0., *SIZE, 0.), (.04, 0., *SIZE, 0.)]
    workspace.merge(detector.detect(image_capture(touching)))
    assert segmenter.calls == 1
    assert not workspace.tracks
    assert workspace.snapshot()['hypotheses']
    assert not SearchCriteria().assess(workspace.snapshot())['met']


class ErodedSegmenter:
    def __init__(self):
        self.calls = 0

    def segment(self, capture, pixels, groups):
        self.calls += 1
        masks = []
        for group in groups:
            uv = pixels[group]
            low, high = uv.min(axis=0), uv.max(axis=0)
            interior = uv[(uv[:, 0] > low[0]) & (uv[:, 0] < high[0]) &
                          (uv[:, 1] > low[1]) & (uv[:, 1] < high[1])]
            mask = np.zeros(capture.depth.shape, dtype=bool)
            mask[interior[:, 1], interior[:, 0]] = True
            masks.append(mask)
        return masks


def test_thin_mask_edges_do_not_become_permanent_duplicate_hypotheses():
    segmenter = ErodedSegmenter()
    detector = Detector(DetectorConfig(known_block_size=SIZE), segmenter)
    scan = feature(detector)
    scan.process(image_capture(BLOCKS), stationary=True)
    scan.request_capture(2.)
    scan.tick(2., stationary=True)
    scan.process(image_capture(BLOCKS, 'wrist', 2., 'left'), stationary=True)
    snap = scan.snapshot()
    assert segmenter.calls == 0
    assert len(snap['workspace']['tracks']) == 2
    assert not snap['workspace']['hypotheses']
    assert snap['search']['result']['criteria_met']


def test_segmented_touching_group_retains_depth_outside_resolved_block():
    class LeftBlockSegmenter:
        def segment(self, capture, pixels, groups):
            mask = np.zeros(capture.depth.shape, dtype=bool)
            uv = pixels[groups[0]]
            left = uv[uv[:, 0] <= 88]
            mask[left[:, 1], left[:, 0]] = True
            return [mask]

    detector = Detector(DetectorConfig(known_block_size=SIZE), LeftBlockSegmenter())
    touching = [(0., 0., *SIZE, 0.), (.075, 0., *SIZE, 0.)]
    workspace = Workspace(scope=(-.2, .2, -.15, .15))
    workspace.merge(detector.detect(image_capture(touching)))
    snap = workspace.snapshot()
    assert len(snap['tracks']) == 1
    assert len(snap['hypotheses']) == 1
    assert not SearchCriteria().assess(snap)['met']


def test_small_above_table_evidence_without_a_fit_still_blocks_success():
    capture = image_capture()
    capture.depth[58:60, 78:80] = .985
    workspace = Workspace(scope=(-.2, .2, -.15, .15))
    workspace.merge(Detector().detect(capture))
    snap = workspace.snapshot()
    assert not snap['tracks'] and not snap['hypotheses']
    assert snap['coverage']['unexplained_cells']
    assert not SearchCriteria().assess(snap)['met']


@pytest.mark.parametrize('background_depth', [1.02, 1.3])
def test_mixed_horizontal_surfaces_without_installation_prior_are_ambiguous(background_depth):
    capture = image_capture()
    capture.depth[:, 80:] = background_depth
    detection = Detector().detect(capture)
    assert detection.plane is None
    scan = feature(Detector())
    scan.process(capture, stationary=True)
    scan.finish()
    assert scan.search.result['status'] == 'failed'
    assert not scan.search.result['criteria_met']


def test_height_prior_selects_table_and_background_does_not_inflate_coverage():
    capture = image_capture()
    capture.depth[:, 80:] = 1.3
    workspace = Workspace(scope=(-.2, .2, -.15, .15))
    detector = Detector(DetectorConfig(table_height_range=(-.005, .005)))
    detection = detector.detect(capture)
    assert detection.plane is not None
    assert abs(detection.plane['offset']) < .001
    workspace.merge(detection)
    coverage = workspace.snapshot()['coverage']
    assert coverage['explored_fraction'] == pytest.approx(.5)
    assert not SearchCriteria().assess(workspace.snapshot())['met']


def test_height_prior_cannot_invent_a_missing_table():
    capture = image_capture()
    capture.depth[:] = 1.3
    assert Detector(DetectorConfig(table_height_range=(-.005, .005))).detect(capture).plane is None


def test_overly_broad_height_prior_does_not_disambiguate_two_surfaces():
    capture = image_capture()
    capture.depth[:, 80:] = 1.3
    detector = Detector(DetectorConfig(table_height_range=(-.4, .1)))
    assert detector.detect(capture).plane is None


def test_object_above_supported_height_remains_unexplained_even_with_high_coverage():
    capture = image_capture([(.1, 0., .025, .025, .25, 0.)])
    detector = Detector(DetectorConfig(table_height_range=(-.005, .005)))
    workspace = Workspace(scope=(-.2, .2, -.15, .15))
    workspace.merge(detector.detect(capture))
    snap = workspace.snapshot()
    assert snap['coverage']['explored_fraction'] > .9
    assert snap['coverage']['unexplained_cells']
    assert not SearchCriteria().assess(snap)['met']


@pytest.mark.parametrize('bounds', [(0., 0.), (.1, -.1), (float('nan'), .1), (0.,)])
def test_invalid_table_height_prior(bounds):
    with pytest.raises(ValueError, match='Table height range'):
        DetectorConfig(table_height_range=bounds)


def fitted_workspace():
    detector = Detector(DetectorConfig(known_block_size=SIZE))
    first = detector.detect(image_capture(BLOCKS[:1]))
    workspace = Workspace(scope=(-.2, .2, -.15, .15))
    workspace.merge(first)
    return workspace, first


def another_view(first, stamp, view, dx=0., yaw=0.):
    detection = deepcopy(first)
    detection.camera, detection.stamp, detection.viewpoint = 'wrist', stamp, view
    detection.candidates[0].position[0] += dx
    detection.candidates[0].quaternion = [math.cos(yaw/2), 0., 0., math.sin(yaw/2)]
    return detection


@pytest.mark.parametrize('dx,yaw', [(.02, 0.), (0., math.radians(12))])
def test_cross_view_pose_conflict_cannot_supply_support_and_needs_new_consistent_views(dx, yaw):
    workspace, first = fitted_workspace()
    original_id = workspace.tracks[0]['id']
    workspace.merge(another_view(first, 2., 'left', dx, yaw))
    track = workspace.tracks[0]
    assert track['id'] == original_id
    assert not track['resolved'] and track['alternatives']
    assert len(track['supporting_views']) == 1
    assert not SearchCriteria().assess(workspace.snapshot())['met']
    workspace.merge(another_view(first, 3., 'left', dx, yaw))
    assert not track['resolved']  # Repeating the same view cannot settle the conflict.
    workspace.merge(another_view(first, 4., 'right', dx, yaw))
    assert track['resolved'] and not track['alternatives']
    assert track['id'] == original_id
    assert len(track['supporting_views']) == 2
    assert SearchCriteria().assess(workspace.snapshot())['met']


def test_compatible_view_residual_is_included_in_reported_position_bound():
    workspace, first = fitted_workspace()
    bound = first.candidates[0].pose_uncertainty['position_m']
    workspace.merge(another_view(first, 2., 'left', dx=.012))
    track = workspace.tracks[0]
    assert track['resolved']
    assert track['pose_uncertainty']['position_m'] >= bound+.006-1e-9
    assert track['position'][0] == pytest.approx(first.candidates[0].position[0]+.006)
    assert not SearchCriteria().assess(workspace.snapshot())['met']


def test_slow_pose_drift_is_checked_against_all_supporting_views():
    workspace, first = fitted_workspace()
    for stamp, dx in [(2., .006), (3., .012), (4., .018)]:
        workspace.merge(another_view(first, stamp, str(stamp), dx))
    assert not workspace.tracks[0]['resolved']
    assert len(workspace.tracks[0]['supporting_views']) == 1
    assert not SearchCriteria().assess(workspace.snapshot())['met']


def test_repeated_view_cannot_walk_its_original_pose_anchor():
    workspace, first = fitted_workspace()
    for stamp, dx in [(2., .006), (3., .012), (4., .018)]:
        workspace.merge(another_view(first, stamp, 'same_view', dx))
    assert not workspace.tracks[0]['resolved']
    assert not SearchCriteria().assess(workspace.snapshot())['met']


def test_half_turn_symmetry_does_not_create_a_false_orientation_conflict():
    workspace, first = fitted_workspace()
    other = another_view(first, 2., 'left')
    w, x, y, z = first.candidates[0].quaternion
    other.candidates[0].quaternion = [-z, y, -x, w]
    workspace.merge(other)
    assert workspace.tracks[0]['resolved']
    assert SearchCriteria().assess(workspace.snapshot())['met']


def test_configured_sensor_errors_contribute_to_pose_bounds():
    detector = Detector(DetectorConfig(known_block_size=SIZE, depth_error=.004,
                        calibration_position_error=.003, calibration_yaw_error=math.radians(2)))
    candidate = detector.detect(image_capture(BLOCKS[:1])).candidates[0]
    assert candidate.pose_uncertainty['position_m'] >= .012
    assert candidate.pose_uncertainty['yaw_rad'] == pytest.approx(math.radians(5))


@pytest.mark.parametrize('key', ['depth_error', 'calibration_position_error', 'calibration_yaw_error'])
@pytest.mark.parametrize('value', [-.001, float('nan'), float('inf')])
def test_invalid_sensor_error_bounds(key, value):
    with pytest.raises(ValueError, match='Measurement error bounds'):
        DetectorConfig(**{key: value})
