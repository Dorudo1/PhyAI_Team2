"""Measured-depth pipeline tests: inputs are images, not known block inventories."""
import json
import math

import numpy as np
import pytest

from phyai_discovery.observations import Capture, Intrinsics, decode_depth
from phyai_discovery.perception import Detector, DetectorConfig
from phyai_discovery.recording import replay
from phyai_discovery.search import Search, SearchState, Stationarity
from phyai_discovery.workspace import Workspace


def image_capture(blocks=(), camera='overhead', stamp=1., viewpoint='overhead', camera_x=0.):
    # Orthogonal downward pinhole camera, 1m above z=0. Ray/surface intersections
    # render top faces at arbitrary yaw. Ground truth is confined to the fixture.
    k = Intrinsics(160, 120, 240., 240., 79.5, 59.5)
    v, u = np.indices((k.height, k.width))
    rx, ry = (u-k.cx)/k.fx, -(v-k.cy)/k.fy
    depth = np.ones((k.height, k.width), dtype=np.float32)
    for x, y, length, width, height, angle in blocks:
        z = 1-height
        px, py = rx*z+camera_x-x, ry*z-y
        a = math.cos(angle)*px+math.sin(angle)*py
        b = -math.sin(angle)*px+math.cos(angle)*py
        mask = (abs(a) <= length/2) & (abs(b) <= width/2)
        depth[mask] = np.minimum(depth[mask], z)
    t = np.diag([1., -1., -1., 1.])
    t[:3,3] = [camera_x, 0., 1.]
    return Capture(camera, stamp, f'{camera}_optical', 'base_link', 'calib_1', k, depth, t, viewpoint)


def test_unknown_count_yaw_and_positions_from_pixels():
    truth = [(-.14, .04, .075, .025, .015, .4), (.12, -.08, .07, .03, .018, -.6)]
    detection = Detector().detect(image_capture(truth))
    assert detection.plane is not None
    assert len(detection.candidates) == 2
    for candidate in detection.candidates:
        nearest = min(truth, key=lambda b: math.dist(candidate.position[:2], b[:2]))
        assert math.dist(candidate.position[:2], nearest[:2]) < .007
        assert abs(candidate.size[0]-nearest[2]) < .008
        assert abs(candidate.size[1]-nearest[3]) < .008
        assert abs(candidate.size[2]-nearest[4]) < .004
        yaw = 2*math.atan2(candidate.quaternion[3], candidate.quaternion[0])
        assert abs(math.sin(yaw-nearest[5])) < .12
        assert candidate.resolved
        assert 'hidden faces' in ' '.join(candidate.reasons)


def test_invalid_depth_is_not_empty_inventory():
    capture = image_capture()
    capture.depth[:] = np.nan
    with pytest.raises(ValueError, match='Insufficient valid depth'):
        Detector().detect(capture)


def test_transform_and_intrinsics_validation():
    capture = image_capture()
    capture.camera_to_map[0,0] = 2.
    with pytest.raises(ValueError, match='proper rotation'):
        capture.validate()
    with pytest.raises(ValueError):
        Intrinsics(160,120,0.,1.,0.,0.).validate()


def test_decode_padded_big_endian_depth():
    rows = bytes([0,100,0,200,0,0, 0,0,1,44,0,0])
    decoded = decode_depth(rows, 2,2,6,'16UC1',True)
    np.testing.assert_allclose(decoded[0], [.1,.2])
    assert np.isnan(decoded[1,0])
    assert decoded[1,1] == pytest.approx(.3)


def test_duplicate_views_preserve_ids_and_missing_views_preserve_tracks():
    workspace = Workspace()
    detector = Detector(DetectorConfig(known_block_size=(.075,.025,.015)))
    blocks = [(.1,0.,.075,.025,.015,.2)]
    workspace.merge(detector.detect(image_capture(blocks)))
    original = workspace.tracks[0]['id']
    workspace.merge(detector.detect(image_capture(blocks, 'wrist', 2., 'wrist_1')))
    assert len(workspace.tracks) == 1
    assert workspace.tracks[0]['id'] == original
    assert workspace.tracks[0]['source_cameras'] == ['overhead','wrist']
    workspace.merge(detector.detect(image_capture([], 'wrist', 3., 'wrist_2', camera_x=.5)))
    assert len(workspace.tracks) == 1
    assert workspace.tracks[0]['last_observed'] == 2.


def test_scope_does_not_fill_missing_depth_or_certify_motion():
    workspace = Workspace(scope=(-.4,.4,-.4,.4))
    workspace.merge(Detector().detect(image_capture()))
    snap = workspace.snapshot(now=20.)
    assert any(cell['state'] == 'unobserved' for cell in snap['coverage']['cells'])
    assert snap['coverage']['observed_fraction'] < 1
    assert snap['motion_authorization'] is False
    assert any(cell['age'] == 19 for cell in snap['coverage']['cells'])


def test_calibration_change_requires_restart():
    workspace = Workspace()
    workspace.merge(Detector().detect(image_capture()))
    detection = Detector().detect(image_capture(stamp=2.))
    detection.calibration_revision = 'changed'
    with pytest.raises(ValueError, match='Calibration changed'):
        workspace.merge(detection)
    assert workspace.revision == 1


def test_irregular_touching_group_is_flagged():
    blocks = [(0.,0.,.075,.025,.015,0.), (.025,.02,.025,.05,.015,0.)]
    candidates = Detector().detect(image_capture(blocks)).candidates
    assert len(candidates) == 1
    assert not candidates[0].resolved


def test_stationarity_freshness_movement_and_feedback_gap():
    stationary = Stationarity(duration=.5)
    for stamp in (0.,.2,.4,.6):
        stationary.update([0.]*7,stamp)
    assert stationary.ready(.6)
    assert not stationary.ready(2.)
    stationary.update([0.]*7,2.)
    assert not stationary.ready(2.)
    stationary.update([.02]*7,2.2)
    assert not stationary.ready(2.3)


def test_timeout_and_retry_budget_allow_wrist_fallback():
    search = Search(Workspace(), viewpoints=['left'], max_retries=0, capture_timeout=1.)
    search.tick(0.,False)
    search.tick(2.,False)
    assert search.state == SearchState.WRIST
    search.request(3.)
    search.tick(5.,False)
    assert search.state == SearchState.FINISHED
    assert search.result['status'] == 'failed'


def test_recorded_scan_finds_blocks_outside_overhead_and_finishes_partial(tmp_path):
    captures = [image_capture([(-.1,0.,.075,.025,.015,0.)]),
                image_capture([(.6,0.,.075,.025,.015,.3)],'wrist',2.,'right',camera_x=.6)]
    entries = []
    for i, capture in enumerate(captures):
        np.save(tmp_path/f'depth{i}.npy', capture.depth)
        entries.append({'camera': capture.camera, 'stamp':capture.stamp,
                        'depth':f'depth{i}.npy','depth_unit':'m',
                        'optical_frame':capture.optical_frame,'map_frame':capture.map_frame,
                        'calibration_revision':capture.calibration_revision,
                        'intrinsics':vars(capture.intrinsics),
                        'camera_to_map':capture.camera_to_map.tolist(),
                        'viewpoint':capture.viewpoint, 'stationary_verified':True})
    path = tmp_path/'scan.json'
    path.write_text(json.dumps({'version':1,'map_frame':'base_link','viewpoints':['right'],'captures':entries}))
    result = replay(path)[-1]
    assert len(result['workspace']['tracks']) == 0
    assert len(result['workspace']['hypotheses']) == 2
    assert result['search']['result']['status'] == 'partial'
    assert result['search']['result']['pick_ready'] is False


def test_invalid_mask_excluded_pixels_do_not_create_objects():
    capture=image_capture([(.1,0.,.075,.025,.015,0.)])
    mask=capture.depth < .99
    masked=Capture(**{**vars(capture),'mask':mask})
    detection=Detector().detect(masked)
    assert not detection.candidates
    assert len(detection.surface_points)>0


def test_clipped_candidates_remain_uncertain():
    candidates=Detector().detect(image_capture([(.31,0.,.075,.025,.015,0.)])).candidates
    assert candidates
    assert not candidates[0].resolved
    assert 'clipped' in ' '.join(candidates[0].reasons)


def test_conflicting_table_geometry_does_not_mutate_map():
    workspace=Workspace()
    workspace.merge(Detector().detect(image_capture()))
    capture=image_capture(stamp=2.)
    capture.camera_to_map[2,3]+=.05
    with pytest.raises(ValueError,match='Table geometry conflicts'):
        workspace.merge(Detector().detect(capture))
    assert workspace.revision==1


def test_ambiguous_association_flags_existing_tracks():
    from copy import deepcopy
    workspace=Workspace(match_distance=.1)
    detection=Detector().detect(image_capture([(-.06,0.,.075,.025,.015,0.),(.06,0.,.075,.025,.015,0.)]))
    workspace.merge(detection)
    assert len(workspace.hypotheses)==2
    ambiguous=deepcopy(detection)
    ambiguous.camera,ambiguous.stamp='wrist',2.
    ambiguous.candidates=ambiguous.candidates[:1]
    ambiguous.candidates[0].position[0]=0.
    workspace.merge(ambiguous)
    assert len(workspace.hypotheses)==2
    assert all(not track['resolved'] for track in workspace.hypotheses)


def test_plane_unknown_is_distinct_from_an_empty_table():
    capture=image_capture()
    capture.camera_to_map[:3,:3]=[[0,0,1],[0,1,0],[-1,0,0]]
    detection=Detector().detect(capture)
    assert detection.plane is None
    assert len(detection.surface_points)==0
    assert detection.warnings
    workspace=Workspace()
    workspace.merge(detection)
    assert workspace.snapshot()['table'] is None
    assert not workspace.coverage


def test_worker_generation_changes_on_timeout_and_restart():
    search=Search(Workspace(),capture_timeout=1.)
    search.tick(0.,False)
    search.begin()
    generation=search.generation
    search.tick(2.,False)
    assert search.generation != generation
    generation=search.generation
    search.reset()
    assert search.generation != generation


def test_noise_and_roi_preserve_geometry_uncertainty():
    capture=image_capture([(.1,0.,.075,.025,.015,.3)])
    noisy=capture.depth+np.random.default_rng(5).normal(0.,.0004,capture.depth.shape).astype(np.float32)
    capture=Capture(**{**vars(capture),'depth':noisy,'roi':(50,25,145,95)})
    detection=Detector().detect(capture)
    assert len(detection.candidates)==1
    assert abs(detection.candidates[0].size[2]-.015)<.004
    workspace=Workspace()
    workspace.merge(detection)
    assert workspace.table['boundary_complete'] is False
    assert workspace.table['observed_surface_bounds'][0]>-.2
