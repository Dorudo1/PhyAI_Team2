"""Identity confirmation and known-shape poses; inventory count is never an input."""
from copy import deepcopy
import math

import numpy as np

from phyai_discovery.perception import Detector, DetectorConfig
from phyai_discovery.workspace import Workspace
from test_discovery import image_capture

SIZE = (.075,.025,.015)


def test_unknown_geometry_needs_distinct_consistent_views():
    workspace = Workspace()
    detector = Detector()
    blocks = [(0.,0.,*SIZE,.2)]
    workspace.merge(detector.detect(image_capture(blocks)))
    assert workspace.tracks == []
    assert len(workspace.snapshot()['hypotheses']) == 1
    assert workspace.hypotheses[0]['id'] is None
    workspace.merge(detector.detect(image_capture(blocks,stamp=2.)))
    assert workspace.tracks == []  # repeated same view is not independent evidence
    workspace.merge(detector.detect(image_capture(blocks,camera='wrist',stamp=3.,viewpoint='left')))
    assert len(workspace.tracks) == 1
    assert workspace.tracks[0]['id'] == 'block_1'


def test_six_known_blocks_keep_ids_under_camera_and_orientation_changes():
    truth = [(x,y,*SIZE,a) for (x,y,a) in [
        (-.16,-.1,.0),(-.16,.0,.3),(-.16,.1,.7),
        (.13,-.1,-.2),(.13,.0,.5),(.13,.1,-.7)]]
    detector = Detector(DetectorConfig(known_block_size=SIZE))
    workspace = Workspace()
    associations = None
    for stamp,camera,offset in [(1.,'overhead',0.),(2.,'wrist',.01),(3.,'wrist',-.015)]:
        workspace.merge(detector.detect(image_capture(truth,camera,stamp,f'view_{stamp}',offset)))
        assert len(workspace.tracks) == 6
        assert not workspace.snapshot()['hypotheses']
        current = {track['id']: min(range(len(truth)),key=lambda j:
            math.dist(track['position'][:2],truth[j][:2])) for track in workspace.tracks}
        assert len(set(current.values())) == len(truth)
        if associations is not None:
            assert current == associations
        associations = current
        for track in workspace.tracks:
            nearest = min(truth,key=lambda b:math.dist(b[:2],track['position'][:2]))
            assert math.dist(nearest[:2],track['position'][:2]) < .006
            yaw = 2*math.atan2(track['quaternion'][3],track['quaternion'][0])
            assert abs(math.sin(yaw-nearest[-1])) < .08
            np.testing.assert_allclose(track['size'],SIZE)
    assert {track['id'] for track in workspace.tracks} == {f'block_{i}' for i in range(1,7)}


def test_partial_overlapping_observation_cannot_create_duplicate_id():
    detector = Detector(DetectorConfig(known_block_size=SIZE))
    workspace = Workspace()
    detection = detector.detect(image_capture([(0.,0.,*SIZE,0.)]))
    workspace.merge(detection)
    original = workspace.tracks[0]['id']
    partial = deepcopy(detection)
    partial.camera,partial.stamp,partial.viewpoint = 'wrist',2.,'left'
    partial.candidates[0].position[0] += .03
    partial.candidates[0].resolved = False
    partial.candidates[0].pose_uncertainty['position_m'] = .04
    workspace.merge(partial)
    assert len(workspace.tracks) == 1
    assert workspace.tracks[0]['id'] == original
    assert not workspace.snapshot()['hypotheses']


def test_ambiguous_view_preserves_ids_without_new_confirmation():
    detector = Detector(DetectorConfig(known_block_size=SIZE))
    workspace = Workspace(match_distance=.08)
    detection = detector.detect(image_capture([(-.06,0.,*SIZE,0.),(.06,0.,*SIZE,0.)]))
    workspace.merge(detection)
    ids = {track['id'] for track in workspace.tracks}
    ambiguous = deepcopy(detection)
    ambiguous.camera,ambiguous.stamp = 'wrist',2.
    ambiguous.candidates = ambiguous.candidates[:1]
    ambiguous.candidates[0].position[0] = 0.
    workspace.merge(ambiguous)
    assert {track['id'] for track in workspace.tracks} == ids
    assert all(not track['resolved'] and track['alternatives'] for track in workspace.tracks)
    assert not workspace.snapshot()['hypotheses']


def test_clipped_known_block_stays_provisional():
    workspace = Workspace()
    detector = Detector(DetectorConfig(known_block_size=SIZE))
    workspace.merge(detector.detect(image_capture([(.31,0.,*SIZE,0.)])))
    assert not workspace.tracks
    assert workspace.snapshot()['hypotheses']
    assert not workspace.hypotheses[0]['resolved']


def test_merged_touching_blocks_do_not_confirm_as_one_known_block():
    workspace = Workspace()
    detector = Detector(DetectorConfig(known_block_size=SIZE))
    workspace.merge(detector.detect(image_capture([(0.,0.,*SIZE,0.),(.04,0.,*SIZE,0.)])))
    assert not workspace.tracks
    assert workspace.hypotheses
    assert not workspace.hypotheses[0]['resolved']


def test_conflicting_orientation_does_not_confirm_an_unknown_shape():
    detector = Detector()
    workspace = Workspace()
    detection = detector.detect(image_capture([(0.,0.,*SIZE,0.)]))
    workspace.merge(detection)
    conflicting = deepcopy(detection)
    conflicting.camera,conflicting.stamp,conflicting.viewpoint = 'wrist',2.,'left'
    conflicting.candidates[0].quaternion = [math.sqrt(.5),0.,0.,math.sqrt(.5)]
    workspace.merge(conflicting)
    assert not workspace.tracks
    assert workspace.hypotheses[0]['alternatives']
    assert not workspace.hypotheses[0]['resolved']


def test_confirmed_quality_is_not_overwritten_by_detector_fit_label():
    workspace = Workspace()
    detector = Detector(DetectorConfig(known_block_size=SIZE))
    for camera,stamp,viewpoint in [('overhead',1.,'overhead'),('wrist',2.,'left')]:
        workspace.merge(detector.detect(image_capture([(0.,0.,*SIZE,0.)],camera,stamp,viewpoint)))
        track=workspace.snapshot(now=30.)['tracks'][0]
        assert track['identity_state']=='confirmed'
        assert track['quality']=='confirmed'
        assert track['pose_state']=='resolved'
        assert track['age']>10.
        assert track['pose_last_observed']==stamp
