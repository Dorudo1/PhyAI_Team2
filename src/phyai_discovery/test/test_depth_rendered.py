"""Optional headless rendered-depth validation, independent of SAM2 weights."""
import math
import os

import numpy as np
import pytest

from phyai_discovery import Capture, Intrinsics
from phyai_discovery.criteria import SearchCriteria
from phyai_discovery.perception import Detector, DetectorConfig
from phyai_discovery.workspace import Workspace


mujoco = pytest.importorskip('mujoco')
pytestmark = pytest.mark.skipif(
    not (os.environ.get('MUJOCO_GL') in ('egl', 'osmesa') or os.environ.get('DISPLAY')),
    reason='Set MUJOCO_GL=egl or osmesa for rendered-depth validation without a display')


@pytest.mark.parametrize('seed', [4, 17, 31])
def test_randomized_blocks_keep_inventory_and_pose_bounds_with_visible_floor(seed):
    rng = np.random.default_rng(seed)
    truth = [(x+rng.uniform(-.008, .008), y+rng.uniform(-.008, .008), rng.uniform(-1.4, 1.4))
             for x in (-.14, 0., .14) for y in (-.09, .09)]
    blocks = ''.join(
        f'<geom type="box" pos="{x} {y} .0075" size=".0375 .0125 .0075" '
        f'quat="{math.cos(yaw/2)} 0 0 {math.sin(yaw/2)}"/>' for x, y, yaw in truth)
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><visual><global offwidth="640" offheight="480"/></visual><worldbody>'
        '<camera name="view" pos="0 0 .8" fovy="54"/>'
        '<geom type="box" pos="0 0 -.0125" size=".25 .2 .0125"/>'
        '<geom type="plane" pos="0 0 -.3" size="2 2 .01"/>'
        + blocks + '</worldbody></mujoco>')
    data = mujoco.MjData(model)
    detector = Detector(DetectorConfig(known_block_size=(.075, .025, .015),
                                      table_height_range=(-.005, .005)))
    workspace = Workspace(scope=(-.22, .22, -.16, .16))
    identities = None
    with mujoco.Renderer(model, height=480, width=640) as renderer:
        renderer.enable_depth_rendering()
        for index, offset in enumerate((0., .012, -.01)):
            model.cam_pos[0, 0] = offset
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera='view')
            depth = renderer.render().copy()
            focal = 480/(2*math.tan(math.radians(54)/2))
            transform = np.eye(4)
            transform[:3, :3] = data.cam_xmat[0].reshape(3, 3) @ np.diag([1., -1., -1.])
            transform[:3, 3] = data.cam_xpos[0]
            capture = Capture('overhead' if index == 0 else 'wrist', index+1., 'optical',
                'base_link', 'test', Intrinsics(640, 480, focal, focal, 319.5, 239.5),
                depth, transform, 'overhead' if index == 0 else f'view_{index}')
            if index == 0:
                assert Detector().detect(capture).plane is None
            workspace.merge(detector.detect(capture))
            snap = workspace.snapshot()
            assert abs(snap['table']['offset']) < .001
            assert len(snap['tracks']) == len(truth)
            assert not snap['hypotheses']
            current = {track['id']: min(range(len(truth)), key=lambda j:
                math.dist(track['position'][:2], truth[j][:2])) for track in snap['tracks']}
            assert len(set(current.values())) == len(truth)
            if identities is not None:
                assert current == identities
            identities = current
            for track in snap['tracks']:
                x, y, yaw = truth[current[track['id']]]
                assert math.dist(track['position'], (x, y, .0075)) <= track['pose_uncertainty']['position_m']
                fitted_yaw = 2*math.atan2(track['quaternion'][3], track['quaternion'][0])
                error = abs(math.atan2(math.sin(2*(fitted_yaw-yaw)), math.cos(2*(fitted_yaw-yaw))))/2
                assert error <= track['pose_uncertainty']['yaw_rad']
        report = SearchCriteria().assess(workspace.snapshot())
        assert report['met'], report['reasons']
