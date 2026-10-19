"""Optional actual-model test: unknown count/yaw from rendered RGB-D only."""
import math
import os

import numpy as np
import pytest

from phyai_discovery.observations import Capture, Intrinsics
from phyai_discovery.perception import Detector, DetectorConfig
from phyai_discovery.segmentation import SAM2Segmenter
from phyai_discovery.workspace import Workspace

CHECKPOINT = os.environ.get('JUNH_TEST_SAM2_CHECKPOINT')
pytestmark = pytest.mark.skipif(not CHECKPOINT, reason='Set JUNH_TEST_SAM2_CHECKPOINT for real SAM 2 inference')


@pytest.mark.parametrize('angles', [(0.,.35,-.65),(.2,-.3,.8,-.5,.1)])
def test_rendered_rgbd_known_geometry_has_unique_ids(angles):
    import mujoco
    truth = [(-.18+i*.085, .06*(-1)**i, a) for i,a in enumerate(angles)]
    geoms = ''.join(
        f'<geom type="box" pos="{x} {y} .0075" size=".0375 .0125 .0075" '
        f'quat="{math.cos(a/2)} 0 0 {math.sin(a/2)}" rgba=".75 .52 .25 1"/>'
        for x,y,a in truth)
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><visual><global offwidth="320" offheight="240"/></visual><worldbody>'
        '<light pos="0 0 2"/><camera name="overhead" pos="0 0 .65" fovy="54"/>'
        '<geom type="plane" size="1 1 .01" rgba=".2 .3 .4 1"/>'
        + geoms + '</worldbody></mujoco>')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    detector = Detector(DetectorConfig(known_block_size=(.075,.025,.015)),
                        SAM2Segmenter(CHECKPOINT))
    workspace = Workspace()
    renderer = mujoco.Renderer(model,height=240,width=320)
    associations = None
    try:
        for i,offset in enumerate([0.,.012,-.01]):
            model.cam_pos[0,0] = offset
            mujoco.mj_forward(model,data)
            renderer.update_scene(data,camera='overhead')
            rgb = renderer.render().copy()
            renderer.enable_depth_rendering()
            depth = renderer.render().copy()
            renderer.disable_depth_rendering()
            focal = 240/(2*math.tan(math.radians(54)/2))
            transform = np.eye(4)
            transform[:3,:3] = data.cam_xmat[0].reshape(3,3) @ np.diag([1.,-1.,-1.])
            transform[:3,3] = data.cam_xpos[0]
            capture = Capture('overhead',i+1.,'optical','base_link','test',
                Intrinsics(320,240,focal,focal,159.5,119.5),depth,transform,
                f'view_{i}',rgb=rgb)
            workspace.merge(detector.detect(capture))
            assert len(workspace.tracks) == len(truth)
            assert not workspace.snapshot()['hypotheses']
            current = {track['id']: min(range(len(truth)),key=lambda j:
                math.dist(track['position'][:2],truth[j][:2])) for track in workspace.tracks}
            assert len(set(current.values())) == len(truth)
            if associations is not None:
                assert current == associations
            associations = current
            for track in workspace.tracks:
                nearest = min(truth,key=lambda b:math.dist(track['position'][:2],b[:2]))
                assert math.dist(track['position'][:2],nearest[:2]) < .004
                yaw = 2*math.atan2(track['quaternion'][3],track['quaternion'][0])
                assert abs(math.sin(yaw-nearest[2])) < .08
        assert len({t['id'] for t in workspace.tracks}) == len(truth)
    finally:
        renderer.close()
