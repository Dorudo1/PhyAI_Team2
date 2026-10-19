"""Verify optical calibration against actual MuJoCo depth rendering."""
import math
import os
from pathlib import Path

import numpy as np
import pytest

mujoco=pytest.importorskip('mujoco')
pytest.importorskip('sensor_msgs')
from piper_mujoco_sim.camera_ros import camera_info, image_msg
from builtin_interfaces.msg import Time


def test_depth_image_and_intrinsics_share_pixel_geometry():
    model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <camera name="test" pos="0 0 1" fovy="60"/>
      <geom type="plane" size="2 2 .1"/>
    </worldbody></mujoco>''')
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    try:renderer=mujoco.Renderer(model,height=120,width=160)
    except Exception as error:pytest.skip(f'Offscreen renderer unavailable: {error}')
    try:
        renderer.update_scene(data,camera='test')
        renderer.enable_depth_rendering();depth=renderer.render().copy()
        np.testing.assert_allclose(depth,1.,atol=1e-4)
        info=camera_info(160,120,60.,'test_optical',Time(sec=1))
        assert info.k[0]==pytest.approx(120/(2*math.tan(math.radians(30))))
        assert info.k[2]==79.5
        msg=image_msg(depth.astype(np.float32),'32FC1','test_optical',info.header.stamp)
        assert msg.step==160*4
        np.testing.assert_allclose(np.frombuffer(bytes(msg.data),dtype=np.float32).reshape(120,160),depth)
        optical=data.cam_xmat[0].reshape(3,3)@np.diag([1.,-1.,-1.])
        point=optical@np.array([0.,0.,depth[60,80]])+data.cam_xpos[0]
        assert abs(point[2])<1e-4
    finally:renderer.close()


def test_on_demand_camera_selection_and_expiry():
    from types import SimpleNamespace
    from piper_mujoco_sim.sim import PiperMujocoSim
    state=SimpleNamespace(camera_on_demand=True,camera_request='wrist',camera_request_received=5.)
    assert PiperMujocoSim.sensor_requested(state,'wrist',5.5)
    assert not PiperMujocoSim.sensor_requested(state,'overhead',5.5)
    assert not PiperMujocoSim.sensor_requested(state,'wrist',6.1)
    state.camera_request=None
    assert not PiperMujocoSim.sensor_requested(state,'wrist',5.5)
    state.camera_on_demand=False
    assert PiperMujocoSim.sensor_requested(state,'overhead',10.)
