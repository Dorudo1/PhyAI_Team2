"""File calibration validation and live depth use without a CameraInfo publisher."""
import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image

from phyai_discovery_ros.calibration import load_calibration
from phyai_discovery_ros.node import DiscoveryNode


def calibration():
    return dict(version=1, frame_id='overhead_optical', calibration_revision='measured_v1',
                intrinsics=dict(width=160,height=120,fx=240.,fy=240.,cx=79.5,cy=59.5))


@pytest.mark.parametrize('change', [
    {'version': 2}, {'frame_id': ''}, {'calibration_revision': ''},
    {'intrinsics': {**calibration()['intrinsics'], 'fx': 0.}},
    {'intrinsics': {**calibration()['intrinsics'], 'width': 160.5}},
    {'distortion': [float('nan')]}, {'projection': [1., 2.]},
])
def test_bad_calibration_is_rejected_with_filename(tmp_path, change):
    path=tmp_path/'camera.json'
    path.write_text(json.dumps({**calibration(), **change}))
    with pytest.raises(ValueError, match='camera.json'):
        load_calibration(path)


def test_live_capture_uses_file_and_retains_frame_and_size_checks(tmp_path):
    path=tmp_path/'camera.json'
    path.write_text(json.dumps(calibration()))
    config=json.loads((Path(__file__).parents[1]/'config/discovery.json').read_text())
    config['cameras']['overhead'].update(intrinsics_file='camera.json', info_topic=None)
    config_path=tmp_path/'discovery.json'
    config_path.write_text(json.dumps(config))
    rclpy.init()
    node=None
    try:
        node=DiscoveryNode(parameter_overrides=[Parameter('config_path',value=str(config_path))])
        assert node.config['cameras']['overhead']['calibration_revision']=='measured_v1'
        # Configured file calibration cannot be overwritten by an unrelated publisher.
        node.on_info('overhead',CameraInfo())
        assert node.info['overhead'].width==160
        submitted=[]
        node.submit=submitted.append
        now=time.monotonic()
        node.stationarity.update([0.]*7, now-.6)
        node.stationarity.update([0.]*7, now)
        node.tf.lookup_transform=lambda *_: SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.,y=0.,z=1.),
            rotation=SimpleNamespace(w=0.,x=1.,y=0.,z=0.)))
        image=Image(width=160,height=120,encoding='32FC1',step=160*4)
        image.header.stamp=node.get_clock().now().to_msg()
        image.data=np.ones((120,160),dtype=np.float32).tobytes()
        image.header.frame_id='wrong_optical'
        node.on_depth('overhead',image)
        assert not submitted and 'frame/dimensions' in node.last_error
        image.header.frame_id='overhead_optical'
        image.width=159
        node.on_depth('overhead',image)
        assert not submitted and 'frame/dimensions' in node.last_error
        image.width=160
        node.on_depth('overhead',image)
        assert len(submitted)==1
        capture=submitted[0]
        assert capture.intrinsics.fx==240. and capture.intrinsics.width==160
        assert capture.optical_frame=='overhead_optical'
        np.testing.assert_allclose(capture.camera_to_map[:3,3],[0.,0.,1.])
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
