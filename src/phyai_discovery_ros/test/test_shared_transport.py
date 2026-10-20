"""Standalone transport integration: no Junh or simulator imports."""
import json
from pathlib import Path
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger

from phyai_discovery_ros.node import DiscoveryNode


def test_recorded_inventory_and_restart_over_shared_interfaces(tmp_path):
    config = json.loads((Path(__file__).parents[1]/'config/discovery.json').read_text())
    config_path = tmp_path/'hardware.json'
    config_path.write_text(json.dumps(config))
    # Downward pinhole at z=1m: one 75x25x15mm cuboid on a horizontal table.
    v, u = np.indices((120, 160))
    depth = np.ones((120, 160), dtype=np.float32)
    top = (abs((u-79.5)/240*.985) <= .0375) & (abs((v-59.5)/240*.985) <= .0125)
    depth[top] = .985
    np.save(tmp_path/'depth.npy', depth)
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = 1.
    entries = []
    for camera, stamp, viewpoint in [('overhead', 1., 'overhead'), ('wrist', 2., 'side')]:
        entries.append(dict(camera=camera, stamp=stamp, viewpoint=viewpoint,
            depth='depth.npy', depth_unit='m', optical_frame=f'{camera}_optical',
            map_frame='base_link', calibration_revision='measured_test',
            intrinsics=dict(width=160,height=120,fx=240.,fy=240.,cx=79.5,cy=59.5),
            camera_to_map=transform.tolist(), stationary_verified=True))
    manifest = tmp_path/'scan.json'
    manifest.write_text(json.dumps(dict(version=1,map_frame='base_link',
                                       viewpoints=['side'],captures=entries)))
    rclpy.init()
    nodes = []
    executor = SingleThreadedExecutor()
    try:
        discovery = DiscoveryNode(parameter_overrides=[
            Parameter('config_path',value=str(config_path)),
            Parameter('recording_path',value=str(manifest))])
        nodes.append(discovery)
        consumer = Node('independent_consumer')
        nodes.append(consumer)
        messages = []
        consumer.create_subscription(String,'discovery/workspace',
                                     lambda msg: messages.append(json.loads(msg.data)),1)
        restart = consumer.create_client(Trigger,'discovery/restart_scan')
        for node in nodes:
            executor.add_node(node)
        deadline = time.monotonic()+6
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.05)
            if messages and messages[-1]['search']['result']:
                break
        assert messages and messages[-1]['search']['result']
        result = messages[-1]
        assert result['input_mode'] == 'recorded'
        assert result['workspace']['frame'] == 'base_link'
        assert len(result['workspace']['tracks']) == 1
        assert result['search']['accepted'] == 2
        assert result['search']['result']['pick_ready'] is False
        assert all('joint_ctrl' not in name for name, _ in discovery.get_publisher_names_and_types_by_node(
            discovery.get_name(), discovery.get_namespace()))
        assert restart.wait_for_service(timeout_sec=2)
        session = discovery.workspace.session
        future = restart.call_async(Trigger.Request())
        executor.spin_until_future_complete(future,timeout_sec=2)
        assert future.done() and future.result().success
        assert discovery.workspace.session != session
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.try_shutdown()
