"""GetBlockInfo inter-app service and discovery state lifecycle."""
import json
from pathlib import Path
import time

import numpy as np
import pytest

rclpy=pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from phyai_discovery import Capture, Intrinsics
from phyai_discovery_interfaces.srv import GetBlockInfo
from phyai_discovery_ros.node import DiscoveryNode


def test_service_gates_inventory_filters_ids_and_rejects_old_sessions(tmp_path):
    cfg=json.loads((Path(__file__).parents[1]/'config/discovery.json').read_text())
    cfg['viewpoints']=['side']
    path=tmp_path/'discovery.json';path.write_text(json.dumps(cfg))
    rclpy.init();nodes=[];executor=SingleThreadedExecutor()
    try:
        discovery=DiscoveryNode(parameter_overrides=[Parameter('config_path',value=str(path))]);nodes.append(discovery)
        consumer=Node('block_info_consumer');nodes.append(consumer)
        for node in nodes:executor.add_node(node)
        client=consumer.create_client(GetBlockInfo,'discovery/get_block_info')
        assert client.wait_for_service(timeout_sec=2)
        def query(**kwargs):
            future=client.call_async(GetBlockInfo.Request(**kwargs))
            executor.spin_until_future_complete(future,timeout_sec=2)
            assert future.done()
            return future.result()
        running=query()
        assert not running.success and not running.finished and not running.snapshot_json
        assert running.discovery_state=='capture_overhead'
        session=running.session_id
        v,u=np.indices((120,160));depth=np.ones((120,160),dtype=np.float32)
        depth[(abs((u-79.5)/240*.985)<=.0375)&(abs((v-59.5)/240*.985)<=.0125)]=.985
        transform=np.diag([1.,-1.,-1.,1.]);transform[2,3]=1.
        def capture(camera,view,stamp):
            return Capture(camera,stamp,camera+'_optical','base_link','test',
                           Intrinsics(160,120,240.,240.,79.5,59.5),depth,transform,view)
        discovery.discovery.process(capture('overhead','overhead',1.),stationary=True)
        discovery.discovery.request_capture(time.monotonic())
        discovery.discovery.tick(time.monotonic(),stationary=True)
        discovery.discovery.process(capture('wrist','side',2.),stationary=True)
        result=query(session_id=session)
        assert result.success and result.finished and result.discovery_state=='search_finished'
        assert result.search_status=='partial'  # sequence complete without configured evidence criteria
        snapshot=json.loads(result.snapshot_json)
        assert snapshot['workspace']['session_id']==session
        assert result.revision==snapshot['workspace']['revision']
        assert result.frame_id=='base_link'
        block=snapshot['workspace']['tracks'][0]
        assert len(block['position'])==3 and len(block['quaternion'])==4
        selected=query(block_id=block['id'],session_id=session)
        assert selected.success
        assert [b['id'] for b in json.loads(selected.snapshot_json)['workspace']['tracks']]==[block['id']]
        missing=query(block_id='missing')
        assert not missing.success and missing.finished and not missing.snapshot_json
        # Late subscribers receive the current compact state, not an independent state machine.
        discovery.publish_progress()
        states=[]
        consumer.create_subscription(String,'discovery/state',lambda msg:states.append(json.loads(msg.data)),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        deadline=time.monotonic()+2
        while not states and time.monotonic()<deadline:executor.spin_once(timeout_sec=.05)
        assert states and states[-1]['finished'] and states[-1]['session_id']==session
        discovery.restart_service(Trigger.Request(),Trigger.Response())
        old=query(session_id=session)
        assert not old.success and old.session_id!=session and not old.snapshot_json
        assert not query().finished
    finally:
        executor.shutdown()
        for node in reversed(nodes):node.destroy_node()
        rclpy.try_shutdown()
