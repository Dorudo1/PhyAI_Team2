"""Simulation driver motion interpolation, ownership and capture races."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
rclpy=pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from junh_master.ros.sim_scan import SimScanNode, simulator_only, smooth_target
from junh_master.ros import sim_scan as module


def test_interpolation_stays_between_endpoints():
    a=[0.]*7;b=[1.,-1.,.5,.2,-.2,.4,.035]
    assert smooth_target(a,b,0.,4.)==a
    assert smooth_target(a,b,4.,4.)==b
    for time in (.1,1.,2.,3.9):
        assert all(min(x,y)<=z<=max(x,y) for x,y,z in zip(a,b,smooth_target(a,b,time,4.)))


def test_command_ownership_rejects_bridge_and_missing_simulator():
    assert not simulator_only([])
    assert simulator_only([SimpleNamespace(node_name='piper_mujoco_sim')])
    assert not simulator_only([SimpleNamespace(node_name='piper_mujoco_sim'),SimpleNamespace(node_name='piper_bridge')])


def test_capture_requested_once_while_discovery_progress_is_delayed(monkeypatch):
    rclpy.init()
    node=SimScanNode(parameter_overrides=[Parameter('config_path',value=str(Path(__file__).parents[1]/'config/mujoco.json'))])
    try:
        monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:100.))
        node.get_subscriptions_info_by_topic=lambda topic:[SimpleNamespace(node_name='piper_mujoco_sim')]
        target=next(iter(node.targets));node.actual=list(node.targets[target])
        node.scene={'paused':False,'simulation_time':1.,'robot':{'joints':[{'limits':[-3.,3.]} for _ in range(6)]}}
        node.scene_received=node.feedback_received=node.discovery_received=100.
        node.session='scan_1'
        node.discovery={'search':{'state':'wait_for_wrist_view','target':target,'viewpoint_index':0,'rejected':0}}
        calls=[]
        future=SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(success=True,message='ok'))
        node.client=SimpleNamespace(service_is_ready=lambda:True,call_async=lambda request:calls.append(request) or future)
        node.command=lambda values:None
        node.tick()
        node.scene['simulation_time']=10.;node.tick()
        node.scene['simulation_time']=11.;node.tick()
        node.tick();node.tick()
        assert len(calls)==1
        node.discovery['search']['rejected']=1
        node.tick()  # retry requires a new settling phase
        assert len(calls)==1
    finally:
        node.destroy_node();rclpy.try_shutdown()


def test_restart_resumes_only_with_discovery_from_new_simulator_epoch():
    from std_msgs.msg import String
    rclpy.init()
    node=SimScanNode(parameter_overrides=[Parameter('config_path',value=str(Path(__file__).parents[1]/'config/mujoco.json'))])
    try:
        def progress(epoch,session):return String(data=json.dumps({'input_mode':'live','sim_restart_id':epoch,
            'workspace':{'session_id':session},'search':{'state':'capture_overhead'}}))
        node.on_scene(String(data=json.dumps({'restart_id':0})))
        node.on_discovery(progress(0,'old'))
        node.on_scene(String(data=json.dumps({'restart_id':1})))
        assert node.fault
        node.on_discovery(progress(0,'old'))
        assert node.fault
        node.on_discovery(progress(1,'new'))
        assert node.fault is None
        assert node.session=='new'
        # New discovery can arrive before the simulator scene callback.
        node.on_discovery(progress(2,'next'))
        assert node.session=='new'
        node.on_scene(String(data=json.dumps({'restart_id':2})))
        node.on_discovery(progress(2,'next'))
        assert node.fault is None
        assert node.session=='next'
    finally:node.destroy_node();rclpy.try_shutdown()
