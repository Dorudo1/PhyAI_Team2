"""Fixed joint-space swivel scan for MuJoCo only, separate from discovery logic."""
import json
import math
import time

import numpy as np
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


NAMES = [f'joint{i}' for i in range(1,8)]


def smooth_target(initial, target, elapsed, duration):
    t = min(1.,max(0.,elapsed/duration))
    blend = t*t*(3-2*t)
    return [float(a+blend*(b-a)) for a,b in zip(initial,target)]


def simulator_only(endpoints):
    """Refuse forwarding if command-topic subscribers include anything else."""
    return bool(endpoints) and all(e.node_name == 'piper_mujoco_sim' for e in endpoints)


class SimScanNode(Node):
    def __init__(self, **kwargs):
        super().__init__('junh_sim_scan', **kwargs)
        self.declare_parameter('config_path',get_package_share_directory('junh_master')+'/config/mujoco.json')
        from pathlib import Path
        config=json.loads(Path(self.get_parameter('config_path').value).read_text())
        scan=config['sim_scan']
        self.targets=scan['targets']
        self.duration=float(scan.get('move_seconds',4.))
        self.approach_duration=float(scan.get('approach_seconds',8.))
        self.settle_seconds=float(scan.get('settle_seconds',.7))
        if not all(math.isfinite(v) and v>0 for v in (self.duration,self.approach_duration,self.settle_seconds)):
            raise ValueError('Scan timing must be positive.')
        labels = config['viewpoints'] + [name for views in config.get('inspection_rounds',[]) for name in views]
        if set(self.targets)!=set(labels):
            raise ValueError('Every scan viewpoint requires a joint target.')
        for values in self.targets.values():
            if len(values)!=7 or not np.isfinite(values).all():
                raise ValueError('Scan targets must contain seven finite joints.')
        self.scene=None;self.scene_received=None;self.restart_id=None
        self.actual=None;self.feedback_received=None
        self.discovery=None;self.discovery_received=None
        self.session=None;self.active=None;self.initial=None;self.started=None
        self.settled_since=None;self.capture_future=None;self.fault=None
        self.moved=False;self.fault_session=None
        self.wait_reason='Waiting for simulator and search startup.'
        self.logged_wait=None
        self.client=self.create_client(Trigger,'junh/capture')
        self.pub=self.create_publisher(JointState,'joint_ctrl_single',1)
        self.status_pub=self.create_publisher(String,'junh/scan_status',1)
        self.create_subscription(String,'sim/scene',self.on_scene,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState,'joint_states',self.on_joints,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(String,'junh/search_progress',self.on_discovery,1)
        self.create_timer(.02,self.tick)
        self.create_timer(.5,self.publish_status)
        self.get_logger().info('Automatic MuJoCo swivel search enabled; no hardware execution support.')

    def clear_motion(self):
        self.active=self.initial=self.started=self.settled_since=None
        self.capture_future=None
        self.request_key=None

    def on_scene(self,msg):
        try:
            scene=json.loads(msg.data)
            restart=scene['restart_id']
            if self.restart_id is not None and restart<self.restart_id:return
            if self.restart_id is not None and restart!=self.restart_id:
                self.clear_motion()
                self.fault='Simulator restarted: restart discovery before resuming the scan.'
                self.fault_session=self.session
                self.actual=None
            self.restart_id=restart
            self.scene=scene;self.scene_received=time.monotonic()
        except (ValueError,KeyError,TypeError):self.scene=None

    def on_joints(self,msg):
        values=dict(zip(msg.name,msg.position))
        if all(name in values and math.isfinite(values[name]) for name in NAMES):
            self.actual=[values[name] for name in NAMES]
            self.feedback_received=time.monotonic()
        else:self.actual=None

    def on_discovery(self,msg):
        try:
            data=json.loads(msg.data)
            if data.get('input_mode')!='live':return
            if 'sim_restart_id' in data and data['sim_restart_id']!=self.restart_id:return
            if self.fault=='Simulator restarted: restart discovery before resuming the scan.' and data.get('sim_restart_id')==self.restart_id:
                self.fault=None;self.fault_session=None
            session=data['workspace']['session_id']
            if self.session != session:
                self.clear_motion();self.moved=False
                if self.fault_session != session:self.fault=None
            self.session=session
            self.discovery=data;self.discovery_received=time.monotonic()
        except (ValueError,KeyError,TypeError):self.discovery=None

    def fault_stop(self,reason):
        self.fault=reason;self.fault_session=self.session
        self.clear_motion()
        self.get_logger().error(reason)
        # MuJoCo position control: hold the measured pose if simulator ownership
        # is still exclusive. This is not a hardware stop implementation.
        if self.actual is not None and simulator_only(self.get_subscriptions_info_by_topic('joint_ctrl_single')):
            self.command(self.actual)

    def command(self,target):
        msg=JointState()
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.name=NAMES;msg.position=target
        self.pub.publish(msg)

    def tick(self):
        now=time.monotonic()
        if self.fault:return
        if self.scene is None:
            self.wait_reason='Waiting for /sim/scene.';return
        if self.actual is None:
            self.wait_reason='Waiting for complete /joint_states.';return
        if self.discovery is None:
            self.wait_reason='Waiting for /junh/search_progress.';return
        endpoints=self.get_subscriptions_info_by_topic('joint_ctrl_single')
        if not simulator_only(endpoints):
            self.wait_reason='Waiting for exclusive MuJoCo command ownership; subscribers: '+(', '.join(e.node_name for e in endpoints) or 'none')
            if self.active:self.fault_stop('Command ownership changed; scan halted.')
            return
        if now-self.scene_received>1. or now-self.feedback_received>1. or now-self.discovery_received>3.:
            self.wait_reason='Waiting for fresh simulator, feedback, and search progress.'
            if self.active:self.fault_stop('Simulator, joint feedback, or discovery became stale.')
            return
        if self.scene.get('paused',False):
            self.wait_reason='Simulator paused.';return
        progress=self.discovery['search']
        if progress['state']=='search_finished':
            if self.active:self.command(self.actual)
            self.wait_reason='Search completed.'
            self.clear_motion();return
        target_name=progress['target']
        if self.active is not None and target_name!=self.active:
            self.clear_motion()
        if self.capture_future is not None:
            self.wait_reason='Waiting for capture acknowledgement.'
            if self.capture_future.done():
                try:
                    result=self.capture_future.result()
                    if not result.success:self.fault_stop('Capture request rejected: '+result.message)
                except Exception as error:self.fault_stop(str(error))
                self.capture_future=None
            return
        if progress['state']!='wait_for_wrist_view':
            self.wait_reason='Discovery: '+progress['state'];return
        key=(self.session,progress['viewpoint_index'],progress['rejected'])
        if getattr(self,'request_key',None)==key:return
        if getattr(self,'request_key',None) is not None:
            self.clear_motion()
        if target_name not in self.targets:
            self.fault_stop('No configured scan pose for '+target_name);return
        target=self.targets[target_name]
        if self.active is None:
            limits=[joint['limits'] for joint in self.scene['robot']['joints']]
            if any(not low<=angle<=high for angle,(low,high) in zip(target[:6],limits)) or not 0<=target[6]<=.035:
                self.fault_stop('Scan target exceeds simulator joint limits.');return
            self.active=target_name;self.initial=list(self.actual)
            self.started=self.scene['simulation_time'];self.settled_since=None
            self.active_duration=self.duration if self.moved else self.approach_duration
            self.wait_reason='Moving to '+target_name
            self.get_logger().info('Swiveling to '+target_name)
        # Use scene time; pauses freeze interpolation instead of skipping ahead.
        elapsed=self.scene['simulation_time']-self.started
        self.command(smooth_target(self.initial,target,elapsed,self.active_duration))
        error=max(abs(a-b) for a,b in zip(self.actual[:6],target[:6]))
        if elapsed>=self.active_duration and error<.025:
            self.wait_reason='Settling at '+target_name
            if self.settled_since is None:self.settled_since=self.scene['simulation_time']
            if self.scene['simulation_time']-self.settled_since>=self.settle_seconds and self.client.service_is_ready():
                self.request_key=key
                self.capture_future=self.client.call_async(Trigger.Request())
                self.moved=True
                self.wait_reason='Capturing '+target_name
                self.get_logger().info('Capturing '+target_name)
        else:self.settled_since=None
        if elapsed>self.active_duration+10.:
            self.fault_stop('Arm failed to settle at '+target_name)

    def publish_status(self):
        reason=self.fault or self.wait_reason
        if reason != self.logged_wait:
            self.get_logger().info(reason);self.logged_wait=reason
        status={'reason':reason,'mode':'mujoco_only','viewpoint':self.active,'fault':self.fault,
                'state':'fault' if self.fault else 'moving_or_capturing' if self.active else 'waiting'}
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    from .runtime import run
    run(SimScanNode,args)
