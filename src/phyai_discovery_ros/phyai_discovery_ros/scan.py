"""Shared PIPER joint-position scan execution for simulation and hardware bridges."""
import json
import math
import time
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


NAMES = [f'joint{i}' for i in range(1,8)]


def smooth_target(initial, target, elapsed, duration):
    t = min(1.,max(0.,elapsed/duration))
    blend = t*t*(3-2*t)
    return [float(a+blend*(b-a)) for a,b in zip(initial,target)]


def move_duration(initial, target, minimum, max_speed, max_acceleration):
    """Bound the peak speed and acceleration of the cubic joint trajectory."""
    distance = max(abs(a-b) for a,b in zip(initial[:6],target[:6]))
    return max(minimum, 1.5*distance/max_speed,
               math.sqrt(6*distance/max_acceleration))


class ScanNode(Node):
    def __init__(self, *, node_name='discovery_scan', config_package='phyai_discovery_ros',
                 config_name='discovery.json', scan_key='scan', interface_prefix='discovery', **kwargs):
        super().__init__(node_name, **kwargs)
        self.declare_parameter('config_path','')
        self.declare_parameter('command_topic','joint_ctrl_single')
        self.declare_parameter('joint_states_topic','joint_states')
        self.command_topic=self.get_parameter('command_topic').value
        config=json.loads(Path(self.get_parameter('config_path').value or
            get_package_share_directory(config_package)+'/config/'+config_name).read_text())
        scan=config.get(scan_key)
        if not isinstance(scan,dict):
            raise ValueError('Automatic scanning requires a '+scan_key+' configuration with installation joint targets.')
        self.config=config
        self.scan=scan
        self.limits=scan.get('joint_limits')
        if scan_key=='scan':
            limits=np.asarray(self.limits,dtype=float)
            if limits.shape!=(6,2) or not np.isfinite(limits).all() or np.any(limits[:,0]>=limits[:,1]):
                raise ValueError('Scan requires six finite joint_limits [lower, upper] pairs.')
        self.targets=scan['targets']
        self.survey_labels=set(config['viewpoints'])
        self.targeted_inspection=bool(scan.get('targeted_inspection',False))
        if scan_key=='scan' and self.targeted_inspection:
            raise ValueError('Shared scan uses configured poses; dynamic inspection requires a geometry planner.')
        self.max_views_per_block=scan.get('max_targeted_views_per_block',2)
        if not isinstance(self.max_views_per_block,int) or isinstance(self.max_views_per_block,bool) or self.max_views_per_block<1:
            raise ValueError('Targeted per-object view budget must be a positive integer.')
        self.planner=None
        self.selected_targets={}
        self.inspection_focus=None
        self.duration=float(scan.get('move_seconds',4.))
        self.approach_duration=float(scan.get('approach_seconds',8.))
        self.settle_seconds=float(scan.get('settle_seconds',.7))
        self.max_speed=float(scan.get('max_joint_speed',1.2))
        self.max_acceleration=float(scan.get('max_joint_acceleration',3.))
        if not all(math.isfinite(v) and v>0 for v in (self.duration,self.approach_duration,
                       self.settle_seconds,self.max_speed,self.max_acceleration)):
            raise ValueError('Scan timing must be positive.')
        labels = config['viewpoints'] + [name for views in config.get('inspection_rounds',[]) for name in views]
        if set(self.targets)!=set(labels):
            raise ValueError('Every scan viewpoint requires a joint target.')
        for values in self.targets.values():
            if len(values)!=7 or not np.isfinite(values).all():
                raise ValueError('Scan targets must contain seven finite joints.')
        self.actual=None;self.feedback_received=None
        self.discovery=None;self.discovery_received=None
        self.session=None;self.active=None;self.initial=None;self.started=None
        self.settled_since=None;self.capture_future=None;self.fault=None
        self.moved=False;self.fault_session=None
        self.wait_reason='Waiting for bridge feedback and discovery startup.'
        self.logged_wait=None
        self.client=self.create_client(Trigger,f'{interface_prefix}/capture')
        self.finish_client=self.create_client(Trigger,f'{interface_prefix}/end_inspection')
        self.finish_future=None
        self.pub=self.create_publisher(JointState,self.command_topic,1)
        self.status_pub=self.create_publisher(String,f'{interface_prefix}/scan_status',1)
        self.stop_client=self.create_client(Trigger,f'{interface_prefix}/finish_scan')
        self.create_service(Trigger,f'{interface_prefix}/stop_scan',self.stop_service)
        self.create_subscription(JointState,self.get_parameter('joint_states_topic').value,self.on_joints,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(String,f'{interface_prefix}/search_progress',self.on_discovery,1)
        self.create_timer(.02,self.tick)
        self.create_timer(.5,self.publish_status)
        self.get_logger().info('PIPER scan enabled using JointState commands and measured arrival feedback.')

    def clear_motion(self):
        self.active=self.initial=self.started=self.settled_since=None
        self.capture_future=None
        self.request_key=None

    def motion_time(self):
        return time.monotonic()

    def environment_ready(self, now):
        return True

    def target_limits(self):
        return self.limits

    def accept_progress(self, data):
        return True

    def can_command(self):
        if not self.get_subscriptions_info_by_topic(self.command_topic):
            return False
        own=(self.get_name(),self.get_namespace())
        return all((e.node_name,e.node_namespace)==own
                   for e in self.get_publishers_info_by_topic(self.command_topic))

    def stop_service(self, request, response):
        self.fault_stop('Scan cancelled by client.')
        if self.stop_client.service_is_ready():
            self.stop_future=self.stop_client.call_async(Trigger.Request())
        response.success=True
        response.message='Scan command stream stopped; bridge remains responsible for hardware stop handling.'
        return response

    def on_joints(self,msg):
        values=dict(zip(msg.name,msg.position))
        if all(name in values and math.isfinite(values[name]) for name in NAMES):
            self.actual=[values[name] for name in NAMES]
            self.feedback_received=time.monotonic()
        else:
            self.actual=None
            if self.active:self.fault_stop('Incomplete or invalid PIPER feedback.')

    def on_discovery(self,msg):
        try:
            data=json.loads(msg.data)
            if data.get('input_mode')!='live':return
            if not self.accept_progress(data):return
            session=data['workspace']['session_id']
            if self.session != session:
                self.clear_motion();self.moved=False
                self.selected_targets={};self.inspection_focus=None
                self.finish_future=None
                if self.planner is not None:self.planner.reset()
                if self.fault_session != session:self.fault=None
            self.session=session
            self.discovery=data;self.discovery_received=time.monotonic()
        except (ValueError,KeyError,TypeError):
            self.discovery=None
            if self.active:self.fault_stop('Invalid discovery progress.')

    def fault_stop(self,reason):
        self.fault=reason;self.fault_session=self.session
        self.clear_motion()
        self.get_logger().error(reason)
        # A measured-position hold uses the agreed position-command contract.
        # Hardware emergency stopping remains the bridge's responsibility.
        if (self.actual is not None and self.feedback_received is not None
                and 0<=time.monotonic()-self.feedback_received<=1. and self.can_command()):
            self.command(self.actual)

    def command(self,target):
        msg=JointState()
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.name=NAMES;msg.position=target
        self.pub.publish(msg)

    def select_target(self,label):
        # Fixed, installation-validated survey and reserve poses. Environment
        # extensions may override this to choose additional perceived-target views.
        return self.targets[label]

    def tick(self):
        now=time.monotonic()
        if self.fault:return
        if not self.environment_ready(now):return
        if self.actual is None:
            if self.active:self.fault_stop('Joint feedback is unavailable.')
            self.wait_reason='Waiting for complete /joint_states.';return
        if self.discovery is None:
            if self.active:self.fault_stop('Discovery progress is unavailable.')
            self.wait_reason='Waiting for discovery search progress.';return
        if not self.can_command():
            self.wait_reason='Waiting for a bridge subscriber and exclusive command publishing.'
            if self.active:self.fault_stop('Command publishing conflict or missing bridge; scan halted.')
            return
        if now-self.feedback_received>1. or now-self.discovery_received>3.:
            self.wait_reason='Waiting for fresh feedback and search progress.'
            if self.active:self.fault_stop('Joint feedback or discovery became stale.')
            return
        progress=self.discovery['search']
        if progress['state']=='search_finished':
            if self.active:self.command(self.actual)
            self.wait_reason='Search completed.'
            self.clear_motion();return
        if self.finish_future is not None:
            self.wait_reason='Waiting for targeted inspection to finish.'
            if self.finish_future.done():
                try:
                    result=self.finish_future.result()
                    if not result.success:self.fault_stop('Inspection stop rejected: '+result.message)
                except Exception as error:self.fault_stop(str(error))
            return
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
        target=self.select_target(target_name)
        if target is None:
            self.wait_reason='Targeted views exhausted; preserving unresolved evidence.'
            if self.finish_client.service_is_ready():
                self.finish_future=self.finish_client.call_async(Trigger.Request())
            return
        if self.active is None:
            limits=self.target_limits()
            if any(not low<=angle<=high for angle,(low,high) in zip(target[:6],limits)) or not 0<=target[6]<=.035:
                self.fault_stop('Scan target exceeds PIPER joint limits.');return
            self.active=target_name;self.initial=list(self.actual)
            self.started=self.motion_time();self.settled_since=None
            self.active_duration=move_duration(self.initial,target,
                self.duration if self.moved else self.approach_duration,
                self.max_speed,self.max_acceleration)
            self.wait_reason='Moving to '+target_name
            self.get_logger().info(f'Moving to {target_name} ({self.active_duration:.2f}s trajectory)')
        elapsed=self.motion_time()-self.started
        self.command(smooth_target(self.initial,target,elapsed,self.active_duration))
        error=max(abs(a-b) for a,b in zip(self.actual[:6],target[:6]))
        gripper_error=abs(self.actual[6]-target[6])
        if elapsed>=self.active_duration and error<.025 and gripper_error<.002:
            self.wait_reason='Settling at '+target_name
            if self.settled_since is None:self.settled_since=self.motion_time()
            if self.motion_time()-self.settled_since>=self.settle_seconds and self.client.service_is_ready():
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
        status={'reason':reason,'mode':'piper_joint_position','viewpoint':self.active,'fault':self.fault,
                'inspection_focus':self.inspection_focus,
                'state':'fault' if self.fault else 'moving_or_capturing' if self.active else 'waiting'}
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    from .runtime import run
    run(ScanNode,args)
