"""MuJoCo geometry and clock extension of the shared PIPER scan controller."""
import json
import time

from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

from phyai_discovery_ros.scan import ScanNode, NAMES, move_duration, smooth_target


def simulator_only(endpoints):
    return bool(endpoints) and all(e.node_name=='piper_mujoco_sim' for e in endpoints)


class SimScanNode(ScanNode):
    def __init__(self, **kwargs):
        super().__init__(node_name='junh_sim_scan', config_package='junh_master',
                         config_name='mujoco.json', scan_key='sim_scan',
                         interface_prefix='junh', **kwargs)
        self.scene=None;self.scene_received=None;self.restart_id=None
        self.declare_parameter('robot_path','/opt/mujoco_menagerie/agilex_piper/piper.xml')
        self.declare_parameter('scene_config',get_package_share_directory('piper_mujoco_sim')+'/config/lab_scene.json')
        self.create_subscription(String,'sim/scene',self.on_scene,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def can_command(self):
        return (simulator_only(self.get_subscriptions_info_by_topic(self.command_topic))
                and super().can_command())

    def motion_time(self):
        return self.scene['simulation_time']

    def target_limits(self):
        return [joint['limits'] for joint in self.scene['robot']['joints']]

    def environment_ready(self, now):
        if self.scene is None:
            self.wait_reason='Waiting for /sim/scene.'
            return False
        if now-self.scene_received>1.:
            self.wait_reason='Waiting for fresh simulator scene.'
            if self.active:self.fault_stop('Simulator scene became stale.')
            return False
        if self.scene.get('paused',False):
            self.wait_reason='Simulator paused.'
            return False
        return True

    def accept_progress(self, data):
        if 'sim_restart_id' in data and data['sim_restart_id']!=self.restart_id:
            return False
        if self.fault=='Simulator restarted: restart discovery before resuming the scan.':
            self.fault=None;self.fault_session=None
        return True

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
            if self.planner is not None and scene.get('scan_geometry'):
                self.planner.sync(scene['scan_geometry'])
        except (ValueError,KeyError,TypeError):self.scene=None

    def select_target(self,label):
        if label in self.selected_targets:
            return self.selected_targets[label]
        target=self.targets[label]
        self.inspection_focus=None
        if self.targeted_inspection and label not in self.survey_labels and self.scene.get('scan_geometry'):
            if self.planner is None:
                import mujoco
                from piper_mujoco_sim.inspection import InspectionPlanner
                from piper_mujoco_sim.scene import build_scene
                model=mujoco.MjModel.from_xml_string(build_scene(
                    self.get_parameter('robot_path').value,self.get_parameter('scene_config').value))
                self.planner=InspectionPlanner(model,list(self.targets.values()),self.max_views_per_block)
                self.planner.sync(self.scene['scan_geometry'])
                for name,joints in self.selected_targets.items():self.planner.remember(name,joints)
            focus=self.discovery['workspace'].get('inspection_targets',[])
            plan=self.planner.choose(focus,self.actual,label)
            if plan:
                target=plan['joints'];self.inspection_focus=plan
                self.get_logger().info(f'Inspection {label}: focus {plan["focus"]} at {plan["position"]}')
            else:
                if focus:
                    self.selected_targets[label]=None
                    self.get_logger().info(f'Inspection {label}: no clear novel targeted pose remains; stopping partial.')
                    return None
                self.get_logger().info(f'Inspection {label}: no uncertain objects; using coverage reserve.')
        self.selected_targets[label]=list(target)
        if self.planner is not None:self.planner.remember(label,target)
        return self.selected_targets[label]


def main(args=None):
    from .runtime import run
    run(SimScanNode,args)
