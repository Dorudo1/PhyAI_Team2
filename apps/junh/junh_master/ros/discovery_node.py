"""Live/recorded fixed-view discovery; no arm command publisher or motion service."""
from concurrent.futures import ThreadPoolExecutor
from collections import deque
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from phyai_discovery.observations import Capture, Intrinsics, decode_depth, load_capture, load_manifest
from phyai_discovery import Discovery, SearchState, Stationarity


def transform_matrix(msg):
    t, q = msg.transform.translation, msg.transform.rotation
    quat = np.array([q.w,q.x,q.y,q.z],dtype=float)
    if not np.isfinite(quat).all() or np.linalg.norm(quat) < 1e-8:
        raise ValueError('Invalid TF quaternion.')
    w,x,y,z = quat/np.linalg.norm(quat)
    matrix = np.eye(4)
    matrix[:3,:3] = [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    matrix[:3,3] = [t.x,t.y,t.z]
    return matrix


class DiscoveryNode(Node):
    def __init__(self, **kwargs):
        super().__init__('junh_discovery', **kwargs)
        self.declare_parameter('config_path',get_package_share_directory('junh_master')+'/config/discovery.json')
        self.declare_parameter('recording_path','')
        config_path = Path(self.get_parameter('config_path').value)
        self.config = json.loads(config_path.read_text())
        recording = self.get_parameter('recording_path').value
        self.recording_entries = None
        if recording:
            manifest,self.recording_base = load_manifest(recording)
            self.recording_entries = manifest['captures']
            self.config.update(map_frame=manifest['map_frame'],viewpoints=manifest['viewpoints'],scope=manifest.get('scope'))
        self.sim_restart_id = None
        self.recording_index = 0
        feature_config = dict(self.config)
        if self.recording_entries is not None:
            feature_config.update(search_criteria=None, inspection_rounds=())
        self.discovery = Discovery(feature_config)
        self.workspace = self.discovery.workspace
        self.search = self.discovery.search
        self.detector = self.discovery.detector
        self.stationarity = Stationarity(self.config.get('stationary_seconds',.5),
            self.config.get('stationary_tolerance',.003),self.config.get('feedback_timeout',1.))
        self.info = {}
        self.rgb = {}
        self.overlay_pubs = {camera:self.create_publisher(Image,f'junh/{camera}/overlay',1)
                             for camera in ('overhead','wrist')}
        self.masks = {}
        self.dynamic_masks = {}
        self.pending_depth = {}
        self.worker = ThreadPoolExecutor(max_workers=1,thread_name_prefix='depth_geometry')
        self.future = None
        self.job = None
        self.request_stamp = self.get_clock().now().nanoseconds/1e9
        self.last_error = None
        self.logged_status = None
        self.last_source_received = {}
        self.data_pub = self.create_publisher(String,'junh/discovery',1)
        self.progress_pub = self.create_publisher(String,'junh/search_progress',1)
        self.camera_request_pub = self.create_publisher(String,self.config['sim_camera_request_topic'],1) if self.config.get('sim_camera_request_topic') and self.recording_entries is None else None
        self.create_service(Trigger,'junh/capture',self.capture_service)
        self.create_service(Trigger,'junh/restart_scan',self.restart_service)
        self.create_service(Trigger,'junh/finish_scan',self.finish_service)
        self.create_service(Trigger,'junh/add_view',self.add_view_service)
        if self.recording_entries is None:
            if self.config.get('sim_camera_request_topic'):
                self.create_subscription(String,'sim/scene',self.on_sim_scene,
                    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf,self)
            self.create_subscription(JointState,'joint_states',self.on_joints,sensor_qos)
            for camera,cfg in self.config['cameras'].items():
                if camera not in ('overhead','wrist'):
                    raise ValueError('Only overhead and wrist camera roles are supported.')
                if cfg.get('mask'):
                    self.masks[camera] = np.load(config_path.parent/cfg['mask'],allow_pickle=False)
                if cfg.get('mask_topic'):
                    self.create_subscription(Image,cfg['mask_topic'],
                        lambda msg,camera=camera:self.on_mask(camera,msg),sensor_qos)
                if cfg.get('rgb_topic'):
                    self.create_subscription(Image,cfg['rgb_topic'],
                        lambda msg,camera=camera:self.on_rgb(camera,msg),sensor_qos)
                self.create_subscription(CameraInfo,cfg['info_topic'],
                    lambda msg,camera=camera:self.on_info(camera,msg),sensor_qos)
                self.create_subscription(Image,cfg['depth_topic'],
                    lambda msg,camera=camera:self.on_depth(camera,msg),sensor_qos)
        self.create_timer(.05,self.tick)
        self.create_timer(.2,self.publish_progress)
        self.create_timer(1.,self.publish)
        self.get_logger().info('Discovery awaits /junh/capture requests from manual or automatic simulation clients.')

    def on_sim_scene(self,msg):
        try:
            restart=json.loads(msg.data)['restart_id']
            if not isinstance(restart,int) or restart<0:return
            if self.sim_restart_id is not None and restart<self.sim_restart_id:return
            changed=self.sim_restart_id is not None and restart!=self.sim_restart_id
            self.sim_restart_id=restart
            if changed:
                self.restart_service(Trigger.Request(),Trigger.Response())
                self.publish_progress()
                self.publish()
                self.get_logger().info('Simulator restarted; cleared map and restarted discovery.')
        except (ValueError,TypeError,KeyError):pass

    def on_joints(self,msg):
        positions = dict(zip(msg.name,msg.position))
        try:
            values = [positions[f'joint{i}'] for i in range(1,8)]
        except KeyError:
            self.stationarity.clear()
            return
        self.stationarity.update(values,time.monotonic())

    def on_mask(self,camera,msg):
        if msg.encoding != 'mono8' or msg.step < msg.width or len(msg.data) != msg.height*msg.step:
            return
        mask=np.ndarray((msg.height,msg.width),dtype=np.uint8,buffer=bytes(msg.data),strides=(msg.step,1)).astype(bool)
        self.dynamic_masks.setdefault(camera,deque(maxlen=4)).append(
            (Time.from_msg(msg.header.stamp).nanoseconds/1e9,msg.header.frame_id,mask))

    def on_rgb(self,camera,msg):
        if msg.encoding not in ('rgb8','bgr8') or msg.step < msg.width*3 or len(msg.data) != msg.height*msg.step:
            return
        rgb = np.ndarray((msg.height,msg.width,3),dtype=np.uint8,buffer=bytes(msg.data),
                         strides=(msg.step,3,1)).copy()
        if msg.encoding == 'bgr8':
            rgb = rgb[:,:,::-1].copy()
        self.rgb[camera] = (Time.from_msg(msg.header.stamp).nanoseconds/1e9,msg.header.frame_id,rgb)

    def on_info(self,camera,msg):
        self.info[camera] = msg

    def on_depth(self,camera,msg):
        now = time.monotonic()
        self.last_source_received[camera] = now
        if self.future is not None or not self.stationarity.ready(now):
            return  # no growing queue and no captures while the arm moves
        if camera == 'overhead' and self.search.state != SearchState.OVERHEAD:
            return
        if camera == 'wrist' and self.search.state != SearchState.CAPTURE:
            return
        stamp = Time.from_msg(msg.header.stamp).nanoseconds/1e9
        age = self.get_clock().now().nanoseconds/1e9-stamp
        if stamp < self.request_stamp or not 0 <= age <= self.config.get('input_max_age',1.):
            self.last_error = 'Image predates capture request or has an invalid/stale timestamp.'
            return
        try:
            info = self.info.get(camera)
            if info is None:
                raise ValueError('Waiting for matching camera calibration.')
            if info.header.frame_id != msg.header.frame_id or (info.width,info.height) != (msg.width,msg.height):
                raise ValueError('Depth image and CameraInfo frame/dimensions differ.')
            cfg = self.config['cameras'][camera]
            if not np.isfinite(list(info.d)).all():
                raise ValueError('Invalid camera distortion calibration.')
            distorted = any(abs(d) > 1e-9 for d in info.d)
            rectified = cfg.get('rectified',True)
            if distorted and (not rectified or info.p[0] <= 0 or info.p[5] <= 0):
                raise ValueError('Distorted depth requires a rectified stream and calibrated projection matrix.')
            p = info.p if rectified else [0.]*12
            k = Intrinsics(info.width,info.height,p[0] if p[0] > 0 else info.k[0],
                p[5] if p[5] > 0 else info.k[4],p[2] if p[0] > 0 else info.k[2],
                p[6] if p[5] > 0 else info.k[5])
            depth = decode_depth(msg.data,msg.width,msg.height,msg.step,msg.encoding,
                bool(msg.is_bigendian),cfg.get('depth_scale',.001))
            tf = self.tf.lookup_transform(self.workspace.frame,msg.header.frame_id,Time.from_msg(msg.header.stamp))
            digest = hashlib.sha256(json.dumps([list(info.k),list(info.p),list(info.d),
                info.header.frame_id,info.width,info.height,cfg.get('depth_scale',.001),
                cfg.get('calibration_revision','unversioned')]).encode()).hexdigest()[:16]
            rgb = self.rgb.get(camera)
            rgb_pixels = rgb[2] if rgb is not None and abs(rgb[0]-stamp) <= .05 and rgb[1] == msg.header.frame_id else None
            if self.detector.segmenter is not None and rgb_pixels is None:
                raise ValueError('Waiting for capture-matched aligned RGB.')
            mask=self.masks.get(camera)
            if cfg.get('mask_topic'):
                matches=[item for item in self.dynamic_masks.get(camera,())
                         if abs(item[0]-stamp)<=.05 and item[1]==msg.header.frame_id]
                dynamic=min(matches,key=lambda item:abs(item[0]-stamp)) if matches else None
                if dynamic is None:
                    raise ValueError('Waiting for the capture-matched robot mask.')
                mask=dynamic[2] if mask is None else mask | dynamic[2]
            capture = Capture(camera,stamp,msg.header.frame_id,self.workspace.frame,digest,k,depth,
                transform_matrix(tf),self.search.target,roi=tuple(cfg['roi']) if cfg.get('roi') else None,
                mask=mask,rgb=rgb_pixels)
            self.pending_depth.pop(camera,None)
            self.submit(capture)
        except (ValueError,TypeError,KeyError,TransformException) as error:
            self.last_error = str(error)
            if isinstance(error,TransformException) or 'capture-matched' in str(error):
                self.pending_depth[camera]=msg
            # Missing TF/calibration can recover before the pending capture times out.

    def submit(self,capture):
        self.job = self.discovery.begin(capture, stationary=True)
        self.future = self.worker.submit(self.detector.detect,capture)
        self.last_error = None

    def capture_service(self,request,response):
        try:
            self.discovery.request_capture(time.monotonic())
            self.request_stamp = self.get_clock().now().nanoseconds/1e9
            response.success = True
            response.message = self.search.reason
        except ValueError as error:
            response.success,response.message = False,str(error)
        return response

    def restart_service(self,request,response):
        self.pending_depth.clear()
        self.discovery.reset()
        self.stationarity.clear()
        self.request_stamp = self.get_clock().now().nanoseconds/1e9
        self.recording_index = 0
        self.last_error = None
        response.success,response.message = True,'Scan restarted; old worker results will be discarded.'
        return response

    def finish_service(self,request,response):
        self.discovery.finish()
        response.success,response.message = True,self.search.reason
        return response

    def add_view_service(self,request,response):
        try:
            self.discovery.add_view()
            response.success,response.message = True,self.search.reason
        except ValueError as error:
            response.success,response.message = False,str(error)
        return response

    def tick(self):
        now = time.monotonic()
        if self.future is None:
            for camera,msg in list(self.pending_depth.items()):
                self.pending_depth.pop(camera,None)
                self.on_depth(camera,msg)
        if self.future is not None and self.future.done():
            future,self.future = self.future,None
            job,self.job = self.job,None
            try:
                detection = future.result()
                if self.discovery.complete(job,detection):
                    self.publish_overlay(job.capture,detection)
            except Exception as error:
                if self.discovery.fail(job,error):
                    self.last_error = str(error)
        if self.recording_entries is not None:
            if self.future is None and self.search.state != SearchState.FINISHED:
                if self.recording_index >= len(self.recording_entries):
                    self.discovery.finish('Recorded observations exhausted.')
                    return
                entry = self.recording_entries[self.recording_index]
                self.recording_index += 1
                try:
                    capture = load_capture(entry,self.recording_base)
                    if entry.get('stationary_verified') is not True:
                        raise ValueError('Recording must verify stationary capture.')
                    if self.search.state == SearchState.WRIST:
                        self.discovery.request_capture(now)
                        self.discovery.tick(now,stationary=True)
                    self.submit(capture)
                except (ValueError,KeyError,OSError) as error:
                    self.search.reject(str(error))
        self.discovery.tick(now,stationary=self.stationarity.ready(now) if self.recording_entries is None else True)

    def publish_overlay(self,capture,detection):
        if capture.rgb is None:
            return
        pixels = capture.rgb.copy()
        for candidate in detection.candidates:
            (x0,y0),(x1,y1) = candidate.pixels
            color = [30,220,60] if candidate.resolved else [255,110,20]
            pixels[y0:y0+2,x0:x1+1] = color
            pixels[max(y0,y1-1):y1+1,x0:x1+1] = color
            pixels[y0:y1+1,x0:x0+2] = color
            pixels[y0:y1+1,max(x0,x1-1):x1+1] = color
        msg = Image()
        msg.header.frame_id = capture.optical_frame
        msg.header.stamp = Time(nanoseconds=int(capture.stamp*1e9)).to_msg()
        msg.height,msg.width = pixels.shape[:2]
        msg.encoding,msg.step = 'rgb8',msg.width*3
        msg.data = pixels.tobytes()
        self.overlay_pubs[capture.camera].publish(msg)

    def publish_progress(self):
        mode='live' if self.recording_entries is None else 'recorded'
        self.progress_pub.publish(String(data=json.dumps({
            'workspace':{'session_id':self.workspace.session},
            'search':self.search.progress(),'input_mode':mode,'last_error':self.last_error,
            'sim_restart_id':self.sim_restart_id})))
        if self.camera_request_pub is not None:
            camera=None
            if self.stationarity.ready(time.monotonic()) and self.future is None:
                if self.search.state == SearchState.OVERHEAD:camera='overhead'
                elif self.search.state == SearchState.CAPTURE:camera='wrist'
            self.camera_request_pub.publish(String(data=json.dumps({'camera':camera})))

    def publish(self):
        stamp = self.get_clock().now().nanoseconds/1e9 if self.recording_entries is None else None
        envelope = {**self.discovery.snapshot(stamp),
                    'input_mode':'live' if self.recording_entries is None else 'recorded',
                    'sim_restart_id':self.sim_restart_id,
                    'last_error':self.last_error,
                    'stationary_feedback_ready':self.stationarity.ready(time.monotonic()) if self.recording_entries is None else None,
                    'camera_stream_ages':{camera:time.monotonic()-received for camera,received in self.last_source_received.items()}}
        status = (self.search.state.value,self.search.accepted,self.search.rejected,self.last_error)
        if status != self.logged_status:
            self.get_logger().info(f'{self.search.state.value}: {self.search.reason} '
                                   f'({len(self.workspace.tracks)} tracks); input: {self.last_error or "ready/waiting"}')
            self.logged_status = status
        self.data_pub.publish(String(data=json.dumps(envelope,allow_nan=False)))

    def destroy_node(self):
        self.worker.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    from .runtime import run
    run(DiscoveryNode,args)
