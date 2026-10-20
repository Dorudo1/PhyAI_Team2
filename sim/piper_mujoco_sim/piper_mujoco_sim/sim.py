import os

# Render offscreen on the GPU; no display is needed, so local and remote users see the same browser view
os.environ.setdefault('MUJOCO_GL', 'egl')

import io  # noqa: E402
import json  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
from http.server import ThreadingHTTPServer

from ament_index_python.packages import get_package_share_directory
from .scatter import BlockScatter
from .scene import build_scene, scene_snapshot, reset_scene  # noqa: E402
from .gizmo import draw_axis_gizmo  # noqa: E402
from .editor import SceneEditor  # noqa: E402
from .web import FrameHub, make_handler  # noqa: E402

import mujoco  # noqa: E402
import rclpy  # noqa: E402
from PIL import Image  # noqa: E402
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError  # noqa: E402
from rclpy.qos import QoSProfile, DurabilityPolicy  # noqa: E402
import numpy as np  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image as RosImage, JointState
from .camera_ros import camera_info, image_msg, transform  # noqa: E402  # noqa: E402
from std_msgs.msg import String  # noqa: E402


class PiperMujocoSim(Node):
    """MuJoCo stand-in for the control PC host stack (piper_bridge).

    Publishes the arm's joint states and drives the simulated arm from joint position
    commands, over the same topics apps use on the real robot. The scene is streamed
    to a browser at http://<dev-pc>:<http_port>.
    """

    def __init__(self):
        super().__init__('piper_mujoco_sim')
        self.declare_parameter('model_path', '')
        self.declare_parameter('robot_path', '/opt/mujoco_menagerie/agilex_piper/piper.xml')
        self.declare_parameter('scene_config', get_package_share_directory('piper_mujoco_sim')
                               + '/config/lab_scene.json')
        self.declare_parameter('state_topic', 'joint_states')
        self.declare_parameter('command_topic', 'joint_ctrl_single')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('http_port', 23517)
        self.declare_parameter('workspace_port', 0)
        self.declare_parameter('scene_editing', False)
        self.declare_parameter('scatter_on_start', False)
        self.declare_parameter('scatter_seed', -1)
        self.declare_parameter('scatter_table', 'all')
        self.declare_parameter('view_fps', 30.0)
        self.declare_parameter('view_width', 960)
        self.declare_parameter('view_height', 720)
        self.declare_parameter('publish_cameras', False)
        self.declare_parameter('camera_rate', 5.0)
        self.declare_parameter('camera_width', 320)
        self.declare_parameter('camera_height', 240)
        self.declare_parameter('camera_on_demand', False)
        param = lambda name: self.get_parameter(name).value

        self.model = (mujoco.MjModel.from_xml_path(param('model_path')) if param('model_path')
                      else mujoco.MjModel.from_xml_string(
                          build_scene(param('robot_path'), param('scene_config'))))
        self.data = mujoco.MjData(self.model)
        home = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, 'home')
        if home >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, home)
        mujoco.mj_forward(self.model, self.data)

        self.joints = [self.model.joint(f'joint{i}') for i in range(1, 9)]
        # Command names -> actuator index; the gripper actuator also answers to 'joint7'
        self.actuators = {self.model.actuator(i).name: i for i in range(self.model.nu)}
        if 'gripper' in self.actuators:
            self.actuators.setdefault('joint7', self.actuators['gripper'])

        self.lock = threading.RLock()
        self.restart_id = 0
        self.command_cutoff = 0
        self.editor = (None if param('model_path') else
                       SceneEditor(self.model, self.data, self.lock, param('scene_config')))
        self.block_scatter = (BlockScatter(self.model, self.data, self.editor.config)
                              if self.editor else None)
        if self.block_scatter:
            seed = param('scatter_seed')
            if seed == -1:
                seed = None
            self.block_scatter.prepare_initial_scene(
                self.editor.config, scatter=param('scatter_on_start'),
                seed=seed, table=param('scatter_table'))
            if param('scatter_on_start'):
                self.get_logger().info(
                    f'Scattered blocks before simulation startup (seed={seed}, '
                    f'table={param("scatter_table")}); reset restores the aligned layout.')
        self.pub = self.create_publisher(JointState, param('state_topic'), 10)
        if self.editor:
            self.scene_pub = self.create_publisher(String, 'sim/scene',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_timer(.2, self.publish_scene)
        self.create_subscription(JointState, param('command_topic'), self.on_command, 10)
        self.publish_period = 1.0 / param('publish_rate')

        self.hubs = {name: FrameHub() for name in ('overview', 'overhead', 'wrist')}
        self.view_period = 1.0 / param('view_fps')
        self.view_size = (param('view_width'), param('view_height'))
        self.camera_publishers = {}
        self.camera_request = None
        self.camera_request_received = 0.
        self.camera_on_demand = bool(param('camera_on_demand'))
        self.sensor_size = (int(param('camera_width')),int(param('camera_height')))
        if min(self.sensor_size)<=0:
            raise ValueError('Camera dimensions must be positive.')
        if param('publish_cameras'):
            if param('camera_rate') <= 0:
                raise ValueError('camera_rate must be positive.')
            self.camera_period = 1.0 / param('camera_rate')
            self.camera_tf = TransformBroadcaster(self)
            self.create_subscription(String,'sim/camera_request',self.on_camera_request,1)
            for name in ('overhead', 'wrist'):
                self.camera_publishers[name] = {
                    'depth': self.create_publisher(RosImage, f'{name}/depth/image_rect_raw', 1),
                    'rgb': self.create_publisher(RosImage, f'{name}/color/image_rect_raw', 1),
                    'info': self.create_publisher(CameraInfo, f'{name}/depth/camera_info', 1),
                    'mask': self.create_publisher(RosImage, f'{name}/robot_mask', 1)}
            base_id = self.model.body('base_link').id
            robot_bodies = []
            for body_id in range(self.model.nbody):
                ancestor = body_id
                while ancestor > 0 and ancestor != base_id:
                    ancestor = self.model.body_parentid[ancestor]
                if ancestor == base_id:
                    robot_bodies.append(body_id)
            self.robot_geom_ids = np.flatnonzero(np.isin(self.model.geom_bodyid, robot_bodies))
        self.server = ThreadingHTTPServer(('0.0.0.0', param('http_port')),
                                         make_handler(self.hubs, self.editor, param('scene_editing'),
                                                      restart=self.restart_simulation,workspace_port=int(param('workspace_port')),
                                                      block_action=self.change_blocks if self.editor else None))
        self.server.daemon_threads = True

        self.get_logger().info(
            f"command: /{param('command_topic')}  state: /{param('state_topic')}  "
            f"view: http://localhost:{self.server.server_port} "
            f"editor: http://localhost:{self.server.server_port}/edit")
        for target in (self.run_physics, self.run_view, self.server.serve_forever):
            threading.Thread(target=target, daemon=True).start()

    def on_camera_request(self,msg):
        try:
            camera=json.loads(msg.data)['camera']
            if camera not in (None,'overhead','wrist'):return
            self.camera_request=camera
            self.camera_request_received=time.monotonic()
        except (ValueError,TypeError,KeyError):pass

    def sensor_requested(self,name,now):
        if not self.camera_on_demand:return True
        return self.camera_request==name and now-self.camera_request_received<=1.

    def on_command(self, msg):
        with self.lock:
            stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
            if self.restart_id and (self.data.time < .25 or (stamp and stamp < self.command_cutoff)):
                return  # Let the example receive the reset before accepting new commands.
            if self.editor and self.editor.paused:
                return
            for name, position in zip(msg.name, msg.position):
                if name in self.actuators:
                    self.data.ctrl[self.actuators[name]] = position

    def publish_scene(self):
        with self.lock:
            snapshot = scene_snapshot(self.model, self.data, self.editor.config,
                                      self.editor.paused, self.restart_id)
        self.scene_pub.publish(String(data=json.dumps(snapshot)))

    def restart_simulation(self, block_poses=None):
        with self.lock:
            reset_scene(self.model, self.data, self.editor.config if self.editor else None)
            if self.block_scatter:
                if block_poses is None:
                    self.block_scatter.restore()
                else:
                    self.block_scatter.apply(block_poses)
            if self.editor:
                self.editor.paused = False
            self.restart_id += 1
            self.command_cutoff = self.get_clock().now().nanoseconds
            restart_id = self.restart_id
        self.publish_state()
        if self.editor:
            self.publish_scene()
        return {'restart_id': restart_id, 'message': 'Simulation restarted.'}

    def change_blocks(self, action, payload):
        with self.lock:
            if action == 'scatter':
                poses = self.block_scatter.plan(self.editor.config,
                    seed=payload.get('seed'), table=payload.get('table', 'all'))
                self.block_scatter.remember()
                result = self.restart_simulation(block_poses=poses)
                result['message'] = 'Blocks scattered and settled; original poses retained for reset.'
            elif action == 'reset':
                result = self.restart_simulation()
                result['message'] = 'Original block positions and rotations restored.'
            else:
                raise ValueError('Unknown block action.')
            result['blocks'] = [pose.tolist() for pose in self.block_scatter.poses(self.data)]
            return result

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            for joint in self.joints:
                msg.name.append(joint.name)
                msg.position.append(float(self.data.qpos[joint.qposadr[0]]))
                msg.velocity.append(float(self.data.qvel[joint.dofadr[0]]))
        self.pub.publish(msg)

    def run_physics(self):
        """Steps physics in real time and publishes joint states at publish_rate."""
        dt = self.model.opt.timestep
        steps_per_publish = max(1, round(self.publish_period / dt))
        start, step = time.monotonic(), 0
        while rclpy.ok():
            with self.lock:
                paused = self.editor is not None and self.editor.paused
                if not paused:
                    mujoco.mj_step(self.model, self.data)
            if paused:
                self.publish_state()
                time.sleep(self.publish_period)
                start = time.monotonic() - step * dt
                continue
            step += 1
            if step % steps_per_publish == 0:
                self.publish_state()
            ahead = start + step * dt - time.monotonic()
            if ahead > 0:
                time.sleep(ahead)

    def publish_camera(self,renderer,name):
        width,height=self.sensor_size
        with self.lock:
            renderer.update_scene(self.data,name)
            stamp=self.get_clock().now().to_msg()
            camera_id=self.model.camera(name).id
            optical_rotation=self.data.cam_xmat[camera_id].reshape(3,3)@np.diag([1.,-1.,-1.])
            q=np.empty(4)
            mujoco.mju_mat2Quat(q,optical_rotation.reshape(-1))
            camera_tf=transform(self.data.cam_xpos[camera_id].copy(),q,'world',f'{name}_optical',stamp)
            base_q=np.empty(4)
            mujoco.mju_mat2Quat(base_q,self.data.body('base_link').xmat.copy())
            base_tf=transform(self.data.body('base_link').xpos.copy(),base_q,'world','base_link',stamp)
            fovy=float(self.model.cam_fovy[camera_id])
        rgb=renderer.render().copy()
        renderer.enable_depth_rendering()
        try:depth=renderer.render().astype(np.float32)
        finally:renderer.disable_depth_rendering()
        renderer.enable_segmentation_rendering()
        try:segments=renderer.render()
        finally:renderer.disable_segmentation_rendering()
        mask=(np.isin(segments[:,:,0],self.robot_geom_ids)&
              (segments[:,:,1]==mujoco.mjtObj.mjOBJ_GEOM)).astype(np.uint8)
        try:
            self.camera_tf.sendTransform([base_tf,camera_tf])
            pubs=self.camera_publishers[name]
            pubs['mask'].publish(image_msg(mask,'mono8',f'{name}_optical',stamp))
            pubs['info'].publish(camera_info(width,height,fovy,f'{name}_optical',stamp))
            pubs['rgb'].publish(image_msg(rgb,'rgb8',f'{name}_optical',stamp))
            pubs['depth'].publish(image_msg(depth,'32FC1',f'{name}_optical',stamp))
        except RCLError:
            if rclpy.ok():raise

    def run_view(self):
        """Prioritize browser frames; render small sensor frames only when needed."""
        width,height=self.view_size
        self.model.vis.global_.offwidth=max(self.model.vis.global_.offwidth,width,self.sensor_size[0])
        self.model.vis.global_.offheight=max(self.model.vis.global_.offheight,height,self.sensor_size[1])
        renderer=None;sensor=None
        try:
            renderer=mujoco.Renderer(self.model,height=height,width=width)
            if self.camera_publishers:
                sensor=mujoco.Renderer(self.model,height=self.sensor_size[1],width=self.sensor_size[0])
            camera=mujoco.MjvCamera()
            camera.azimuth,camera.elevation,camera.distance=120.,-40.,2.8
            camera.lookat[:]=(0.,0.,0.)
            next_camera=0.
            while rclpy.ok():
                started=time.monotonic()
                for name,hub in self.hubs.items():
                    if hub.clients<=0:continue
                    selected=camera if name=='overview' else name
                    if name!='overview' and mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_CAMERA,name)<0:
                        selected=camera
                    with self.lock:renderer.update_scene(self.data,selected)
                    frame=Image.fromarray(renderer.render())
                    if name in ('overview','overhead'):
                        axes='XY' if name=='overhead' and selected==name else 'XYZ'
                        draw_axis_gizmo(frame,renderer.scene,axes)
                    buf=io.BytesIO();frame.save(buf,format='JPEG',quality=80)
                    hub.publish(buf.getvalue())
                if sensor is not None and time.monotonic()>=next_camera:
                    next_camera=time.monotonic()+self.camera_period
                    for name,pubs in self.camera_publishers.items():
                        if (self.sensor_requested(name,time.monotonic())
                            and any(p.get_subscription_count() for p in pubs.values())
                            and mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_CAMERA,name)>=0):
                            self.publish_camera(sensor,name)
                time.sleep(max(0.,self.view_period-(time.monotonic()-started)))
        except Exception as error:
            if rclpy.ok():self.get_logger().error(f'Camera rendering failed: {error}')
        finally:
            if sensor is not None:sensor.close()
            if renderer is not None:renderer.close()

    def destroy_node(self):
        self.server.shutdown()
        super().destroy_node()


def main():
    rclpy.init()
    node = PiperMujocoSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
