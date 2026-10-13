import os

# Render offscreen on the GPU; no display is needed, so local and remote users see the same browser view
os.environ.setdefault('MUJOCO_GL', 'egl')

import io  # noqa: E402
import json  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
from http.server import ThreadingHTTPServer

from ament_index_python.packages import get_package_share_directory
from .scene import build_scene, scene_snapshot, reset_scene  # noqa: E402
from .gizmo import draw_axis_gizmo  # noqa: E402
from .editor import SceneEditor  # noqa: E402
from .web import FrameHub, make_handler  # noqa: E402

import mujoco  # noqa: E402
import rclpy  # noqa: E402
from PIL import Image  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, DurabilityPolicy  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
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
        self.declare_parameter('scene_editing', False)
        self.declare_parameter('view_fps', 30.0)
        self.declare_parameter('view_width', 960)
        self.declare_parameter('view_height', 720)
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

        self.lock = threading.Lock()
        self.restart_id = 0
        self.command_cutoff = 0
        self.editor = (None if param('model_path') else
                       SceneEditor(self.model, self.data, self.lock, param('scene_config')))
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
        self.server = ThreadingHTTPServer(('0.0.0.0', param('http_port')),
                                         make_handler(self.hubs, self.editor, param('scene_editing'),
                                                      restart=self.restart_simulation))
        self.server.daemon_threads = True

        self.get_logger().info(
            f"command: /{param('command_topic')}  state: /{param('state_topic')}  "
            f"view: http://localhost:{self.server.server_port} "
            f"editor: http://localhost:{self.server.server_port}/edit")
        for target in (self.run_physics, self.run_view, self.server.serve_forever):
            threading.Thread(target=target, daemon=True).start()

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

    def restart_simulation(self):
        with self.lock:
            reset_scene(self.model, self.data, self.editor.config if self.editor else None)
            if self.editor:
                self.editor.paused = False
            self.restart_id += 1
            self.command_cutoff = self.get_clock().now().nanoseconds
            restart_id = self.restart_id
        self.publish_state()
        if self.editor:
            self.publish_scene()
        return {'restart_id': restart_id, 'message': 'Simulation restarted.'}

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

    def run_view(self):
        """Renders JPEG frames for the browser view, only while someone is watching."""
        width, height = self.view_size
        self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, width)
        self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, height)
        try:
            renderer = mujoco.Renderer(self.model, height, width)
        except Exception as e:  # e.g. no GPU / EGL in the container
            self.get_logger().error(f'browser view disabled, offscreen rendering failed: {e}')
            return
        camera = mujoco.MjvCamera()
        camera.azimuth, camera.elevation, camera.distance = 120.0, -40.0, 2.8
        camera.lookat[:] = (0.0, 0.0, 0.0)
        while rclpy.ok():
            started = time.monotonic()
            for name, hub in self.hubs.items():
                if hub.clients > 0:
                    selected = camera if name == 'overview' else name
                    if name != 'overview' and mujoco.mj_name2id(
                            self.model, mujoco.mjtObj.mjOBJ_CAMERA, name) < 0:
                        selected = camera  # Custom models may not define lab cameras.
                    with self.lock:
                        renderer.update_scene(self.data, selected)
                    buf = io.BytesIO()
                    frame = Image.fromarray(renderer.render())
                    if name in ('overview', 'overhead'):
                        # A missing overhead camera falls back to the 3D overview.
                        axes = 'XY' if name == 'overhead' and selected == name else 'XYZ'
                        draw_axis_gizmo(frame, renderer.scene, axes)
                    frame.save(buf, format='JPEG', quality=80)
                    hub.publish(buf.getvalue())
            time.sleep(max(0.0, self.view_period - (time.monotonic() - started)))
        renderer.close()

    def destroy_node(self):
        self.server.shutdown()
        super().destroy_node()


def main():
    rclpy.init()
    node = PiperMujocoSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
