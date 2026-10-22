#!/usr/bin/env python3
"""실제 Piper · RealSense 토픽 → PhyAI_Team2 README 로봇 인터페이스.

팀원 앱은 시뮬레이터(MuJoCo)와 같은 이름 · 단위로 실제 로봇을 본다. 하드웨어 · 기존 도구는 건드리지 않고 이름만 바꿔 내보낸다.

  /slave/joint_states_single (joint1~6, gripper=폭)  →  /joint_states (joint1~8, joint7=폭/2, joint8=-폭/2)
  /joint_ctrl_single (joint1~7, joint7=폭/2)          →  /policy/joint_cmd (joint1~6, gripper=폭) → piper-follow → 슬레이브
  /cam2/… (천장)  →  /overhead/color/image_rect_raw · depth/image_rect_raw · {color,depth}/camera_info  (frame overhead_optical)
  /cam1/… (손목)  →  /wrist/…  (frame wrist_optical)
  tf (고정): world → base_link,  base_link → overhead_optical,  link6 → wrist_optical   (캘리브레이션 결과)
  주의: 펌웨어 end_pose 는 link6 에서 약 23 mm 떨어진 점이라(end_pose_in_flange) 손목 카메라는 그만큼 옮겨 붙인다.
  base_link → link1 … link8 은 같이 띄우는 robot_state_publisher 가 /joint_states 로 계산한다 (/robot_description 도 발행).

명령은 슬레이브로 직접 보내지 않는다. 안전 브리지(piper-follow)가 실행 중일 때만 팔이 움직인다.
"""
import json
import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import StaticTransformBroadcaster

ARM = [f'joint{i}' for i in range(1, 7)]
GRIPPER_HALF_MAX = 0.035          # README: joint7 = 손가락 하나 이동량 (전체 벌림 = 2배)


def _tf(T, parent, child, stamp):
    from scipy.spatial.transform import Rotation
    m = TransformStamped()
    m.header.frame_id, m.header.stamp, m.child_frame_id = parent, stamp, child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, T[:3, 3])
    x, y, z, w = Rotation.from_matrix(T[:3, :3]).as_quat()
    m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = x, y, z, w
    return m


class ReadmeBridge(Node):
    def __init__(self):
        super().__init__('readme_bridge')
        p = {k: self.declare_parameter(k, v).value for k, v in dict(
            slave_state_topic='/slave/joint_states_single', state_topic='/joint_states',
            command_topic='/joint_ctrl_single', bridge_topic='/policy/joint_cmd',
            overhead_source='cam2', wrist_source='cam1',
            world_frame='world', base_frame='base_link', flange_frame='link6',
            end_pose_in_flange=[-0.01766, 0.00005, 0.01516],
            calibration_file='~/piper_test/data/calib/calibration.json').items()}
        self.p = p
        self.last_state = None        # 슬레이브 현재 [j1..j6, 폭]
        self.last_cmd = None          # 마지막 목표 [j1..j6, 폭] — 일부 관절만 온 명령은 나머지를 여기서 채움 (시뮬레이터와 같은 동작)

        # 팔 상태 · 명령 (시뮬레이터와 같은 QoS: reliable, depth 10)
        self.state_pub = self.create_publisher(JointState, p['state_topic'], 10)
        self.bridge_pub = self.create_publisher(JointState, p['bridge_topic'], 10)
        self.create_subscription(JointState, p['slave_state_topic'], self.on_state, 10)
        self.create_subscription(JointState, p['command_topic'], self.on_command, 10)

        # 카메라: 메시지는 그대로, frame_id · 토픽 이름만 바꿔 다시 발행 (깊이 16UC1 mm 유지 — discovery 는 16UC1 · 32FC1 모두 지원)
        img_qos = QoSProfile(depth=1)  # reliable: best effort · reliable 구독자 모두 받음 (시뮬레이터와 같음)
        for name, src in (('overhead', p['overhead_source']), ('wrist', p['wrist_source'])):
            frame = f'{name}_optical'
            pubs = {k: self.create_publisher(t, f'/{name}/{k}', img_qos) for k, t in (
                ('color/image_rect_raw', Image), ('depth/image_rect_raw', Image),
                ('color/camera_info', CameraInfo), ('depth/camera_info', CameraInfo))}
            self.create_subscription(Image, f'/{src}/color/image_raw',
                                     self._relay(frame, [pubs['color/image_rect_raw']]), qos_profile_sensor_data)
            self.create_subscription(Image, f'/{src}/aligned_depth_to_color/image_raw',
                                     self._relay(frame, [pubs['depth/image_rect_raw']]), qos_profile_sensor_data)
            # 깊이는 컬러에 정렬돼 있으므로 깊이 camera_info = 컬러 camera_info
            self.create_subscription(CameraInfo, f'/{src}/color/camera_info',
                                     self._relay(frame, [pubs['color/camera_info'], pubs['depth/camera_info']]),
                                     qos_profile_sensor_data)

        self.static_br = StaticTransformBroadcaster(self)
        self.publish_static_tf()
        self.warned = set()
        self.get_logger().info(f"{p['slave_state_topic']} → {p['state_topic']},  {p['command_topic']} → {p['bridge_topic']} "
                               f"(piper-follow 가 실행 중일 때만 팔이 움직임),  /{p['overhead_source']} → /overhead, "
                               f"/{p['wrist_source']} → /wrist")

    # ---- 팔 ----
    def on_state(self, m):
        if len(m.position) < 7:
            return
        pos, vel, eff = list(m.position), list(m.velocity), list(m.effort)
        g = pos[6]
        self.last_state = pos[:7]
        out = JointState()
        out.header = m.header
        out.name = ARM + ['joint7', 'joint8']
        out.position = pos[:6] + [g / 2, -g / 2]
        out.velocity = (vel[:6] + [0.0, 0.0]) if len(vel) >= 6 else []
        out.effort = (eff[:6] + [eff[6] / 2, -eff[6] / 2]) if len(eff) >= 7 else []
        self.state_pub.publish(out)

    def on_command(self, m):
        if m.name and len(m.name) != len(m.position):
            return self.warn_once('len', f'name {len(m.name)}개 ≠ position {len(m.position)}개 → 버림')
        cmd = dict(zip(m.name, m.position)) if m.name else dict(zip(ARM + ['joint7'], m.position))
        base = self.last_cmd or self.last_state
        if base is None and not all(n in cmd for n in ARM):
            return self.warn_once('nostate', '슬레이브 상태가 아직 없어 빠진 관절을 채울 수 없음 → 버림')
        target = list(base) if base else [0.0] * 7
        for i, n in enumerate(ARM):
            if n in cmd:
                target[i] = cmd[n]
        half = cmd.get('joint7', cmd.get('gripper'))          # 시뮬레이터는 'gripper' 이름도 받음 (같은 단위)
        if half is not None:
            target[6] = 2 * min(max(half, 0.0), GRIPPER_HALF_MAX)
        if not all(math.isfinite(v) for v in target):
            return self.warn_once('nan', 'NaN / inf 포함 → 버림')
        self.last_cmd = target
        out = JointState()
        out.header.stamp = m.header.stamp if m.header.stamp.sec else self.get_clock().now().to_msg()
        out.name = ARM + ['gripper']
        out.position = [float(v) for v in target]
        self.bridge_pub.publish(out)

    def warn_once(self, key, text):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warning(text)

    # ---- 카메라 ----
    @staticmethod
    def _relay(frame, pubs):
        def cb(m):
            m.header.frame_id = frame
            for pub in pubs:
                pub.publish(m)
        return cb

    def publish_static_tf(self):
        p, now = self.p, self.get_clock().now().to_msg()
        tfs = [_tf(np.eye(4), p['world_frame'], p['base_frame'], now)]
        path = os.path.expanduser(p['calibration_file'])
        try:
            with open(path) as f:
                cal = json.load(f)
        except OSError:
            cal = {}
            self.get_logger().warning(f'캘리브레이션 파일 없음: {path} → 카메라 tf 를 발행하지 않음')
        parent_of = {'T_base_cam': p['base_frame'], 'T_flange_cam': p['flange_frame']}
        for name, src in (('overhead', p['overhead_source']), ('wrist', p['wrist_source'])):
            c = cal.get(src)
            if c and c.get('type') in parent_of:
                T = np.array(c['T'])
                if c['type'] == 'T_flange_cam':      # 캘리브레이션은 펌웨어 end_pose 기준 → link6 기준으로
                    off = np.eye(4)
                    off[:3, 3] = p['end_pose_in_flange']
                    T = off @ T
                tfs.append(_tf(T, parent_of[c['type']], f'{name}_optical', now))
            elif cal:
                self.get_logger().warning(f'{src} 캘리브레이션 없음 → {name}_optical tf 없음')
        self.static_br.sendTransform(tfs)


def main():
    rclpy.init()
    node = ReadmeBridge()
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
