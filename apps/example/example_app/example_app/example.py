import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .pick_drop import plan


class Example(Node):
    """Run the fixed block pickup/drop example once per simulation restart."""

    def __init__(self):
        super().__init__('example')
        self.latest = None
        self.scene = None
        self.restart_id = None
        self.scene_received = 0
        self.await_scene_after = None
        self.status = 'waiting for simulator scene and joint feedback'
        self.stages = None
        self.index = 0
        self.stage_start = None
        self.stage_initial = None
        self.selected = None
        self.lifted = False
        self.landed = False
        self.done = False
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)
        self.create_subscription(String, 'sim/scene', self.on_scene,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub = self.create_publisher(String, 'example/heartbeat', 10)
        self.command_pub = self.create_publisher(JointState, 'joint_ctrl_single', 10)
        self.create_timer(.02, self.move_arm)
        self.create_timer(1.0, self.tick)

    def on_joint_states(self, msg):
        self.latest = dict(zip(msg.name, msg.position))

    def on_scene(self, msg):
        try:
            scene = json.loads(msg.data)
            restart_id = scene.get('restart_id', 0)
            if self.restart_id is not None and restart_id < self.restart_id:
                return  # Ignore a scene snapshot queued before the restart.
            self.scene = scene
            self.scene_received = time.monotonic()
            if restart_id != self.restart_id:
                self.restart_id = restart_id
                self.latest = None  # Wait for feedback from the reset arm.
                self.stages = None
                self.index = 0
                self.stage_start = None
                self.stage_initial = None
                self.await_scene_after = None
                self.selected = None
                self.lifted = self.landed = self.done = False
                self.status = 'waiting for reset arm and blocks to settle'
        except (ValueError, TypeError):
            self.get_logger().error('Invalid simulator scene message.')

    def simulation_time(self):
        elapsed = 0 if self.scene['paused'] else time.monotonic()-self.scene_received
        return self.scene['simulation_time'] + elapsed

    def move_arm(self):
        names = [f'joint{i}' for i in range(1, 8)]
        if self.done or self.scene is None or self.latest is None or not all(name in self.latest for name in names):
            return
        if self.scene['paused'] or time.monotonic()-self.scene_received > 1:
            return
        actual = [self.latest[name] for name in names]
        if self.await_scene_after is not None:
            if self.scene['simulation_time'] <= self.await_scene_after:
                return
            self.await_scene_after = None
        if self.stages is None:
            if self.scene['simulation_time'] < .25:
                return
            self.await_scene_after = self.scene['simulation_time']
            try:
                self.selected, self.stages = plan(self.scene, actual)
            except ValueError as error:
                self.status = str(error)
                self.get_logger().error(self.status)
                self.done = True
                return
            self.status = f'selected {self.selected}'
            self.get_logger().info(self.status)
            # Wait for a fresh scene clock before starting the sequence.
            self.stage_initial = actual
            return
        now = self.simulation_time()
        stage = self.stages[self.index]
        if self.stage_start is None:
            self.stage_start = now
            # Maintain the previous commanded gripper closure through lift
            # waypoints. Starting from the blocked finger's measured opening
            # would unload the grip force and let the block slip at each step.
            self.stage_initial = actual if self.index == 0 else self.stages[self.index-1].target
            self.status = f'{self.selected}: {stage.name}'
            self.get_logger().info(self.status)
        progress = min(1, max(0, (now-self.stage_start)/stage.duration))
        blend = progress*progress*(3-2*progress)
        target = [float(initial + blend*(goal-initial)) for initial, goal in zip(self.stage_initial, stage.target)]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position = names, target
        self.command_pub.publish(msg)
        # Grasped fingers stop against the block before reaching zero opening.
        if progress < 1 or max(abs(value-goal) for value, goal in zip(actual[:6], stage.target[:6])) > .04:
            if now-self.stage_start > stage.duration+5:
                self.status = f'Aborted: arm did not reach {stage.name}.'
                self.get_logger().error(self.status)
                self.done = True
            return
        block = next((block for block in self.scene['blocks'] if block['name'] == self.selected), None)
        if stage.name == 'hold above table':
            self.lifted = block is not None and block['position'][2] > .06
            if not self.lifted:
                self.get_logger().error('Pickup did not lift the block; opening the gripper and returning home.')
        elif stage.name == 'wait for block to land':
            self.landed = block is not None and abs(block['position'][2]-self.scene['block_size'][2]/2) < .008
        self.index += 1
        self.stage_start = None
        if self.index == len(self.stages):
            self.done = True
            if self.lifted and self.landed:
                self.status = f'Completed: picked up {self.selected} and dropped it on the table.'
                self.get_logger().info(self.status)
            else:
                self.status = 'Scenario finished without a verified pickup and landing.'
                self.get_logger().error(self.status)

    def tick(self):
        self.pub.publish(String(data=self.status))
        if self.stages is None and not self.done:
            self.get_logger().info(self.status)


def main():
    rclpy.init()
    node = Example()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
