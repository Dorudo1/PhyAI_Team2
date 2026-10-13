"""Live editing of lab poses without replacing the running physics model."""
import copy
import json
import math
import os
from pathlib import Path
import tempfile

import mujoco


VECTORS = {
    'post_position': 2,
    'overhead_position': 3,
    'overhead_quaternion': 4,
    'wrist_camera_mount_position': 3,
    'wrist_camera_mount_quaternion': 4,
    'wrist_camera_position': 3,
    'wrist_camera_quaternion': 4,
}
SCALARS = {'robot_yaw_degrees', 'wrist_camera_roll_degrees', 'camera_fovy'}


class SceneEditor:
    def __init__(self, model, data, lock, config_path):
        self.model, self.data, self.lock = model, data, lock
        self.path = Path(config_path).resolve()
        self.disk_text = self.path.read_text()
        self.config = json.loads(self.disk_text)
        self.saved = copy.deepcopy(self.config)
        self.paused = False

    def snapshot(self):
        with self.lock:
            return {'enabled': True, 'config': copy.deepcopy(self.config),
                    'paused': self.paused, 'dirty': self.config != self.saved,
                    'path': str(self.path)}

    def validate(self, changes):
        if not isinstance(changes, dict):
            raise ValueError('Changes must be an object.')
        config = copy.deepcopy(self.config)

        def number(value):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('Use finite numeric values.')
            return float(value)

        for key, value in changes.items():
            if key in VECTORS:
                if not isinstance(value, list) or len(value) != VECTORS[key]:
                    raise ValueError(f'{key} requires {VECTORS[key]} values.')
                value = [number(item) for item in value]
                if key.endswith('quaternion'):
                    norm = math.sqrt(sum(item * item for item in value))
                    if norm < 1e-8 or not math.isfinite(norm):
                        raise ValueError('Rotation quaternion must be nonzero and finite.')
                    value = [item / norm for item in value]
                elif any(abs(item) > 10 for item in value):
                    raise ValueError('Positions must be within 10 meters of their parent.')
            elif key in SCALARS:
                value = number(value)
                if key == 'camera_fovy' and not 1 <= value <= 150:
                    raise ValueError('Field of view must be between 1 and 150 degrees.')
            elif key == 'block_positions':
                if not isinstance(value, list) or len(value) != len(self.config[key]):
                    raise ValueError('Keep the existing number of blocks.')
                positions = []
                for position in value:
                    if not isinstance(position, list) or len(position) != 2:
                        raise ValueError('Each block requires X and Y.')
                    position = [number(item) for item in position]
                    if any(abs(item) > 10 for item in position):
                        raise ValueError('Block positions must be within 10 meters.')
                    positions.append(position)
                value = positions
            else:
                raise ValueError(f'{key} is not editable in the live editor.')
            config[key] = value
        return config

    def apply(self, config):
        """Caller holds lock. Fixed-body edits keep the arm joints intact."""
        model, data = self.model, self.data
        base = model.body('base_link')
        base.pos[:] = [*config['post_position'], 0]
        yaw = math.radians(config['robot_yaw_degrees']) / 2
        base.quat[:] = [math.cos(yaw), 0, 0, math.sin(yaw)]
        model.geom('mount_post').pos[:2] = config['post_position']
        for body, position, quaternion in (
                ('overhead_mount', 'overhead_position', 'overhead_quaternion'),
                ('wrist_camera_mount', 'wrist_camera_mount_position', 'wrist_camera_mount_quaternion'),
                ('wrist_camera_body', 'wrist_camera_position', 'wrist_camera_quaternion')):
            model.body(body).pos[:] = config[position]
            model.body(body).quat[:] = config.get(quaternion, [1, 0, 0, 0])
        roll = math.radians(config.get('wrist_camera_roll_degrees', 0)) / 2
        model.camera('wrist').quat[:] = [math.cos(roll), 0, 0, math.sin(roll)]
        for name in ('wrist', 'overhead'):
            model.camera(name).fovy[0] = config['camera_fovy']
        # Move only blocks whose configured positions changed; preserve other state.
        for index, position in enumerate(config['block_positions']):
            if position != self.config['block_positions'][index]:
                joint = model.joint(f'block_{index + 1}_joint')
                qpos, dof = joint.qposadr[0], joint.dofadr[0]
                data.qpos[qpos:qpos + 7] = [*position, config['block_size'][2] / 2 + .001, 1, 0, 0, 0]
                data.qvel[dof:dof + 6] = 0
        self.config = copy.deepcopy(config)
        mujoco.mj_forward(model, data)

    def action(self, action, payload):
        with self.lock:
            if action == 'preview':
                config = self.validate(payload.get('changes', {}))
                self.apply(config)
                self.paused = True
            elif action == 'save':
                config = self.validate(payload.get('changes', {}))
                if self.path.read_text() != self.disk_text:
                    raise ValueError('The scene file changed outside the editor. Reload the file before saving.')
                text = json.dumps(config, indent=2, allow_nan=False) + '\n'
                # Resolve symlinks and replace the target, preserving its permissions.
                temporary = None
                try:
                    with tempfile.NamedTemporaryFile(mode='w', dir=self.path.parent, delete=False) as stream:
                        temporary = stream.name
                        stream.write(text)
                    os.chmod(temporary, self.path.stat().st_mode & 0o777)
                    os.replace(temporary, self.path)
                    temporary = None
                finally:
                    if temporary is not None:
                        os.unlink(temporary)
                self.apply(config)
                self.saved = copy.deepcopy(config)
                self.disk_text = text
            elif action == 'revert':
                self.apply(self.saved)
            elif action == 'reload':
                text = self.path.read_text()
                loaded = json.loads(text)
                if not isinstance(loaded, dict):
                    raise ValueError('Scene file must contain an object.')
                editable = set(VECTORS) | SCALARS | {'block_positions'}
                fixed = {key: value for key, value in self.saved.items() if key not in editable}
                if fixed != {key: value for key, value in loaded.items() if key not in editable}:
                    raise ValueError('Table, block dimensions, or other structural settings changed. Restart the simulator to load them.')
                for key in editable - {'overhead_quaternion', 'wrist_camera_mount_quaternion'}:
                    if key not in loaded:
                        raise ValueError(f'Scene file is missing {key}.')
                config = self.validate({key: value for key, value in loaded.items() if key in editable})
                for key in ('overhead_quaternion', 'wrist_camera_mount_quaternion'):
                    if key not in loaded:
                        config.pop(key, None)
                self.apply(config)
                self.saved = copy.deepcopy(config)
                self.disk_text = text
                self.paused = True
            elif action == 'pause':
                if not isinstance(payload.get('paused'), bool):
                    raise ValueError('Paused must be true or false.')
                self.paused = payload['paused']
            else:
                raise ValueError('Unknown editor action.')
            return {'enabled': True, 'config': copy.deepcopy(self.config),
                    'paused': self.paused, 'dirty': self.config != self.saved,
                    'path': str(self.path)}
