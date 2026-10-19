"""Temporary randomized block layouts, validated and settled by MuJoCo."""
import math

import mujoco
import numpy as np

from .scene import reset_scene


class BlockScatter:
    """Caller holds the simulator lock for planning, capture, and restore."""
    def __init__(self, model, data, config):
        self.model, self.data = model, data
        self.joints = [model.joint(f'block_{i+1}_joint')
                       for i in range(len(config['block_positions']))]
        self.body_ids = {model.body(f'block_{i+1}').id for i in range(len(self.joints))}
        self.baseline = None

    def poses(self, data):
        return [data.qpos[j.qposadr[0]:j.qposadr[0]+7].copy() for j in self.joints]

    def apply(self, poses):
        for joint, pose in zip(self.joints, poses):
            q, v = joint.qposadr[0], joint.dofadr[0]
            self.data.qpos[q:q+7] = pose
            self.data.qvel[v:v+6] = 0
            self.data.qacc_warmstart[v:v+6] = 0
        mujoco.mj_forward(self.model, self.data)

    def remember(self):
        if self.baseline is None:
            self.baseline = self.poses(self.data)

    def restore(self):
        if self.baseline is not None:
            self.apply(self.baseline)

    def _clear(self, data, active):
        for contact in data.contact:
            if contact.dist >= 0:
                continue
            bodies = self.model.geom_bodyid[[contact.geom1, contact.geom2]]
            if any(int(body) in active for body in bodies):
                return False
        return True

    def plan(self, config, seed=None, table='all'):
        if seed is not None and (isinstance(seed, bool) or not isinstance(seed, int) or seed < 0):
            raise ValueError('seed must be a nonnegative integer.')
        tables = [(r, c) for r in range(2) for c in range(2)]
        if table != 'all':
            if table not in ('0_0', '0_1', '1_0', '1_1'):
                raise ValueError('table must be all, 0_0, 0_1, 1_0, or 1_1.')
            tables = [tuple(map(int, table.split('_')))]
        rng = np.random.default_rng(seed)
        scratch = mujoco.MjData(self.model)
        reset_scene(self.model, scratch, config)
        width, depth = config['table_size']
        gap = config['table_gap']
        length, short, height = config['block_size']
        radius = math.hypot(length, short)/2
        margin = radius+.015
        if min(width, depth)/2 <= margin:
            raise ValueError('Blocks do not fit on the configured tables.')
        # Hide unplaced blocks below the scene while checking each proposal.
        for i, joint in enumerate(self.joints):
            q = joint.qposadr[0]
            scratch.qpos[q:q+3] = [0, 0, -10-i]
        centers = []
        active = set()
        for index, joint in enumerate(self.joints):
            active.add(self.model.body(f'block_{index+1}').id)
            q = joint.qposadr[0]
            for _ in range(2000):
                row, column = tables[int(rng.integers(len(tables)))]
                center = np.array([(column-.5)*(width+gap), (row-.5)*(depth+gap)])
                xy = center+rng.uniform([-width/2+margin, -depth/2+margin],
                                         [width/2-margin, depth/2-margin])
                if any(np.linalg.norm(xy-other) < 2*radius+.01 for other in centers):
                    continue
                yaw = rng.uniform(-math.pi, math.pi)
                pose = [*xy, height/2+.003, math.cos(yaw/2), 0, 0, math.sin(yaw/2)]
                # Check the full vertical drop corridor for robot/scene contacts.
                clear = True
                for lift in (0., .01, .02):
                    scratch.qpos[q:q+7] = pose
                    scratch.qpos[q+2] += lift
                    mujoco.mj_forward(self.model, scratch)
                    if not self._clear(scratch, active):
                        clear = False
                        break
                if clear:
                    centers.append(xy)
                    break
            else:
                raise ValueError('Could not fit a collision-free layout; try another seed or all tables.')
        scratch.qvel[:] = 0
        # Gravity, friction, contact constraints, and angular dynamics determine
        # final poses. Never expose the live scene to an unverified placement.
        stable = 0
        for _ in range(math.ceil(3./self.model.opt.timestep)):
            mujoco.mj_step(self.model, scratch)
            speed = max((np.max(np.abs(scratch.qvel[j.dofadr[0]:j.dofadr[0]+6]))
                         for j in self.joints), default=0.)
            stable = stable+1 if speed < .001 else 0
            if stable*self.model.opt.timestep >= .2:
                break
        else:
            raise ValueError('Blocks did not settle; try another seed.')
        poses = self.poses(scratch)
        for pose in poses:
            # Every block must remain upright and supported on one tabletop.
            w, x, y, z = pose[3:]
            if abs(pose[2]-height/2) > .002 or 1-2*(x*x+y*y) < .999:
                raise ValueError('Scattered blocks lost tabletop support; try another seed.')
            if not any(abs(pose[0]-(c-.5)*(width+gap)) <= width/2-radius
                       and abs(pose[1]-(r-.5)*(depth+gap)) <= depth/2-radius
                       for r, c in tables):
                raise ValueError('A block moved too close to a table edge; try another seed.')
        # Verify settled poses against the actual reset robot configuration.
        reset_scene(self.model, scratch, config)
        for joint, pose in zip(self.joints, poses):
            q = joint.qposadr[0]
            scratch.qpos[q:q+7] = pose
        mujoco.mj_forward(self.model, scratch)
        for contact in scratch.contact:
            if contact.dist >= -.001:
                continue
            bodies = self.model.geom_bodyid[[contact.geom1, contact.geom2]]
            if any(int(body) in self.body_ids for body in bodies):
                raise ValueError('Settled layout intersects scene geometry; try another seed.')
        return poses
