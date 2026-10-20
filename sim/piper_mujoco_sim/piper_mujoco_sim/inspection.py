"""Joint-only inspection planning from detected object positions.

Uses known robot/camera/table geometry. Free-body geometry is disabled in the
planning model: simulator block poses and block count never enter view scoring.
"""
import copy
import math

import mujoco
import numpy as np


def scan_geometry(model):
    """Installation settings only; intentionally excludes free-body state."""
    return {
        'bodies': {name: {'position': model.body(name).pos.tolist(),
                          'quaternion': model.body(name).quat.tolist()}
                   for name in ('base_link', 'overhead_mount', 'wrist_camera_mount', 'wrist_camera_body')},
        'cameras': {name: {'quaternion': model.camera(name).quat.tolist(),
                           'fovy': float(model.camera(name).fovy[0])}
                    for name in ('wrist', 'overhead')},
        'post_position': model.geom('mount_post').pos.tolist()}


class InspectionPlanner:
    def __init__(self, model, survey_targets, max_views_per_block=2):
        if not isinstance(max_views_per_block,int) or isinstance(max_views_per_block,bool) or max_views_per_block<1:
            raise ValueError('Per-object inspection budget must be a positive integer.')
        self.model = copy.copy(model)
        self.data = mujoco.MjData(self.model)
        self.addresses = [model.joint(f'joint{i}').qposadr[0] for i in range(1, 7)]
        self.limits = np.array([model.joint(f'joint{i}').range for i in range(1, 7)])
        self.camera = model.camera('wrist').id
        self.seeds = np.asarray(survey_targets, dtype=float)
        self.max_views_per_block = max_views_per_block
        self.history = {}
        self.geometry = None
        # Ignore all free-body objects for contacts and rays. Their actual poses
        # must not affect a plan derived from perception's estimated positions.
        free_bodies = set(int(model.jnt_bodyid[j]) for j in range(model.njnt)
                          if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE)
        for body in range(model.nbody):
            ancestor = body
            while ancestor and ancestor not in free_bodies:
                ancestor = int(model.body_parentid[ancestor])
            if ancestor in free_bodies:
                geoms = np.flatnonzero(model.geom_bodyid == body)
                self.model.geom_contype[geoms] = 0
                self.model.geom_conaffinity[geoms] = 0
                self.model.geom_group[geoms] = 5
        self.ray_groups = np.array([1, 1, 1, 1, 1, 0], dtype=np.uint8)
        self.sync(scan_geometry(model))

    def sync(self, geometry):
        if geometry == self.geometry:
            return
        for name, body in geometry['bodies'].items():
            self.model.body(name).pos[:] = body['position']
            self.model.body(name).quat[:] = body['quaternion']
        for name, camera in geometry['cameras'].items():
            self.model.camera(name).quat[:] = camera['quaternion']
            self.model.camera(name).fovy[0] = camera['fovy']
        self.model.geom('mount_post').pos[:] = geometry['post_position']
        self.geometry = copy.deepcopy(geometry)
        self.reset()
        self._build_pool()

    def reset(self):
        self.history = {}
        self.attempts = {}

    def pose(self, joints):
        self.data.qpos[self.addresses] = joints[:6]
        self.data.qpos[self.model.joint('joint7').qposadr[0]] = 0.0
        self.data.qpos[self.model.joint('joint8').qposadr[0]] = 0.0
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_camlight(self.model, self.data)
        return (self.data.cam_xpos[self.camera].copy(),
                self.data.cam_xmat[self.camera].reshape(3, 3).copy())

    def remember(self, label, joints):
        self.history[label] = self.pose(joints)

    def _build_pool(self):
        rng = np.random.default_rng(711)
        samples = [self.seeds[:, :6]]
        for seed in self.seeds:
            samples.append(seed[:6] + rng.normal(size=(160, 6))*[.45, .22, .22, .32, .12, 0])
        samples.append(rng.uniform(self.limits[:, 0], self.limits[:, 1], size=(3000, 6)))
        joints = np.unique(np.clip(np.vstack(samples), self.limits[:, 0], self.limits[:, 1]), axis=0)
        self.pool, self.positions, self.rotations = [], [], []
        for q in joints:
            q[5] = 0  # joint6 rolls the gripper, not the fixed camera on link5
            p, r = self.pose(q)
            if .24 < p[2] < .85 and r[2, 2] > .3 and r[2, 1] > .15:
                self.pool.append([*q, 0.0])
                self.positions.append(p)
                self.rotations.append(r)
        self.pool = np.asarray(self.pool)
        self.positions = np.asarray(self.positions)
        self.rotations = np.asarray(self.rotations)

    def _clear_path(self, initial, target):
        for t in np.linspace(0, 1, 25):
            self.data.qpos[self.addresses] = np.asarray(initial[:6])*(1-t)+np.asarray(target[:6])*t
            self.data.qpos[self.model.joint('joint7').qposadr[0]] = 0.0
            self.data.qpos[self.model.joint('joint8').qposadr[0]] = 0.0
            mujoco.mj_forward(self.model, self.data)
            if any(contact.dist < -.001 for contact in self.data.contact):
                return False
        return True

    def _visible(self, q, point, radius):
        p, _ = self.pose(q)
        mujoco.mj_forward(self.model, self.data)
        # Test the centre and surrounding top-face evidence against the robot
        # and known fixtures; looking at a centre hidden by the gripper is useless.
        for dx, dy in ((0, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)):
            endpoint = np.asarray(point)+[dx*radius, dy*radius, .01]
            ray = endpoint-p
            distance = np.linalg.norm(ray)
            hit = mujoco.mj_ray(self.model, self.data, p, ray/distance,
                                self.ray_groups, 1, self.model.camera(self.camera).bodyid[0],
                                np.zeros(1, dtype=np.int32))
            if 0 <= hit < distance-.015:
                return False
        return True

    def choose(self, targets, current, label):
        """Return a clear novel view of a detected uncertain object, or None."""
        if not targets or not len(self.pool):
            return None
        f = 240/(2*math.tan(math.radians(float(self.model.cam_fovy[self.camera]))/2))
        possibilities = []
        for target in targets:
            attempts = self.attempts.get(target['id'],0)
            if attempts >= self.max_views_per_block:
                continue
            point = np.asarray(target['position'], dtype=float)
            radius = max(float(target['size'][0])/2, .045)
            local = np.einsum('ni,nij->nj', point-self.positions, self.rotations)
            depth = -local[:, 2]
            visible = ((depth > .15) & (depth < 1.15)
                       & (abs(local[:, 0])+radius < depth*160/f)
                       & (abs(local[:, 1])+radius < depth*120/f))
            novelty = np.ones(len(self.pool))
            # Physical camera poses, not just new labels, establish novelty.
            for old_p, old_r in self.history.values():
                separation = np.linalg.norm(self.positions-old_p, axis=1)
                angles = np.arccos(np.clip(np.einsum('ni,i->n', self.rotations[:, :, 2], old_r[:, 2]), -1, 1))
                visible &= (separation > .035) | (angles > math.radians(10))
            for _, supporting_label in target.get('supporting_views', []):
                if supporting_label in self.history:
                    old_p, old_r = self.history[supporting_label]
                    angles = np.arccos(np.clip(np.einsum('ni,i->n', self.rotations[:, :, 2], old_r[:, 2]), -1, 1))
                    novelty += np.minimum(angles/math.radians(30), 1)*.25
            # Prefer a larger top face, centred in the image, over a wider sweep.
            area = float(np.prod(target['size'][:2]))
            pixel_area = f*f*area*self.rotations[:, 2, 2]/np.maximum(depth, .15)**2
            centring = np.sum((local[:, :2]/np.maximum(depth[:, None], .15))**2, axis=1)
            travel = np.max(abs(self.pool[:, :6]-np.asarray(current[:6])), axis=1)
            score = np.sqrt(pixel_area)*novelty - 12*centring - 2*travel
            for i in np.argsort(np.where(visible, score, -np.inf))[-40:]:
                if visible[i]:
                    possibilities.append((target['priority'],attempts,float(score[i]),int(i),target,radius))
        # Resolve provisional objects and missing support before refining an
        # already supported pose. Rotate between equally urgent objects.
        for _,_,score,index,target,radius in sorted(possibilities,key=lambda item:(-item[0],item[1],-item[2])):
            q = self.pool[index].tolist()
            if self._visible(q, target['position'], radius) and self._clear_path(current, q):
                self.remember(label, q)
                self.attempts[target['id']] = self.attempts.get(target['id'],0)+1
                return {'joints': q, 'focus': target['id'], 'position': target['position'],
                        'reason': 'New close view of uncertain detected object', 'score': score}
        return None
