"""Incremental static-scene inventory and discovery coverage, not a collision map."""
from dataclasses import asdict
import math
import uuid

import numpy as np


def pose_difference(a, b):
    distance = float(np.linalg.norm(np.asarray(a['position'])-b['position']))
    w, x, y, z = b['quaternion']
    symmetric = (-z, y, -x, w)
    dot = max(abs(np.dot(a['quaternion'], q)) for q in (b['quaternion'], symmetric))
    return distance, 2*math.acos(float(np.clip(dot, 0., 1.)))


def pose_evidence(candidate, detection):
    return {'position': list(candidate.position), 'quaternion': list(candidate.quaternion),
            'position_m': candidate.pose_uncertainty['position_m'],
            'yaw_rad': candidate.pose_uncertainty['yaw_rad'],
            'camera': detection.camera, 'viewpoint': detection.viewpoint, 'stamp': detection.stamp}


def fuse_pose(evidence):
    """Average independent view anchors with the cuboid's half-turn symmetry."""
    reference = evidence[0]['quaternion']
    aligned = []
    for measurement in evidence:
        q = np.asarray(measurement['quaternion'])
        w, x, y, z = q
        symmetric = np.array([-z, y, -x, w])
        aligned.append(max((q, -q, symmetric, -symmetric), key=lambda v: np.dot(v, reference)))
    quaternion = np.mean(aligned, axis=0)
    quaternion /= np.linalg.norm(quaternion)
    return {'position': np.mean([m['position'] for m in evidence], axis=0).tolist(),
            'quaternion': quaternion.tolist()}


class Workspace:
    def __init__(self, frame='base_link', cell_size=.02, scope=None,
                 match_distance=.025, dimension_tolerance=.02, min_cell_points=3):
        if not frame or not np.isfinite([cell_size, match_distance, dimension_tolerance]).all() or min(
                cell_size, match_distance, dimension_tolerance) <= 0:
            raise ValueError('Invalid workspace configuration.')
        if scope is not None and (len(scope) != 4 or not np.isfinite(scope).all()
                or scope[0] >= scope[1] or scope[2] >= scope[3]):
            raise ValueError('Scope requires xmin,xmax,ymin,ymax in the fixed frame.')
        self.frame, self.cell_size, self.scope = frame, cell_size, scope
        self.match_distance, self.dimension_tolerance = match_distance, dimension_tolerance
        self.min_cell_points = min_cell_points
        self.reset()

    def reset(self):
        self.session = str(uuid.uuid4())
        self.revision = 0
        self.table = None
        self.hypotheses = []
        self.coverage = {}
        self.calibrations = {}
        self.last_stamps = {}
        self.warnings = []
        self.next_id = 1
        self.next_hypothesis = 1

    def _cells(self, points):
        if not len(points):
            return []
        cells, counts = np.unique(np.floor(points[:, :2]/self.cell_size).astype(int), axis=0, return_counts=True)
        return [tuple(int(v) for v in cell) for cell, count in zip(cells, counts) if count >= self.min_cell_points]

    def merge(self, detection):
        if detection.frame != self.frame:
            raise ValueError('Observation and map frames differ.')
        camera = detection.camera
        previous = self.calibrations.get(camera)
        if previous is not None and previous != detection.calibration_revision:
            raise ValueError('Calibration changed; restart the static-scene scan.')
        if detection.stamp <= self.last_stamps.get(camera, 0.):
            raise ValueError('Out-of-order or duplicate camera observation.')
        if detection.plane is not None and self.table is not None:
            a, b = np.asarray(self.table['normal']), np.asarray(detection.plane['normal'])
            sample = detection.surface_points.mean(axis=0)
            if a @ b < math.cos(math.radians(3)) or abs(a @ sample+self.table['offset']) > .01:
                raise ValueError('Table geometry conflicts with this scan; restart or inspect calibration.')
        self.calibrations[camera] = detection.calibration_revision
        self.last_stamps[camera] = detection.stamp
        self.warnings = list(dict.fromkeys(self.warnings + detection.warnings))
        self.revision += 1
        if detection.plane is None:
            return
        if self.table is None:
            self.table = dict(detection.plane)
        else:
            # Preserve geometry frame; combine consistent plane measurements only.
            weight = detection.plane['support'] / (self.table['support'] + detection.plane['support'])
            n = (1-weight)*np.asarray(self.table['normal']) + weight*np.asarray(detection.plane['normal'])
            norm = np.linalg.norm(n)
            self.table['normal'] = (n/norm).tolist()
            self.table['offset'] = ((1-weight)*self.table['offset']+weight*detection.plane['offset'])/norm
            self.table['support'] += detection.plane['support']
            self.table['fit_error'] = max(self.table['fit_error'], detection.plane['fit_error'])
        surface_xy = detection.surface_points[:, :2]
        observed_bounds = [float(surface_xy[:,0].min()),float(surface_xy[:,0].max()),
                           float(surface_xy[:,1].min()),float(surface_xy[:,1].max())]
        previous_bounds = self.table.get('observed_surface_bounds')
        if previous_bounds:
            observed_bounds = [min(previous_bounds[0],observed_bounds[0]),
                               max(previous_bounds[1],observed_bounds[1]),
                               min(previous_bounds[2],observed_bounds[2]),
                               max(previous_bounds[3],observed_bounds[3])]
        # Bounds of measured patches only; gaps and true table edges stay unknown.
        self.table['observed_surface_bounds'] = observed_bounds
        for points, state, explored, unexplained in (
                (detection.surface_points, 'observed', True, False),
                (detection.object_points, 'uncertain', True, False),
                (detection.uncertain_points, 'uncertain', False, False),
                (detection.unexplained_points, 'uncertain', False, True)):
            for cell in self._cells(points):
                self.coverage[cell] = {'state': state, 'last_observed': detection.stamp,
                                       'camera': camera, 'explored': explored, 'unexplained': unexplained}
        self._associate(detection)

    @property
    def tracks(self):
        """Only evidence-confirmed inventory is exposed to the stacking layer."""
        return [track for track in self.hypotheses if track['id'] is not None]

    def _associate(self, detection):
        candidates = detection.candidates
        matches = []
        # Use conservative geometry gates for association before deciding whether
        # an observation is precise enough to update a pose.
        for candidate in candidates:
            potential = []
            for index, track in enumerate(self.hypotheses):
                distance = np.linalg.norm(np.asarray(candidate.position)-track['position'])
                size_error = np.max(np.abs(np.asarray(candidate.size)-track['size']))
                uncertainty = max(candidate.pose_uncertainty.get('position_m',0),
                                  track.get('pose_uncertainty',{}).get('position_m',0))
                gate = self.match_distance + min(uncertainty,.04)
                if distance <= gate and (size_error <= self.dimension_tolerance or
                        not candidate.resolved or not track['resolved']):
                    potential.append(index)
            matches.append(potential)
        claimed = set()
        for candidate, potential in zip(candidates, matches):
            competitors = [j for j, options in enumerate(matches) if potential and set(options) & set(potential)]
            if len(potential) > 1 or (potential and len(competitors) > 1):
                for index in potential:
                    track = self.hypotheses[index]
                    track['resolved'] = False
                    track['quality'] = 'uncertain'
                    track['supporting_views'] = []
                    track['pose_evidence'] = []
                    track['pose_conflict'] = True
                    track['reasons'] = list(dict.fromkeys(
                        track['reasons'] + ['ambiguous cross-view association']))
                    track['alternatives'] = (track['alternatives'] + [{
                        'position': candidate.position, 'quaternion': candidate.quaternion,
                        'camera': detection.camera, 'stamp': detection.stamp}])[-4:]
                continue
            values = asdict(candidate)
            values.pop('pixels', None)
            values.pop('quality')
            values['fit_quality'] = 'shape_supported' if candidate.resolved else 'incomplete'
            values['quality'] = 'uncertain'
            view = (detection.camera,detection.viewpoint)
            if potential:
                index = potential[0]
                if index in claimed:
                    continue
                claimed.add(index)
                track = self.hypotheses[index]
                if candidate.resolved:
                    evidence = track.get('pose_evidence', [])
                    current = pose_evidence(candidate, detection)
                    differences = [(old, *pose_difference(current, old)) for old in evidence]
                    conflicts = [old for old, distance, angle in differences
                                 if distance > current['position_m']+old['position_m']
                                 or angle > current['yaw_rad']+old['yaw_rad']]
                    if conflicts:
                        track['alternatives'] = (track['alternatives']+conflicts)[-4:]
                        track['pose_conflict'] = True
                        evidence = []
                        values['reasons'] += ['cross-view pose disagreement exceeds measurement bounds']
                    # Keep the first measurement as each physical view's anchor;
                    # repeated captures must not walk that anchor through drift.
                    if not any((old['camera'], old['viewpoint']) == view for old in evidence):
                        evidence = evidence+[current]
                    track['pose_evidence'] = evidence
                    track['supporting_views'] = [(old['camera'], old['viewpoint']) for old in evidence]
                    if track.get('pose_conflict') and len(evidence) < 2:
                        values['resolved'] = False
                        values['pose_uncertainty']['position_m'] = max(
                            [current['position_m']]+[distance+old['position_m'] for old, distance, _ in differences])
                        values['pose_uncertainty']['yaw_rad'] = max(
                            [current['yaw_rad']]+[angle+old['yaw_rad'] for old, _, angle in differences])
                    else:
                        track['pose_conflict'] = False
                        track['alternatives'] = []
                        values.update(fuse_pose(evidence))
                        # Bounds contain every supporting measurement and its
                        # sensor error, around the fused pose rather than just
                        # the latest fit. Repeated views do not bias the mean.
                        residuals = [(old, *pose_difference(values, old)) for old in evidence+[current]]
                        values['pose_uncertainty']['position_m'] = max(
                            distance+old['position_m'] for old, distance, _ in residuals)
                        values['pose_uncertainty']['yaw_rad'] = max(
                            angle+old['yaw_rad'] for old, _, angle in residuals)
                # Partial evidence does not replace an already fitted full pose.
                if candidate.resolved or not track['resolved']:
                    track.update(values)
                    if candidate.resolved:
                        track['pose_last_observed'] = detection.stamp
                track['source_cameras'] = sorted(set(track['source_cameras'] + [detection.camera]))
                track['last_observed'] = detection.stamp
                track['observations'] += 1
            else:
                # Do not mint an identity for an overlapping hypothesis rejected
                # by the precise association gate (e.g. a partial-face centre).
                nearby = [track for track in self.hypotheses if
                    np.linalg.norm(np.asarray(candidate.position)-track['position'])
                    < (np.linalg.norm(track['size'][:2])+np.linalg.norm(candidate.size[:2]))/4]
                if nearby:
                    for track in nearby:
                        track['alternatives'] = (track['alternatives'] + [{
                            'position': candidate.position, 'quaternion': candidate.quaternion,
                            'camera': detection.camera, 'stamp': detection.stamp}])[-4:]
                        track['quality'] = 'uncertain'
                        track['resolved'] = False
                        track['supporting_views'] = []
                        track['pose_evidence'] = []
                        track['pose_conflict'] = True
                    continue
                track = {**values, 'id': None,
                    'hypothesis_id': f'hypothesis_{self.next_hypothesis}',
                    'source_cameras': [detection.camera], 'last_observed': detection.stamp,
                    'observations': 1, 'pose_last_observed': detection.stamp if candidate.resolved else None,
                    'supporting_views': [view] if candidate.resolved else [],
                    'pose_evidence': [pose_evidence(candidate, detection)] if candidate.resolved else [],
                    'pose_conflict': False,
                    'alternatives': []}
                self.hypotheses.append(track)
                self.next_hypothesis += 1
            # A complete known-shape observation can confirm immediately.
            # Otherwise require consistent resolved evidence from distinct views.
            if track['id'] is None and track['resolved'] and not track['alternatives'] and (
                    (candidate.known_geometry and candidate.resolved)
                    or len(track['supporting_views']) >= 2):
                track['id'] = f'block_{self.next_id}'
                self.next_id += 1
                track['quality'] = 'confirmed'
            track['identity_state'] = 'confirmed' if track['id'] is not None else 'provisional'
            if track['id'] is not None and track['resolved']:
                track['quality'] = 'confirmed'

    def snapshot(self, now=None):
        latest = max(self.last_stamps.values(), default=0.)
        now = latest if now is None else now
        cells = dict(self.coverage)
        if self.scope:
            xmin, xmax, ymin, ymax = self.scope
            for x in range(math.floor(xmin/self.cell_size), math.ceil(xmax/self.cell_size)):
                for y in range(math.floor(ymin/self.cell_size), math.ceil(ymax/self.cell_size)):
                    cells.setdefault((x, y), {'state': 'unobserved', 'last_observed': None, 'camera': None})
        entries = [{'index': list(cell), **info,
                    'age': max(0., now-info['last_observed']) if info['last_observed'] is not None else None}
                   for cell, info in sorted(cells.items())]
        scope_cells = entries
        if self.scope:
            xmin, xmax, ymin, ymax = self.scope
            scope_cells = [entry for entry in entries if
                xmin <= (entry['index'][0]+.5)*self.cell_size < xmax and
                ymin <= (entry['index'][1]+.5)*self.cell_size < ymax]
        observed = sum(entry['state'] == 'observed' for entry in scope_cells)
        uncertain = sum(entry['state'] == 'uncertain' and entry.get('explored', True) for entry in scope_cells)
        unexplained = sum(entry.get('unexplained', False) for entry in scope_cells)
        explored = (observed+uncertain)/len(scope_cells) if scope_cells else 0.
        fraction = observed/len(scope_cells) if scope_cells else 0.
        return {'version': 1, 'frame': self.frame, 'session_id': self.session,
                'revision': self.revision, 'stamp': latest, 'table': self.table,
                'tracks': [{**track, 'quality':'confirmed' if track['resolved'] else 'uncertain',
                            'identity_state':'confirmed',
                            'pose_state':'resolved' if track['resolved'] else 'needs_observation',
                            'age': max(0., now-track['last_observed'])} for track in self.tracks],
                'hypotheses': [{**track, 'age': max(0., now-track['last_observed'])}
                               for track in self.hypotheses if track['id'] is None],
                'coverage': {'cell_size': self.cell_size, 'scope': self.scope,
                             'observed_fraction': fraction, 'explored_fraction': explored,
                             'unobserved_fraction': 1.-explored, 'unexplained_cells': unexplained, 'cells': entries},
                'calibrations': dict(self.calibrations), 'warnings': list(self.warnings),
                'motion_authorization': False}
