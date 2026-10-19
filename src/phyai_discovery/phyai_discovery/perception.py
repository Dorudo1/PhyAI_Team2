"""Depth geometry for stationary, separated blocks on a near-horizontal table.

Uses measured depth only. Rectangles describe visible evidence, not certified
complete object geometry. Ambiguous groups never become grasp-ready poses.
"""
from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np

from .observations import Capture


@dataclass(frozen=True)
class DetectorConfig:
    min_depth: float = .05
    max_depth: float = 3.0
    plane_tolerance: float = .003
    min_object_height: float = .006
    max_object_height: float = .20
    cell_size: float = .005
    min_points: int = 12
    min_plane_points: int = 80
    min_plane_fraction: float = .25
    max_tilt_degrees: float = 15.
    min_rectangularity: float = .78
    max_block_length: float = .20
    # Map-space boxes exclude robot/camera geometry; masked pixels stay uncertain.
    exclusion_boxes: tuple = ()
    known_block_size: tuple = ()
    # Optional measured installation prior: map-frame tabletop Z interval (m).
    table_height_range: tuple = ()
    depth_error: float = .001
    calibration_position_error: float = .001
    calibration_yaw_error: float = math.radians(1)

    def __post_init__(self):
        vals = [self.min_depth, self.max_depth, self.plane_tolerance,
                self.min_object_height, self.max_object_height, self.cell_size,
                self.min_plane_fraction, self.max_tilt_degrees,
                self.min_rectangularity, self.max_block_length]
        if not np.isfinite(vals).all() or min(vals) <= 0 or self.min_depth >= self.max_depth:
            raise ValueError('Invalid detector configuration.')
        if self.min_object_height >= self.max_object_height or min(self.min_points, self.min_plane_points) < 3:
            raise ValueError('Invalid object/plane thresholds.')
        if self.min_plane_fraction > 1 or self.min_rectangularity > 1 or self.max_tilt_degrees >= 90:
            raise ValueError('Invalid fit thresholds.')
        errors = (self.depth_error, self.calibration_position_error, self.calibration_yaw_error)
        if not np.isfinite(errors).all() or min(errors) < 0:
            raise ValueError('Measurement error bounds must be finite and nonnegative.')
        if self.table_height_range:
            bounds = np.asarray(self.table_height_range)
            if bounds.shape != (2,) or not np.isfinite(bounds).all() or bounds[0] >= bounds[1]:
                raise ValueError('Table height range requires finite zmin < zmax.')
        if self.known_block_size:
            size = np.asarray(self.known_block_size)
            if size.shape != (3,) or not np.isfinite(size).all() or min(size) <= 0 or size[0] <= size[1]:
                raise ValueError('Known block dimensions require positive length > width, and height.')
        for box in self.exclusion_boxes:
            b = np.asarray(box)
            if b.shape != (6,) or not np.isfinite(b).all() or not (b[:3] < b[3:]).all():
                raise ValueError('Exclusion boxes require xmin,ymin,zmin,xmax,ymax,zmax.')


@dataclass
class Candidate:
    position: list
    size: list
    quaternion: list  # w,x,y,z; symmetry of long axis remains ambiguous
    quality: str
    resolved: bool
    reasons: list
    dimension_ranges: list
    pixels: list = field(default_factory=list)  # rectangle in source image, for overlay
    pose_uncertainty: dict = field(default_factory=dict)
    known_geometry: bool = False


@dataclass
class Detection:
    camera: str
    stamp: float
    frame: str
    calibration_revision: str
    viewpoint: str
    plane: object
    surface_points: np.ndarray
    object_points: np.ndarray
    uncertain_points: np.ndarray
    candidates: list
    warnings: list
    unexplained_points: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))


def backproject(capture, config):
    capture.validate()
    depth = capture.depth
    valid = np.isfinite(depth) & (depth >= config.min_depth) & (depth <= config.max_depth)
    if capture.mask is not None:
        valid &= ~capture.mask
    if capture.roi is not None:
        x0, y0, x1, y1 = capture.roi
        roi = np.zeros(depth.shape, dtype=bool)
        roi[y0:y1, x0:x1] = True
        valid &= roi
    v, u = np.nonzero(valid)
    z = depth[v, u]
    k = capture.intrinsics
    xyz = np.column_stack(((u-k.cx)*z/k.fx, (v-k.cy)*z/k.fy, z))
    xyz = xyz @ capture.camera_to_map[:3, :3].T + capture.camera_to_map[:3, 3]
    keep = np.ones(len(xyz), dtype=bool)
    for box in config.exclusion_boxes:
        b = np.asarray(box)
        keep &= ~((xyz >= b[:3]) & (xyz <= b[3:])).all(axis=1)
    return xyz[keep], np.column_stack((u, v))[keep]


def fit_plane(points, config):
    if config.table_height_range:
        low, high = config.table_height_range
        points = points[(points[:, 2] >= low) & (points[:, 2] <= high)]
    if len(points) < config.min_plane_points:
        return None
    # Deterministic bounded RANSAC; coarse evidence, full-resolution classification.
    rng = np.random.default_rng(17)
    coarse = points[rng.choice(len(points), min(1800, len(points)), replace=False)]
    best, best_count = None, 0
    supported_planes = []
    for _ in range(80):
        a, b, c = coarse[rng.choice(len(coarse), 3, replace=False)]
        n = np.cross(b-a, c-a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if n[2] < 0:
            n = -n
        if n[2] < math.cos(math.radians(config.max_tilt_degrees)):
            continue
        d = -float(n @ a)
        distances = coarse @ n + d
        count = int((np.abs(distances) <= config.plane_tolerance).sum())
        if count/len(coarse) >= config.min_plane_fraction:
            supported_planes.append((n.copy(), d))
        # Do not call a block top a table with a large measured lower surface.
        if (distances < -2*config.plane_tolerance).mean() > .05:
            continue
        if count > best_count:
            best, best_count = (n, d), count
    if best is None or best_count/len(coarse) < config.min_plane_fraction:
        return None
    n, d = best
    # A height interval that still contains multiple broad surfaces is ambiguous
    # too. Compare at a measured point so this does not depend on map translation.
    anchor = coarse[np.abs(coarse @ n+d) <= config.plane_tolerance].mean(axis=0)
    for other_n, other_d in supported_planes:
        if (n @ other_n > math.cos(math.radians(3))
                and abs(other_n @ anchor+other_d) > 4*config.plane_tolerance):
            return None
    if (coarse @ n+d > config.max_object_height).mean() > .05:
        return None
    inliers = points[np.abs(points @ n + d) <= config.plane_tolerance]
    if len(inliers) < config.min_plane_points:
        return None
    centre = inliers.mean(axis=0)
    _, _, axes = np.linalg.svd(inliers[::max(1, len(inliers)//2500)]-centre, full_matrices=False)
    n = axes[-1]
    if n[2] < 0:
        n = -n
    if n[2] < math.cos(math.radians(config.max_tilt_degrees)):
        return None
    return {'normal': n.tolist(), 'offset': -float(n @ centre),
            'fit_error': float(np.std(inliers @ n)), 'support': len(inliers),
            'boundary_complete': False}


def components(points, cell_size):
    cells, inverse = np.unique(np.floor(points[:, :2]/cell_size).astype(np.int64), axis=0, return_inverse=True)
    lookup = {tuple(cell): i for i, cell in enumerate(cells)}
    labels = np.full(len(cells), -1, dtype=int)
    label = 0
    for i, cell in enumerate(cells):
        if labels[i] >= 0:
            continue
        labels[i] = label
        todo = deque([tuple(cell)])
        while todo:
            x, y = todo.popleft()
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)):
                j = lookup.get((x+dx, y+dy))
                if j is not None and labels[j] < 0:
                    labels[j] = label
                    todo.append((x+dx, y+dy))
        label += 1
    point_labels = labels[inverse]
    order = np.argsort(point_labels, kind='stable')
    return np.split(order, np.flatnonzero(np.diff(point_labels[order]))+1)


def rotation_quaternion(rotation):
    # Matrix-to-quaternion via largest eigenvector, stable at 180 degree rotations.
    r = rotation
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[0,1]+r[1,0], r[0,2]+r[2,0], r[2,1]-r[1,2]],
                  [r[0,1]+r[1,0], r[1,1]-r[0,0]-r[2,2], r[1,2]+r[2,1], r[0,2]-r[2,0]],
                  [r[0,2]+r[2,0], r[1,2]+r[2,1], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], r.trace()]])/3
    _, vecs = np.linalg.eigh(k)
    q = vecs[:, -1][[3,0,1,2]]
    return (q if q[0] >= 0 else -q).tolist()


def fit_candidate(points, pixels, plane, config):
    n = np.asarray(plane['normal'])
    ex = np.array([1., 0., 0.])
    ex -= n*(n @ ex)
    ex /= np.linalg.norm(ex)
    ey = np.cross(n, ex)
    uv = points @ np.column_stack((ex, ey))
    # Minimum-area rectangle sampled at 1 degree; avoids a noisy PCA yaw estimate.
    angles = np.deg2rad(np.arange(90))
    best = None
    for angle in angles:
        r = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        local = uv @ r
        lo, hi = local.min(axis=0), local.max(axis=0)
        area = np.prod(hi-lo)
        if best is None or area < best[0]:
            best = area, r, lo, hi
    area, r, lo, hi = best
    dims = hi-lo + config.cell_size  # sampling uncertainty, not exact dimensions
    xy = ((hi+lo)/2) @ r.T
    height = points @ n + plane['offset']
    top = float(np.percentile(height, 90))
    centre = ex*xy[0] + ey*xy[1] + n*(-plane['offset']+top/2)
    rotation = np.column_stack((ex, ey)) @ r
    if dims[0] < dims[1]:
        dims = dims[::-1]
        rotation = np.column_stack((rotation[:, 1], -rotation[:, 0]))
    occupied = len(np.unique(np.floor((uv @ r-lo)/config.cell_size).astype(int), axis=0))
    fill = min(1., occupied*config.cell_size**2/max(area, config.cell_size**2))
    reasons = ['long-axis orientation has 180-degree symmetry',
               'hidden faces and dimensions are not verified']
    resolved = bool(fill >= config.min_rectangularity and dims[0] <= config.max_block_length)
    if not resolved:
        reasons.append('cluster may contain touching/overlapping blocks or incomplete geometry')
    low_height = float(np.percentile(height, 10))
    if top-low_height > max(.008, top*.45):
        resolved = False
        reasons.append('multiple heights or tilted geometry need another view')
    ranges = [[max(0., float(d-config.cell_size)), float(d+config.cell_size)] for d in dims]
    ranges.append([max(config.min_object_height, low_height), float(top+config.plane_tolerance)])
    bounds = [pixels.min(axis=0).tolist(), pixels.max(axis=0).tolist()]
    return Candidate(centre.tolist(), [float(dims[0]), float(dims[1]), top],
                     rotation_quaternion(np.column_stack((rotation, n))),
                     'provisional' if resolved else 'uncertain', resolved, reasons, ranges, bounds,
                     {'position_m': config.cell_size + config.depth_error + plane['fit_error']
                                    + config.calibration_position_error,
                      'yaw_rad': math.radians(3) + config.calibration_yaw_error,
                      'yaw_symmetry_rad': math.pi,
                      'bounds_type': 'sampling_and_configured_sensor_bounds; not calibrated probability'})


def fit_known_candidate(points, pixels, plane, config):
    """Fit a table-supported known cuboid; partial faces never confirm a pose."""
    n = np.asarray(plane['normal'])
    ex = np.array([1., 0., 0.])
    ex -= n*(n @ ex)
    ex /= np.linalg.norm(ex)
    ey = np.cross(n, ex)
    height = points @ n + plane['offset']
    length, width, thickness = config.known_block_size
    top = points[np.abs(height-thickness) <= config.plane_tolerance]
    candidate = fit_candidate(points, pixels, plane, config)
    candidate.known_geometry = True
    candidate.size = [length, width, thickness]
    candidate.dimension_ranges = [[v, v] for v in candidate.size]
    candidate.reasons = ['long-axis orientation has 180-degree symmetry']
    candidate.resolved = False
    if len(top) < config.min_points:
        candidate.reasons.append('known top face is missing; tilted, stacked, or partial object')
    else:
        uv = top @ np.column_stack((ex, ey))
        best = None
        for angle in np.deg2rad(np.arange(180)):
            r = np.array([[math.cos(angle), -math.sin(angle)],
                          [math.sin(angle), math.cos(angle)]])
            local = uv @ r
            lo, hi = local.min(axis=0), local.max(axis=0)
            span = hi-lo
            # Prefer evidence covering the known long and short axes.
            score = np.sum((span-np.array([length,width]))**2)
            if best is None or score < best[0]:
                best = score, r, lo, hi
        _, r, lo, hi = best
        span = hi-lo
        xy = ((hi+lo)/2) @ r.T
        rotation = np.column_stack((ex, ey)) @ r
        candidate.position = (ex*xy[0]+ey*xy[1]+n*(-plane['offset']+thickness/2)).tolist()
        candidate.quaternion = rotation_quaternion(np.column_stack((rotation,n)))
        local_all = points @ rotation
        centre_local = ((hi+lo)/2)
        outside = np.maximum(np.abs(local_all-centre_local)-np.array([length,width])/2,0)
        residual = float(np.percentile(np.linalg.norm(outside,axis=1),95))
        occupied = len(np.unique(np.floor((uv @ r-lo)/config.cell_size).astype(int),axis=0))
        fill = min(1.,occupied*config.cell_size**2/max(float(np.prod(span)),1e-9))
        candidate.resolved = bool(
            np.all(span >= np.array([length,width])-2*config.cell_size)
            and np.all(span <= np.array([length,width])+config.cell_size)
            and residual <= config.plane_tolerance
            and fill >= config.min_rectangularity
            and np.percentile(height,95) <= thickness+config.plane_tolerance)
        if not candidate.resolved:
            candidate.reasons.append('known cuboid pose requires more visible evidence')
    candidate.quality = 'provisional' if candidate.resolved else 'uncertain'
    candidate.pose_uncertainty = {
        'position_m': (config.cell_size + config.depth_error + plane['fit_error']
                       + config.calibration_position_error) if candidate.resolved else max(length,width)/2,
        'yaw_rad': (math.radians(3) + config.calibration_yaw_error) if candidate.resolved else math.pi/2,
        'yaw_symmetry_rad': math.pi,
        'model': 'table_supported_cuboid',
        'bounds_type': 'sampling_and_configured_sensor_bounds; not calibrated probability'}
    return candidate


class Detector:
    def __init__(self, config=None, segmenter=None):
        self.config = config or DetectorConfig()
        self.segmenter = segmenter

    def detect(self, capture: Capture):
        c = self.config
        points, pixels = backproject(capture, c)
        if len(points) < c.min_plane_points:
            raise ValueError('Insufficient valid depth; observation is unusable.')
        plane = fit_plane(points, c)
        if plane is None:
            return Detection(capture.camera, capture.stamp, capture.map_frame,
                capture.calibration_revision, capture.viewpoint, None,
                np.empty((0,3)), np.empty((0,3)), points, [],
                ['Insufficient table surface evidence; geometry remains unknown.'])
        heights = points @ np.asarray(plane['normal']) + plane['offset']
        surface = np.abs(heights) <= c.plane_tolerance
        above = (heights >= c.min_object_height) & (heights <= c.max_object_height)
        candidates = []
        object_points, object_pixels = points[above], pixels[above]
        groups = components(object_points, c.cell_size) if above.any() else []
        fitter = fit_known_candidate if c.known_block_size else fit_candidate
        unresolved_groups = []
        accounted = np.zeros(len(object_points), dtype=bool)

        def fit(indices):
            candidate = fitter(object_points[indices], object_pixels[indices], plane, c)
            low, high = candidate.pixels
            x0, y0, x1, y1 = capture.roi or (0, 0, capture.intrinsics.width, capture.intrinsics.height)
            if low[0] <= x0 or low[1] <= y0 or high[0] >= x1-1 or high[1] >= y1-1:
                candidate.resolved = False
                candidate.quality = 'uncertain'
                candidate.reasons.append('candidate is clipped by the capture boundary')
            return candidate

        # Depth already supplies a complete fit for separated blocks. Running
        # segmentation on those groups costs inference time and can manufacture
        # extra hypotheses from harmless mask boundary differences.
        for indices in groups:
            if len(indices) >= c.min_points:
                candidate = fit(indices)
                if candidate.resolved:
                    candidates.append(candidate)
                    accounted[indices] = True
                else:
                    unresolved_groups.append((indices, candidate))

        masks = (self.segmenter.segment(capture, object_pixels,
                 [indices for indices, _ in unresolved_groups])
                 if self.segmenter is not None and unresolved_groups else [])
        for indices, original in unresolved_groups:
            segmented = []
            for mask in masks:
                selected = indices[mask[object_pixels[indices, 1], object_pixels[indices, 0]]]
                if len(selected) >= c.min_points:
                    candidate = fit(selected)
                    if candidate.resolved:
                        segmented.append(candidate)
                        candidates.append(candidate)
            if not segmented:
                # A missing or partial mask cannot remove a measured depth group.
                candidates.append(original)
                accounted[indices] = True
                continue
            # A resolved mask explains omitted edge pixels inside its fitted box.
            # Residual depth outside every box may be another touching object.
            explained = np.zeros(len(indices), dtype=bool)
            for candidate in segmented:
                w, x, y, z = candidate.quaternion
                rotation = np.array([
                    [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                    [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                    [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
                local = (object_points[indices]-candidate.position) @ rotation
                explained |= (np.abs(local) <= np.asarray(candidate.size)/2+c.cell_size).all(axis=1)
            accounted[indices[explained]] = True
            remaining = indices[~explained]
            if len(remaining):
                for group in components(object_points[remaining], c.cell_size):
                    residual = remaining[group]
                    if len(residual) >= c.min_points:
                        candidate = fit(residual)
                        candidate.resolved = False
                        candidate.quality = 'uncertain'
                        candidate.reasons.append('above-table depth outside resolved segment')
                        candidates.append(candidate)
                        accounted[residual] = True
        warnings = []
        if self.segmenter is not None and above.any() and not candidates:
            warnings.append("SAM 2 did not resolve above-table evidence; inspect another view.")
        if any(not candidate.resolved for candidate in candidates):
            warnings.append('Unresolved object groups require inspection.')
        return Detection(capture.camera, capture.stamp, capture.map_frame,
            capture.calibration_revision, capture.viewpoint, plane, points[surface],
            points[above], points[~(surface | above)], candidates, warnings,
            np.concatenate((object_points[~accounted], points[heights > c.max_object_height])))
