"""Validated RGB-D capture contracts; no scene-file or robot-command dependency."""
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def validate(self):
        if self.width <= 0 or self.height <= 0 or not all(np.isfinite(
                [self.fx, self.fy, self.cx, self.cy])) or min(self.fx, self.fy) <= 0:
            raise ValueError('Invalid calibrated intrinsics.')


@dataclass(frozen=True)
class Capture:
    camera: str
    stamp: float
    optical_frame: str
    map_frame: str
    calibration_revision: str
    intrinsics: Intrinsics
    depth: np.ndarray  # optical-axis distance, metres
    camera_to_map: np.ndarray
    viewpoint: str = 'overhead'
    rgb: Optional[np.ndarray] = None
    roi: Optional[tuple] = None  # pixel x0, y0, x1, y1
    mask: Optional[np.ndarray] = None  # True means exclude (robot/contamination)

    def validate(self):
        self.intrinsics.validate()
        if self.camera not in ('overhead', 'wrist'):
            raise ValueError('Camera must be overhead or wrist.')
        if not all(isinstance(s, str) and s for s in (self.optical_frame,
                self.map_frame, self.calibration_revision, self.viewpoint)):
            raise ValueError('Capture frames, viewpoint and calibration revision are required.')
        if not np.isfinite(self.stamp) or self.stamp <= 0:
            raise ValueError('Capture requires a positive acquisition timestamp.')
        shape = (self.intrinsics.height, self.intrinsics.width)
        if self.depth.shape != shape or self.depth.dtype.kind != 'f':
            raise ValueError('Depth must be a matching floating-point image in metres.')
        if self.rgb is not None and (self.rgb.shape != (*shape, 3) or self.rgb.dtype != np.uint8):
            raise ValueError('RGB must be matching uint8 RGB pixels.')
        if self.mask is not None and (self.mask.shape != shape or self.mask.dtype != np.bool_):
            raise ValueError('Exclusion mask must be a matching boolean image.')
        if self.roi is not None:
            x0, y0, x1, y1 = self.roi
            if not all(isinstance(v, int) for v in self.roi) or not (
                    0 <= x0 < x1 <= shape[1] and 0 <= y0 < y1 <= shape[0]):
                raise ValueError('Invalid pixel ROI.')
        t = np.asarray(self.camera_to_map)
        if t.shape != (4, 4) or not np.isfinite(t).all() or not np.allclose(t[3], [0, 0, 0, 1]):
            raise ValueError('Missing or invalid capture-time transform.')
        r = t[:3, :3]
        if not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(r), 1., atol=1e-5):
            raise ValueError('Camera transform must contain a proper rotation.')
        return self


def decode_depth(data, width, height, step, encoding, bigendian=False, scale=.001):
    """Decode ROS depth rows including padding; 16UC1 uses configured scale."""
    if encoding not in ('16UC1', '32FC1'):
        raise ValueError('Supported depth encodings are 16UC1 and 32FC1.')
    if not np.isfinite(scale) or scale <= 0 or min(width, height) <= 0:
        raise ValueError('Invalid depth scale or dimensions.')
    dtype = np.dtype(('>' if bigendian else '<') + ('u2' if encoding == '16UC1' else 'f4'))
    if step < width*dtype.itemsize or len(data) != height*step:
        raise ValueError('Depth buffer does not match row stride and dimensions.')
    values = np.ndarray((height, width), dtype=dtype, buffer=bytes(data), strides=(step, dtype.itemsize))
    depth = values.astype(np.float32)
    if encoding == '16UC1':
        depth *= scale
    depth[~np.isfinite(depth) | (depth <= 0)] = np.nan
    return depth


def load_capture(entry, base):
    """Read a manifest entry containing measured depth, intrinsics and extrinsics."""
    base = Path(base)
    depth = np.load(base / entry['depth'], allow_pickle=False)
    if entry.get('depth_unit') == 'mm':
        depth = depth.astype(np.float32)*.001
    elif entry.get('depth_unit') == 'm':
        depth = depth.astype(np.float32)
    else:
        raise ValueError('Recorded depth_unit must explicitly be m or mm.')
    capture = Capture(
        camera=entry['camera'], stamp=float(entry['stamp']),
        optical_frame=entry['optical_frame'], map_frame=entry['map_frame'],
        calibration_revision=entry['calibration_revision'],
        intrinsics=Intrinsics(**entry['intrinsics']), depth=depth,
        camera_to_map=np.asarray(entry['camera_to_map'], dtype=float),
        viewpoint=entry['viewpoint'],
        rgb=np.load(base / entry['rgb'], allow_pickle=False) if entry.get('rgb') else None,
        roi=tuple(entry['roi']) if entry.get('roi') else None,
        mask=np.load(base / entry['mask'], allow_pickle=False) if entry.get('mask') else None)
    return capture.validate()


def load_manifest(path):
    path = Path(path)
    manifest = json.loads(path.read_text())
    if manifest.get('version') != 1 or not isinstance(manifest.get('captures'), list):
        raise ValueError('Expected version 1 manifest with captures list.')
    return manifest, path.parent
