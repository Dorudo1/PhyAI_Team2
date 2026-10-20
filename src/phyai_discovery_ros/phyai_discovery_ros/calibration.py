"""JSON calibration adapter; converts file data to the existing CameraInfo path."""
import json
from pathlib import Path

import numpy as np
from sensor_msgs.msg import CameraInfo

from phyai_discovery import Intrinsics


def load_calibration(path):
    """Return (CameraInfo, revision) for a version-1 pinhole calibration file."""
    try:
        value = json.loads(Path(path).read_text())
        if value['version'] != 1 or isinstance(value['version'], bool):
            raise ValueError('Unsupported calibration version.')
        frame, revision = value['frame_id'], value['calibration_revision']
        if not all(isinstance(item, str) and item.strip() for item in (frame, revision)):
            raise ValueError('Calibration requires frame_id and calibration_revision.')
        k = Intrinsics(**value['intrinsics'])
        if any(type(item) is not int for item in (k.width, k.height)):
            raise ValueError('Calibration image dimensions must be integers.')
        if any(isinstance(item, bool) for item in (k.fx, k.fy, k.cx, k.cy)):
            raise ValueError('Calibration intrinsics must be numeric.')
        k.validate()
        distortion = np.asarray(value.get('distortion', []), dtype=float)
        projection = np.asarray(value.get('projection', [0.]*12), dtype=float)
        if (distortion.ndim != 1 or not np.isfinite(distortion).all()
                or projection.shape != (12,) or not np.isfinite(projection).all()):
            raise ValueError('Invalid distortion or rectified projection.')
        info = CameraInfo(width=k.width, height=k.height)
        info.header.frame_id = frame
        info.k = [float(k.fx), 0., float(k.cx), 0., float(k.fy), float(k.cy), 0., 0., 1.]
        info.d = distortion.tolist()
        info.p = projection.tolist()
        return info, revision
    except (KeyError, TypeError, ValueError, OSError) as error:
        raise ValueError(f'Invalid calibration file {path}: {error}') from error
