"""Rendered depth fixture for app adapter/handoff integration tests."""
import math
import numpy as np
from phyai_discovery import Capture, Intrinsics


def image_capture(blocks=(), camera='overhead', stamp=1., viewpoint='overhead', camera_x=0.):
    # Orthogonal downward pinhole camera, 1m above z=0. Ray/surface intersections
    # render top faces at arbitrary yaw. Ground truth is confined to the fixture.
    k = Intrinsics(160, 120, 240., 240., 79.5, 59.5)
    v, u = np.indices((k.height, k.width))
    rx, ry = (u-k.cx)/k.fx, -(v-k.cy)/k.fy
    depth = np.ones((k.height, k.width), dtype=np.float32)
    for x, y, length, width, height, angle in blocks:
        z = 1-height
        px, py = rx*z+camera_x-x, ry*z-y
        a = math.cos(angle)*px+math.sin(angle)*py
        b = -math.sin(angle)*px+math.cos(angle)*py
        mask = (abs(a) <= length/2) & (abs(b) <= width/2)
        depth[mask] = np.minimum(depth[mask], z)
    t = np.diag([1., -1., -1., 1.])
    t[:3,3] = [camera_x, 0., 1.]
    return Capture(camera, stamp, f'{camera}_optical', 'base_link', 'calib_1', k, depth, t, viewpoint)
