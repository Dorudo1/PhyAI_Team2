"""MuJoCo rendered RGB-D and calibration in ROS optical-frame conventions."""
import math

import numpy as np
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image


def camera_info(width, height, fovy, frame, stamp):
    focal = height / (2 * math.tan(math.radians(fovy) / 2))
    cx, cy = (width-1)/2, (height-1)/2
    msg = CameraInfo()
    msg.header.frame_id, msg.header.stamp = frame, stamp
    msg.width, msg.height = width, height
    msg.distortion_model, msg.d = 'plumb_bob', [0.]*5
    msg.k = [focal,0.,cx,0.,focal,cy,0.,0.,1.]
    msg.r = [1.,0.,0.,0.,1.,0.,0.,0.,1.]
    msg.p = [focal,0.,cx,0.,0.,focal,cy,0.,0.,0.,1.,0.]
    return msg


def image_msg(pixels, encoding, frame, stamp):
    pixels = np.ascontiguousarray(pixels)
    msg = Image()
    msg.header.frame_id, msg.header.stamp = frame, stamp
    msg.height, msg.width = pixels.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = msg.width * pixels.dtype.itemsize * (3 if encoding == 'rgb8' else 1)
    msg.data = pixels.tobytes()
    return msg


def transform(position, quaternion, parent, child, stamp):
    msg = TransformStamped()
    msg.header.frame_id, msg.header.stamp, msg.child_frame_id = parent, stamp, child
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float,position)
    w,x,y,z = quaternion
    msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z = map(float,(w,x,y,z))
    return msg
