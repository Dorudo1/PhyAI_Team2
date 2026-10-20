"""Generate a synthetic depth recording to exercise discovery without ROS/cameras.

Ground truth is used only to render depth pixels. The detector reads the resulting
images and camera calibration; the manifest contains no block count or poses.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def render(camera_x, objects):
    width,height,fx=320,240,480.
    v,u=np.indices((height,width))
    cx,cy=(width-1)/2,(height-1)/2
    rx,ry=(u-cx)/fx,-(v-cy)/fx
    depth=np.ones((height,width),dtype=np.float32)
    rgb=np.full((height,width,3),40,dtype=np.uint8)
    for x,y,length,width_m,h,yaw in objects:
        z=1-h
        px,py=rx*z+camera_x-x,ry*z-y
        a,b=np.cos(yaw)*px+np.sin(yaw)*py,-np.sin(yaw)*px+np.cos(yaw)*py
        mask=(abs(a)<length/2)&(abs(b)<width_m/2)
        depth[mask]=np.minimum(depth[mask],z)
        rgb[mask]=[190,150,100]
    return depth,rgb,dict(width=width,height=height,fx=fx,fy=fx,cx=cx,cy=cy)


def create(directory):
    directory=Path(directory)
    directory.mkdir(parents=True,exist_ok=True)
    objects=[(-.10,.04,.075,.025,.015,.4),(.13,-.06,.07,.03,.018,-.6),
             (.65,.04,.075,.025,.015,.3)]
    captures=[]
    for i,(camera,viewpoint,x) in enumerate([('overhead','overhead',0.),('wrist','right',.6),('wrist','centre',0.)]):
        depth,rgb,intrinsics=render(x,objects)
        np.save(directory/f'depth_{i}.npy',depth)
        np.save(directory/f'rgb_{i}.npy',rgb)
        transform=np.diag([1.,-1.,-1.,1.]);transform[:3,3]=[x,0.,1.]
        captures.append(dict(camera=camera,viewpoint=viewpoint,stamp=float(i+1),
            depth=f'depth_{i}.npy',depth_unit='m',rgb=f'rgb_{i}.npy',
            optical_frame=f'{camera}_optical',map_frame='base_link',calibration_revision='synthetic_v1',
            intrinsics=intrinsics,camera_to_map=transform.tolist(),stationary_verified=True))
    manifest=dict(version=1,map_frame='base_link',viewpoints=['right','centre'],
                  scope=[-.4,1.,-.4,.4],captures=captures)
    path=directory/'scan.json'
    path.write_text(json.dumps(manifest,indent=2)+'\n')
    return path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    print(create(parser.parse_args().directory))


if __name__=='__main__':main()
