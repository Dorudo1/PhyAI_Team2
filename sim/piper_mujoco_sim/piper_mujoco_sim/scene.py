"""Compose the measured lab layout with the pinned Menagerie robot.

World origin is the combined tabletop center, Z up. The mounting post and
overhead camera use offsets from that origin. Wrist camera mount pose uses the
gripper root frame at zero wrist roll, fixed to the arm casing; camera pose is
relative to that mount.
"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco


def vector(values):
    return ' '.join(str(value) for value in values)


def reset_scene(model, data, config=None):
    """Restore the robot home pose and blocks, keeping active scene settings."""
    home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'home')
    if home >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home)
    else:
        mujoco.mj_resetData(model, data)
    if config is not None:
        for index, (x, y) in enumerate(config['block_positions'], 1):
            joint = model.joint(f'block_{index}_joint')
            offset = joint.qposadr[0]
            data.qpos[offset:offset + 7] = [x, y, config['block_size'][2]/2 + .001, 1, 0, 0, 0]
    mujoco.mj_forward(model, data)


def scene_snapshot(model, data, config, paused=False, restart_id=0):
    """Simulator-only ground-truth poses for ROS example planning and checks."""
    from .inspection import scan_geometry
    return {
        'simulation_time': float(data.time), 'paused': paused, 'restart_id': restart_id,
        'scan_geometry': scan_geometry(model),
        'robot': {
            'base_position': model.body('base_link').pos.tolist(),
            'base_quaternion': model.body('base_link').quat.tolist(),
            'joints': [{'position': model.body(f'link{i}').pos.tolist(),
                        'quaternion': model.body(f'link{i}').quat.tolist(),
                        'axis': model.joint(f'joint{i}').axis.tolist(),
                        'limits': model.joint(f'joint{i}').range.tolist()}
                       for i in range(1, 7)]},
        'block_size': config['block_size'],
        'blocks': [{'name': f'block_{i + 1}',
                    'position': data.body(f'block_{i + 1}').xpos.tolist(),
                    'quaternion': data.body(f'block_{i + 1}').xquat.tolist()}
                   for i in range(len(config['block_positions']))]}


def build_scene(robot_path, config_path):
    config = json.loads(Path(config_path).read_text())
    root = ET.parse(robot_path).getroot()
    root.set('model', 'piper_block_stacking_lab')
    compiler = root.find('compiler')
    compiler.set('meshdir', str((Path(robot_path).parent / compiler.get('meshdir', '')).resolve()))
    world = root.find('worldbody')
    for light in world.findall('light'):
        world.remove(light)
    width, depth = config['table_size']
    gap, thickness = config['table_gap'], config['table_thickness']
    height = config['table_height']
    ET.SubElement(root, 'visual')
    visual = root.find('visual')
    ET.SubElement(visual, 'global', offwidth='960', offheight='720')
    ET.SubElement(visual, 'map', znear='0.005', zfar='10')
    # Office-style ceiling lighting with uniform room fill. Relative RGB
    # strengths approximate the appearance; MuJoCo does not calibrate lux here.
    lighting = config.get('lighting', {})
    ET.SubElement(visual, 'headlight', ambient=vector(lighting.get('ambient', [.3, .3, .3])),
                  diffuse=vector(lighting.get('camera_fill', [.2, .2, .2])),
                  specular='0 0 0')
    for row in range(2):
        for column in range(2):
            x = (column - .5) * (width + gap)
            y = (row - .5) * (depth + gap)
            ET.SubElement(world, 'light', name=f'ceiling_light_{row}_{column}',
                          pos=vector([x, y, lighting.get('ceiling_height', 2.25)]),
                          dir='0 0 -1', cutoff='90', exponent='0', attenuation='1 0 0',
                          ambient='0 0 0', diffuse=vector(lighting.get('diffuse', [.3, .3, .3])),
                          specular=vector(lighting.get('specular', [.02, .02, .02])),
                          castshadow='false')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='3 3 .1',
                  pos=vector([0, 0, -height]), rgba='.22 .24 .26 1')
    for row in range(2):
        for column in range(2):
            x, y = (column - .5) * (width + gap), (row - .5) * (depth + gap)
            ET.SubElement(world, 'geom', name=f'table_{row}_{column}', type='box',
                          pos=vector([x, y, -thickness / 2]),
                          size=vector([width / 2, depth / 2, thickness / 2]),
                          rgba='.07 .075 .085 1', friction='0.8 .005 .0001')
            for dx in (-width / 2 + .04, width / 2 - .04):
                for dy in (-depth / 2 + .04, depth / 2 - .04):
                    ET.SubElement(world, 'geom', type='box',
                                  pos=vector([x + dx, y + dy, -(height + thickness) / 2]),
                                  size=vector([.02, .02, (height - thickness) / 2]),
                                  rgba='.55 .57 .6 1')
    radius = config['post_radius']
    base_position = [*config['post_position'], 0]
    base = world.find("body[@name='base_link']")
    base.set('pos', vector(base_position))
    yaw = math.radians(config['robot_yaw_degrees']) / 2
    base.set('quat', vector([math.cos(yaw), 0, 0, math.sin(yaw)]))
    ET.SubElement(world, 'geom', name='mount_post', type='cylinder',
                  pos=vector([*base_position[:2], -height / 2]),
                  size=vector([radius, height / 2]), rgba='.4 .42 .45 1')

    def camera(parent, name, position, quaternion, wrist_camera=False):
        body = ET.SubElement(parent, 'body', name='wrist_camera_body' if wrist_camera else name + '_mount',
                             pos=vector(position), quat=vector(quaternion))
        # Lens at body origin; housing behind the lens so it does not obscure the view.
        ET.SubElement(body, 'geom', name=name + '_housing', type='box',
                      size='.07 .018 .012' if wrist_camera else '.07 .012 .012',
                      pos='0 0 .016', rgba='.75 .77 .8 1',
                      contype='0', conaffinity='0', density='0')
        if wrist_camera:
            # Camera-frame X is the bar's width; camera-frame Y points upward
            # toward gripper-root +X. The sensor face looks along camera -Z.
            ET.SubElement(body, 'geom', name='wrist_sensor_face', type='box',
                          size='.065 .014 .001', pos='0 0 .003', rgba='.08 .09 .1 1',
                          contype='0', conaffinity='0', density='0')
            for index, offset in enumerate((-.045, 0, .045)):
                ET.SubElement(body, 'geom', name=f'wrist_lens_{index}', type='cylinder',
                              size='.005 .0005', pos=vector([offset, 0, .0015]),
                              rgba='.04 .08 .12 1', contype='0', conaffinity='0', density='0')
            for index, offset in enumerate((-.04, .04)):
                ET.SubElement(body, 'geom', name=f'wrist_rear_screw_{index}', type='cylinder',
                              size='.0018 .001', pos=vector([offset, .009, .029]),
                              rgba='.35 .37 .4 1', contype='0', conaffinity='0', density='0')
        # Optical roll changes image orientation while the housing stays across the wrist.
        roll = math.radians(config.get('wrist_camera_roll_degrees', 0)) / 2 if wrist_camera else 0
        fovy = config.get('overhead_camera_fovy', config['camera_fovy']) if name == 'overhead' else config['camera_fovy']
        ET.SubElement(body, 'camera', name=name, fovy=str(fovy),
                      quat=vector([math.cos(roll), 0, 0, math.sin(roll)]))

    camera(world, 'overhead', config['overhead_position'],
           config.get('overhead_quaternion', [1, 0, 0, 0]))
    arm_casing = world.find(".//body[@name='link5']")
    wrist = arm_casing.find("body[@name='link6']")
    # The camera sits on the casing before joint6, so it must not inherit the
    # gripper's wrist roll. Copy only the zero-roll wrist transform (no joint).
    # This preserves the configured placement and axes at joint6 == 0.
    arm_reference = ET.SubElement(arm_casing, 'body', name='wrist_mount_reference',
                                  pos=wrist.get('pos'), quat=wrist.get('quat'))
    finger = wrist.find("body[@name='link7']")
    gripper_root = ET.SubElement(arm_reference, 'body', name='gripper_root_mount',
                                 pos=finger.get('pos'), quat=finger.get('quat'))
    # The pad defines the assembly frame; lens/housing pose is relative to it.
    camera_mount = ET.SubElement(gripper_root, 'body', name='wrist_camera_mount',
                                 pos=vector(config['wrist_camera_mount_position']),
                                 quat=vector(config.get('wrist_camera_mount_quaternion', [1, 0, 0, 0])))
    ET.SubElement(camera_mount, 'geom', name='wrist_camera_mount_plate', type='box',
                  pos='0 0 0', size='.006 .012 .022', rgba='.2 .22 .24 1',
                  contype='0', conaffinity='0', density='0')
    camera(camera_mount, 'wrist', config['wrist_camera_position'],
           config['wrist_camera_quaternion'], wrist_camera=True)
    colors = ['.72 .48 .25 1', '.85 .65 .36 1', '.65 .38 .18 1']
    block_qpos = []
    for index, (x, y) in enumerate(config['block_positions']):
        z = config['block_size'][2] / 2 + .001
        body = ET.SubElement(world, 'body', name=f'block_{index + 1}', pos=vector([x, y, z]))
        ET.SubElement(body, 'freejoint', name=f'block_{index + 1}_joint')
        ET.SubElement(body, 'geom', type='box',
                      size=vector([value / 2 for value in config['block_size']]),
                      mass=str(config['block_mass']), friction=vector(config['block_friction']),
                      condim='4', rgba=colors[index % len(colors)])
        block_qpos.extend([x, y, z, 1, 0, 0, 0])
    # Free bodies add seven position values each; preserve the arm's home pose.
    for key in root.findall('./keyframe/key'):
        key.set('qpos', key.get('qpos', '') + ' ' + vector(block_qpos))
    return ET.tostring(root, encoding='unicode')
