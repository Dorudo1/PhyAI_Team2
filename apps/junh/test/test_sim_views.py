"""Validate configured scan poses with the fixed camera and actual arm model."""
import json
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')
from piper_mujoco_sim.scene import build_scene, reset_scene

ROOT = Path(__file__).resolve().parents[3]
ROBOT = Path('/opt/mujoco_menagerie/agilex_piper/piper.xml')
pytestmark = pytest.mark.skipif(not ROBOT.exists(), reason='Menagerie model is required')


def test_scan_views_are_upright_varied_and_paths_clear_the_scene():
    scene_path = ROOT/'sim/piper_mujoco_sim/config/lab_scene.json'
    config = json.loads((ROOT/'apps/junh/config/mujoco.json').read_text())
    scene = json.loads(scene_path.read_text())
    model = mujoco.MjModel.from_xml_string(build_scene(ROBOT, scene_path))
    data = mujoco.MjData(model)
    reset_scene(model, data, scene)
    addresses = [model.joint(f'joint{i}').qposadr[0] for i in range(1, 7)]
    limits = np.array([model.joint(f'joint{i}').range for i in range(1, 7)])
    labels = config['viewpoints'] + [view for views in config['inspection_rounds'] for view in views]
    positions, pitches = [], []
    previous = data.qpos[addresses].copy()
    for label in labels:
        commanded = config['sim_scan']['targets'][label]
        assert commanded[6] == 0.0
        data.qpos[model.joint('joint7').qposadr[0]] = commanded[6]
        data.qpos[model.joint('joint8').qposadr[0]] = -commanded[6]
        target = np.array(commanded[:6])
        assert np.all(target >= limits[:, 0]) and np.all(target <= limits[:, 1])
        for blend in np.linspace(0, 1, 31):
            data.qpos[addresses] = previous + blend*(target-previous)
            mujoco.mj_forward(model, data)
            # Ignore settled block/table contacts; reject robot penetration.
            for contact in data.contact:
                if contact.dist < -.001:
                    bodies = model.geom_bodyid[[contact.geom1, contact.geom2]]
                    assert not any(0 < body < model.body('block_1').id for body in bodies)
        camera = data.camera('wrist')
        rotation = camera.xmat.reshape(3, 3)
        assert camera.xpos[2] > .25
        assert rotation[2, 2] > .3  # optical forward (-Z) points down
        assert rotation[2, 1] > .15  # image up points upward
        positions.append(camera.xpos.copy())
        pitches.append(rotation[2, 2])
        previous = target
    # A base-only orbit cannot supply these changes in height and viewing pitch.
    assert np.ptp(np.array(positions)[:, 2]) > .1
    assert np.ptp(pitches) > .1
