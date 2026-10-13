"""Verify the example against real MuJoCo grasp and release physics."""
import json
import os
from pathlib import Path

import numpy as np
import pytest

from example_app.pick_drop import PICK_PATH, plan

mujoco = pytest.importorskip('mujoco')
scene_module = pytest.importorskip('piper_mujoco_sim.scene')
ROBOT = Path(os.environ.get('PIPER_TEST_ROBOT', '/opt/mujoco_menagerie/agilex_piper/piper.xml'))
pytestmark = pytest.mark.skipif(not ROBOT.exists(), reason='Menagerie PIPER model required')


@pytest.fixture
def lab():
    root = next(parent for parent in Path(__file__).parents if (parent / 'sim/piper_mujoco_sim/config/lab_scene.json').exists())
    path = root / 'sim/piper_mujoco_sim/config/lab_scene.json'
    config = json.loads(path.read_text())
    model = mujoco.MjModel.from_xml_string(scene_module.build_scene(ROBOT, path))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key('home').id)
    mujoco.mj_forward(model, data)
    return model, data, config


def test_fixed_waypoints_follow_known_block(lab):
    model, data, config = lab
    for angles, height in zip(PICK_PATH, np.linspace(.14, .025, len(PICK_PATH))):
        for i, value in enumerate(angles, 1):
            data.qpos[model.joint(f'joint{i}').qposadr[0]] = value
        mujoco.mj_forward(model, data)
        wrist = data.body('link6')
        expected_orientation = wrist.xmat.reshape(3, 3)
        position = wrist.xpos + expected_orientation @ [0, 0, .12]
        np.testing.assert_allclose(position, [.4, -.33, height], atol=.0003)


def test_reset_restores_blocks_and_preserves_active_scene(lab):
    model, data, config = lab
    model.body('wrist_camera_mount').pos[0] += .01
    mount_position = model.body('wrist_camera_mount').pos.copy()
    config['block_positions'][0] = [.2, -.15]
    data.qpos[model.joint('block_1_joint').qposadr[0]:][:3] = [.3, -.1, .2]
    data.qvel[:] = 1
    data.time = 12
    scene_module.reset_scene(model, data, config)
    np.testing.assert_allclose(data.qpos[:8], model.key('home').qpos[:8])
    np.testing.assert_allclose(data.qvel, 0)
    np.testing.assert_allclose(data.body('block_1').xpos, [.2, -.15, .0085])
    np.testing.assert_allclose(model.body('wrist_camera_mount').pos, mount_position)
    assert data.time == 0


def test_block_physically_lifts_and_lands(lab):
    model, data, config = lab
    initial = data.qpos[:7].tolist()
    selected, stages = plan(scene_module.scene_snapshot(model, data, config), initial)
    limits = np.array([model.joint(f'joint{i}').range for i in range(1, 7)])
    start = initial
    lifted_height = 0
    for _ in range(250):
        mujoco.mj_step(model, data)
    for stage in stages:
        assert all(type(value) is float for value in stage.target)
        assert np.all(np.array(stage.target[:6]) >= limits[:, 0])
        assert np.all(np.array(stage.target[:6]) <= limits[:, 1])
        steps = round(stage.duration / model.opt.timestep)
        for step in range(steps):
            progress = (step + 1) / steps
            blend = progress*progress*(3-2*progress)
            data.ctrl[:] = [a + blend*(b-a) for a, b in zip(start, stage.target)]
            mujoco.mj_step(model, data)
        for _ in range(100):
            mujoco.mj_step(model, data)
        start = stage.target
        if stage.name == 'hold above table':
            lifted_height = data.body(selected).xpos[2]
            assert data.qpos[6] > .005  # Fingers stopped against the block.
    assert lifted_height > .08
    assert data.qpos[6] > .028  # Open after releasing.
    np.testing.assert_allclose(data.body(selected).xpos[2], config['block_size'][2]/2, atol=.001)
    assert abs(data.body(selected).xpos[0]) < config['table_size'][0]
    assert abs(data.body(selected).xpos[1]) < config['table_size'][1]
    # No block attachment/weld was added to fake the grasp.
    assert model.neq == 1
