"""Random layouts use actual contact dynamics and preserve a rotated baseline."""
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
from http.server import ThreadingHTTPServer
from types import MethodType, SimpleNamespace

import mujoco
import numpy as np
import pytest

from piper_mujoco_sim.scatter import BlockScatter
from piper_mujoco_sim.scene import build_scene, reset_scene, scene_snapshot
from piper_mujoco_sim.web import make_handler

ROBOT = Path(os.environ.get('PIPER_TEST_ROBOT', '/opt/mujoco_menagerie/agilex_piper/piper.xml'))
pytestmark = pytest.mark.skipif(not ROBOT.exists(), reason='Menagerie PIPER model is required')


@pytest.fixture
def scene():
    path = Path(__file__).parents[1]/'config/lab_scene.json'
    config = json.loads(path.read_text())
    model = mujoco.MjModel.from_xml_string(build_scene(ROBOT, path))
    data = mujoco.MjData(model)
    reset_scene(model, data, config)
    return model, data, config, BlockScatter(model, data, config)


@pytest.mark.parametrize('table', ['all', '0_1'])
def test_seeded_layout_is_settled_supported_and_does_not_mutate_live_scene(scene, table):
    model, data, config, scatter = scene
    original = data.qpos.copy()
    poses = scatter.plan(config, seed=42, table=table)
    np.testing.assert_array_equal(data.qpos, original)
    assert scatter.baseline is None
    np.testing.assert_array_equal(poses, scatter.plan(config, seed=42, table=table))
    assert not np.allclose(poses, scatter.plan(config, seed=43, table=table))
    assert any(abs(pose[6]) > .1 for pose in poses)
    scatter.apply(poses)
    for _ in range(500):
        mujoco.mj_step(model, data)
    for joint, expected in zip(scatter.joints, poses):
        q, v = joint.qposadr[0], joint.dofadr[0]
        np.testing.assert_allclose(data.qpos[q:q+3], expected[:3], atol=.001)
        assert np.max(np.abs(data.qvel[v:v+6])) < .001
        assert abs(data.qpos[q+2]-config['block_size'][2]/2) < .001
        if table == '0_1':
            assert data.qpos[q] > 0 and data.qpos[q+1] < 0
    for contact in data.contact:
        bodies = model.geom_bodyid[[contact.geom1, contact.geom2]]
        if any(int(body) in scatter.body_ids for body in bodies):
            assert contact.dist > -.001
            # Blocks contact only table surfaces, never each other or the robot.
            names = [model.geom(int(g)).name for g in (contact.geom1, contact.geom2)]
            assert any(name and name.startswith('table_') for name in names)


def test_repeated_scatters_restore_exact_original_positions_and_quaternions(scene):
    model, data, config, scatter = scene
    joint = scatter.joints[0]
    q = joint.qposadr[0]
    data.qpos[q+3:q+7] = [np.cos(.3), 0, 0, np.sin(.3)]
    original = scatter.poses(data)
    scatter.remember()
    for seed in (3, 8):
        scatter.apply(scatter.plan(config, seed=seed))
        scatter.remember()  # Never replace the original baseline.
    reset_scene(model, data, config)
    scatter.restore()
    np.testing.assert_array_equal(scatter.poses(data), original)
    for joint in scatter.joints:
        v = joint.dofadr[0]
        np.testing.assert_array_equal(data.qvel[v:v+6], 0)
    scatter.apply(scatter.plan(config, seed=2))
    scatter.restore()
    np.testing.assert_array_equal(scatter.poses(data), original)


def test_startup_scatter_precedes_first_scene_and_reset_restores_aligned_layout(scene):
    model, data, config, scatter = scene
    aligned = [np.array([*xy, config['block_size'][2]/2+.001, 1, 0, 0, 0])
               for xy in config['block_positions']]
    scatter.prepare_initial_scene(config, scatter=True, seed=42)
    for pose, expected in zip(scatter.baseline, aligned):
        np.testing.assert_array_equal(pose, expected)
    assert any(not np.allclose(pose, expected)
               for pose, expected in zip(scatter.poses(data), aligned))
    first_scene = scene_snapshot(model, data, config)
    assert any(not np.allclose(block['position'], expected[:3])
               for block, expected in zip(first_scene['blocks'], aligned))
    scatter.apply(scatter.plan(config, seed=43))
    scatter.restore()
    for pose, expected in zip(scatter.poses(data), aligned):
        np.testing.assert_array_equal(pose, expected)


@pytest.mark.parametrize('options', [{'seed': -1}, {'seed': True}, {'table': 'missing'}])
def test_invalid_request_preserves_scene_and_baseline(scene, options):
    _, data, config, scatter = scene
    original = data.qpos.copy()
    with pytest.raises(ValueError):
        scatter.plan(config, **options)
    np.testing.assert_array_equal(data.qpos, original)
    assert scatter.baseline is None


def test_cli_http_and_simulator_restart_restore_baseline(scene):
    pytest.importorskip('rclpy')
    from piper_mujoco_sim.sim import PiperMujocoSim
    model, data, config, scatter = scene
    original = scatter.poses(data)
    scatter.prepare_initial_scene(config, scatter=True, seed=42)
    assert any(not np.allclose(pose, aligned)
               for pose, aligned in zip(scatter.poses(data), original))
    node = SimpleNamespace(model=model, data=data, block_scatter=scatter,
        editor=SimpleNamespace(config=config, paused=True), lock=threading.RLock(),
        restart_id=0, command_cutoff=0,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=123)),
        publish_state=lambda: None, publish_scene=lambda: None)
    node.restart_simulation = MethodType(PiperMujocoSim.restart_simulation, node)
    node.change_blocks = MethodType(PiperMujocoSim.change_blocks, node)
    server = ThreadingHTTPServer(('127.0.0.1', 0),
        make_handler({}, block_action=node.change_blocks))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    script = Path(__file__).parents[3]/'apps/junh/scatter_blocks.py'
    try:
        for action, extra in [('scatter', ['--seed', '1']), ('scatter', ['--seed', '9']), ('reset', [])]:
            result = subprocess.run([sys.executable, str(script), action,
                '--url', f'http://127.0.0.1:{server.server_port}', *extra],
                capture_output=True, text=True, timeout=10)
            assert result.returncode == 0, result.stderr
            assert 'restart_id=' in result.stdout
            assert not node.editor.paused
        np.testing.assert_array_equal(scatter.poses(data), original)
        assert node.restart_id == 3
        node.change_blocks('scatter', {'seed': 10})
        node.restart_simulation()  # Existing browser restart also restores baseline.
        np.testing.assert_array_equal(scatter.poses(data), original)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
