"""Exercise live edits against the actual Menagerie model and browser API."""
import copy
import json
import os
from pathlib import Path
import threading
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import mujoco
import numpy as np
import pytest

from piper_mujoco_sim.editor import SceneEditor
from piper_mujoco_sim.scene import build_scene, reset_scene
from piper_mujoco_sim.web import FrameHub, make_handler


ROBOT = Path(os.environ.get('PIPER_TEST_ROBOT', '/opt/mujoco_menagerie/agilex_piper/piper.xml'))
pytestmark = pytest.mark.skipif(not ROBOT.exists(), reason='Menagerie PIPER model is required')


@pytest.fixture
def editor(tmp_path):
    source = Path(__file__).parents[1] / 'config/lab_scene.json'
    path = tmp_path / 'scene.json'
    path.write_text(source.read_text())
    model = mujoco.MjModel.from_xml_string(build_scene(ROBOT, path))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key('home').id)
    mujoco.mj_forward(model, data)
    return SceneEditor(model, data, threading.Lock(), path)


def test_parent_translation_rotation_and_revert(editor):
    data = editor.data
    lens_position = data.camera('wrist').xpos.copy()
    lens_rotation = data.camera('wrist').xmat.copy()
    joints = data.qpos.copy()
    config = copy.deepcopy(editor.config)
    position = [value + delta for value, delta in zip(config['wrist_camera_mount_position'], [.01, -.02, .03])]
    editor.action('preview', {'changes': {
        'wrist_camera_mount_position': position,
        'wrist_camera_mount_quaternion': [2**-.5, 0, 0, 2**-.5]}})
    mount = data.body('wrist_camera_mount')
    rotation = mount.xmat.reshape(3, 3)
    np.testing.assert_allclose(rotation.T @ (data.camera('wrist').xpos - mount.xpos),
                               config['wrist_camera_position'], atol=1e-12)
    np.testing.assert_allclose(data.geom('wrist_camera_mount_plate').xpos, mount.xpos)
    np.testing.assert_allclose(data.qpos, joints)
    assert editor.paused
    assert editor.snapshot()['dirty']
    editor.action('revert', {})
    np.testing.assert_allclose(data.camera('wrist').xpos, lens_position)
    np.testing.assert_allclose(data.camera('wrist').xmat, lens_rotation)
    assert not editor.snapshot()['dirty']


def test_camera_rigidly_follows_casing_not_gripper_roll(editor):
    model, data = editor.model, editor.data
    assert model.body('wrist_mount_reference').parentid == model.body('link5').id
    for name in ('wrist_mount_reference', 'gripper_root_mount', 'wrist_camera_mount', 'wrist_camera_body'):
        assert model.body(name).jntnum[0] == 0

    def relative(body):
        casing = data.body('link5')
        rotation = casing.xmat.reshape(3, 3)
        return (rotation.T @ (body.xpos - casing.xpos),
                rotation.T @ body.xmat.reshape(3, 3))

    expected_camera = relative(data.camera('wrist'))
    expected_pad = relative(data.geom('wrist_camera_mount_plate'))
    original_camera = data.camera('wrist').xpos.copy()
    original_gripper = data.body('link6').xmat.copy()
    # Include whole-arm motion, the final wrist roll, and opening both fingers.
    for joint, value in [('joint1', .4), ('joint2', 1.2), ('joint3', -1.1),
                         ('joint4', .3), ('joint5', .2), ('joint6', .5),
                         ('joint7', .02), ('joint8', -.02), ('joint6', -1.0)]:
        data.qpos[model.joint(joint).qposadr[0]] = value
        mujoco.mj_forward(model, data)
        for actual, expected in zip(relative(data.camera('wrist')), expected_camera):
            np.testing.assert_allclose(actual, expected, atol=1e-12)
        for actual, expected in zip(relative(data.geom('wrist_camera_mount_plate')), expected_pad):
            np.testing.assert_allclose(actual, expected, atol=1e-12)
    assert not np.allclose(data.camera('wrist').xpos, original_camera)
    assert not np.allclose(data.body('link6').xmat, original_gripper)


def test_arm_overhead_and_block_edits(editor):
    blocks = copy.deepcopy(editor.config['block_positions'])
    blocks[0] = [.22, -.12]
    untouched = editor.data.body('block_2').xpos.copy()
    editor.action('preview', {'changes': {'post_position': [.2, -.1],
        'overhead_position': [-.2, .1, 1], 'overhead_quaternion': [1, 0, 0, 0],
        'camera_fovy': 70, 'block_positions': blocks}})
    np.testing.assert_allclose(editor.data.body('base_link').xpos, [.2, -.1, 0])
    np.testing.assert_allclose(editor.data.geom('mount_post').xpos[:2], [.2, -.1])
    np.testing.assert_allclose(editor.data.camera('overhead').xpos, [-.2, .1, 1])
    np.testing.assert_allclose(editor.data.body('block_1').xpos[:2], blocks[0])
    np.testing.assert_allclose(editor.data.body('block_2').xpos, untouched)
    assert editor.model.camera('wrist').fovy[0] == 70


@pytest.mark.parametrize('changes', [
    {'wrist_camera_quaternion': [0, 0, 0, 0]},
    {'wrist_camera_position': [float('nan'), 0, 0]},
    {'wrist_camera_position': [1, 2]},
    {'camera_fovy': 0}, {'camera_fovy': True},
    {'table_size': [1, 1]}, {'block_positions': []}])
def test_invalid_edits_are_atomic(editor, changes):
    config = copy.deepcopy(editor.config)
    position = editor.data.camera('wrist').xpos.copy()
    with pytest.raises(ValueError):
        editor.action('preview', {'changes': changes})
    assert editor.config == config
    np.testing.assert_allclose(editor.data.camera('wrist').xpos, position)
    assert not editor.paused


def test_save_symlink_and_external_edits(editor, tmp_path):
    link = tmp_path / 'installed_scene.json'
    link.symlink_to(editor.path)
    linked = SceneEditor(editor.model, editor.data, editor.lock, link)
    linked.action('save', {'changes': {'camera_fovy': 60}})
    assert link.is_symlink()
    assert json.loads(editor.path.read_text())['camera_fovy'] == 60
    disk = json.loads(editor.path.read_text())
    disk['post_position'] = [.3, -.2]
    editor.path.write_text(json.dumps(disk))
    with pytest.raises(ValueError, match='changed outside'):
        linked.action('save', {'changes': {'camera_fovy': 65}})
    assert json.loads(editor.path.read_text())['camera_fovy'] == 60
    linked.action('reload', {})
    np.testing.assert_allclose(linked.data.body('base_link').xpos[:2], [.3, -.2])
    linked.action('save', {'changes': {'camera_fovy': 65}})
    assert json.loads(editor.path.read_text())['camera_fovy'] == 65


def test_http_editor_and_stream(editor):
    hubs = {name: FrameHub() for name in ('overview', 'overhead', 'wrist')}
    server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(hubs, editor))
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f'http://127.0.0.1:{server.server_port}'
    try:
        with urlopen(url, timeout=3) as response:
            page = response.read()
            assert b'Read-only viewer' in page
            assert b'Save scene' not in page
        with urlopen(url + '/edit', timeout=3) as response:
            assert b'Save scene' in response.read()
        with urlopen(url + '/edit/scene', timeout=3) as response:
            assert json.load(response)['enabled']
        request = Request(url + '/edit/scene/preview', data=json.dumps({'changes': {'camera_fovy': 65}}).encode(),
                          headers={'Content-Type': 'application/json'})
        with urlopen(request, timeout=3) as response:
            assert json.load(response)['paused']
        request.add_header('Origin', 'http://unrelated.example')
        with pytest.raises(HTTPError) as error:
            urlopen(request, timeout=3)
        assert error.value.code == 403
        with urlopen(url + '/stream?view=overview', timeout=3) as response:
            with hubs['overview'].cond:
                assert hubs['overview'].cond.wait_for(lambda: hubs['overview'].clients > 0, timeout=3)
                hubs['overview'].publish(b'test-jpeg')
            assert response.readline() == b'--frame\r\n'
            assert response.readline() == b'Content-Type: image/jpeg\r\n'
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)


@pytest.mark.parametrize('scene_editing', [False, True])
def test_explicit_editor_routes_and_launch_mode(editor, scene_editing):
    server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler({}, editor, scene_editing))
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f'http://127.0.0.1:{server.server_port}'
    disk_text = editor.path.read_text()
    config = copy.deepcopy(editor.config)
    try:
        with urlopen(url, timeout=3) as response:
            page = response.read()
            assert (b'Save scene' in page) == scene_editing
            assert b'<title>PIPER MuJoCo</title>' in page
        for path in ('/edit', '/edit/'):
            with urlopen(url + path, timeout=3) as response:
                assert b'Save scene' in response.read()
        # Visiting the editor does not globally promote other viewer requests.
        with urlopen(url, timeout=3) as response:
            assert (b'Save scene' in response.read()) == scene_editing
        for operation in ('preview', 'save', 'revert', 'reload', 'pause'):
            request = Request(url + '/scene/' + operation, data=b'{}',
                              headers={'Content-Type': 'application/json'})
            with pytest.raises(HTTPError) as error:
                urlopen(request, timeout=3)
            assert error.value.code == 404
        assert editor.path.read_text() == disk_text
        assert editor.config == config
        assert not editor.paused
        with urlopen(url + '/edit/scene', timeout=3) as response:
            assert json.load(response)['enabled']
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)


def test_custom_model_stays_view_only():
    server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler({}, None, scene_editing=True))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f'http://127.0.0.1:{server.server_port}'
    try:
        with urlopen(url, timeout=3) as response:
            assert b'Read-only viewer' in response.read()
        with urlopen(url + '/edit/scene', timeout=3) as response:
            assert not json.load(response)['enabled']
        request = Request(url + '/edit/scene/save', data=b'{}',
                          headers={'Content-Type': 'application/json'})
        with pytest.raises(HTTPError) as error:
            urlopen(request, timeout=3)
        assert error.value.code == 409
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)


def test_restart_resets_runtime_without_saving_scene(editor):
    original_file = editor.path.read_text()
    mount_position = editor.config['wrist_camera_mount_position'].copy()
    mount_position[0] += .01
    editor.action('preview', {'changes': {'wrist_camera_mount_position': mount_position}})
    editor.data.qpos[0] = .4
    editor.data.time = 10
    generation = [0]

    def restart():
        with editor.lock:
            reset_scene(editor.model, editor.data, editor.config)
            editor.paused = False
            generation[0] += 1
        return {'restart_id': generation[0], 'message': 'Simulation restarted.'}

    server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler({}, editor, restart=restart))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f'http://127.0.0.1:{server.server_port}'
    try:
        for index in (1, 2):
            request = Request(url + '/simulation/restart', data=b'{}',
                              headers={'Content-Type': 'application/json'})
            with urlopen(request, timeout=3) as response:
                assert json.load(response)['restart_id'] == index
            assert editor.data.time == 0
            np.testing.assert_allclose(editor.data.qpos[:8], editor.model.key('home').qpos[:8])
            np.testing.assert_allclose(editor.model.body('wrist_camera_mount').pos, mount_position)
            assert editor.snapshot()['dirty']
            assert not editor.paused
            assert editor.path.read_text() == original_file
        request.add_header('Origin', 'http://unrelated.example')
        with pytest.raises(HTTPError) as error:
            urlopen(request, timeout=3)
        assert error.value.code == 403
        assert generation[0] == 2
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)
