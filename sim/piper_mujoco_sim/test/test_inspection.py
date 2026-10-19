"""Targeted views use measured positions and fixed geometry, never block truth."""
import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

from piper_mujoco_sim.inspection import InspectionPlanner, scan_geometry
from piper_mujoco_sim.scene import build_scene

ROOT = Path(__file__).resolve().parents[3]
ROBOT = Path('/opt/mujoco_menagerie/agilex_piper/piper.xml')
pytestmark = pytest.mark.skipif(not ROBOT.exists(), reason='Menagerie model is required')


@pytest.fixture
def planner():
    model = mujoco.MjModel.from_xml_string(build_scene(ROBOT,
        ROOT/'sim/piper_mujoco_sim/config/lab_scene.json'))
    config = json.loads((ROOT/'apps/junh/config/mujoco.json').read_text())
    return InspectionPlanner(model,list(config['sim_scan']['targets'].values()))


def target(x=.12,y=.47):
    return {'id':'uncertain_block','position':[x,y,.0075],
            'size':[.075,.025,.015],'priority':3,'supporting_views':[]}


def test_focus_fits_image_and_joint_pose_is_novel_without_camera_edit(planner):
    geometry = scan_geometry(planner.model)
    current = [0.,1.57,-1.3485,0.,0.,0.,.035]
    first = planner.choose([target()],current,'close_1')
    assert first is not None and first['focus']=='uncertain_block'
    p,r = planner.pose(first['joints'])
    local = (np.array(target()['position'])-p)@r
    tangent = np.tan(np.deg2rad(geometry['cameras']['wrist']['fovy'])/2)
    assert abs(local[0])+.045 < -local[2]*tangent*4/3
    assert abs(local[1])+.045 < -local[2]*tangent
    second = planner.choose([target()],first['joints'],'close_2')
    assert second is not None
    next_p,next_r = planner.pose(second['joints'])
    angle = np.arccos(np.clip(r[:,2]@next_r[:,2],-1,1))
    assert np.linalg.norm(next_p-p)>.035 or angle>np.deg2rad(10)
    assert scan_geometry(planner.model)==geometry


def test_simulator_object_truth_cannot_change_selected_view(planner):
    current = [0.,1.57,-1.3485,0.,0.,0.,.035]
    before = planner.choose([target()],current,'close_1')
    planner.reset()
    # Put simulator truth directly across the camera rays. These objects must
    # remain irrelevant; only detected positions may influence the planner.
    for j in range(planner.model.njnt):
        if planner.model.jnt_type[j]==mujoco.mjtJoint.mjJNT_FREE:
            address = planner.model.jnt_qposadr[j]
            planner.data.qpos[address:address+3]=[.12,.47,.2]
    after = planner.choose([target()],current,'close_1')
    assert before is not None and after is not None
    np.testing.assert_allclose(before['joints'],after['joints'])


def test_unreachable_or_blocked_target_does_not_produce_motion(planner,monkeypatch):
    current = [0.,1.57,-1.3485,0.,0.,0.,.035]
    assert planner.choose([target(20,20)],current,'far') is None
    monkeypatch.setattr(planner,'_clear_path',lambda initial,target:False)
    assert planner.choose([target()],current,'blocked') is None
    assert not planner.history


def test_higher_urgency_precedes_easy_target_and_attempts_are_bounded(planner):
    planner.max_views_per_block=1
    current=[0.,1.57,-1.3485,0.,0.,0.,.035]
    urgent=target();urgent['id']='provisional';urgent['priority']=3
    easy=target(-.3,-.15);easy['id']='supported';easy['priority']=1
    first=planner.choose([easy,urgent],current,'first')
    assert first is not None and first['focus']=='provisional'
    second=planner.choose([easy,urgent],first['joints'],'second')
    assert second is not None and second['focus']=='supported'
    assert planner.choose([easy,urgent],second['joints'],'third') is None
