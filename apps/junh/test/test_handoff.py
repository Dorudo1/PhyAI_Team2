"""Sensor-independent master contracts work without a ROS installation."""
import json

from junh_master.handoff import Master, Placement
from phyai_discovery.perception import Detector, DetectorConfig
from phyai_discovery.search import Search
from phyai_discovery.workspace import Workspace
from app_scene import image_capture


def envelope():
    workspace=Workspace()
    workspace.merge(Detector(DetectorConfig(known_block_size=(.075,.025,.015))).detect(image_capture([(.1,0.,.075,.025,.015,0.)])))
    search=Search(workspace)
    search.accepted=1
    search.finish()
    return {'workspace':workspace.snapshot(),'search':search.progress()}


def test_master_dimension_noise_and_session_restart():
    master=Master()
    data=envelope()
    master.consume(data)
    data['workspace']['revision']+=1
    # New snapshot geometry, not mutation of the prior observation.
    data=json.loads(json.dumps(data))
    data['workspace']['tracks'][0]['size'][0]+=.002
    master.consume(data)
    assert master.status()['inventory_stable']
    old=json.loads(json.dumps(data))
    data['workspace']['session_id']='new_session'
    data['workspace']['revision']=1
    master.consume(data)
    master.consume(old)
    assert master.observation.session_id=='new_session'


def test_proposal_remains_unexecuted_and_orientation_changes_are_not_stable():
    class Policy:
        def propose(self,observation):
            return Placement(observation.tracks[0]['id'],(.2,0.,.015),(1.,0.,0.,0.),'Review only')
    master=Master(policy=Policy())
    data=envelope()
    master.consume(data)
    assert master.state=='placement_proposed'
    assert master.status()['verified_stack_height']==0.
    data=json.loads(json.dumps(data));data['workspace']['revision']+=1
    data['workspace']['tracks'][0]['quaternion']=[.70710678,0.,0.,.70710678]
    master.consume(data)
    assert not master.status()['inventory_stable']


def test_policy_cannot_select_unresolved_confirmed_track():
    import pytest
    class Policy:
        def propose(self, observation):
            return Placement(observation.tracks[0]['id'],(.2,0.,.015),(1.,0.,0.,0.),'Review')
    data = envelope()
    data['workspace']['tracks'][0]['resolved'] = False
    with pytest.raises(ValueError, match='unresolved'):
        Master(policy=Policy()).consume(data)


def test_empty_confirmed_inventory_does_not_call_policy():
    class Policy:
        def propose(self, observation):
            raise AssertionError('No confirmed blocks to select')
    data = envelope()
    data['workspace']['tracks'] = []
    master = Master(policy=Policy())
    master.consume(data)
    assert master.proposal is None
