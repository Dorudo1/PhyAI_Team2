import copy
import math

import pytest

from junh_master.legacy.sim_adapter import Config, Master, Placement, State, discover


def scene():
    return {'restart_id': 0, 'simulation_time': 0., 'paused': False,
            'block_size': [.075, .025, .015],
            'blocks': [{'name': 'block_1', 'position': [.4, -.25, .0075],
                        'quaternion': [1., 0., 0., 0.]}]}


def advance(master, snapshot, stamp):
    snapshot['simulation_time'] = stamp
    master.consume(snapshot)


def test_stable_inventory_waits_for_model():
    master, snapshot = Master(), scene()
    master.consume(snapshot, joints_ready=False)
    assert master.state == State.WAITING
    advance(master, snapshot, .2)
    assert master.state == State.SETTLING
    advance(master, snapshot, .9)
    assert master.state == State.WAITING_MODEL
    assert master.status()['blocks'][0]['name'] == 'block_1'
    assert master.status()['verified_stack_height'] == 0


def test_slow_drift_is_compared_to_original_anchor():
    master, snapshot = Master(), scene()
    advance(master, snapshot, 0.)
    for stamp in (.2, .4, .6):
        snapshot['blocks'][0]['position'][0] += .002
        advance(master, snapshot, stamp)
        assert master.state == State.SETTLING
    advance(master, snapshot, 1.1)
    assert master.state == State.WAITING_MODEL


def test_orientation_motion_resets_settling_and_quaternion_sign_does_not():
    master, snapshot = Master(), scene()
    advance(master, snapshot, 0.)
    snapshot['blocks'][0]['quaternion'] = [math.cos(.1), 0., 0., math.sin(.1)]
    advance(master, snapshot, .6)
    assert master.state == State.SETTLING
    snapshot['blocks'][0]['quaternion'] = [-v for v in snapshot['blocks'][0]['quaternion']]
    advance(master, snapshot, 1.3)
    assert master.state == State.WAITING_MODEL


def test_repeated_clock_pause_restart_and_old_snapshots():
    master, snapshot = Master(), scene()
    advance(master, snapshot, 0.)
    advance(master, snapshot, 0.)
    assert master.state == State.SETTLING
    advance(master, snapshot, .8)
    old = copy.deepcopy(snapshot)
    snapshot['paused'] = True
    master.consume(snapshot)
    assert master.state == State.PAUSED
    assert master.inventory == ()
    snapshot.update(paused=False, restart_id=1)
    advance(master, snapshot, 0.)
    assert master.state == State.SETTLING
    master.consume(old)
    assert master.restart_id == 1
    assert master.state == State.SETTLING


@pytest.mark.parametrize('position,rotation', [
    ([.02, -.25, .0075], [1, 0, 0, 0]),  # Over the table seam.
    ([.4, -.25, .10], [1, 0, 0, 0]),  # Already raised/stacked.
    ([.4, -.25, .0075], [.707, .707, 0, 0]),  # Tipped over.
])
def test_rejects_blocks_outside_table_or_not_resting(position, rotation):
    snapshot = scene()
    snapshot['blocks'][0].update(position=position, quaternion=rotation)
    master = Master()
    master.consume(snapshot)
    assert master.state == State.SEARCHING


def test_disappearance_requires_new_stability_window():
    master, snapshot = Master(), scene()
    advance(master, snapshot, 0.)
    advance(master, snapshot, .8)
    block = snapshot['blocks'].pop()
    advance(master, snapshot, 1.)
    assert master.state == State.SEARCHING
    snapshot['blocks'].append(block)
    advance(master, snapshot, 1.2)
    assert master.state == State.SETTLING


@pytest.mark.parametrize('field,value', [('position', [float('nan'), 0, 0]),
                                       ('quaternion', [0, 0, 0, 0])])
def test_invalid_pose(field, value):
    snapshot = scene()
    snapshot['blocks'][0][field] = value
    with pytest.raises(ValueError):
        discover(snapshot, Config())


def test_policy_proposal_is_not_counted_as_executed():
    class Policy:
        def propose(self, observation):
            assert observation.frame == 'world'
            return Placement(observation.blocks[0].name, (.5, -.3, .0075),
                             (1., 0., 0., 0.), 'Start a tower base.')
    master, snapshot = Master(policy=Policy()), scene()
    advance(master, snapshot, 0.)
    advance(master, snapshot, .8)
    assert master.state == State.PROPOSAL
    assert master.proposal.block_name == 'block_1'
    assert master.status()['verified_stack_height'] == 0


def test_policy_cannot_select_a_missing_block():
    class Policy:
        def propose(self, observation):
            return Placement('missing', (.5, -.3, .0075), (1., 0., 0., 0.), '')
    master, snapshot = Master(policy=Policy()), scene()
    advance(master, snapshot, 0.)
    with pytest.raises(ValueError, match='outside'):
        advance(master, snapshot, .8)
