"""ROS-independent startup controller. All lengths are metres in sim world frame."""
from dataclasses import asdict, dataclass
from enum import Enum
import math
from typing import Optional, Protocol, Tuple


Vector3 = Tuple[float, float, float]


class State(str, Enum):
    WAITING = 'waiting_for_inputs'
    PAUSED = 'paused'
    SEARCHING = 'searching_blocks'
    SETTLING = 'waiting_for_stable_blocks'
    WAITING_MODEL = 'waiting_for_model'
    PROPOSAL = 'placement_proposed'


@dataclass(frozen=True)
class Config:
    # Interior of the default scene's lower-right tabletop; avoids the seams.
    bounds: Tuple[float, float, float, float] = (.04, .78, -.50, -.04)
    table_z: float = 0.0
    settle_seconds: float = .6
    position_tolerance: float = .003
    orientation_tolerance: float = .03  # radians

    def __post_init__(self):
        xmin, xmax, ymin, ymax = self.bounds
        if not all(math.isfinite(v) for v in (*self.bounds, self.table_z,
                self.settle_seconds, self.position_tolerance, self.orientation_tolerance)):
            raise ValueError('Configuration must be finite.')
        if xmin >= xmax or ymin >= ymax or min(self.settle_seconds,
                self.position_tolerance, self.orientation_tolerance) <= 0:
            raise ValueError('Invalid search bounds or settling thresholds.')


@dataclass(frozen=True)
class Block:
    name: str
    position: Vector3
    size: Vector3
    quaternion: Tuple[float, float, float, float]  # w, x, y, z


@dataclass(frozen=True)
class Observation:
    blocks: Tuple[Block, ...]
    simulation_time: float
    restart_id: int
    frame: str = 'world'


@dataclass(frozen=True)
class Placement:
    block_name: str
    target_position: Vector3  # block centre, not gripper pose
    target_quaternion: Tuple[float, float, float, float]
    reason: str


class StackingPolicy(Protocol):
    """Teammate model adapter. None means no placement can be proposed yet.

    Proposals require a separate motion planner and execution verification.
    This startup scaffold never executes or credits a proposed placement.
    """
    def propose(self, observation: Observation) -> Optional[Placement]: ...


class PendingPolicy:
    def propose(self, observation):
        return None


def vector(value, length):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f'Expected {length} numeric values.')
    result = tuple(float(v) for v in value)
    if not all(math.isfinite(v) for v in result):
        raise ValueError('Pose values must be finite.')
    return result


def quaternion(value):
    q = vector(value, 4)
    norm = math.sqrt(sum(v*v for v in q))
    if norm < 1e-8:
        raise ValueError('Quaternion must be nonzero.')
    return tuple(v/norm for v in q)


def discover(scene, config):
    """Inventory upright blocks resting on the configured tabletop.

    Uses simulator ground truth, not image detection or a reachability test.
    A conservative half-diagonal margin keeps the entire block inside bounds.
    """
    size = vector(scene['block_size'], 3)
    if min(size) <= 0:
        raise ValueError('Block dimensions must be positive.')
    found = []
    seen = set()
    xmin, xmax, ymin, ymax = config.bounds
    margin = math.hypot(size[0], size[1])/2
    for raw in scene['blocks']:
        name = raw['name']
        if not isinstance(name, str) or not name or name in seen:
            raise ValueError('Block names must be unique nonempty strings.')
        seen.add(name)
        x, y, z = position = vector(raw['position'], 3)
        q = quaternion(raw['quaternion'])
        upright = 1-2*(q[1]**2+q[2]**2) >= math.cos(.10)
        on_table = abs(z-(config.table_z+size[2]/2)) <= config.position_tolerance
        if upright and on_table and xmin+margin <= x <= xmax-margin and ymin+margin <= y <= ymax-margin:
            found.append(Block(name, position, size, q))
    return tuple(sorted(found, key=lambda block: block.name))


class Master:
    def __init__(self, config=None, policy=None):
        self.config = config or Config()
        self.policy = policy or PendingPolicy()
        self.restart_id = None
        self.reset()

    def reset(self):
        self.state = State.WAITING
        self.reason = 'Waiting for fresh scene and joint feedback.'
        self.inventory = ()
        self.proposal = None
        self.last_time = None
        self.anchor = ()
        self.stable_since = None

    def wait(self, reason):
        self.reset()
        self.reason = reason

    def consume(self, scene, joints_ready=True):
        restart = scene['restart_id']
        if not isinstance(restart, int) or isinstance(restart, bool) or restart < 0:
            raise ValueError('Invalid restart ID.')
        if self.restart_id is not None and restart < self.restart_id:
            return
        if restart != self.restart_id:
            self.reset()
            self.restart_id = restart
        now = float(scene['simulation_time'])
        if not math.isfinite(now) or now < 0 or not isinstance(scene['paused'], bool):
            raise ValueError('Invalid scene clock or pause flag.')
        if scene['paused'] or not joints_ready:
            self.wait('Simulator paused.' if scene['paused'] else 'Waiting for fresh joint feedback.')
            self.state = State.PAUSED if scene['paused'] else State.WAITING
            return
        if self.last_time is not None and now <= self.last_time:
            if now < self.last_time:
                self.wait('Scene clock moved backwards; searching again.')
            return
        blocks = discover(scene, self.config)
        self.last_time = now
        self.inventory = blocks
        self.proposal = None
        if not blocks:
            self.anchor = ()
            self.stable_since = None
            self.state = State.SEARCHING
            self.reason = 'No upright blocks on the configured tabletop; keep observing.'
            return
        same = len(blocks) == len(self.anchor) and all(
            a.name == b.name and a.size == b.size
            and math.dist(a.position, b.position) <= self.config.position_tolerance
            and abs(sum(x*y for x, y in zip(a.quaternion, b.quaternion)))
                >= math.cos(self.config.orientation_tolerance/2)
            for a, b in zip(blocks, self.anchor))
        if not same:
            self.anchor = blocks
            self.stable_since = now
        if now-self.stable_since < self.config.settle_seconds:
            self.state = State.SETTLING
            self.reason = f'Found {len(blocks)} blocks; checking pose stability.'
            return
        proposal = self.policy.propose(Observation(blocks, now, restart))
        if proposal is None:
            self.state = State.WAITING_MODEL
            self.reason = f'{len(blocks)} stable blocks ready for the model.'
        else:
            if proposal.block_name not in {block.name for block in blocks}:
                raise ValueError('Policy selected a block outside the current inventory.')
            vector(proposal.target_position, 3)
            quaternion(proposal.target_quaternion)
            self.proposal = proposal
            self.state = State.PROPOSAL
            self.reason = 'Proposal ready for a future motion planner; no motion executed.'

    def status(self):
        return {'state': self.state.value, 'reason': self.reason,
                'restart_id': self.restart_id, 'frame': 'world',
                'blocks': [asdict(block) for block in self.inventory],
                'proposal': asdict(self.proposal) if self.proposal else None,
                'verified_stack_height': 0.0}
