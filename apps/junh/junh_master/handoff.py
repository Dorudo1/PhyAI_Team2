"""Sensor-independent master handoff; perception runs without a stacking model."""
from dataclasses import dataclass, asdict
import math
from typing import Optional, Protocol


@dataclass(frozen=True)
class Observation:
    tracks: tuple
    table: dict
    coverage: dict
    stamp: float
    session_id: str
    revision: int
    frame: str
    search_status: str
    pick_ready: bool = False


@dataclass(frozen=True)
class Placement:
    block_name: str
    target_position: tuple
    target_quaternion: tuple
    reason: str


class StackingPolicy(Protocol):
    def propose(self, observation: Observation) -> Optional[Placement]: ...


class PendingPolicy:
    def propose(self, observation):
        return None


class Master:
    def __init__(self, policy=None, position_tolerance=.005, dimension_tolerance=.01, orientation_tolerance=.05):
        if not all(math.isfinite(v) and v > 0 for v in (position_tolerance,dimension_tolerance,orientation_tolerance)):
            raise ValueError('Invalid inventory stability tolerances.')
        self.orientation_tolerance = orientation_tolerance
        self.policy = policy or PendingPolicy()
        self.position_tolerance, self.dimension_tolerance = position_tolerance, dimension_tolerance
        self.reset()

    def reset(self):
        self.state = 'waiting_for_discovery'
        self.reason = 'Waiting for workspace and search progress.'
        self.observation = None
        self.proposal = None
        self.previous = None
        self.retired_sessions = set()

    def wait(self, reason):
        self.state, self.reason = 'waiting_for_discovery', reason
        self.observation = self.proposal = None

    def consume(self, envelope):
        workspace, search = envelope['workspace'], envelope['search']
        if workspace['version'] != 1 or not workspace['frame'] or not workspace['session_id']:
            raise ValueError('Invalid discovery version, frame or session.')
        if workspace['motion_authorization'] is not False:
            raise ValueError('Discovery may not authorize robot motion.')
        stamp, revision = float(workspace['stamp']), int(workspace['revision'])
        if not math.isfinite(stamp) or stamp < 0 or revision < 0:
            raise ValueError('Invalid discovery timestamp/revision.')
        session = workspace['session_id']
        if session in self.retired_sessions:
            return
        if self.previous and self.previous.session_id != session:
            self.retired_sessions.add(self.previous.session_id)
            self.previous = None
        if self.previous and revision < self.previous.revision:
            return
        tracks = workspace['tracks']
        ids = set()
        for track in tracks:
            if not track['id'] or track['id'] in ids:
                raise ValueError('Invalid or duplicate track ID.')
            ids.add(track['id'])
            for key in ('position','size'):
                if len(track[key]) != 3 or not all(math.isfinite(float(v)) for v in track[key]):
                    raise ValueError('Invalid track geometry.')
            if min(track['size']) <= 0 or len(track['quaternion']) != 4 or not all(
                    math.isfinite(float(v)) for v in track['quaternion']):
                raise ValueError('Invalid dimensions/orientation.')
            if abs(sum(float(v)**2 for v in track['quaternion'])-1.) > .01:
                raise ValueError('Track quaternion must be normalized.')
        result = search.get('result')
        status = result['status'] if result else 'in_progress'
        if status not in ('complete','partial','failed','in_progress'):
            raise ValueError('Invalid search result.')
        obs = Observation(tuple(tracks), workspace['table'], workspace['coverage'],
                          stamp, session, revision, workspace['frame'], status, False)
        stable = False
        if self.previous and revision > self.previous.revision:
            before = {t['id']:t for t in self.previous.tracks}
            stable = set(before) == ids and bool(ids) and all(
                math.dist(track['position'],before[track['id']]['position']) <= self.position_tolerance
                and max(abs(a-b) for a,b in zip(track['size'],before[track['id']]['size'])) <= self.dimension_tolerance
                and self.orientation_close(track['quaternion'],before[track['id']]['quaternion'])
                for track in tracks)
        self.observation = obs
        self.proposal = None
        if not result:
            self.state, self.reason = 'discovering', search['reason']
        elif status == 'failed':
            self.state, self.reason = 'discovery_failed', result['reason']
        else:
            self.state = 'waiting_for_model'
            self.reason = f'Search {status}: {len(tracks)} tracks available; picking is not authorized.'
            # Model can review partial geometry, but never receives pick authorization.
            self.proposal = self.policy.propose(obs) if tracks else None
            if self.proposal:
                if self.proposal.block_name not in ids:
                    raise ValueError('Policy selected an unavailable block.')
                selected = next(t for t in tracks if t['id'] == self.proposal.block_name)
                if not selected['resolved'] or selected.get('alternatives'):
                    raise ValueError('Policy selected a block with unresolved pose or identity evidence.')
                p, q = self.proposal.target_position, self.proposal.target_quaternion
                if len(p) != 3 or len(q) != 4 or not all(math.isfinite(float(v)) for v in (*p,*q)) or abs(sum(v*v for v in q)-1.) > .01:
                    raise ValueError('Invalid proposed placement pose.')
                self.state = 'placement_proposed'
        self.stable = stable if revision != getattr(self.previous, 'revision', None) else getattr(self,'stable',False)
        self.previous = obs

    def orientation_close(self, actual, previous):
        w,x,y,z = previous
        symmetric = (-z,y,-x,w)  # rectangular long-axis 180-degree symmetry
        dot = max(abs(sum(a*b for a,b in zip(actual,q))) for q in (previous,symmetric))
        return dot >= math.cos(self.orientation_tolerance/2)

    def status(self):
        return {'state':self.state,'reason':self.reason,
                'observation':asdict(self.observation) if self.observation else None,
                'inventory_stable':getattr(self,'stable',False) if self.observation else False,
                'proposal':asdict(self.proposal) if self.proposal else None,
                'pick_ready':False,'verified_stack_height':0.}
