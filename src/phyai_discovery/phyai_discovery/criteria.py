"""Evidence requirements for stopping a static-scene search, not probabilities."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SearchCriteria:
    min_explored_fraction: float = .90
    min_resolved_views: int = 2
    max_position_uncertainty: float = .01
    max_yaw_uncertainty: float = math.radians(5)
    min_coverage_gain: float = .005
    max_no_progress_rounds: int = 1

    def __post_init__(self):
        values = (self.min_explored_fraction,self.max_position_uncertainty,
                  self.max_yaw_uncertainty,self.min_coverage_gain)
        if not all(math.isfinite(v) for v in values) or not 0 < self.min_explored_fraction <= 1:
            raise ValueError('Invalid search coverage threshold.')
        if min(self.max_position_uncertainty,self.max_yaw_uncertainty) <= 0 or self.min_coverage_gain < 0:
            raise ValueError('Invalid search uncertainty/gain thresholds.')
        if any(not isinstance(v,int) or isinstance(v,bool) or v < 1
               for v in (self.min_resolved_views,self.max_no_progress_rounds)):
            raise ValueError('Search view and plateau counts must be positive integers.')

    def assess(self, snapshot):
        reasons = []
        coverage = snapshot['coverage'].get('explored_fraction',snapshot['coverage']['observed_fraction'])
        if snapshot['table'] is None:
            reasons.append('table plane is unknown')
        if snapshot['coverage']['scope'] is None:
            reasons.append('search scope is unspecified')
        if coverage < self.min_explored_fraction:
            reasons.append(f'explored scope {coverage:.1%} below {self.min_explored_fraction:.1%}')
        if snapshot['coverage'].get('unexplained_cells', 0):
            reasons.append('unexplained above-table depth remains')
        pending = snapshot.get('hypotheses',[])
        if pending:
            reasons.append(f'{len(pending)} provisional object hypotheses')
        blocks = []
        for track in snapshot['tracks']:
            uncertainty = track.get('pose_uncertainty',{})
            views = len(set(tuple(v) for v in track.get('supporting_views',[])))
            checks = {
                'resolved_pose': bool(track['resolved'] and not track.get('alternatives')),
                'resolved_views': views >= self.min_resolved_views,
                'position_bound': uncertainty.get('position_m',math.inf) <= self.max_position_uncertainty,
                'yaw_bound': uncertainty.get('yaw_rad',math.inf) <= self.max_yaw_uncertainty}
            passed = all(checks.values())
            blocks.append({'id':track['id'],'meets_criteria':passed,'checks':checks,'resolved_views':views})
            if not passed:
                reasons.append(track['id']+': '+', '.join(k for k,v in checks.items() if not v))
        # Support deficit provides progress even before an object meets every check.
        support = sum(min(b['resolved_views'],self.min_resolved_views) for b in blocks)
        return {'met':not reasons,'reasons':reasons,'explored_fraction':coverage,
                'min_explored_fraction':self.min_explored_fraction,
                'min_resolved_views':self.min_resolved_views,
                'max_position_uncertainty':self.max_position_uncertainty,
                'max_yaw_uncertainty':self.max_yaw_uncertainty,
                'blocks':blocks,'pending_hypotheses':len(pending),'support_count':support,
                'basis':'evidence thresholds; not calibrated probabilities'}

    def improved(self, previous, current):
        return (current['explored_fraction']-previous['explored_fraction'] >= self.min_coverage_gain
                or current['pending_hypotheses'] < previous['pending_hypotheses']
                or current['support_count'] > previous['support_count']
                or sum(b['meets_criteria'] for b in current['blocks']) >
                   sum(b['meets_criteria'] for b in previous['blocks']))
