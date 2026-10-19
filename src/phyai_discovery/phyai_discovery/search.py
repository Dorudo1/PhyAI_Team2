"""Fixed capture coordination. Never generates or publishes robot commands."""
from enum import Enum
import math

import numpy as np


class SearchState(str, Enum):
    OVERHEAD = 'capture_overhead'
    WRIST = 'wait_for_wrist_view'
    STATIONARY = 'wait_for_stationary_arm'
    CAPTURE = 'capture_wrist'
    PROCESSING = 'detect_and_merge'
    FINISHED = 'search_finished'


class Stationarity:
    def __init__(self, duration=.5, tolerance=.003, timeout=1.):
        if not all(math.isfinite(v) and v > 0 for v in (duration, tolerance, timeout)):
            raise ValueError('Stationary thresholds must be positive.')
        self.duration, self.tolerance, self.timeout = duration, tolerance, timeout
        self.clear()

    def clear(self):
        self.anchor = None
        self.since = self.latest = None

    def update(self, positions, now):
        q = np.asarray(positions, dtype=float)
        if q.shape != (7,) or not np.isfinite(q).all() or not math.isfinite(now):
            self.clear()
            return
        if self.latest is not None and (now <= self.latest or now-self.latest > self.timeout):
            self.clear()
        if self.anchor is None or np.max(np.abs(q-self.anchor)) > self.tolerance:
            self.anchor, self.since = q.copy(), now
        self.latest = now

    def ready(self, now):
        return self.latest is not None and 0 <= now-self.latest <= self.timeout and now-self.since >= self.duration


class Search:
    def __init__(self, workspace, viewpoints=('wrist_1', 'wrist_2'), max_retries=2, capture_timeout=15., criteria=None, inspection_rounds=()):
        if len(set(viewpoints)) != len(viewpoints) or any(not isinstance(v, str) or not v or v == 'overhead' for v in viewpoints):
            raise ValueError('Wrist viewpoints require unique nonempty labels.')
        if not viewpoints or max_retries < 0 or not math.isfinite(capture_timeout) or capture_timeout <= 0:
            raise ValueError('Invalid search configuration.')
        from .criteria import SearchCriteria
        self.criteria = SearchCriteria(**criteria) if isinstance(criteria,dict) else criteria
        self.base_viewpoints = tuple(viewpoints)
        self.inspection_rounds = tuple(tuple(views) for views in inspection_rounds)
        labels = list(self.base_viewpoints) + [view for views in self.inspection_rounds for view in views]
        if any(not views for views in self.inspection_rounds) or len(set(labels)) != len(labels):
            raise ValueError('Inspection rounds require unique new physical viewpoint labels.')
        if any(not isinstance(v,str) or not v or v=='overhead' for v in labels):
            raise ValueError('Invalid inspection viewpoint label.')
        self.workspace, self.viewpoints = workspace, self.base_viewpoints
        self.max_retries, self.timeout = max_retries, capture_timeout
        self.reset()

    def reset(self):
        self.viewpoints = self.base_viewpoints
        self.inspection_round = 0
        self.no_progress_rounds = 0
        self.assessment = None
        self.previous_assessment = None
        self.state = SearchState.OVERHEAD
        self.index = 0
        self.accepted = self.rejected = 0
        self.attempts = 0
        self.reason = 'Waiting for a fresh overhead capture.'
        self.pending_since = None
        self.generation = getattr(self, 'generation', 0)+1
        self.result = None

    @property
    def target(self):
        overhead = self.state == SearchState.OVERHEAD or (self.state == SearchState.PROCESSING and self.processing_camera == 'overhead')
        return 'overhead' if overhead else self.viewpoints[min(self.index, len(self.viewpoints)-1)]

    def request(self, now):
        if self.state != SearchState.WRIST:
            raise ValueError('A wrist capture can be requested only when waiting for a viewpoint.')
        self.state = SearchState.STATIONARY
        self.pending_since = now
        self.reason = f'Waiting for stationary feedback at {self.target}.'

    def tick(self, now, stationary):
        if self.state == SearchState.STATIONARY and stationary:
            self.state = SearchState.CAPTURE
            self.reason = f'Waiting for a fresh wrist image at {self.target}.'
        if self.state in (SearchState.OVERHEAD, SearchState.STATIONARY, SearchState.CAPTURE, SearchState.PROCESSING):
            if self.pending_since is None:
                self.pending_since = now
            if now-self.pending_since > self.timeout:
                self.reject('Capture or processing timed out.')

    def accepts(self, capture, stationary=True):
        return ((self.state == SearchState.OVERHEAD and capture.camera == 'overhead' and capture.viewpoint == 'overhead')
                or (self.state == SearchState.CAPTURE and capture.camera == 'wrist'
                    and capture.viewpoint == self.target and stationary))

    def begin(self):
        if self.state not in (SearchState.OVERHEAD, SearchState.CAPTURE):
            raise ValueError('Capture is not currently requested.')
        self.processing_camera = 'overhead' if self.state == SearchState.OVERHEAD else 'wrist'
        self.state = SearchState.PROCESSING
        self.reason = 'Detecting and merging capture.'

    def accept(self, detection):
        if self.state != SearchState.PROCESSING:
            raise ValueError('No capture is being processed.')
        self.workspace.merge(detection)
        if self.criteria is not None:
            self.assessment = self.criteria.assess(self.workspace.snapshot())
        self.accepted += 1
        self.attempts = 0
        self.pending_since = None
        if detection.camera == 'overhead':
            self.state = SearchState.WRIST
        else:
            self.index += 1
            if self.inspection_round and self.assessment is not None and self.assessment['met']:
                self.finish('Configured observation criteria met.', stop_reason='criteria_met')
                return
            if self.index >= len(self.viewpoints):
                self.end_pass('Predefined capture sequence finished.')
                return
            self.state = SearchState.WRIST
        self.reason = f'Position the wrist camera at {self.target}, then request capture.'

    def reject(self, reason):
        overhead = self.state == SearchState.OVERHEAD or (
            self.state == SearchState.PROCESSING and self.processing_camera == 'overhead')
        self.rejected += 1
        self.attempts += 1
        self.generation += 1  # invalidate worker results after timeout/rejection
        self.pending_since = None
        self.reason = reason
        if self.attempts > self.max_retries:
            self.attempts = 0
            if overhead:
                self.state = SearchState.WRIST
                self.reason = f'Overhead unavailable; position wrist at {self.target}.'
            else:
                self.index += 1
                if self.index >= len(self.viewpoints):
                    self.end_pass('Capture retry budget exhausted; scan coverage is incomplete.')
                else:
                    self.state = SearchState.WRIST
        else:
            self.state = SearchState.OVERHEAD if overhead else SearchState.WRIST

    def end_pass(self, reason):
        """Append configured novel views, without resetting map or identities."""
        if self.criteria is None:
            self.finish(reason)
            return
        self.assessment = self.criteria.assess(self.workspace.snapshot())
        if self.assessment['met']:
            self.finish('Configured observation criteria met.', stop_reason='criteria_met')
            return
        if self.previous_assessment is not None:
            if self.criteria.improved(self.previous_assessment,self.assessment):
                self.no_progress_rounds = 0
            else:
                self.no_progress_rounds += 1
            if self.no_progress_rounds >= self.criteria.max_no_progress_rounds:
                self.finish('Additional views did not improve observation evidence.', stop_reason='no_progress')
                return
        if self.inspection_round >= len(self.inspection_rounds):
            self.finish('Configured inspection views exhausted; observation criteria remain unmet.',
                        stop_reason='view_budget_exhausted')
            return
        self.previous_assessment = self.assessment
        self.viewpoints += self.inspection_rounds[self.inspection_round]
        self.inspection_round += 1
        self.state = SearchState.WRIST
        self.pending_since = None
        self.result = None
        self.reason = f'Inspection round {self.inspection_round}: '+ '; '.join(self.assessment['reasons'])

    def finish(self, reason='Operator finished the scan.', stop_reason='operator_or_sequence'):
        self.generation += 1
        self.state = SearchState.FINISHED
        self.reason = reason
        snap = self.workspace.snapshot()
        if self.criteria is not None:
            self.assessment = self.criteria.assess(snap)
        complete = (snap['coverage']['scope'] is not None
                    and snap['coverage']['observed_fraction'] == 1.
                    and not any(not track['resolved'] for track in snap['tracks'])
                    and not snap.get('hypotheses')
                    and not snap['warnings'])
        self.result = {'status': 'failed' if not self.accepted or snap['table'] is None
                       else 'complete' if complete else 'partial',
                       'snapshot_revision': snap['revision'], 'reason': reason,
                       'stop_reason':stop_reason,
                       'criteria_met': self.assessment['met'] if self.assessment is not None else None,
                       'pick_ready': False}  # grasp geometry and safeguard remain unverified

    def progress(self):
        return {'state': self.state.value, 'viewpoint_index': self.index,
                'viewpoint_count': len(self.viewpoints), 'target': self.target,
                'accepted': self.accepted, 'rejected': self.rejected,
                'attempts': self.attempts, 'reason': self.reason, 'result': self.result,
                'inspection_round':self.inspection_round,'inspection_round_limit':len(self.inspection_rounds),
                'assessment':self.assessment}
