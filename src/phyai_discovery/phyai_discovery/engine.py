"""Discovery lifecycle shared by ROS adapters and in-process applications.

Call lifecycle methods from one owner thread. Only detector.detect(job.capture)
may run on a worker; complete() rejects results invalidated by reset or timeout.
Sensor synchronization, capture freshness, motion, and transport belong to callers.
"""
from copy import deepcopy
from dataclasses import dataclass

from .observations import Capture
from .perception import Detector, DetectorConfig
from .search import Search, SearchState
from .segmentation import make_segmenter
from .workspace import Workspace


@dataclass(frozen=True)
class CaptureJob:
    generation: int
    capture: Capture


class Discovery:
    def __init__(self, config=None, *, detector=None):
        self.config = deepcopy(config or {})
        cfg = self.config
        self.workspace = Workspace(frame=cfg.get('map_frame', 'base_link'),
            cell_size=cfg.get('cell_size', .02), scope=cfg.get('scope'))
        self.search = Search(self.workspace, cfg.get('viewpoints', ('wrist_1', 'wrist_2')),
            cfg.get('max_retries', 2), cfg.get('capture_timeout', 15.),
            criteria=cfg.get('search_criteria'), inspection_rounds=cfg.get('inspection_rounds', ()))
        self.detector = detector if detector is not None else Detector(
            DetectorConfig(**cfg.get('detector', {})), make_segmenter(cfg.get('segmentation')))
        self._active_job = None

    def request_capture(self, now):
        """Request the current wrist view; tick waits for stationary feedback."""
        self.search.request(now)

    def tick(self, now, *, stationary):
        self.search.tick(now, stationary)

    def begin(self, capture, *, stationary):
        """Validate a requested capture before scheduling detection on a worker."""
        capture.validate()
        if capture.map_frame != self.workspace.frame:
            raise ValueError('Observation and map frames differ.')
        if not stationary or not self.search.accepts(capture, stationary):
            raise ValueError('Capture requires a stationary arm and the requested camera/viewpoint.')
        self.search.begin()
        job = CaptureJob(self.search.generation, capture)
        self._active_job = job
        return job

    def _current(self, job):
        return (job is self._active_job and job.generation == self.search.generation
                and self.search.state == SearchState.PROCESSING)

    def complete(self, job, detection):
        """Merge a current worker result; return False for an invalidated job."""
        if not self._current(job):
            return False
        self.search.accept(detection)
        self._active_job = None
        return True

    def fail(self, job, reason):
        """Reject a current worker job without disturbing a newer scan."""
        if not self._current(job):
            return False
        self.search.reject(str(reason))
        self._active_job = None
        return True

    def process(self, capture, *, stationary):
        """Synchronously detect and merge one requested, stationary capture."""
        job = self.begin(capture, stationary=stationary)
        try:
            detection = self.detector.detect(capture)
            self.complete(job, detection)
        except Exception as error:
            self.fail(job, error)
            raise
        return detection

    def reset(self):
        self.workspace.reset()
        self.search.reset()
        self._active_job = None

    def finish(self, reason='Operator finished the scan.', stop_reason='operator_or_sequence'):
        self.search.finish(reason,stop_reason=stop_reason)
        self._active_job = None

    def add_view(self):
        """Append a manual inspection view after finishing, up to three views."""
        base_count = len(self.config.get('viewpoints', ('wrist_1', 'wrist_2')))
        if self.search.state != SearchState.FINISHED or len(self.search.viewpoints) >= base_count+3:
            raise ValueError('Finish the scan first; at most three additional views are supported.')
        label = f'inspection_{len(self.search.viewpoints)-base_count+1}'
        self.search.viewpoints += (label,)
        self.search.index = len(self.search.viewpoints)-1
        self.search.result = None
        self.search.state = SearchState.WRIST
        self.search.pending_since = None
        self.search.reason = f'Position the wrist camera at {label}, then request capture.'
        return label

    def snapshot(self, stamp=None):
        """Return the workspace/search handoff shared by pipeline consumers."""
        return {'workspace': self.workspace.snapshot(stamp), 'search': self.search.progress()}
