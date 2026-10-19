"""Discovery feature for shared applications; importing requires only NumPy."""
from .engine import Discovery, CaptureJob
from .observations import Capture, Intrinsics
from .perception import Detection, Detector, DetectorConfig
from .search import SearchState, Stationarity

__all__ = ['Discovery', 'CaptureJob', 'Capture', 'Intrinsics', 'Detection',
           'Detector', 'DetectorConfig', 'SearchState', 'Stationarity']
