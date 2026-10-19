"""Additional observations are bounded, measurable, and preserve identity."""
from dataclasses import replace

from phyai_discovery.criteria import SearchCriteria
from phyai_discovery.perception import Detector, DetectorConfig
from phyai_discovery.search import Search, SearchState
from phyai_discovery.workspace import Workspace
from test_discovery import image_capture

BLOCKS=[(.1,0.,.075,.025,.015,0.)]


def accept(search, detector, camera, stamp, viewpoint):
    if camera=='wrist':
        search.request(stamp)
        search.tick(stamp,True)
    search.begin()
    search.accept(detector.detect(image_capture(BLOCKS,camera,stamp,viewpoint)))


def test_uncertain_object_cells_are_explored_not_free_table():
    workspace=Workspace(scope=(-.2,.2,-.15,.15))
    workspace.merge(Detector().detect(image_capture(BLOCKS)))
    coverage=workspace.snapshot()['coverage']
    assert coverage['observed_fraction'] < coverage['explored_fraction']
    assert coverage['explored_fraction']==1.
    assert any(c['state']=='uncertain' for c in coverage['cells'])


def test_threshold_satisfied_stops_without_using_reserve_views():
    workspace=Workspace(scope=(-.2,.2,-.15,.15))
    detector=Detector(DetectorConfig(known_block_size=(.075,.025,.015)))
    search=Search(workspace,viewpoints=('first',),criteria=SearchCriteria(),
                  inspection_rounds=[['extra']])
    accept(search,detector,'overhead',1.,'overhead')
    accept(search,detector,'wrist',2.,'first')
    assert search.state==SearchState.FINISHED
    assert search.result['criteria_met']
    assert search.result['stop_reason']=='criteria_met'
    assert search.inspection_round==0


def test_unmet_coverage_appends_novel_views_without_resetting_blocks():
    workspace=Workspace(scope=(-.5,.5,-.5,.5))
    detector=Detector(DetectorConfig(known_block_size=(.075,.025,.015)))
    search=Search(workspace,viewpoints=('first',),criteria=SearchCriteria(),
                  inspection_rounds=[['extra'],['last']])
    accept(search,detector,'overhead',1.,'overhead')
    accept(search,detector,'wrist',2.,'first')
    original=workspace.tracks[0]['id']
    assert search.state==SearchState.WRIST
    assert search.target=='extra'
    assert search.result is None
    assert search.inspection_round==1
    accept(search,detector,'wrist',3.,'extra')
    assert workspace.tracks[0]['id']==original
    assert search.state==SearchState.FINISHED
    assert search.result['stop_reason']=='no_progress'
    assert not search.result['criteria_met']
    assert search.result['status']=='partial'
    search.reset()
    assert search.viewpoints==('first',)
    assert search.inspection_round==0
    assert search.assessment is None


def test_view_budget_cannot_repeat_forever():
    workspace=Workspace(scope=(-.5,.5,-.5,.5))
    detector=Detector(DetectorConfig(known_block_size=(.075,.025,.015)))
    search=Search(workspace,viewpoints=('first',),criteria=SearchCriteria(max_no_progress_rounds=5),
                  inspection_rounds=[['extra']])
    accept(search,detector,'overhead',1.,'overhead')
    accept(search,detector,'wrist',2.,'first')
    accept(search,detector,'wrist',3.,'extra')
    assert search.result['stop_reason']=='view_budget_exhausted'
    assert search.inspection_round==1


def test_known_geometry_needs_support_views_to_meet_scan_threshold():
    workspace=Workspace(scope=(-.2,.2,-.15,.15))
    workspace.merge(Detector(DetectorConfig(known_block_size=(.075,.025,.015))).detect(image_capture(BLOCKS)))
    report=SearchCriteria().assess(workspace.snapshot())
    assert len(workspace.tracks)==1
    assert not report['met']
    assert not report['blocks'][0]['checks']['resolved_views']
    assert report['blocks'][0]['checks']['position_bound']
    assert 'probabilities' in report['basis']


def test_contradictory_pose_blocks_success_even_with_old_support():
    workspace=Workspace(scope=(-.2,.2,-.15,.15))
    detector=Detector(DetectorConfig(known_block_size=(.075,.025,.015)))
    workspace.merge(detector.detect(image_capture(BLOCKS)))
    workspace.merge(detector.detect(image_capture(BLOCKS,'wrist',2.,'first')))
    workspace.tracks[0]['resolved']=False
    report=SearchCriteria().assess(workspace.snapshot())
    assert not report['met']
    assert not report['blocks'][0]['checks']['resolved_pose']


def test_inspection_stops_early_when_new_view_meets_criteria():
    workspace=Workspace(scope=(-.2,.2,-.15,.15))
    detector=Detector(DetectorConfig(known_block_size=(.075,.025,.015)))
    search=Search(workspace,viewpoints=('first',),
                  criteria=SearchCriteria(min_resolved_views=3),
                  inspection_rounds=[['extra','unused']])
    accept(search,detector,'overhead',1.,'overhead')
    accept(search,detector,'wrist',2.,'first')
    assert search.target=='extra'
    original=workspace.tracks[0]['id']
    accept(search,detector,'wrist',3.,'extra')
    assert search.state==SearchState.FINISHED
    assert search.result['stop_reason']=='criteria_met'
    assert search.index < len(search.viewpoints)
    assert workspace.tracks[0]['id']==original
