#!/usr/bin/env python3
"""Evaluate configured scan poses on rendered seeded scenes, without ROS motion.

The detector receives RGB-D, calibration and robot masks. Simulator block poses
are used only by the evaluator after detection, never by discovery or planning.
Run with MUJOCO_GL=egl when no display is available.
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'src/phyai_discovery'), str(ROOT/'sim/piper_mujoco_sim')]
from phyai_discovery import Discovery  # noqa: E402
from phyai_discovery.observations import Capture, Intrinsics  # noqa: E402
from phyai_discovery.search import SearchState  # noqa: E402
from phyai_discovery.criteria import inspection_targets  # noqa: E402
from piper_mujoco_sim.inspection import InspectionPlanner  # noqa: E402
from piper_mujoco_sim.scatter import BlockScatter  # noqa: E402
from piper_mujoco_sim.scene import build_scene, reset_scene  # noqa: E402
from evaluate_scatter import evaluate  # noqa: E402


def capture_view(model, data, renderer, robot_geoms, camera, viewpoint, stamp):
    renderer.update_scene(data, camera=camera)
    rgb = renderer.render().copy()
    renderer.enable_depth_rendering()
    depth = renderer.render().astype(np.float32).copy()
    renderer.disable_depth_rendering()
    renderer.enable_segmentation_rendering()
    segmentation = renderer.render().copy()
    renderer.disable_segmentation_rendering()
    mask = (np.isin(segmentation[:, :, 0], robot_geoms)
            & (segmentation[:, :, 1] == mujoco.mjtObj.mjOBJ_GEOM))
    height, width = depth.shape
    camera_id = model.camera(camera).id
    focal = height/(2*math.tan(math.radians(float(model.cam_fovy[camera_id]))/2))
    transform = np.eye(4)
    transform[:3, :3] = data.cam_xmat[camera_id].reshape(3, 3) @ np.diag([1., -1., -1.])
    transform[:3, 3] = data.cam_xpos[camera_id]
    return Capture(camera, stamp, camera+'_optical', 'world', 'rendered_benchmark',
                   Intrinsics(width, height, focal, focal, (width-1)/2, (height-1)/2),
                   depth, transform, viewpoint, rgb=rgb, mask=mask)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seeds', nargs='+', type=int, default=[42, 17, 43, 7, 101])
    parser.add_argument('--table', default='all', choices=('all', '0_0', '0_1', '1_0', '1_1'))
    parser.add_argument('--config', type=Path, default=ROOT/'apps/junh/config/mujoco.json')
    parser.add_argument('--backend', choices=('depth', 'sam2'), default='depth')
    parser.add_argument('--output', type=Path, help='Write per-capture metrics and final GT scores')
    parser.add_argument('--fixed-inspection', action='store_true', help='Compare against the fixed reserve sweep')
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    if args.backend == 'depth':
        config['segmentation'] = {'backend': 'depth'}
    else:
        os.environ.setdefault('JUNH_SAM2_CHECKPOINT', str(ROOT/'apps/junh/.models/sam2.1_hiera_tiny.pt'))
    scene_path = ROOT/'sim/piper_mujoco_sim/config/lab_scene.json'
    scene = json.loads(scene_path.read_text())
    model = mujoco.MjModel.from_xml_string(build_scene(
        Path('/opt/mujoco_menagerie/agilex_piper/piper.xml'), scene_path))
    data = mujoco.MjData(model)
    scatter = BlockScatter(model, data, scene)
    base = model.body('base_link').id
    robot_bodies = []
    for body in range(model.nbody):
        ancestor = body
        while ancestor > 0 and ancestor != base:
            ancestor = model.body_parentid[ancestor]
        if ancestor == base:
            robot_bodies.append(body)
    robot_geoms = np.flatnonzero(np.isin(model.geom_bodyid, robot_bodies))
    joint_addresses = [model.joint(f'joint{i}').qposadr[0] for i in range(1, 7)]
    discovery = Discovery(config)
    planner = (InspectionPlanner(model,list(config['sim_scan']['targets'].values()),
                               config['sim_scan'].get('max_targeted_views_per_block',2))
               if config['sim_scan'].get('targeted_inspection') and not args.fixed_inspection else None)
    results = []
    with mujoco.Renderer(model, height=240, width=320) as renderer:
        for seed in args.seeds:
            reset_scene(model, data, scene)
            truth = scatter.plan(scene, seed=seed, table=args.table)
            scatter.apply(truth)
            discovery.reset()
            if planner:planner.reset()
            history = []
            started = time.monotonic()
            while discovery.search.state != SearchState.FINISHED:
                search = discovery.search
                camera = 'overhead' if search.state == SearchState.OVERHEAD else 'wrist'
                viewpoint = search.target
                stamp = float(search.accepted+search.rejected+1)
                plan = None
                if camera == 'wrist':
                    target = config['sim_scan']['targets'][viewpoint]
                    if planner and search.inspection_round:
                        focus = inspection_targets(discovery.workspace.hypotheses,search.assessment)
                        plan = planner.choose(focus,
                                              [*data.qpos[joint_addresses],.035],viewpoint)
                        if plan:target=plan['joints']
                        elif focus:
                            discovery.finish('No clear novel targeted views remain within the per-object inspection budget.',
                                             stop_reason='targeted_views_exhausted')
                            break
                    if planner:planner.remember(viewpoint,target)
                    data.qpos[joint_addresses] = target[:6]
                    data.qpos[model.joint('joint7').qposadr[0]] = target[6]
                    data.qpos[model.joint('joint8').qposadr[0]] = -target[6]
                    discovery.request_capture(stamp)
                    discovery.tick(stamp, stationary=True)
                mujoco.mj_forward(model, data)
                capture = capture_view(model, data, renderer, robot_geoms, camera, viewpoint, stamp)
                discovery.process(capture, stationary=True)
                snapshot = discovery.snapshot()
                score = evaluate(snapshot, truth, scene)
                history.append({'viewpoint': viewpoint, 'captures': search.accepted,
                                'inspection_focus':plan,
                                'unmet_reasons':(search.assessment or {}).get('reasons',[]),
                                'confirmed_truth': sum(b['confirmed'] for b in score['blocks']),
                                'hypotheses': score['hypotheses'], **score['scope_coverage']})
            score = evaluate(discovery.snapshot(), truth, scene)
            score.update(seed=seed, backend=args.backend, history=history,
                         assessment=search.assessment,
                         rendering_detection_seconds=round(time.monotonic()-started, 3))
            results.append(score)
            print(json.dumps({key: score[key] for key in
                  ('seed', 'captures', 'confirmed', 'hypotheses', 'scope_coverage', 'result')}), flush=True)
    if args.output:
        args.output.write_text(json.dumps(results, indent=2)+'\n')


if __name__ == '__main__':
    main()
