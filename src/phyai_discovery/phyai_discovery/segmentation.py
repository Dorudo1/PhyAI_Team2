"""Prompted SAM 2 masks from measured RGB-D; model loaded once per detector."""
from pathlib import Path
import os

import numpy as np


class SAM2Segmenter:
    def __init__(self, checkpoint, model_config='configs/sam2.1/sam2.1_hiera_t.yaml',
                 device='cuda', min_score=.7):
        try:
            import torch
            from sam2.build_sam import build_sam2
            from sam2.sam2_image_predictor import SAM2ImagePredictor
        except ImportError as error:
            raise ValueError('SAM 2 runtime unavailable. Install PyTorch and SAM 2 in the application runtime.') from error
        if not 0 <= min_score <= 1:
            raise ValueError('SAM 2 min_score must be between zero and one.')
        checkpoint = Path(os.path.expandvars(os.path.expanduser(checkpoint)))
        if not checkpoint.is_file():
            raise ValueError(f'SAM 2 checkpoint missing: {checkpoint}. Configure segmentation.checkpoint with an existing local weights file.')
        if device == 'cuda' and not torch.cuda.is_available():
            raise ValueError('SAM 2 CUDA requested but unavailable; configure device=cpu explicitly.')
        self.torch = torch
        self.predictor = SAM2ImagePredictor(build_sam2(model_config, str(checkpoint), device=device))
        self.min_score = min_score

    def segment(self, capture, pixels, groups):
        if capture.rgb is None:
            raise ValueError('SAM 2 requires capture-matched aligned RGB.')
        masks = []
        with self.torch.inference_mode():
            self.predictor.set_image(capture.rgb)
            for group in groups:
                uv = pixels[group]
                lo, hi = uv.min(axis=0), uv.max(axis=0)
                box = np.array([max(0,lo[0]-2),max(0,lo[1]-2),
                                min(capture.intrinsics.width-1,hi[0]+2),
                                min(capture.intrinsics.height-1,hi[1]+2)])
                # A foreground seed remains on measured above-table evidence.
                seed = uv[np.argmin(np.linalg.norm(uv-uv.mean(axis=0),axis=1))]
                predicted, scores, _ = self.predictor.predict(
                    point_coords=seed[None,:], point_labels=np.array([1]),
                    box=box, multimask_output=True)
                order = np.argsort(scores)[::-1]
                for index in order:
                    if scores[index] < self.min_score:
                        continue
                    mask = predicted[index].astype(bool)
                    # Do not let a large background region claim other proposals.
                    bounded = np.zeros_like(mask)
                    bounded[int(box[1]):int(box[3])+1,int(box[0]):int(box[2])+1] = True
                    mask &= bounded
                    if capture.mask is not None:
                        mask &= ~capture.mask
                    if not mask[uv[:,1],uv[:,0]].any():
                        continue
                    if not any(np.count_nonzero(mask & old) /
                            max(1,np.count_nonzero(mask | old)) > .5 for old in masks):
                        masks.append(mask)
                    break
        return masks


def make_segmenter(config):
    if not config or config.get('backend', 'depth') == 'depth':
        return None
    if config.get('backend') != 'sam2':
        raise ValueError('Unknown segmentation backend.')
    checkpoint=config.get('checkpoint') or os.environ.get('PHYAI_SAM2_CHECKPOINT')
    if not checkpoint:
        raise ValueError('SAM2 checkpoint is not configured. Run scripts/setup_discovery_sam2.sh and source its env.sh.')
    return SAM2Segmenter(checkpoint,
        config.get('model_config','configs/sam2.1/sam2.1_hiera_t.yaml'),
        config.get('device','cuda'), config.get('min_score',.7))
