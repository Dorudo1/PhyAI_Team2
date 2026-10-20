"""Provision the pinned discovery segmenter and generate its launch environment."""
import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
from urllib.request import urlopen

SAM2_COMMIT = '2b90b9f5ceec907a1c18123530e92e794ad901a4'
CHECKPOINT_URL = 'https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt'
PROBE = '''import importlib.metadata as m, json, sys, sysconfig
import torch, torchvision, hydra, numpy
assert (1,24) <= tuple(map(int,numpy.__version__.split('.')[:2])) < (2,0), 'Unexpected NumPy version'
from sam2.build_sam import build_sam2
assert torch.__version__.split('+')[0] == '2.5.1', 'Unexpected PyTorch version'
assert torchvision.__version__.split('+')[0] == '0.20.1', 'Unexpected torchvision version'
assert m.version('hydra-core') == '1.3.2', 'Unexpected Hydra version'
origin=json.loads(m.distribution('SAM-2').read_text('direct_url.json') or '{}')
assert origin.get('vcs_info',{}).get('commit_id') == sys.argv[1], 'Unexpected SAM2 source revision'
if len(sys.argv)>2:
    build_sam2('configs/sam2.1/sam2.1_hiera_t.yaml',sys.argv[2],device='cpu')
print(json.dumps({'site':sysconfig.get_paths()['purelib'],'cuda':torch.cuda.is_available()}))
'''


def probe(python, checkpoint=None):
    args=[str(python),'-c',PROBE,SAM2_COMMIT]
    if checkpoint is not None:args.append(str(checkpoint))
    result=subprocess.run(args,check=True,text=True,capture_output=True)
    return json.loads(result.stdout.strip().splitlines()[-1])


def download_checkpoint(destination, python):
    """Validate downloaded weights before publishing the cache file atomically."""
    destination.parent.mkdir(parents=True,exist_ok=True)
    fd,name=tempfile.mkstemp(prefix=destination.name+'.partial-',dir=destination.parent)
    partial=Path(name)
    try:
        with os.fdopen(fd,'wb') as output, urlopen(CHECKPOINT_URL,timeout=120) as source:
            shutil.copyfileobj(source,output)
        probe(python,partial)
        os.replace(partial,destination)
    finally:
        partial.unlink(missing_ok=True)


def write_runtime(runtime, python, checkpoint, info, device, config):
    """Resolve camera asset paths before moving the config into the runtime cache."""
    config_path=Path(config).resolve()
    data=json.loads(config_path.read_text())
    for camera in data.get('cameras',{}).values():
        for key in ('mask','intrinsics_file'):
            if camera.get(key):camera[key]=str((config_path.parent/camera[key]).resolve())
    data['segmentation']={**data.get('segmentation',{}),'backend':'sam2',
        'checkpoint':str(checkpoint),'device':device,
        'model_config':'configs/sam2.1/sam2.1_hiera_t.yaml'}
    generated=runtime/'discovery.json'
    generated.write_text(json.dumps(data,indent=2)+'\n')
    environment=(f'export PHYAI_SAM2_CHECKPOINT={shlex.quote(str(checkpoint))}\n'
        f'export JUNH_SAM2_CHECKPOINT={shlex.quote(str(checkpoint))}\n'
        f'export PHYAI_DISCOVERY_CONFIG={shlex.quote(str(generated))}\n'
        f'export PHYAI_SAM2_PYTHON={shlex.quote(str(python))}\n'
        f'export PYTHONPATH={shlex.quote(info["site"])}:"${{PYTHONPATH:-}}"\n')
    (runtime/'env.sh').write_text(environment)
    return generated


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime-dir',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--venv',type=Path,help='Reuse/provision a specific virtual environment')
    parser.add_argument('--checkpoint',type=Path,help='Reuse/provision a specific checkpoint')
    parser.add_argument('--device',choices=('auto','cpu','cuda'),default='auto')
    args=parser.parse_args(argv)
    config=json.loads(args.config.read_text())
    if not isinstance(config,dict) or not config.get('cameras'):
        raise ValueError('--config must be a discovery ROS configuration with cameras.')
    runtime=args.runtime_dir.resolve();runtime.mkdir(parents=True,exist_ok=True)
    # Prevent concurrent first launches from installing into the same environment.
    import fcntl
    with (runtime/'setup.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        venv=(args.venv or runtime/'venv').resolve()
        checkpoint=(args.checkpoint or runtime/'models/sam2.1_hiera_tiny.pt').resolve()
        python=venv/'bin/python'
        try:
            info=probe(python)
        except (OSError,subprocess.CalledProcessError,ValueError):
            print('Installing pinned SAM2 dependencies (first setup requires network access).',flush=True)
            subprocess.run([sys.executable,'-m','venv','--system-site-packages',str(venv)],check=True)
            pip=[str(python),'-m','pip','install']
            subprocess.run([*pip,'numpy>=1.24,<2','hydra-core==1.3.2','wheel'],check=True)
            subprocess.run([*pip,'torch==2.5.1','torchvision==0.20.1',
                            '--index-url','https://download.pytorch.org/whl/cu121'],check=True)
            subprocess.run([*pip,'--no-build-isolation',
                'SAM-2 @ git+https://github.com/facebookresearch/sam2.git@'+SAM2_COMMIT],
                env={**os.environ,'SAM2_BUILD_CUDA':'0'},check=True)
            info=probe(python)
        if not checkpoint.is_file() or checkpoint.stat().st_size==0:
            print('Downloading SAM2.1 tiny checkpoint.',flush=True)
            download_checkpoint(checkpoint,python)
        # Build/load the actual model, rather than trusting a nonempty weights file.
        info=probe(python,checkpoint)
        device=('cuda' if info['cuda'] else 'cpu') if args.device=='auto' else args.device
        generated=write_runtime(runtime,python,checkpoint,info,device,args.config)
        print(f'SAM2 ready ({device}). Config: {generated}\nEnvironment: {runtime/"env.sh"}',flush=True)
    return 0


if __name__=='__main__':
    try:
        raise SystemExit(main())
    except (OSError,ValueError,subprocess.CalledProcessError) as error:
        print(f'SAM2 setup failed: {error}',file=sys.stderr)
        if isinstance(error,subprocess.CalledProcessError) and error.stderr:
            print(error.stderr,file=sys.stderr)
        raise SystemExit(1)
