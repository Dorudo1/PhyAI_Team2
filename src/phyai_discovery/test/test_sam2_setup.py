"""Automatic model provisioning, config path handling and atomic downloads."""
import io
import json
from pathlib import Path
import subprocess

import pytest
from phyai_discovery import setup_sam2 as setup
from phyai_discovery import segmentation


def test_generated_config_preserves_installation_and_quotes_environment(tmp_path):
    source=tmp_path/'installation';source.mkdir()
    config=source/'scan.json'
    original={'cameras':{'wrist':{'mask':'mask.npy','intrinsics_file':'camera.json'}},
              'scan':{'targets':{'side':[0.]*7}},'segmentation':{'backend':'depth','min_score':.8}}
    config.write_text(json.dumps(original))
    runtime=tmp_path/'cache with spaces';runtime.mkdir()
    checkpoint=runtime/'weights.pt'
    site=runtime/'python site'
    generated=setup.write_runtime(runtime,runtime/'venv/bin/python',checkpoint,
                                  {'site':str(site)},'cpu',config)
    data=json.loads(generated.read_text())
    assert data['scan']==original['scan']
    assert data['cameras']['wrist']['mask']==str(source/'mask.npy')
    assert data['cameras']['wrist']['intrinsics_file']==str(source/'camera.json')
    assert data['segmentation']['backend']=='sam2' and data['segmentation']['min_score']==.8
    assert json.loads(config.read_text())==original
    result=subprocess.run(['bash','-c','source "$1"; printf "%s" "$PHYAI_SAM2_CHECKPOINT"',
                           'check',str(runtime/'env.sh')],check=True,capture_output=True,text=True)
    assert result.stdout==str(checkpoint)


@pytest.mark.parametrize('valid',[True,False])
def test_download_is_published_only_after_model_validation(tmp_path,monkeypatch,valid):
    checkpoint=tmp_path/'weights.pt'
    monkeypatch.setattr(setup,'urlopen',lambda *_args,**_kwargs:io.BytesIO(b'weights'))
    def validate(python,path):
        assert not checkpoint.exists()
        assert path.read_bytes()==b'weights'
        if not valid:raise ValueError('Bad checkpoint')
    monkeypatch.setattr(setup,'probe',validate)
    if valid:
        setup.download_checkpoint(checkpoint,Path('/python'))
        assert checkpoint.read_bytes()==b'weights'
    else:
        with pytest.raises(ValueError,match='Bad checkpoint'):
            setup.download_checkpoint(checkpoint,Path('/python'))
        assert not checkpoint.exists()
    assert not list(tmp_path.glob('*.partial-*'))


def test_ready_runtime_reuses_dependencies_and_checkpoint(tmp_path,monkeypatch):
    config=tmp_path/'config.json';config.write_text(json.dumps({'cameras':{'overhead':{}}}))
    checkpoint=tmp_path/'model.pt';checkpoint.write_bytes(b'previously cached')
    monkeypatch.setattr(setup,'probe',lambda *_:{'site':'/venv/site','cuda':False})
    monkeypatch.setattr(setup.subprocess,'run',lambda *_args,**_kwargs:pytest.fail('Unexpected reinstall'))
    monkeypatch.setattr(setup,'download_checkpoint',lambda *_:pytest.fail('Unexpected download'))
    assert setup.main(['--runtime-dir',str(tmp_path/'runtime'),'--checkpoint',str(checkpoint),
                       '--config',str(config)])==0
    data=json.loads((tmp_path/'runtime/discovery.json').read_text())
    assert data['segmentation']['checkpoint']==str(checkpoint)
    assert data['segmentation']['device']=='cpu'


def test_missing_config_fails_before_installation(tmp_path,monkeypatch):
    monkeypatch.setattr(setup,'probe',lambda *_:pytest.fail('Must validate config first'))
    with pytest.raises(OSError):
        setup.main(['--runtime-dir',str(tmp_path/'runtime'),'--config',str(tmp_path/'missing.json')])


def test_segmenter_uses_provisioned_environment_unless_explicitly_overridden(monkeypatch):
    calls=[]
    monkeypatch.setenv('PHYAI_SAM2_CHECKPOINT','/shared/model.pt')
    monkeypatch.setattr(segmentation,'SAM2Segmenter',lambda *args:calls.append(args) or 'ready')
    assert segmentation.make_segmenter({'backend':'sam2'})=='ready'
    assert calls[-1][0]=='/shared/model.pt'
    segmentation.make_segmenter({'backend':'sam2','checkpoint':'/app/model.pt'})
    assert calls[-1][0]=='/app/model.pt'
