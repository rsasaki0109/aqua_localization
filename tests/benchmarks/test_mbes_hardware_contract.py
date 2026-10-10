"""Hardware eligibility and pre-reference freeze gates, not accuracy evidence."""
import sys
from pathlib import Path
from types import SimpleNamespace as N
import json
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts/benchmarks'))
import mbes_hardware_score as m

POLICY={'max_horizontal_accuracy_m':0.1,'max_vertical_accuracy_m':0.2}


def packet(**kwargs):
    values=dict(gnssFixOK=True,diffSoln=True,invalidLlh=False,validTime=True,
                fullyResolved=True,carrSoln=2,hAcc=0.05,vAcc=0.1,raw=N(fixType=3))
    return N(**(values|kwargs))


@pytest.mark.parametrize('change,reason',[
    ({'carrSoln':1},'carrSoln'),({'gnssFixOK':False},'gnssFixOK'),
    ({'invalidLlh':True},'invalidLlh'),({'hAcc':0.100001},'hAcc'),
    ({'vAcc':float('nan')},'vAcc'),({'validTime':False},'validTime')])
def test_quality_cannot_substitute_float_invalid_or_imprecise_rtk(change,reason):
    assert reason in m.quality_reasons(packet(**change),POLICY)


def test_exact_quality_limits_included():
    assert not m.quality_reasons(packet(hAcc=0.1,vAcc=0.2),POLICY)


def test_ecef_preserves_meter_scale():
    assert np.allclose(m.ecef(0,0,0),[6378137,0,0])
    assert np.allclose(m.ecef(0,0,1)-m.ecef(0,0,0),[1,0,0])


def test_missing_freeze_blocks_reference_reader(tmp_path,monkeypatch):
    monkeypatch.setattr(m,'validate',lambda root:{})
    def forbidden(*args):raise AssertionError('reference reader opened before freeze')
    monkeypatch.setattr(m,'Reader',forbidden)
    with pytest.raises(FileNotFoundError):m.score(tmp_path)
    assert not (tmp_path/'hardware-scored').exists()


def test_failed_trial_blocks_reference_reader(tmp_path,monkeypatch):
    (tmp_path/'plan.json').write_text('{}')
    plan={'trials':[{'id':'gicp-1'}]};monkeypatch.setattr(m,'validate',lambda root:plan)
    freeze={'all_four_outcomes_frozen':True,'plan_sha256':m.sha(tmp_path/'plan.json'),
            'trials':[{'id':'gicp-1','status':'failed'}]}
    (tmp_path/'freeze.json').write_text(json.dumps(freeze))
    monkeypatch.setattr(m,'Reader',lambda *args:pytest.fail('reference reader opened before successful freeze'))
    with pytest.raises(ValueError,match='failed/mutated trial'):m.score(tmp_path)
    assert not (tmp_path/'hardware-scored').exists()


def test_fixed_chain_respects_mount_rotation(tmp_path):
    p=tmp_path/'calibration.urdf'
    p.write_text('<robot name="test"><joint name="a" type="fixed"><parent link="base"/><child link="sonar"/><origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/></joint><joint name="b" type="fixed"><parent link="base"/><child link="antenna"/><origin xyz="1 1 0" rpy="0 0 0"/></joint></robot>')
    transforms=m.fixed_transforms(p)
    lever=(np.linalg.inv(transforms['sonar'])@transforms['antenna'])[:3,3]
    assert np.allclose(lever,[1,0,0],atol=1e-12)
