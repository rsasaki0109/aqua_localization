import hashlib
import importlib.util
import json
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location('ntnu_hf', Path(__file__).resolve().parents[2]/'scripts/benchmarks/acquire_ntnu_hf.py')
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
DATA = bytes(range(128))
DIGEST = hashlib.sha256(DATA).hexdigest()
IDENTITY = {'source': m.URL, 'bytes': len(DATA), 'etag': '"fixture-v1"'}


def fetch(start, end, identity=None):
    assert identity is None or identity == IDENTITY
    return DATA[start:end+1], dict(IDENTITY)


def acquire(root, **kwargs):
    return m.acquire(root, size=len(DATA), expected_sha=DIGEST, chunk_bytes=16,
                     workers=1, fetch=fetch, **kwargs)


def test_full_hash_and_repeat_native_gate(tmp_path):
    result=acquire(tmp_path)
    assert result['verified_sha256']==DIGEST and result['evaluation_ready'] is False
    assert (tmp_path/'fjord_1.bag').read_bytes()==DATA
    assert acquire(tmp_path)==result


def incomplete(root):
    m.save(root/'identity.json', dict(identity=IDENTITY,publisher_sha256=DIGEST,chunk_bytes=16))
    (root/'ranges').mkdir()
    (root/'fjord_1.bag.partial').write_bytes(b'wrong interrupted bytes'+bytes(105))


def test_unknown_interrupted_bytes_preserved_and_remotely_replaced(tmp_path):
    incomplete(tmp_path);old=(tmp_path/'fjord_1.bag.partial').read_bytes()
    acquire(tmp_path)
    assert (tmp_path/'fjord_1.bag').read_bytes()==DATA
    backups=sorted(tmp_path.glob('interrupted-range-*.bin'))
    assert backups and backups[0].read_bytes()==old[:16]


def test_committed_range_corruption_refuses_reuse(tmp_path):
    incomplete(tmp_path)
    m.save(tmp_path/'ranges/000000000000.json', dict(identity=IDENTITY,start=0,end=15,
           sha256=hashlib.sha256(DATA[:16]).hexdigest()))
    old=(tmp_path/'fjord_1.bag.partial').read_bytes()
    with pytest.raises(ValueError,match='verified sensor range corrupted'):acquire(tmp_path)
    assert (tmp_path/'fjord_1.bag.partial').read_bytes()==old
    assert not (tmp_path/'acquired.json').exists()


def test_changed_remote_etag_refuses_resume(tmp_path):
    incomplete(tmp_path)
    def changed(a,b,identity=None):return DATA[a:b+1],dict(IDENTITY,etag='"changed"')
    with pytest.raises(ValueError,match='retained identity/contract differs'):
        m.acquire(tmp_path,size=128,expected_sha=DIGEST,chunk_bytes=16,fetch=changed)


def test_full_publisher_hash_mismatch_is_not_native_ready(tmp_path):
    with pytest.raises(ValueError,match='publisher LFS hash mismatch'):
        m.acquire(tmp_path,size=128,expected_sha='0'*64,chunk_bytes=16,workers=1,fetch=fetch)
    assert (tmp_path/'fjord_1.bag.partial').exists()
    assert not (tmp_path/'acquired.json').exists()
    assert not (tmp_path/'fjord_1.bag').exists()


@pytest.mark.parametrize('url',['http://us.aws.cdn.hf.co/file','https://foreign.invalid/file'])
def test_unsafe_redirect_refused(url):
    with pytest.raises(ValueError,match='unregistered or non-TLS'):
        m.OfficialRedirect().redirect_request(None,None,302,'',None,url)
