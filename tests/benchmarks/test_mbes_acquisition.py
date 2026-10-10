"""Archive integrity and pre-request TLS redirect gates, using local fixtures."""
import gzip
import importlib.util
import io
from pathlib import Path
import tarfile
import urllib.request
import pytest

spec=importlib.util.spec_from_file_location('mbes_acquisition',Path(__file__).resolve().parents[2]/'scripts/benchmarks/acquire_mbes_verified.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def archive(name='native.bag'):
    out=io.BytesIO()
    with tarfile.open(fileobj=out,mode='w') as t:
        member=tarfile.TarInfo(name);member.size=7;t.addfile(member,io.BytesIO(b'fixture'))
    return gzip.compress(out.getvalue())


def fake_remote(monkeypatch, data):
    identity={'source':m.URL,'bytes':len(data),'etag':'"fixture"','tls_verified':True}
    def get(start,end,expected=None):
        if expected:assert expected==identity
        return data[start:end+1],identity
    monkeypatch.setattr(m,'request_range',get)


def test_complete_archive_and_resume_recheck_all_bytes(tmp_path,monkeypatch):
    fake_remote(monkeypatch,archive())
    first=m.acquire(tmp_path)
    assert (tmp_path/'native/native.bag').read_bytes()==b'fixture'
    assert m.acquire(tmp_path)==first
    (tmp_path/'native/native.bag').write_bytes(b'changed')
    with pytest.raises(ValueError,match='retained native bytes differ'):m.acquire(tmp_path)


def test_gzip_footer_corruption_never_ready(tmp_path,monkeypatch):
    data=bytearray(archive());data[-8]^=1;fake_remote(monkeypatch,bytes(data))
    with pytest.raises(gzip.BadGzipFile):m.acquire(tmp_path)
    assert not (tmp_path/'acquired.json').exists()


def test_unsafe_tar_path_never_extracted(tmp_path,monkeypatch):
    fake_remote(monkeypatch,archive('../escape'))
    with pytest.raises(ValueError,match='unsafe TAR member'):m.acquire(tmp_path)
    assert not (tmp_path.parent/'escape').exists()


@pytest.mark.parametrize('destination',['http://seaward.science/file','https://example.com/file'])
def test_blocks_redirect_before_untrusted_request(destination):
    handler=m.OfficialTLSRedirect()
    with pytest.raises(ValueError,match='registered official HTTPS'):
        handler.redirect_request(urllib.request.Request(m.URL),None,302,'',{},destination)


def test_corrupt_retained_range_is_not_reused(tmp_path,monkeypatch):
    fake_remote(monkeypatch,archive());m.acquire(tmp_path)
    (tmp_path/'ranges/000000000000.bin').write_bytes(b'bad')
    with pytest.raises(ValueError,match='retained range integrity'):m.acquire(tmp_path)
