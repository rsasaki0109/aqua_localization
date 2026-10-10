"""TLS/range-verified native acquisition; no reference trajectory deserialization."""
import argparse
import concurrent.futures
import fcntl
import gzip
import hashlib
import json
from pathlib import Path
import re
import tarfile
import urllib.request
import urllib.parse

URL = 'https://www.seaward.science/files/pos-datasets/bag/beach_pond.tar.gz'
CHUNK = 8 * 1024 * 1024


class OfficialTLSRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        parsed = urllib.parse.urlsplit(newurl)
        if parsed.scheme != 'https' or parsed.hostname not in ('seaward.science', 'www.seaward.science'):
            raise ValueError('redirect outside registered official HTTPS hosts')
        return super().redirect_request(req, fp, code, msg, headers, newurl)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1024*1024), b''): h.update(b)
    return h.hexdigest()


def new_json(path, value):
    with path.open('x') as f: json.dump(value, f, indent=2); f.write('\n')


def request_range(start, end, identity=None, url=URL):
    headers = {'Range': f'bytes={start}-{end}', 'Accept-Encoding': 'identity'}
    if identity: headers['If-Match'] = identity['etag']
    opener = urllib.request.build_opener(OfficialTLSRedirect())
    with opener.open(urllib.request.Request(url, headers=headers), timeout=30) as r:
        if not r.url.startswith('https://'): raise ValueError('non-TLS redirect')
        match = re.fullmatch(r'bytes (\d+)-(\d+)/(\d+)', r.headers.get('Content-Range', ''))
        if r.status != 206 or not match or tuple(map(int, match.groups()[:2])) != (start, end):
            raise ValueError('range response mismatch')
        total = int(match[3]); etag = r.headers.get('ETag')
        if not etag or etag.startswith('W/'): raise ValueError('strong ETag required')
        if identity and (total != identity['bytes'] or etag != identity['etag']):
            raise ValueError('remote object identity changed')
        b = r.read(end-start+2)
        if len(b) != end-start+1: raise ValueError('range length mismatch')
        return b, {'source': url, 'bytes': total, 'etag': etag, 'tls_verified': True}


def acquire(root, url=URL):
    parsed = urllib.parse.urlsplit(url)
    filename = Path(parsed.path).name
    if (parsed.scheme != 'https' or parsed.hostname not in ('seaward.science','www.seaward.science') or
        parsed.path != '/files/pos-datasets/bag/'+filename or filename not in (
            'beach_pond.tar.gz','north_river_confluence.tar.gz','dutch_harbor.tar.gz','wiggles_bank.tar.gz')):
        raise ValueError('use a registered official survey archive')
    root.mkdir(parents=True, exist_ok=True)
    with (root/'acquisition.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        _, identity = request_range(0, 0, url=url)
        ip = root/'identity.json'
        if ip.exists():
            if json.loads(ip.read_text()) != identity: raise ValueError('retained identity mismatch')
        else: new_json(ip, identity)
        parts = root/'ranges'; parts.mkdir(exist_ok=True)
        def fetch(start):
            end = min(start+CHUNK, identity['bytes'])-1
            part = parts/f'{start:012d}.bin'; receipt = part.with_suffix('.json')
            if part.exists() and receipt.exists():
                d = json.loads(receipt.read_text())
                if (d['identity'] != identity or d['start'] != start or d['end'] != end or
                    part.stat().st_size != end-start+1 or digest(part) != d['sha256']):
                    raise ValueError('retained range integrity failure')
                return
            if part.exists() or receipt.exists(): raise ValueError('incomplete range retained; inspect before resuming')
            last = None
            for attempt in range(3):
                try: b, _ = request_range(start, end, identity, url=url); break
                except Exception as exc: last = exc
            else: raise RuntimeError(f'range {start}-{end} failed') from last
            with part.open('xb') as f: f.write(b)
            new_json(receipt, dict(identity=identity, start=start, end=end, sha256=hashlib.sha256(b).hexdigest()))
            print(f'verified range {start}-{end}', flush=True)
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            list(pool.map(fetch, range(0, identity['bytes'], CHUNK)))
        archive = root/filename
        if not archive.exists():
            with archive.open('xb') as out:
                for start in range(0, identity['bytes'], CHUNK):
                    with (parts/f'{start:012d}.bin').open('rb') as f:
                        for b in iter(lambda: f.read(1024*1024), b''): out.write(b)
        if archive.stat().st_size != identity['bytes']: raise ValueError('full archive size mismatch; retained')
        # Always bind the completed archive to every separately validated range.
        with archive.open('rb') as f:
            for start in range(0, identity['bytes'], CHUNK):
                b = f.read(min(CHUNK, identity['bytes']-start))
                if hashlib.sha256(b).hexdigest() != json.loads((parts/f'{start:012d}.json').read_text())['sha256']:
                    raise ValueError('archive/range byte mismatch')
        native = root/'native'; native.mkdir(exist_ok=True); members = []
        # Streaming through the full gzip object checks its footer CRC/ISIZE.
        with gzip.open(archive, 'rb') as gz, tarfile.open(fileobj=gz, mode='r|') as tar:
            for member in tar:
                relative = Path(member.name)
                if relative.is_absolute() or '..' in relative.parts or not (member.isfile() or member.isdir()):
                    raise ValueError('unsafe TAR member')
                target = native/relative
                if target.is_symlink() or not target.resolve().is_relative_to(native.resolve()):
                    raise ValueError('native path escapes destination')
                if member.isdir(): target.mkdir(parents=True, exist_ok=True); continue
                target.parent.mkdir(parents=True, exist_ok=True)
                source = tar.extractfile(member); h = hashlib.sha256(); count = 0
                existing = target.open('rb') if target.exists() else None
                output = None if existing else target.open('xb')
                try:
                    for b in iter(lambda: source.read(1024*1024), b''):
                        h.update(b); count += len(b)
                        if existing:
                            if existing.read(len(b)) != b: raise ValueError('retained native bytes differ; preserved')
                        else: output.write(b)
                    if count != member.size or (existing and existing.read(1)): raise ValueError('native member size mismatch')
                finally:
                    if existing: existing.close()
                    if output: output.close()
                members.append({'name': member.name, 'bytes': count, 'sha256': h.hexdigest()})
            # tar termination may precede gzip EOF; drain to validate footer.
            while gz.read(1024*1024): pass
        result = dict(schema='mbes-native-acquisition-v1', identity=identity,
                      archive_sha256=digest(archive), members=members,
                      hash_provenance='local TLS/range/gzip/native verification; not publisher checksum',
                      reference_values_read=False, sensor_messages_decoded=False)
        rp = root/'acquired.json'
        if rp.exists():
            if json.loads(rp.read_text()) != result: raise ValueError('retained acquisition report differs')
        else: new_json(rp, result)
        return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--root', type=Path, required=True)
    p.add_argument('--sequence', choices=['beach_pond','north_river_confluence','dutch_harbor','wiggles_bank'], default='beach_pond')
    args = p.parse_args()
    url = 'https://www.seaward.science/files/pos-datasets/bag/'+args.sequence+'.tar.gz'
    print(json.dumps(acquire(args.root,url), indent=2))
