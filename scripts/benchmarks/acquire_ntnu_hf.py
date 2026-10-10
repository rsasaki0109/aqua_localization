"""Pinned HF sensor object acquisition; bounded TLS ranges and publisher LFS hash."""
import argparse
import concurrent.futures
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import urllib.parse
import urllib.request
import uuid

URL = ('https://huggingface.co/datasets/ntnu-arl/underwater-datasets/resolve/'
       'd79489fc4742ebe66f872d1b195d9f0a249925da/subset-fjord/fjord_1/fjord_1.bag')
SIZE = 12456215809
SHA256 = 'b0f2a43d049efcc139b7629fc813a14460fb3e3090a1fa6cff3af8135a3aaa6d'
HOSTS = {'huggingface.co', 'us.aws.cdn.hf.co'}


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1024*1024), b''): h.update(b)
    return h.hexdigest()


def save(path, value):
    with Path(path).open('x') as f: json.dump(value, f, indent=2); f.write('\n')


class OfficialRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        parsed = urllib.parse.urlsplit(newurl)
        if parsed.scheme != 'https' or parsed.hostname not in HOSTS:
            raise ValueError('unregistered or non-TLS sensor redirect')
        return super().redirect_request(req, fp, code, msg, headers, newurl)


def fetch_range(start, end, identity=None):
    # Fresh public resolve URL renews expired signed redirects, without credentials.
    url = URL + '?download=true&acquisition_nonce=' + uuid.uuid4().hex
    headers = {'Range': f'bytes={start}-{end}', 'Accept-Encoding': 'identity'}
    opener = urllib.request.build_opener(OfficialRedirect())
    with opener.open(urllib.request.Request(url, headers=headers), timeout=45) as r:
        parsed = urllib.parse.urlsplit(r.url)
        if parsed.scheme != 'https' or parsed.hostname not in HOSTS:
            raise ValueError('unexpected final sensor host')
        match = re.fullmatch(r'bytes (\d+)-(\d+)/(\d+)', r.headers.get('Content-Range', ''))
        if r.status != 206 or not match or tuple(map(int, match.groups()[:2])) != (start, end):
            raise ValueError('sensor range response mismatch')
        metadata = {'source': URL, 'bytes': int(match[3]), 'etag': r.headers.get('ETag')}
        if not metadata['etag'] or metadata['etag'].startswith('W/'):
            raise ValueError('strong sensor ETag required')
        if identity is not None and metadata != identity:
            raise ValueError('sensor object identity changed')
        data = r.read(end-start+2)
        if len(data) != end-start+1: raise ValueError('sensor range length mismatch')
        return data, metadata


def acquire(root, *, size=SIZE, expected_sha=SHA256, chunk_bytes=32*1024*1024,
            workers=4, fetch=fetch_range):
    root = Path(root); root.mkdir(parents=True, exist_ok=True)
    with (root/'acquisition.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        _, identity = fetch(0, 0)
        if identity['bytes'] != size: raise ValueError('publisher object size mismatch')
        contract = dict(identity=identity, publisher_sha256=expected_sha, chunk_bytes=chunk_bytes)
        ip = root/'identity.json'; part = root/'fjord_1.bag.partial'; final = root/'fjord_1.bag'
        if ip.exists():
            if json.loads(ip.read_text()) != contract: raise ValueError('retained identity/contract differs')
        else:
            if part.exists() or final.exists(): raise ValueError('unknown retained file; preserve and inspect')
            save(ip, contract)
        if final.exists():
            if final.stat().st_size != size or digest(final) != expected_sha:
                raise ValueError('retained completed sensor object corrupted')
            return json.loads((root/'acquired.json').read_text())
        receipts = root/'ranges'; receipts.mkdir(exist_ok=True)
        fresh = not part.exists()
        fd = os.open(part, os.O_RDWR | (os.O_CREAT | os.O_EXCL if fresh else 0), 0o600)
        try:
            if fresh: os.ftruncate(fd, size)
            if os.fstat(fd).st_size != size: raise ValueError('retained partial extent changed')
            starts = list(range(0, size, chunk_bytes))
            # Validate EVERY committed local range before launching download workers.
            for start in starts:
                end = min(start+chunk_bytes, size)-1; receipt = receipts/f'{start:012d}.json'
                if receipt.exists():
                    d = json.loads(receipt.read_text()); b = os.pread(fd, end-start+1, start)
                    if (d['identity'] != identity or d['start'] != start or d['end'] != end or
                            len(b) != end-start+1 or hashlib.sha256(b).hexdigest() != d['sha256']):
                        raise ValueError('retained verified sensor range corrupted')
            def download(start):
                end = min(start+chunk_bytes, size)-1; receipt = receipts/f'{start:012d}.json'
                if receipt.exists(): return
                for attempt in range(3):
                    try:
                        b, current = fetch(start, end, identity)
                        if current != identity or len(b) != end-start+1:
                            raise ValueError('sensor range identity/length differs')
                        break
                    except Exception:
                        if attempt == 2: raise
                retained = os.pread(fd, len(b), start)
                # Unknown interrupted bytes are compared remotely; never trusted as a prefix.
                if not fresh and retained != b and retained.strip(b'\0'):
                    backup = root/f'interrupted-range-{start:012d}-{uuid.uuid4().hex}.bin'
                    with backup.open('xb') as f: f.write(retained)
                    save(backup.with_suffix('.json'), dict(start=start, end=end,
                         reason='uncommitted interrupted bytes preserved before verified replacement',
                         retained_sha256=hashlib.sha256(retained).hexdigest(),
                         replacement_sha256=hashlib.sha256(b).hexdigest(), identity=identity))
                written = 0
                while written < len(b):
                    n = os.pwrite(fd, b[written:], start+written)
                    if n <= 0: raise OSError('short sensor write')
                    written += n
                os.fsync(fd)
                save(receipt, dict(identity=identity, start=start, end=end,
                     sha256=hashlib.sha256(b).hexdigest()))
                print(f'verified sensor range {start}-{end}', flush=True)
            with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as pool:
                list(pool.map(download, starts))
        finally: os.close(fd)
        actual = digest(part)
        if actual != expected_sha: raise ValueError('full publisher LFS hash mismatch; partial retained')
        result = dict(schema='ntnu-hf-native-object-v1', identity=identity,
             publisher_sha256=expected_sha, verified_sha256=actual, bytes=size,
             complete=True, native_object_verified=True, native_messages_decoded=False,
             reference_values_read=False, evaluation_ready=False)
        rp = root/'acquired.json'
        if rp.exists():
            if json.loads(rp.read_text()) != result: raise ValueError('completed acquisition report differs')
        else: save(rp, result)
        os.link(part, final)  # atomic no-overwrite publication
        part.unlink()
        return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('--root', type=Path, required=True)
    args = parser.parse_args(); print(json.dumps(acquire(args.root), indent=2))
