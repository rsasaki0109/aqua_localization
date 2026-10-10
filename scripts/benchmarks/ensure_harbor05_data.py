"""Foreground acquisition and complete verification, separate from conversion.

Expected hashes are the user's previously verified observations, not checksums
published by LIRMM. No reference trajectory is downloaded or read here.
"""
import argparse
import gzip
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import ssl
import tarfile
import urllib.parse
import urllib.request
import urllib.error
import threading
from concurrent.futures import ThreadPoolExecutor, wait, FIRST_COMPLETED
import hashlib
import uuid

from audit_native_imu_support import (CAMERA_TOPIC, IMU_TOPIC, PRESSURE_TOPIC,
                                      scan_native_bag, sha256, write_new_json)

URL = "https://seafile.lirmm.fr/d/79b03788f29148ca84e5/files/?p=%2FHarbor_sites_sequences%2Fharbor_sequence_05_bag.tar.gz&dl=1"
ARCHIVE_BYTES = 793867402
ARCHIVE_SHA256 = "b62c4075cfe7f53757c491a73cf2f6b5fb061a28b5fd01f2c1bb7096c252cc47"
BAG_MEMBER = "bag_files/harbor_sequence_5.bag"
BAG_BYTES = 1155720393
BAG_SHA256 = "109c97c1d1b655226b9ca1da312c154b28b3cc8464d58150b7cd4d91e6ba3ecb"
EXPECTED_TOPIC_COUNTS = {IMU_TOPIC: 34576, CAMERA_TOPIC: 3458, PRESSURE_TOPIC: 1650}
EXPECTED_ALL_MESSAGES = 86126
CHUNK_BYTES = 256 * 1024
MAX_REQUESTS = 32
MAX_BUFFERED_CHUNKS = 256  # 64 MiB; prevents one slow range idling all workers.


class HTTPSOnlyRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        if urllib.parse.urlsplit(newurl).scheme != "https":
            raise ValueError("refusing a non-HTTPS download redirect")
        return super().redirect_request(req, fp, code, msg, headers, newurl)


def verify_file(path, size, digest):
    if Path(path).stat().st_size != size or sha256(path) != digest:
        raise ValueError(f"size or SHA-256 mismatch: {path}; file retained")


def validate_resume(metadata, size, digest, url, partial_bytes, headers):
    if metadata != {"url": url, "bytes": size, "sha256": digest,
                    "etag": headers.get("ETag")}:
        raise ValueError("resume identity/size/ETag changed; partial file retained")
    etag = metadata["etag"]
    # Seafile returns a non-weak but unquoted opaque ETag. Preserve it exactly.
    if not etag or etag.startswith("W/") or "\r" in etag or "\n" in etag:
        raise ValueError("partial resumption requires a non-weak ETag; partial file retained")
    if not 0 < partial_bytes < size:
        raise ValueError("invalid partial range")


def verified_prefix_records(partial, offset, size, etag, logs):
    """Check retained bytes against complete prior validated range evidence.

    Unknown tails still require a new remote comparison. The registered full
    archive hash remains mandatory, regardless of any local range evidence.
    """
    best, best_ledger = [], None
    for log in logs:
        records, covered = [], 0
        with Path(log).open() as manifest, Path(partial).open("rb") as stream:
            for line in manifest:
                try:
                    r = json.loads(line)
                except json.JSONDecodeError:
                    break  # A crash may leave the final record incomplete.
                if (type(r.get("start")) is not int or type(r.get("end")) is not int or
                    r["start"] != covered or not covered <= r["end"] < offset or
                    r.get("total") != size or r.get("etag") != etag or
                    r.get("purpose") not in {"download", "retained_prefix_verification", "retained_prefix_from_verified_range_ledger"}):
                    break
                block = stream.read(r["end"] - covered + 1)
                if hashlib.sha256(block).hexdigest() != r.get("sha256"):
                    raise ValueError("retained prefix checksum differs from prior verified range ledger; retained")
                records.append(r)
                covered = r["end"] + 1
        if records and (not best or covered > best[-1]["end"] + 1):
            best, best_ledger = records, sha256(log)
    return best, best_ledger


def download_verified(path, size, digest, url=URL):
    path = Path(path)
    if path.exists():
        verify_file(path, size, digest)
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_name(path.name + ".partial")
    metadata_path = path.with_name(path.name + ".download.json")
    # A completed retained partial needs no network request after full verification.
    if partial.exists() and partial.stat().st_size == size:
        verify_file(partial, size, digest)
        os.link(partial, path)
        partial.unlink()
        return
    if urllib.parse.urlsplit(url).scheme != "https":
        raise ValueError("download URL must use HTTPS")
    opener = urllib.request.build_opener(HTTPSOnlyRedirect(),
        urllib.request.HTTPSHandler(context=ssl.create_default_context()))
    base_headers = {"Accept-Encoding": "identity"}
    def read_head():
        headers = dict(base_headers, **{"Cache-Control": "no-cache"})
        with opener.open(urllib.request.Request(url, headers=headers, method="HEAD"), timeout=60) as response:
            if response.status != 200 or int(response.headers.get("Content-Length", -1)) != size:
                raise ValueError("remote archive size/status does not match registered input")
            target = getattr(response, "geturl", lambda: url)()
            if (urllib.parse.urlsplit(target).scheme != "https" or
                urllib.parse.urlsplit(target).hostname != urllib.parse.urlsplit(url).hostname):
                raise ValueError("resolved download must remain on the registered HTTPS host")
            return {"ETag": response.headers.get("ETag")}, target
    remote, download_url = read_head()
    link = {"url": download_url}
    refresh_lock = threading.Lock()
    metadata = {"url": url, "bytes": size, "sha256": digest, "etag": remote.get("ETag")}
    offset = partial.stat().st_size if partial.exists() else 0
    if offset:
        if not metadata_path.exists():
            raise ValueError("unidentified partial file retained; cannot resume")
        validate_resume(json.loads(metadata_path.read_text()), size, digest, url, offset, remote)
    else:
        if partial.exists() and not metadata_path.exists():
            raise ValueError("unidentified partial file retained; cannot resume")
        if metadata_path.exists():
            if json.loads(metadata_path.read_text()) != metadata:
                raise ValueError("download identity changed; prior metadata retained")
        else:
            write_new_json(metadata_path, metadata)
    def fetch_range(start, end):
        headers = dict(base_headers, Range=f"bytes={start}-{end}")
        if metadata["etag"]:
            headers["If-Range"] = metadata["etag"]
        # Reuse the HEAD-resolved public download object. Re-generating a Seafile
        # public-link redirect for every range is extremely slow under concurrency.
        # The token is ephemeral and never logged; original URL/ETag stay registered.
        for attempt in range(2):
            target = link["url"]
            try:
                with opener.open(urllib.request.Request(target, headers=headers), timeout=60) as response:
                    if response.status != 206:
                        raise ValueError("server did not honor the registered range")
                    if response.headers.get("Content-Range") != f"bytes {start}-{end}/{size}":
                        raise ValueError("Content-Range mismatch; partial file retained")
                    if int(response.headers.get("Content-Length", -1)) != end - start + 1:
                        raise ValueError("download response length mismatch")
                    if response.headers.get("ETag") != metadata["etag"]:
                        raise ValueError("GET ETag differs from verified HEAD")
                    payload = response.read(end - start + 2)
                    if len(payload) != end - start + 1:
                        raise ValueError("range payload length mismatch")
                    return payload
            except urllib.error.HTTPError as error:
                error.close()
                if error.code != 403 or attempt:
                    raise
                with refresh_lock:
                    if link["url"] == target:
                        renewed, new_target = read_head()
                        if renewed != remote:
                            raise ValueError("object identity changed during public download renewal")
                        if new_target == target:
                            raise ValueError("403 not resolved by fresh public HEAD; retained; no repeated retries")
                        link["url"] = new_target
                        print(json.dumps({"event": "public_download_url_renewed", "etag": metadata["etag"]}), flush=True)

    audit_path = path.with_name(path.name + f".ranges-{uuid.uuid4().hex}.jsonl")
    def ranges(begin, limit):
        # Refill completed requests independently of ordered output. A slow first
        # response must not leave 31 workers idle. Buffer at most 64 MiB.
        with ThreadPoolExecutor(max_workers=MAX_REQUESTS) as pool:
            starts = iter(range(begin, limit, CHUNK_BYTES))
            pending, ready = {}, {}
            following = begin
            exhausted = False
            while following < limit:
                while not exhausted and len(pending) < MAX_REQUESTS and len(pending) + len(ready) < MAX_BUFFERED_CHUNKS:
                    start = next(starts, None)
                    if start is None:
                        exhausted = True
                        break
                    end = min(start + CHUNK_BYTES, limit) - 1
                    pending[pool.submit(fetch_range, start, end)] = (start, end)
                if following in ready:
                    end, payload = ready.pop(following)
                    start, following = following, end + 1
                    yield start, end, payload
                else:
                    done, _ = wait(pending, return_when=FIRST_COMPLETED)
                    for future in done:
                        start, end = pending.pop(future)
                        ready[start] = (end, future.result())

    with audit_path.open("x") as range_log:
        def publish_record(entry):
            range_log.write(json.dumps(entry) + "\n")
            range_log.flush()
            os.fsync(range_log.fileno())
        def record(start, end, payload, purpose):
            publish_record({"start": start, "end": end, "total": size,
                "etag": metadata["etag"], "sha256": hashlib.sha256(payload).hexdigest(),
                "purpose": purpose})
        if offset:
            previous = [p for p in path.parent.glob(path.name + ".ranges-*.jsonl") if p != audit_path]
            evidence, ledger_digest = verified_prefix_records(partial, offset, size, metadata["etag"], previous)
            covered = evidence[-1]["end"] + 1 if evidence else 0
            for entry in evidence:
                publish_record(dict(entry, purpose="retained_prefix_from_verified_range_ledger",
                                    previous_ledger_sha256=ledger_digest))
            print(json.dumps({"event": "retained_prefix_checked", "verified_bytes": covered,
                              "partial_bytes": offset, "prior_ledger_sha256": ledger_digest}), flush=True)
            with partial.open("rb") as retained:
                retained.seek(covered)
                for start, end, payload in ranges(covered, offset):
                    if hashlib.sha256(retained.read(len(payload))).digest() != hashlib.sha256(payload).digest():
                        raise ValueError("retained prefix checksum differs from verified remote range")
                    record(start, end, payload, "retained_prefix_verification")
        # One acquisition process/lock, at most MAX_REQUESTS in flight.
        with partial.open("ab" if partial.exists() else "xb") as stream:
            for start, end, payload in ranges(offset, size):
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
                record(start, end, payload, "download")
                print(json.dumps({"event": "download", "bytes": end + 1, "total": size}), flush=True)
            stream.flush()
            os.fsync(stream.fileno())
    verify_file(partial, size, digest)
    os.link(partial, path)
    partial.unlink()


def safe_members(archive):
    result, names = [], set()
    for member in archive:
        path = PurePosixPath(member.name)
        if path.is_absolute() or ".." in path.parts or not path.parts:
            raise ValueError("unsafe TAR member path")
        if not (member.isfile() or member.isdir()):
            raise ValueError("TAR links/devices/other special members are forbidden")
        normalized = str(path)
        if normalized in names:
            raise ValueError("duplicate TAR member path")
        names.add(normalized)
        result.append(member)
    return result


def extract_verified_bag(archive_path, native_path):
    # Read all gzip output through EOF to enforce footer CRC and ISIZE verification.
    uncompressed = 0
    with gzip.open(archive_path, "rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            uncompressed += len(block)
            if uncompressed > BAG_BYTES * 2 + 64 * 1024 * 1024:
                raise ValueError("unexpected archive expansion size")
    native_path = Path(native_path)
    with tarfile.open(archive_path, "r:gz") as archive:
        members = safe_members(archive)
        candidates = [m for m in members if str(PurePosixPath(m.name)) == BAG_MEMBER]
        if len(candidates) != 1 or not candidates[0].isfile() or candidates[0].size != BAG_BYTES:
            raise ValueError("native bag member does not match registered input")
        native_path.parent.mkdir(parents=True, exist_ok=True)
        partial = native_path.with_name(native_path.name + ".partial")
        existing = native_path if native_path.exists() else partial
        # Compare every retained prefix byte to this checksum-verified archive.
        with archive.extractfile(candidates[0]) as source:
            offset = 0
            if existing.exists():
                with existing.open("rb") as old:
                    for block in iter(lambda: old.read(4 * 1024 * 1024), b""):
                        if source.read(len(block)) != block:
                            raise ValueError("retained native bag differs from archive; retained")
                        offset += len(block)
                if offset > BAG_BYTES or (existing == native_path and offset != BAG_BYTES):
                    raise ValueError("retained native bag has invalid size")
            if existing != native_path:
                with partial.open("ab" if partial.exists() else "xb") as output:
                    shutil.copyfileobj(source, output, length=4 * 1024 * 1024)
                    output.flush()
                    os.fsync(output.fileno())
                verify_file(partial, BAG_BYTES, BAG_SHA256)
                os.link(partial, native_path)
                partial.unlink()
    verify_file(native_path, BAG_BYTES, BAG_SHA256)
    return {"gzip_footer_verified": True, "safe_tar_members": len(members),
            "native_bag_matches_archive_bytes": True}


def ensure_data(root, wait_for_lock=False):
    root = Path(root)
    # Linux advisory lock prevents duplicate download/extraction/verification.
    import fcntl
    root.mkdir(parents=True, exist_ok=True)
    with (root / "acquisition.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | (0 if wait_for_lock else fcntl.LOCK_NB))
        archive = root / "harbor_sequence_05_bag.tar.gz"
        native = root / "native" / BAG_MEMBER
        download_verified(archive, ARCHIVE_BYTES, ARCHIVE_SHA256)
        report = extract_verified_bag(archive, native)
        counts, _, _ = scan_native_bag(native)
        if sum(counts.values()) != EXPECTED_ALL_MESSAGES or any(counts.get(t) != n for t, n in EXPECTED_TOPIC_COUNTS.items()):
            raise ValueError("full ROS message scan does not match expected native counts")
        report.update(schema="harbor05-native-data-ready-v1", status="native_data_verified",
                      archive_bytes=ARCHIVE_BYTES, archive_sha256=ARCHIVE_SHA256,
                      native_bag_bytes=BAG_BYTES, native_bag_sha256=BAG_SHA256,
                      hash_provenance="user-reported previously verified observations; not publisher checksums",
                      all_ros_message_count=sum(counts.values()), topic_counts=counts,
                      estimator_stream_conversion="not_executed", reference_access="not_performed")
        ready = root / "ready.json"
        if ready.exists():
            if json.loads(ready.read_text()) != report:
                raise ValueError("existing ready report differs; retained")
        else:
            write_new_json(ready, report)
        return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--wait-for-lock", action="store_true",
                        help="wait in foreground for an existing acquisition; never start concurrent work")
    args = parser.parse_args()
    print(json.dumps(ensure_data(args.root, args.wait_for_lock), sort_keys=True))
