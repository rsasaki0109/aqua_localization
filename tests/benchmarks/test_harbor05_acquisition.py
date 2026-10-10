"""Synthetic fixtures test integrity gates only; they are not accuracy evidence."""
import gzip
import hashlib
import io
import json
from pathlib import Path
import sys
import tarfile
from email.message import Message

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[2] / "scripts/benchmarks"))
import audit_native_imu_support as audit
import ensure_harbor05_data as data
import openvins_supported_bag as supported
import prepare_vio_comparison as registration
import verify_openvins_stream as independent


def tiny_native_bag(path):
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_typestore
    store = get_typestore(Stores.ROS1_NOETIC)
    types = store.types
    def header(ns):
        return types["std_msgs/msg/Header"](seq=0,
            stamp=types["builtin_interfaces/msg/Time"](sec=100, nanosec=ns), frame_id="test")
    def imu(ns):
        return types["sensor_msgs/msg/Imu"](header=header(ns),
            orientation=types["geometry_msgs/msg/Quaternion"](x=0., y=0., z=0., w=1.),
            orientation_covariance=np.zeros(9),
            angular_velocity=types["geometry_msgs/msg/Vector3"](x=0., y=0., z=0.),
            angular_velocity_covariance=np.zeros(9),
            linear_acceleration=types["geometry_msgs/msg/Vector3"](x=0., y=0., z=9.80665),
            linear_acceleration_covariance=np.zeros(9))
    def image(ns):
        return types["sensor_msgs/msg/Image"](header=header(ns), height=1, width=1,
            encoding="mono8", is_bigendian=0, step=1, data=np.array([42], dtype=np.uint8))
    pressure = types["sensor_msgs/msg/FluidPressure"](header=header(1), fluid_pressure=101325., variance=0.)
    samples = [(0, audit.IMU_TOPIC, imu(0)), (1, audit.PRESSURE_TOPIC, pressure),
               (33842804, audit.CAMERA_TOPIC, image(33842804)),
               (50000000, audit.IMU_TOPIC, imu(50000000)),
               (80000000, audit.CAMERA_TOPIC, image(80000000)),
               (100000000, audit.IMU_TOPIC, imu(100000000))]
    with Writer(path) as writer:
        connections = {}
        for ns, topic, msg in samples:
            msgtype = msg.__msgtype__
            if topic not in connections:
                connections[topic] = writer.add_connection(topic, msgtype, typestore=store)
            writer.write(connections[topic], 100000000000 + ns, store.serialize_ros1(msg, msgtype))


@pytest.fixture
def native_archive(tmp_path, monkeypatch):
    bag = tmp_path / "fixture.bag"
    tiny_native_bag(bag)
    archive = tmp_path / "fixture.tar.gz"
    with tarfile.open(archive, "w:gz") as stream:
        stream.add(bag, arcname=data.BAG_MEMBER)
    monkeypatch.setattr(data, "BAG_BYTES", bag.stat().st_size)
    monkeypatch.setattr(data, "BAG_SHA256", audit.sha256(bag))
    monkeypatch.setattr(data, "ARCHIVE_BYTES", archive.stat().st_size)
    monkeypatch.setattr(data, "ARCHIVE_SHA256", audit.sha256(archive))
    monkeypatch.setattr(data, "EXPECTED_ALL_MESSAGES", 6)
    monkeypatch.setattr(data, "EXPECTED_TOPIC_COUNTS", {
        audit.IMU_TOPIC: 3, audit.CAMERA_TOPIC: 2, audit.PRESSURE_TOPIC: 1})
    return bag, archive


def test_native_bag_audit_accounts_for_excluded_camera(tmp_path):
    bag = tmp_path / "test.bag"
    tiny_native_bag(bag)
    result = audit.audit_native_bag(bag)
    assert result["all_ros_message_count"] == 6
    assert result["native_observation_counts"] == {"camera": 2, "pressure": 1}
    assert result["excluded_observation_counts"] == {"camera": 1, "pressure": 0}
    excluded = [r for r in result["records"] if not r["included"]]
    assert excluded[0]["native_ordinal"] == 0
    assert excluded[0]["distance_outside_support_ns"] == 6537851
    assert len(excluded[0]["source_message_sha256"]) == 64
    assert result["estimator_stream_conversion"] == "not_executed"
    assert result["reference_access"] == "not_performed"


def test_supported_stream_keeps_original_and_accounts_for_every_sensor(tmp_path):
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore
    bag = tmp_path / "native.bag"
    tiny_native_bag(bag)
    original = bag.read_bytes()
    output = tmp_path / "v2"
    result = supported.convert_supported_bag(bag, output)
    assert bag.read_bytes() == original
    assert result["supported_topic_counts"] == {
        audit.IMU_TOPIC: 3, audit.CAMERA_TOPIC: 1, audit.PRESSURE_TOPIC: 1}
    assert result["excluded_observation_counts"] == {"camera": 1, "pressure": 0}
    ledger = json.loads((output / "input_audit.json").read_text())
    assert len(ledger["records"]) == 3
    excluded = [r for r in ledger["records"] if not r["included"]]
    assert [(r["sensor"], r["native_ordinal"], r["distance_outside_support_ns"])
            for r in excluded] == [("camera", 0, 6537851)]
    store = get_typestore(Stores.ROS1_NOETIC)
    with Reader(output / "supported.bag") as reader:
        cameras = [(stamp, store.deserialize_ros1(raw, c.msgtype))
                   for c, stamp, raw in reader.messages() if c.topic == audit.CAMERA_TOPIC]
    assert cameras[0][0] == 100000000000 + 80000000 - 40380655
    assert cameras[0][1].data.tolist() == [42]
    checked = independent.verify(bag, output / "supported.bag")
    assert checked["verified_topic_counts"] == result["supported_topic_counts"]
    assert checked["native_imu_and_pressure_raw_bytes_match"] is True
    assert checked["eligible_camera_bytes_match_after_restoring_header_clock"] is True
    assert checked["independently_excluded"][0]["native_ordinal"] == 0
    assert result["reference_access"] == "not_performed"
    retained = (output / "conversion.json").read_bytes()
    with pytest.raises(FileExistsError):
        supported.convert_supported_bag(bag, output)
    assert (output / "conversion.json").read_bytes() == retained


@pytest.mark.parametrize("changed_topic", [audit.IMU_TOPIC, audit.PRESSURE_TOPIC, audit.CAMERA_TOPIC])
def test_independent_verifier_rejects_changed_payload_with_correct_counts(tmp_path, changed_topic):
    from rosbags.rosbag1 import Reader, Writer
    from rosbags.typesys import Stores, get_typestore
    native = tmp_path / "native.bag"
    tiny_native_bag(native)
    supported.convert_supported_bag(native, tmp_path / "input")
    corrupt = tmp_path / "changed.bag"
    store = get_typestore(Stores.ROS1_NOETIC)
    with Reader(tmp_path / "input" / "supported.bag") as reader, Writer(corrupt) as writer:
        connections = {c.id: writer.add_connection(c.topic, c.msgtype, typestore=store)
                       for c in reader.connections}
        for c, stamp, raw in reader.messages():
            if c.topic == changed_topic:
                msg = store.deserialize_ros1(raw, c.msgtype)
                if c.topic == audit.IMU_TOPIC:
                    msg.angular_velocity.z += 1.0
                elif c.topic == audit.PRESSURE_TOPIC:
                    msg.fluid_pressure += 1.0
                else:
                    msg.data = np.array([43], dtype=np.uint8)
                raw = store.serialize_ros1(msg, c.msgtype)
            writer.write(connections[c.id], stamp, raw)
    with pytest.raises(ValueError, match="content/mask differs"):
        independent.verify(native, corrupt)


def test_complete_data_is_idempotent_and_separate_from_stream_readiness(native_archive, tmp_path):
    _, archive = native_archive
    root = tmp_path / "data"
    root.mkdir()
    (root / "harbor_sequence_05_bag.tar.gz").write_bytes(archive.read_bytes())
    first = data.ensure_data(root)
    ready_bytes = (root / "ready.json").read_bytes()
    assert data.ensure_data(root) == first
    assert (root / "ready.json").read_bytes() == ready_bytes
    assert first["status"] == "native_data_verified"
    assert first["estimator_stream_conversion"] == "not_executed"


def test_registration_freezes_new_sources_and_leaves_old_plans(native_archive, tmp_path, monkeypatch):
    bag, archive = native_archive
    root = tmp_path / "data"
    root.mkdir()
    (root / "harbor_sequence_05_bag.tar.gz").write_bytes(archive.read_bytes())
    data.ensure_data(root)
    old = root / "vio_comparison"
    old.mkdir()
    (old / "plan.json").write_text('{"status":"legacy_failure"}')
    monkeypatch.setattr(registration, "BAG_SHA256", audit.sha256(bag))
    monkeypatch.setattr(registration, "EXPECTED_ALL_MESSAGES", 6)
    repo = Path(__file__).parents[2]
    target = registration.register(root, repo)
    plan = json.loads((target / "plan.json").read_text())
    assert plan["comparison_executable"] is False
    assert len(plan["trials"]) == 4
    assert plan["reference_access"] == "not_performed"
    assert plan["input_contract"]["camera_offset_ns"] == -40380655
    assert plan["scoring_plan_sha256"] == audit.sha256(target / "scoring-plan.json")
    for name, digest in plan["source_sha256"].items():
        assert audit.sha256(target / "source" / name) == digest
    before = (target / "plan.json").read_bytes()
    with pytest.raises(FileExistsError):
        registration.register(root, repo)
    assert (target / "plan.json").read_bytes() == before
    assert (old / "plan.json").read_text() == '{"status":"legacy_failure"}'


def test_unexpected_boundary_requires_new_registration(tmp_path):
    bag = tmp_path / "fixture.bag"
    tiny_native_bag(bag)
    contract = {"expected_native_counts": {"imu": 3, "camera": 2, "pressure": 1},
                "camera_offset_ns": -40380655, "pressure_offset_ns": 0,
                "imu_extrapolation": False, "shared_camera_mask": True,
                "expected_exclusions": []}
    with pytest.raises(ValueError, match="boundary differs"):
        supported.convert_supported_bag(bag, tmp_path / "output", contract)
    assert not (tmp_path / "output").exists()


def test_retained_extraction_prefix_is_compared_before_resumption(native_archive, tmp_path):
    bag, archive = native_archive
    native = tmp_path / "resumed.bag"
    partial = native.with_name(native.name + ".partial")
    partial.write_bytes(bag.read_bytes()[:73])
    data.extract_verified_bag(archive, native)
    assert native.read_bytes() == bag.read_bytes()
    assert not partial.exists()


def test_wrong_extraction_prefix_is_retained(native_archive, tmp_path):
    _, archive = native_archive
    native = tmp_path / "bad.bag"
    partial = native.with_name(native.name + ".partial")
    partial.write_bytes(b"wrong")
    with pytest.raises(ValueError, match="differs from archive"):
        data.extract_verified_bag(archive, native)
    assert partial.read_bytes() == b"wrong"
    assert not native.exists()


def test_corrupt_gzip_crc_is_not_accepted(native_archive, tmp_path):
    _, archive = native_archive
    content = bytearray(archive.read_bytes())
    content[-8] ^= 1
    archive.write_bytes(content)
    with pytest.raises(gzip.BadGzipFile):
        data.extract_verified_bag(archive, tmp_path / "corrupt.bag")


@pytest.mark.parametrize("name, kind", [("../escape", tarfile.REGTYPE),
    ("/absolute", tarfile.REGTYPE), ("link", tarfile.SYMTYPE), ("link", tarfile.LNKTYPE)])
def test_unsafe_tar_is_rejected(name, kind):
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w") as stream:
        member = tarfile.TarInfo(name)
        member.type = kind
        stream.addfile(member)
    buffer.seek(0)
    with tarfile.open(fileobj=buffer, mode="r") as stream:
        with pytest.raises(ValueError):
            data.safe_members(stream)


@pytest.mark.parametrize("etag", [None, 'W/"same"', '"different"'])
def test_resume_checks_size_identity_and_strong_etag(etag):
    metadata = {"url": data.URL, "bytes": 100, "sha256": "hash", "etag": '"same"'}
    with pytest.raises(ValueError):
        data.validate_resume(metadata, 100, "hash", data.URL, 50, {"ETag": etag})


def test_exclusive_reports_do_not_replace_existing_failure_or_registration(tmp_path):
    path = tmp_path / "plan.json"
    audit.write_new_json(path, {"status": "old_failure"})
    with pytest.raises(FileExistsError):
        audit.write_new_json(path, {"status": "success"})
    assert json.loads(path.read_text())["status"] == "old_failure"


def test_completed_partial_is_reused_only_after_full_checksum(tmp_path):
    path = tmp_path / "download.bin"
    partial = path.with_name(path.name + ".partial")
    partial.write_bytes(b"complete")
    digest = hashlib.sha256(b"complete").hexdigest()
    data.download_verified(path, 8, digest)
    assert path.read_bytes() == b"complete"
    assert not partial.exists()


def fake_http(monkeypatch, payload, *, offset=0, status=None, changed_etag=False,
              on_range=None, resolved_url=None):
    class Response(io.BytesIO):
        def __init__(self, body, code, pairs):
            super().__init__(body)
            self.status = code
            self.headers = Message()
            for key, value in pairs.items():
                self.headers[key] = str(value)
        def geturl(self):
            return resolved_url or data.URL
    class Opener:
        def open(self, request, timeout):
            if request.get_method() == "HEAD":
                return Response(b"", 200, {"content-length": len(payload), "etag": '"same"'})
            request_headers = {k.lower(): v for k, v in request.header_items()}
            start, end = map(int, request_headers["range"].removeprefix("bytes=").split("-"))
            if on_range is not None:
                on_range(start, request)
            pairs = {"content-length": end - start + 1,
                     "etag": '"changed"' if changed_etag else '"same"'}
            assert request_headers["if-range"] == '"same"'
            pairs["content-range"] = f"bytes {start}-{end}/{len(payload)}"
            return Response(payload[start:end + 1], status or 206, pairs)
    monkeypatch.setattr(data.urllib.request, "build_opener", lambda *args: Opener())


def test_http_header_case_does_not_drop_etag(tmp_path, monkeypatch):
    payload = b"complete"
    fake_http(monkeypatch, payload)
    path = tmp_path / "case.bin"
    data.download_verified(path, len(payload), hashlib.sha256(payload).hexdigest())
    assert path.read_bytes() == payload
    metadata = json.loads(path.with_name(path.name + ".download.json").read_text())
    assert metadata["etag"] == '"same"'


@pytest.mark.parametrize("status, changed_etag", [(206, False), (200, False), (206, True)])
def test_resume_range_and_etag_are_checked_before_appending(tmp_path, monkeypatch, status, changed_etag):
    payload = b"complete"
    path = tmp_path / "range.bin"
    partial = path.with_name(path.name + ".partial")
    partial.write_bytes(payload[:3])
    digest = hashlib.sha256(payload).hexdigest()
    audit.write_new_json(path.with_name(path.name + ".download.json"),
        {"url": data.URL, "bytes": len(payload), "sha256": digest, "etag": '"same"'})
    fake_http(monkeypatch, payload, offset=3, status=status, changed_etag=changed_etag)
    if status == 206 and not changed_etag:
        data.download_verified(path, len(payload), digest)
        assert path.read_bytes() == payload
    else:
        with pytest.raises(ValueError):
            data.download_verified(path, len(payload), digest)
        assert partial.read_bytes() == payload[:3]
        assert not path.exists()


def test_duplicate_acquisition_is_rejected_before_network(tmp_path):
    import fcntl
    root = tmp_path / "locked"
    root.mkdir()
    with (root / "acquisition.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(BlockingIOError):
            data.ensure_data(root)


def test_seafile_opaque_nonweak_etag_is_preserved():
    etag = "99e827e9790a80567ea3a6fa9265e5a32ca216ea"
    metadata = {"url": data.URL, "bytes": 100, "sha256": "hash", "etag": etag}
    data.validate_resume(metadata, 100, "hash", data.URL, 50, {"ETag": etag})


def test_corrupt_retained_download_prefix_is_not_reused(tmp_path, monkeypatch):
    payload = b"complete"
    path = tmp_path / "corrupt-prefix.bin"
    partial = path.with_name(path.name + ".partial")
    partial.write_bytes(b"bad")
    digest = hashlib.sha256(payload).hexdigest()
    audit.write_new_json(path.with_name(path.name + ".download.json"),
        {"url": data.URL, "bytes": len(payload), "sha256": digest, "etag": '"same"'})
    fake_http(monkeypatch, payload, offset=3)
    with pytest.raises(ValueError, match="retained prefix checksum"):
        data.download_verified(path, len(payload), digest)
    assert partial.read_bytes() == b"bad"
    assert not path.exists()


def test_empty_identified_partial_can_restart_without_overwriting_metadata(tmp_path, monkeypatch):
    payload = b"complete"
    path = tmp_path / "empty-prefix.bin"
    path.with_name(path.name + ".partial").write_bytes(b"")
    metadata_path = path.with_name(path.name + ".download.json")
    digest = hashlib.sha256(payload).hexdigest()
    audit.write_new_json(metadata_path,
        {"url": data.URL, "bytes": len(payload), "sha256": digest, "etag": '"same"'})
    original = metadata_path.read_bytes()
    fake_http(monkeypatch, payload)
    data.download_verified(path, len(payload), digest)
    assert path.read_bytes() == payload
    assert metadata_path.read_bytes() == original


def test_slow_first_range_does_not_idle_the_other_workers(tmp_path, monkeypatch):
    import threading
    third_started = threading.Event()
    calls = []
    payload = b"a meaningful ordered binary fixture"
    monkeypatch.setattr(data, "CHUNK_BYTES", 4)
    monkeypatch.setattr(data, "MAX_REQUESTS", 2)
    monkeypatch.setattr(data, "MAX_BUFFERED_CHUNKS", 4)
    resolved = "https://seafile.lirmm.fr/seafhttp/files/test-public-object/file"
    def on_range(start, request):
        assert request.full_url == resolved
        calls.append(start)
        if start == 0:
            assert third_started.wait(3), "head-of-line request prevented worker refill"
        if start == 8:
            third_started.set()
    fake_http(monkeypatch, payload, resolved_url=resolved, on_range=on_range)
    path = tmp_path / "ordered.bin"
    data.download_verified(path, len(payload), hashlib.sha256(payload).hexdigest())
    assert path.read_bytes() == payload
    assert sorted(calls) == list(range(0, len(payload), 4))
    records = [json.loads(line) for log in tmp_path.glob("*.jsonl") for line in log.read_text().splitlines()]
    assert [r["start"] for r in records] == list(range(0, len(payload), 4))


@pytest.mark.parametrize("resolved", ["http://seafile.lirmm.fr/file", "https://other.example/file"])
def test_resolved_download_cannot_change_registered_https_host(tmp_path, monkeypatch, resolved):
    payload = b"complete"
    fake_http(monkeypatch, payload, resolved_url=resolved)
    with pytest.raises(ValueError, match="registered HTTPS host"):
        data.download_verified(tmp_path / "bad-host.bin", len(payload), hashlib.sha256(payload).hexdigest())
    assert not (tmp_path / "bad-host.bin.partial").exists()


def test_known_range_ledger_is_checked_and_reused_without_redownloading_prefix(tmp_path, monkeypatch):
    payload = b"complete"
    path = tmp_path / "cached.bin"
    path.with_name(path.name + ".partial").write_bytes(payload[:3])
    digest = hashlib.sha256(payload).hexdigest()
    audit.write_new_json(path.with_name(path.name + ".download.json"),
        {"url": data.URL, "bytes": len(payload), "sha256": digest, "etag": '"same"'})
    evidence = {"start": 0, "end": 2, "total": 8, "etag": '"same"',
                "sha256": hashlib.sha256(payload[:3]).hexdigest(), "purpose": "download"}
    (tmp_path / (path.name + ".ranges-old.jsonl")).write_text(json.dumps(evidence) + "\n")
    requests = []
    fake_http(monkeypatch, payload, on_range=lambda start, _: requests.append(start))
    data.download_verified(path, len(payload), digest)
    assert requests == [3]
    assert path.read_bytes() == payload


def test_corrupt_cached_range_is_retained_and_not_reused(tmp_path):
    partial = tmp_path / "bad.partial"
    partial.write_bytes(b"bad")
    log = tmp_path / "old.jsonl"
    log.write_text(json.dumps({"start": 0, "end": 2, "total": 8, "etag": '"same"',
        "sha256": hashlib.sha256(b"com").hexdigest(), "purpose": "download"}) + "\n")
    with pytest.raises(ValueError, match="prior verified range ledger"):
        data.verified_prefix_records(partial, 3, 8, '"same"', [log])
    assert partial.read_bytes() == b"bad"


@pytest.mark.parametrize("change_identity", [False, True])
def test_expired_public_object_can_only_be_renewed_with_unchanged_identity(tmp_path, monkeypatch, change_identity):
    class Response(io.BytesIO):
        def __init__(self, body, status, headers, target):
            super().__init__(body)
            self.status, self.headers, self.target = status, Message(), target
            for key, value in headers.items(): self.headers[key] = str(value)
        def geturl(self): return self.target
    class Opener:
        heads = 0
        def open(self, request, timeout):
            if request.get_method() == "HEAD":
                self.heads += 1
                target = "https://seafile.lirmm.fr/" + ("expired" if self.heads == 1 else "renewed")
                etag = '"changed"' if self.heads > 1 and change_identity else '"same"'
                return Response(b"", 200, {"Content-Length": 8, "ETag": etag}, target)
            if request.full_url.endswith("expired"):
                raise data.urllib.error.HTTPError(request.full_url, 403, "Expired", {}, None)
            return Response(b"complete", 206, {"Content-Length": 8, "ETag": '"same"',
                "Content-Range": "bytes 0-7/8"}, request.full_url)
    opener = Opener()
    monkeypatch.setattr(data.urllib.request, "build_opener", lambda *args: opener)
    path = tmp_path / "renewed.bin"
    if change_identity:
        with pytest.raises(ValueError, match="identity changed"):
            data.download_verified(path, 8, hashlib.sha256(b"complete").hexdigest())
        assert not path.exists()
    else:
        data.download_verified(path, 8, hashlib.sha256(b"complete").hexdigest())
        assert path.read_bytes() == b"complete"
    assert opener.heads == 2
