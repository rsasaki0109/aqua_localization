"""Tests for check_mbes_loop_allowlist_replay.py."""

import importlib.util
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "check_mbes_loop_allowlist_replay.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("check_mbes_loop_allowlist_replay", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    import sys
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_csv(path: Path, text: str):
    path.write_text(text.strip() + "\n", encoding="utf-8")


def test_format_report_flags_off_allowlist_accepted_loop(tmp_path):
    module = load_module()
    allowlist = tmp_path / "selected.csv"
    status = tmp_path / "status.csv"
    write_csv(
        allowlist,
        """
timestamp,current_id,candidate_id
1.0,10,2
2.0,20,4
""",
    )
    write_csv(
        status,
        """
timestamp,current_id,candidate_id,accepted,status,fitness_score,correction_translation_m,correction_rotation_rad
1.0,10,2,1,accepted,0.10,1.0,0.10
2.0,21,4,1,accepted,0.20,1.5,0.20
3.0,30,8,0,loop selection rejected,0.30,1.8,0.30
""",
    )

    allow_pairs = module.read_allowlist(allowlist)
    rows = module.read_status_rows(status)
    text = module.format_report(allowlist, status, allow_pairs, rows, max_rows=10)

    assert "- Allowlisted pairs: 2" in text
    assert "- Accepted replay loops: 2" in text
    assert "- Accepted allowlisted loops: 1" in text
    assert "- Accepted outside allowlist: 1" in text
    assert "| 1 | 2.0 | 4 -> 21 | accepted | 0.20 | 1.5 | 0.20 |" in text


def test_strict_main_fails_when_off_allowlist_loop_is_accepted(tmp_path):
    module = load_module()
    allowlist = tmp_path / "selected.csv"
    status = tmp_path / "status.csv"
    out = tmp_path / "audit.md"
    write_csv(
        allowlist,
        """
timestamp,current_id,candidate_id
1.0,10,2
""",
    )
    write_csv(
        status,
        """
timestamp,current_id,candidate_id,accepted,status
1.0,11,2,1,accepted
""",
    )

    rc = module.main([
        "--allowlist", str(allowlist),
        "--status", str(status),
        "--out", str(out),
        "--strict",
    ])

    assert rc == 2
    assert "FAIL: selected replay accepted loop pairs" in out.read_text(encoding="utf-8")


def test_strict_main_passes_when_signature_matches_drifted_ids(tmp_path):
    module = load_module()
    allowlist = tmp_path / "selected.csv"
    status = tmp_path / "status.csv"
    out = tmp_path / "audit.md"
    write_csv(
        allowlist,
        """
timestamp,current_id,candidate_id,fitness_score,correction_translation_m,correction_rotation_rad
10.0,10,2,0.10,1.0,0.10
""",
    )
    write_csv(
        status,
        """
timestamp,current_id,candidate_id,accepted,status,fitness_score,correction_translation_m,correction_rotation_rad
10.2,110,20,1,accepted,0.11,1.2,0.12
""",
    )

    rc = module.main([
        "--allowlist", str(allowlist),
        "--status", str(status),
        "--out", str(out),
        "--strict",
        "--signature-timestamp-window-s", "0.5",
        "--signature-max-fitness-delta", "0.05",
        "--signature-max-translation-delta-m", "0.5",
        "--signature-max-rotation-delta-rad", "0.05",
    ])

    text = out.read_text(encoding="utf-8")
    assert rc == 0
    assert "- Accepted by exact ID: 0" in text
    assert "- Accepted by signature: 1" in text
    assert "- Accepted outside allowlist: 0" in text
    assert "matched the allowlist by exact ID or configured signature" in text


def test_signature_match_uses_candidate_endpoint_timestamp_when_available(tmp_path):
    module = load_module()
    allowlist = tmp_path / "selected.csv"
    status = tmp_path / "status.csv"
    out = tmp_path / "audit.md"
    write_csv(
        allowlist,
        """
timestamp,current_keyframe_timestamp,candidate_keyframe_timestamp,current_id,candidate_id,fitness_score,correction_translation_m,correction_rotation_rad
10.0,10.0,5.0,10,2,0.10,1.0,0.10
""",
    )
    write_csv(
        status,
        """
timestamp,current_keyframe_timestamp,candidate_keyframe_timestamp,current_id,candidate_id,accepted,status,fitness_score,correction_translation_m,correction_rotation_rad
10.1,10.2,99.0,110,20,1,accepted,0.11,1.2,0.12
""",
    )

    rc = module.main([
        "--allowlist", str(allowlist),
        "--status", str(status),
        "--out", str(out),
        "--strict",
        "--signature-timestamp-window-s", "0.5",
        "--signature-max-fitness-delta", "0.05",
        "--signature-max-translation-delta-m", "0.5",
        "--signature-max-rotation-delta-rad", "0.05",
    ])

    text = out.read_text(encoding="utf-8")
    assert rc == 2
    assert "- Endpoint timestamp signatures: 1" in text
    assert "- Accepted outside allowlist: 1" in text
