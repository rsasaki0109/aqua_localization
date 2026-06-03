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
