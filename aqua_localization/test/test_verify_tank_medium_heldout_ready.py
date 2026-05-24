"""Tests for verify_tank_medium_heldout_ready.py."""

import importlib.util
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "verify_tank_medium_heldout_ready.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("verify_tank_medium_heldout_ready", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_tum(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join([
            "0.000000000 0.000000000 0.000000000 0.000000000 0.0 0.0 0.0 1.0",
            "1.000000000 1.000000000 0.000000000 0.000000000 0.0 0.0 0.0 1.0",
            "2.000000000 2.000000000 0.000000000 0.000000000 0.0 0.0 0.0 1.0",
        ])
        + "\n",
        encoding="utf-8",
    )


def write_ros1_odom_csv(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join([
            "%time,field.pose.pose.position.x,field.pose.pose.position.y,field.pose.pose.position.z,field.pose.pose.orientation.x,field.pose.pose.orientation.y,field.pose.pose.orientation.z,field.pose.pose.orientation.w",
            "0,0.0,0.0,0.0,0.0,0.0,0.0,1.0",
            "1.0,1.0,0.0,0.0,0.0,0.0,0.0,1.0",
            "2.0,2.0,0.0,0.0,0.0,0.0,0.0,1.0",
        ])
        + "\n",
        encoding="utf-8",
    )


def write_benchmark_row(path: Path, samples=20, matched_s=19.0):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join([
            "| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |",
            "|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|",
            (
                "| Tank Dataset | Medium | AQUA-SLAM | SE(3) | "
                f"{samples} | {matched_s:.2f} | 0.0100 | 0.0100 | 0.0200 | 0.0300 | baseline |"
            ),
        ])
        + "\n",
        encoding="utf-8",
    )


def ready_args(module, tmp_path, extra=None):
    extra = extra or []
    profile = tmp_path / "best_profile.yaml"
    bag = tmp_path / "medium_ros2"
    reference = tmp_path / "Medium_gt.tum"
    visual = tmp_path / "Medium_visual_frontend.tum"
    csv = tmp_path / "aqua_slam_medium_orb_odom.csv"
    row = tmp_path / "Medium_aqua_slam_benchmark_row.md"
    write_tum(reference)
    write_tum(visual)
    write_ros1_odom_csv(csv)
    write_benchmark_row(row)
    profile.write_text("name: rank1\n", encoding="utf-8")
    bag.mkdir()
    return module.parse_args([
        "--reference",
        str(reference),
        "--csv",
        str(csv),
        "--baseline-row",
        str(row),
        "--benchmark-markdown",
        str(tmp_path / "missing_docs.md"),
        "--profile",
        str(profile),
        "--bag",
        str(bag),
        "--visual",
        str(visual),
        "--locator-root",
        str(tmp_path),
        "--out-dir",
        str(tmp_path / "verify"),
        *extra,
    ])


def test_missing_inputs_reports_locator_next_action(tmp_path):
    module = load_module()

    verify = module.build_verify_report(module.parse_args([
        "--locator-root",
        str(tmp_path),
        "--out-dir",
        str(tmp_path / "verify"),
    ]))
    text = module.format_report(verify, verify.readiness_report.args)

    assert verify.ready is False
    assert verify.next_action.title == "Find Medium reference TUM"
    assert module.locator.OFFICIAL_DOWNLOAD_URL in text
    assert "locate_tank_heldout_inputs.py" in text


def test_locator_candidate_yields_reference_link_command(tmp_path):
    module = load_module()
    candidate = tmp_path / "data/HalfTankMedium_gt.tum"
    write_tum(candidate)
    target = tmp_path / "defaults/tank_medium_gt.tum"

    args = module.parse_args([
        "--reference",
        str(target),
        "--locator-root",
        str(tmp_path / "data"),
        "--out-dir",
        str(tmp_path / "verify"),
    ])
    verify = module.build_verify_report(args)

    assert verify.next_action.title == "Link Medium reference TUM"
    assert verify.next_action.command == ("ln", "-sfn", str(candidate.resolve()), str(target))


def test_apply_located_links_safely_links_reference_candidate(tmp_path):
    module = load_module()
    candidate = tmp_path / "data/HalfTankMedium_gt.tum"
    write_tum(candidate)
    target = tmp_path / "defaults/tank_medium_gt.tum"

    args = module.parse_args([
        "--reference",
        str(target),
        "--locator-root",
        str(tmp_path / "data"),
        "--out-dir",
        str(tmp_path / "verify"),
        "--apply-located-links",
    ])
    verify = module.apply_located_links(args)
    text = module.format_report(verify, args)

    assert target.is_symlink()
    assert target.resolve() == candidate.resolve()
    assert verify.applied_links == ((candidate.resolve(), target),)
    assert verify.next_action.title == "Find Medium ROS 2 bag"
    assert "## Applied Links" in text


def test_ros1_bag_candidate_yields_convert_command(tmp_path):
    module = load_module()
    reference = tmp_path / "Medium_gt.tum"
    ros1_bag = tmp_path / "scan/HalfTankMedium.bag"
    target_bag = tmp_path / "defaults/tank_medium_ros2_visual"
    write_tum(reference)
    ros1_bag.parent.mkdir(parents=True)
    ros1_bag.write_bytes(b"bag")

    args = module.parse_args([
        "--reference",
        str(reference),
        "--bag",
        str(target_bag),
        "--locator-root",
        str(tmp_path / "scan"),
        "--out-dir",
        str(tmp_path / "verify"),
    ])
    verify = module.build_verify_report(args)

    assert verify.next_action.title == "Convert Medium ROS 1 bag to ROS 2"
    assert verify.next_action.command == (
        "ros2",
        "run",
        "aqua_localization",
        "convert_tank_dataset_bag.py",
        "--src",
        str(ros1_bag.resolve()),
        "--dst",
        str(target_bag),
        "--include-cameras",
    )


def test_source_candidate_yields_csv_link_command(tmp_path):
    module = load_module()
    args = ready_args(module, tmp_path)
    args.csv.unlink()
    candidate = tmp_path / "scan/Medium_aqua_slam_orb_odom.csv"
    write_ros1_odom_csv(candidate)
    args.locator_root = [tmp_path / "scan"]

    verify = module.build_verify_report(args)

    assert verify.next_action.title == "Link Medium AQUA-SLAM CSV"
    assert verify.next_action.command == ("ln", "-sfn", str(candidate.resolve()), str(args.csv))


def test_baseline_candidate_must_pass_gates_before_link(tmp_path):
    module = load_module()
    args = ready_args(module, tmp_path)
    args.baseline_row.unlink()
    bad = tmp_path / "scan/bad/Medium_aqua_slam_benchmark_row.md"
    good = tmp_path / "scan/good/Medium_aqua_slam_benchmark_row.md"
    write_benchmark_row(bad, samples=3, matched_s=2.0)
    write_benchmark_row(good, samples=20, matched_s=19.0)
    args.locator_root = [tmp_path / "scan"]

    verify = module.build_verify_report(args)

    assert verify.next_action.title == "Link Medium AQUA-SLAM baseline row"
    assert verify.next_action.command == ("ln", "-sfn", str(good.resolve()), str(args.baseline_row))


def test_visual_candidate_yields_link_command(tmp_path):
    module = load_module()
    args = ready_args(module, tmp_path)
    args.visual.unlink()
    candidate = tmp_path / "scan/Medium_visual_frontend.tum"
    write_tum(candidate)
    args.locator_root = [tmp_path / "scan"]

    verify = module.build_verify_report(args)

    assert verify.next_action.title == "Link Medium visual TUM"
    assert verify.next_action.command == ("ln", "-sfn", str(candidate.resolve()), str(args.visual))


def test_smoke_baseline_row_requests_reingest(tmp_path):
    module = load_module()
    args = ready_args(module, tmp_path)
    write_benchmark_row(args.baseline_row, samples=3, matched_s=2.0)

    verify = module.build_verify_report(args)
    text = module.format_report(verify, args)

    assert verify.ready is False
    assert verify.next_action.title == "Ingest AQUA-SLAM baseline row"
    assert "rejected=1" in text
    assert "ingest_tank_aqua_slam_baseline.py" in text


def test_ready_report_prints_validation_bundle_command(tmp_path):
    module = load_module()
    args = ready_args(module, tmp_path)

    verify = module.build_verify_report(args)
    text = module.format_report(verify, args)

    assert verify.ready is True
    assert verify.next_action.title == "Run Medium held-out validation bundle"
    assert "Status: `PASS`" in text
    assert "run_tank_dvl_validation_bundle.py" in text
    assert "--min-target-samples 10" in text
    assert "--min-target-matched-s 10.0" in text


def test_main_writes_report(tmp_path):
    module = load_module()
    out = tmp_path / "verify.md"

    rc = module.main([
        "--locator-root",
        str(tmp_path),
        "--out",
        str(out),
    ])

    assert rc == 1
    assert "# Tank Medium Held-Out Verifier" in out.read_text(encoding="utf-8")
