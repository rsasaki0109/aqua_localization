#!/usr/bin/env python3
"""Drive Lichtblick (the OSS Foxglove Studio fork) headlessly via Playwright
to render a screenshot of an MCAP bag laid out by an exported layout JSON.

Use case: produce README-bound screenshots from `aqua_localization` demo bags
without requiring the contributor to run a browser by hand.

Lichtblick is an Apache-2.0 fork of Foxglove Studio that did not pick up the
account-required gate Foxglove Studio added in late 2024. Its layout JSON
schema is the same as Foxglove's, so the same `docs/foxglove/*.json` layouts
can drive both.

Quick usage:

  pip install --user playwright
  playwright install chromium

  ./aqua_localization/scripts/lichtblick_screenshot.py \\
    --bag aqua_localization/datasets/public/tank_dataset/demo_with_estimate \\
    --layout docs/foxglove/aqua_tank_demo.json \\
    --out docs/media/tank_dataset_lichtblick.png \\
    --seek 8.0
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

try:
    from playwright.sync_api import sync_playwright
except ImportError:
    sys.stderr.write(
        "playwright not installed. Run `pip install --user playwright && "
        "playwright install chromium`.\n"
    )
    raise


LICHTBLICK_URL = "https://lichtblick-suite.github.io/lichtblick/"


def find_mcap(bag_path: Path) -> Path:
    """Resolve a single .mcap file from either a bag dir or a direct file."""
    if bag_path.is_file() and bag_path.suffix == ".mcap":
        return bag_path
    if bag_path.is_dir():
        candidates = sorted(bag_path.glob("*.mcap"))
        if not candidates:
            raise SystemExit(f"no .mcap files in {bag_path}")
        if len(candidates) > 1:
            print(f"multiple .mcap in {bag_path}; using {candidates[0].name}")
        return candidates[0]
    raise SystemExit(f"unsupported bag path: {bag_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bag", type=Path, required=True,
        help="MCAP file or rosbag2 directory to load")
    parser.add_argument(
        "--layout", type=Path, required=True,
        help="Layout JSON exported from Foxglove/Lichtblick")
    parser.add_argument(
        "--out", type=Path, required=True,
        help="Output screenshot PNG")
    parser.add_argument(
        "--seek", type=float, default=0.0,
        help="Seconds into the bag to seek before screenshotting (default: 0)")
    parser.add_argument(
        "--width", type=int, default=1920,
        help="Browser viewport width (default: 1920)")
    parser.add_argument(
        "--height", type=int, default=1080,
        help="Browser viewport height (default: 1080)")
    parser.add_argument(
        "--headless", action="store_true",
        help="Run Chromium in headless mode (default: headed for stability)")
    parser.add_argument(
        "--video-dir", type=Path, default=None,
        help="If set, save a session video (WEBM) into this directory")
    parser.add_argument(
        "--load-timeout-s", type=float, default=90.0,
        help="Seconds to wait for the bag to load")
    return parser.parse_args()


def dismiss_initial_dialog(page) -> None:
    """Close the data-source modal by uploading the bag instead of clicking it."""
    try:
        page.locator('[data-testid="DataSourceDialog"]').wait_for(
            state="visible", timeout=15_000
        )
    except Exception:
        # Dialog may not appear if Lichtblick changed defaults. Ignore.
        pass


def upload_bag(page, mcap: Path) -> None:
    """Drive the file input that backs the 'Open local file(s)...' button."""
    inputs = page.locator("input[type=file]")
    count = inputs.count()
    if count == 0:
        raise SystemExit("no file input found on Lichtblick page")
    inputs.first.set_input_files(str(mcap))


def wait_for_bag_loaded(page, timeout_s: float) -> None:
    """Bag is loaded when the data-source dialog disappears AND we see topic data
    indicators in the toolbar (timestamp text)."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            count = page.locator('[data-testid="DataSourceDialog"]').count()
            if count == 0:
                # Also wait until the playback bar shows something other than
                # "No data source".
                header_text = page.locator("body").inner_text(timeout=2000)
                if "No data source" not in header_text:
                    return
        except Exception:
            pass
        time.sleep(0.5)
    raise SystemExit("bag did not load in time (Lichtblick may be slow over network)")


def import_layout(page, layout_path: Path) -> None:
    """Open the Layouts sidebar and import the layout JSON file."""
    # Click the Layouts tab (it's one of the left-rail tabs).
    layouts_tab = page.locator('button:has-text("Layouts"), a:has-text("Layouts"), [aria-label="Layouts"]').first
    if layouts_tab.count() == 0:
        # Fallback: try the inline tab text from the screenshot.
        layouts_tab = page.locator("text=Layouts").first
    layouts_tab.click()
    page.wait_for_timeout(500)

    # Lichtblick has a "+" / context menu in the Layouts pane that exposes Import.
    # We rely on a hidden input[type=file] that the import action drives.
    # Try clicking the Import button via aria-label first.
    for sel in [
        '[aria-label="Import layout"]',
        'button:has-text("Import")',
        'text="Import from file"',
        'text="Import"',
    ]:
        loc = page.locator(sel)
        if loc.count():
            try:
                loc.first.click(timeout=2000)
                break
            except Exception:
                continue

    # Some Lichtblick versions wire import to the existing file input. Try the
    # most-recent file input (the import handler usually creates a fresh one).
    page.wait_for_timeout(500)
    inputs = page.locator("input[type=file]")
    if inputs.count() == 0:
        # Fall back to the JSON-paste path: read the layout and inject it via
        # the (undocumented) localStorage layout cache key. Skipped here because
        # it depends on the Lichtblick build's storage schema.
        raise SystemExit("could not find the layout import file input")
    inputs.last.set_input_files(str(layout_path))
    page.wait_for_timeout(2000)


def play_to_seek(page, seek_seconds: float) -> None:
    """Press play, wait, pause near the requested timestamp.

    Lichtblick's playback button has no aria-label, so we locate it by its
    SVG path or by keyboard shortcut. The space bar toggles play/pause."""
    if seek_seconds <= 0:
        return
    # Focus the page (so keyboard input lands somewhere sensible).
    page.locator("body").click()
    page.keyboard.press("Space")
    time.sleep(seek_seconds)
    page.keyboard.press("Space")
    page.wait_for_timeout(500)


def main() -> int:
    args = parse_args()
    mcap = find_mcap(args.bag)
    if not args.layout.exists():
        sys.stderr.write(f"layout JSON not found: {args.layout}\n")
        return 1
    args.out.parent.mkdir(parents=True, exist_ok=True)

    record_video_dir = None
    if args.video_dir is not None:
        args.video_dir.mkdir(parents=True, exist_ok=True)
        record_video_dir = str(args.video_dir)

    with sync_playwright() as p:
        browser = p.chromium.launch(headless=args.headless)
        ctx_kwargs = {"viewport": {"width": args.width, "height": args.height}}
        if record_video_dir:
            ctx_kwargs["record_video_dir"] = record_video_dir
            ctx_kwargs["record_video_size"] = {
                "width": args.width, "height": args.height,
            }
        context = browser.new_context(**ctx_kwargs)
        page = context.new_page()
        page.goto(LICHTBLICK_URL)
        page.wait_for_load_state("networkidle", timeout=60_000)

        dismiss_initial_dialog(page)
        upload_bag(page, mcap)
        wait_for_bag_loaded(page, args.load_timeout_s)
        try:
            import_layout(page, args.layout)
        except SystemExit as e:
            print(f"layout import skipped: {e}")
        play_to_seek(page, args.seek)

        # Collapse the left sidebar so the screenshot shows panel content only.
        # The layout selector tab toggles the sidebar when re-clicked.
        try:
            page.locator('button:has-text("Layouts"), text="Layouts"').first.click(timeout=2000)
        except Exception:
            pass

        page.wait_for_timeout(2000)
        page.screenshot(path=str(args.out), full_page=False)
        print(f"wrote {args.out}")

        context.close()
        browser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
