#!/usr/bin/env python3
"""Merge legacy per-module logs into src-level package logs."""

import argparse
import os
import re
import shutil
import tempfile
from collections import defaultdict
from pathlib import Path


MODULE_TO_PACKAGE = {
    "a1_rl_policy": "pct_scan_gazebo",
    "dynamic_perception_3d": "dynamic_perception_3d",
    "global_pct_planner": "global_pct_planner",
    "global_pct_tomography": "global_pct_planner",
    "go2_gait_publisher": "scan_planner",
    "goal_interactive_marker": "pct_scan_navigation",
    "map_publisher": "scan_planner",
    "mpc_controller": "scan_planner",
    "navigation_manager": "pct_scan_navigation",
    "navigation_supervisor": "navigation_supervisor",
    "odom_visualization": "scan_planner",
    "open_loop_controller": "scan_planner",
    "pct_planner": "pct_planner",
    "pct_scan_gazebo_draw_force": "pct_scan_gazebo",
    "pct_scan_gazebo_livox": "pct_scan_gazebo",
    "pct_scan_pointcloud_bridge": "pct_scan_gazebo",
    "pct_tomography_node": "pct_planner",
    "pointcloud_render_node": "scan_planner",
    "pointcloud_tomography": "pct_planner",
    "scan_planner_node": "scan_planner",
    "state_from_gazebo": "pct_scan_gazebo",
}

TIMESTAMP = re.compile(r"^\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{6})\]")


def read_records(path):
    records = []
    current = None
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            match = TIMESTAMP.match(line)
            if match:
                if current is not None:
                    records.append(current)
                current = [match.group(1), line]
            elif current is None:
                current = ["", line]
            else:
                current[1] += line
    if current is not None:
        records.append(current)
    return records


def merge_package(root, package, sources, delete):
    package_dir = root / package
    package_dir.mkdir(parents=True, exist_ok=True)
    records_by_date = defaultdict(list)
    consumed = []

    for source in sources:
        source_dir = root / source
        if not source_dir.is_dir():
            continue
        for path in sorted(source_dir.glob("*.log")):
            records_by_date[path.name].extend(read_records(path))
        if source != package:
            consumed.append(source_dir)

    for filename, records in sorted(records_by_date.items()):
        records.sort(key=lambda item: item[0])
        destination = package_dir / filename
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=package_dir, prefix=".merge-", delete=False
        ) as temporary:
            temporary_path = Path(temporary.name)
            for _, record in records:
                temporary.write(record)
        os.replace(temporary_path, destination)
        print(f"merged {len(records):6d} records -> {destination}")

    if delete:
        for source_dir in consumed:
            shutil.rmtree(source_dir)
            print(f"removed {source_dir}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path, help="workspace log directory")
    parser.add_argument(
        "--delete", action="store_true", help="remove legacy module directories after merging"
    )
    parser.add_argument(
        "--delete-only",
        action="store_true",
        help="remove known legacy module directories without merging again",
    )
    args = parser.parse_args()
    root = args.root.resolve()
    if not root.is_dir():
        parser.error(f"log directory does not exist: {root}")

    known_sources = set(MODULE_TO_PACKAGE)
    known_packages = set(MODULE_TO_PACKAGE.values())
    unknown = sorted(
        path.name
        for path in root.iterdir()
        if path.is_dir() and path.name not in known_sources and path.name not in known_packages
    )
    if unknown:
        print("unrecognized directories (left untouched):")
        for name in unknown:
            print(f"  {name}")

    if args.delete_only:
        for module, package in sorted(MODULE_TO_PACKAGE.items()):
            source_dir = root / module
            if module == package or not source_dir.is_dir():
                continue
            shutil.rmtree(source_dir)
            print(f"removed {source_dir}")
        return

    packages = defaultdict(list)
    for module, package in MODULE_TO_PACKAGE.items():
        packages[package].append(module)
    for package, sources in sorted(packages.items()):
        merge_package(root, package, sources, args.delete)


if __name__ == "__main__":
    main()
