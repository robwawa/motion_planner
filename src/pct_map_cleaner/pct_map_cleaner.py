#!/usr/bin/env python3
"""Launch the independent C++ PCT map cleaner.

The Python entry point intentionally contains no point-cloud processing.  It
keeps the historical command line while selecting the compiled backend and
providing the bundled YAML configuration by default.
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
from pathlib import Path
from typing import Iterable, Optional


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
DEFAULT_CONFIG = SCRIPT_DIRECTORY / "pct_map_cleaner.yaml"


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Launch the independent C++ PCT point-cloud cleaner."
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG),
        help=f"YAML configuration file (default: {DEFAULT_CONFIG})",
    )
    parser.add_argument("--input", dest="input_pcd", help="Override input_pcd from YAML.")
    parser.add_argument(
        "--output", dest="output_pcd", help="Override output_pcd from YAML."
    )
    return parser


def _find_backend() -> Path:
    configured = os.environ.get("PCT_MAP_CLEANER_BIN")
    candidates = []
    if configured:
        candidates.append(Path(configured).expanduser())
    candidates.append(SCRIPT_DIRECTORY / "build" / "pct_map_cleaner")

    from_path = shutil.which("pct_map_cleaner")
    if from_path:
        candidates.append(Path(from_path))

    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate.resolve()

    raise FileNotFoundError(
        "pct_map_cleaner C++ backend was not found. Build it with:\n"
        f"  cd {SCRIPT_DIRECTORY}\n"
        "  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release\n"
        '  cmake --build build -j"$(nproc)"'
    )


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = _build_argument_parser()
    args = parser.parse_args(argv)

    try:
        config_path = Path(args.config).expanduser().resolve()
        command = [str(_find_backend()), "--config", str(config_path)]
        if args.input_pcd:
            command.extend(("--input", args.input_pcd))
        if args.output_pcd:
            command.extend(("--output", args.output_pcd))
        os.execv(command[0], command)
    except FileNotFoundError as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 1
    except OSError as error:
        print(f"[ERROR] failed to start C++ backend: {error}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
