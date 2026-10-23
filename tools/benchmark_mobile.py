#!/usr/bin/env python3
"""Run the preregistered RB-Y1 mobile native-gripper comparison matrix."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

from rby1_tray.cli import run_episode
from rby1_tray.experiments import (
    CASES,
    VARIANTS,
    parse_names,
    parse_seeds,
    repository_source_snapshot,
    run_benchmark,
)
from rby1_tray.scene import load_config


REPOSITORY = Path(__file__).resolve().parents[1]


def _git_provenance() -> dict[str, object]:
    commit = subprocess.run(
        ["git", "-C", str(REPOSITORY), "rev-parse", "HEAD"],
        capture_output=True,
        text=True,
        check=False,
    )
    status = subprocess.run(
        ["git", "-C", str(REPOSITORY), "status", "--porcelain"],
        capture_output=True,
        text=True,
        check=False,
    )
    return {
        "repository": str(REPOSITORY),
        "commit": commit.stdout.strip() if commit.returncode == 0 else None,
        "source_dirty_at_start": bool(status.stdout.strip()) if status.returncode == 0 else None,
    }


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description="Preregistered one-factor RB-Y1 mobile task comparisons"
    )
    result.add_argument("--config", default=str(REPOSITORY / "configs/mobile-native.json"))
    result.add_argument("--output", required=True)
    result.add_argument("--cases", default="nominal", help=f"comma-separated: {','.join(CASES)}")
    result.add_argument("--seeds", default="0", help="comma-separated nonnegative integers")
    result.add_argument(
        "--variants", default="coop-depth-cad", help=f"comma-separated: {','.join(VARIANTS)}"
    )
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        cases = parse_names(args.cases, CASES, "cases")
        seeds = parse_seeds(args.seeds)
        variants = parse_names(args.variants, VARIANTS, "variants")
        config_path = Path(args.config).resolve()
        base_config = load_config(config_path)
        summary = run_benchmark(
            base_config=base_config,
            output=Path(args.output),
            cases=cases,
            seeds=seeds,
            variants=variants,
            run_episode=lambda config, output: run_episode(config, output, render=False),
            source_snapshot=lambda: repository_source_snapshot(REPOSITORY, config_path),
            provenance={**_git_provenance(), "config_path": str(config_path)},
        )
    except (FileExistsError, FileNotFoundError, ValueError, RuntimeError) as exc:
        print(f"benchmark error: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if summary["complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
