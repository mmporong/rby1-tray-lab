#!/usr/bin/env python3
"""Rerun a validated study case and publish one verified 8x overview video."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
from typing import Any


REPOSITORY = Path(__file__).resolve().parents[1]
TOOLS = Path(__file__).resolve().parent
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))
if str(REPOSITORY / "src") not in sys.path:
    sys.path.insert(0, str(REPOSITORY / "src"))

from audit_transport_contacts import _sha256, _verify_run  # noqa: E402
from build_playback import _VideoEncoder, _verify_video  # noqa: E402
from rby1_tray.cli import source_hashes  # noqa: E402
from rby1_tray.playback import (  # noqa: E402
    BASE_SPEED,
    FPS,
    HEIGHT,
    SENSOR_CAMERAS,
    WIDTH,
    StagedOutput,
    canonical_config_sha256,
    nearest_step_count,
    sha256_file,
)


def _git_state() -> tuple[str, bool]:
    commit = subprocess.run(
        ["git", "-C", str(REPOSITORY), "rev-parse", "HEAD"],
        capture_output=True,
        text=True,
        check=False,
    )
    dirty = subprocess.run(
        ["git", "-C", str(REPOSITORY), "status", "--porcelain"],
        capture_output=True,
        text=True,
        check=False,
    )
    return (
        commit.stdout.strip() if commit.returncode == 0 else "unknown",
        dirty.returncode != 0 or bool(dirty.stdout.strip()),
    )


def _write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _assert_close(label: str, expected: Any, actual: Any, tolerance: float = 1e-9) -> None:
    if isinstance(expected, bool) or expected is None or isinstance(expected, str):
        if expected != actual:
            raise RuntimeError(f"replay mismatch for {label}: expected {expected!r}, got {actual!r}")
        return
    if isinstance(expected, (int, float)) and isinstance(actual, (int, float)):
        if abs(float(expected) - float(actual)) > tolerance:
            raise RuntimeError(
                f"replay mismatch for {label}: expected {expected!r}, got {actual!r}, tolerance {tolerance}"
            )
        return
    raise RuntimeError(f"unsupported replay comparison for {label}")


def _validate_replay(source_result: dict[str, Any], replay: dict[str, Any]) -> None:
    _assert_close("success", source_result.get("success"), replay.get("success"))
    _assert_close("failure_reason", source_result.get("failure_reason"), replay.get("failure_reason"))
    _assert_close(
        "simulation_time_s",
        source_result.get("simulation_time_s"),
        replay.get("simulation_time_s"),
    )
    if "failure_time_s" in source_result:
        _assert_close("failure_time_s", source_result.get("failure_time_s"), replay.get("failure_time_s"))
    expected_metrics = source_result.get("metrics")
    actual_metrics = replay.get("metrics")
    if not isinstance(expected_metrics, dict) or not isinstance(actual_metrics, dict):
        raise RuntimeError("source and replay results require metrics objects")
    if set(expected_metrics) != set(actual_metrics):
        raise RuntimeError(
            "replay metric keys differ: "
            f"expected {sorted(expected_metrics)}, got {sorted(actual_metrics)}"
        )
    for key, expected in expected_metrics.items():
        _assert_close(f"metrics.{key}", expected, actual_metrics[key])


def render_study_video(
    run_directory: str | Path,
    output_directory: str | Path,
    *,
    case_name: str | None = None,
    profile_name: str | None = None,
) -> dict[str, Any]:
    """Rerun a source run and publish only after deterministic result validation."""

    import shutil

    ffmpeg = shutil.which("ffmpeg")
    ffprobe = shutil.which("ffprobe")
    if ffmpeg is None or ffprobe is None:
        raise RuntimeError("ffmpeg and ffprobe are required")
    run = Path(run_directory).resolve()
    run_manifest, config, _ = _verify_run(run)
    result_path = run / "result.json"
    expected_result_hash = run_manifest.get("artifact_sha256", {}).get("result.json")
    if expected_result_hash is None or _sha256(result_path) != expected_result_hash:
        raise RuntimeError("result.json does not match the run manifest")
    source_result = json.loads(result_path.read_text(encoding="utf-8"))

    replay_commit, replay_dirty = _git_state()
    if replay_dirty:
        raise RuntimeError("study video must be rendered from a clean Git worktree")
    sources_before = source_hashes()
    if run_manifest.get("commit") != replay_commit:
        raise RuntimeError(
            "source run and replay must use the same Git commit: "
            f"run={run_manifest.get('commit')!r}, replay={replay_commit!r}"
        )
    if run_manifest.get("source_sha256") != sources_before:
        raise RuntimeError("source run and replay do not have identical controller source hashes")

    from rby1_tray.mission import MobileSession

    destination = StagedOutput(Path(output_directory).resolve())
    stage = destination.create()
    video_path = stage / "video.mp4"
    encoder = _VideoEncoder(video_path, ffmpeg)
    session = None
    frame_count = 0
    started = time.monotonic()
    try:
        session = MobileSession(config)

        def capture() -> None:
            nonlocal frame_count
            frame = session.render(
                width=WIDTH,
                height=HEIGHT,
                azimuth=105,
                elevation=-20,
                view="overview",
            )
            encoder.write(frame)
            frame_count += 1

        capture()
        physics_dt_s = float(config["physics_dt_s"])
        next_frame_index = 1
        while not session.finished:
            target_time_s = next_frame_index * BASE_SPEED / FPS
            session.step(nearest_step_count(float(session.data.time), target_time_s, physics_dt_s))
            capture()
            next_frame_index += 1
            if next_frame_index > 100_000:
                raise RuntimeError("study replay exceeded the frame safety limit")

        session.evaluator.finalize(float(session.data.time))
        final_snapshot = session.snapshot()
        evaluation = session.evaluator.result()
        replay_result = {
            **evaluation,
            "simulation_time_s": float(session.data.time),
            "metrics": final_snapshot["metrics"],
        }
        _validate_replay(source_result, replay_result)
        camera_coverage = sorted(
            {
                str(frame.metadata.get("camera_name"))
                for frame in [*session.sensor_frames, *getattr(session, "monitor_frames", ())]
            }
        )
        missing_cameras = sorted(SENSOR_CAMERAS - set(camera_coverage))
        if missing_cameras:
            raise RuntimeError(f"replay did not produce required RGB-D observations: {missing_cameras}")

        encoder.finish()
        _verify_video(video_path, ffprobe, frame_count)
        sources_after = source_hashes()
        if sources_after != sources_before:
            raise RuntimeError("source changed during study video rendering")
        case = case_name or run.name.split("__", 1)[0]
        profile = profile_name or (run.name.split("__", 1)[1] if "__" in run.name else "unspecified")
        manifest = {
            "schema_version": 1,
            "provenance_statement": "fresh deterministic MobileSession rerun of the source config; not trace interpolation",
            "case": case,
            "profile": profile,
            "seed": int(config["seed"]),
            "source_run": run.as_posix(),
            "source_run_manifest_sha256": _sha256(run / "manifest.json"),
            "source_commit": run_manifest.get("commit"),
            "replay_source_commit": replay_commit,
            "replay_source_dirty": replay_dirty,
            "source_sha256": sources_before,
            "config_sha256": canonical_config_sha256(config),
            "config_file_sha256": _sha256(run / "config.json"),
            "result_file_sha256": _sha256(run / "result.json"),
            "result_match_tolerance": 1e-9,
            "validated_result": replay_result,
            "rendering": {
                "base_speed": BASE_SPEED,
                "fps": FPS,
                "width": WIDTH,
                "height": HEIGHT,
                "frame_count": frame_count,
                "duration_sim_s": float(session.data.time),
                "duration_video_s": frame_count / FPS,
                "view": "overview",
                "sampling": "nearest physics state to i/fps*base_speed; terminal frame appended by the deterministic rerun",
            },
            "rgbd_observations": {
                "camera_coverage": camera_coverage,
                "controller_frame_count": len(session.sensor_frames),
                "monitor_frame_count": len(getattr(session, "monitor_frames", ())),
            },
            "wall_time_s": time.monotonic() - started,
            "artifact_sha256": {"video.mp4": sha256_file(video_path)},
        }
        _write_json(stage / "manifest.json", manifest)
        closing, session = session, None
        closing.close()
        destination.commit_if_sources_match(sources_before, source_hashes())
        return manifest
    except BaseException:
        encoder.terminate()
        raise
    finally:
        try:
            if session is not None:
                session.close()
        finally:
            destination.cleanup()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Render a verified 8x study video by deterministic rerun")
    parser.add_argument("run", help="validated source run directory")
    parser.add_argument("--output", required=True, help="new output directory")
    parser.add_argument("--case")
    parser.add_argument("--profile")
    args = parser.parse_args(argv)

    from rby1_tray.cli import rendering_environment

    rendering_environment()
    manifest = render_study_video(
        args.run,
        args.output,
        case_name=args.case,
        profile_name=args.profile,
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
