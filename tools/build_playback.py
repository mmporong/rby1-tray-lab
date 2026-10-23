#!/usr/bin/env python3
"""Build a verified 8x/60 FPS playback bundle from one physical rollout."""

from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
from typing import Any


REPOSITORY = Path(__file__).resolve().parents[1]
if str(REPOSITORY / "src") not in sys.path:
    sys.path.insert(0, str(REPOSITORY / "src"))

from rby1_tray.playback import (  # noqa: E402
    BASE_SPEED,
    FPS,
    HEIGHT,
    SENSOR_CAMERAS,
    SCHEMA_VERSION,
    VIEWS,
    WIDTH,
    StagedOutput,
    canonical_config_sha256,
    encode_sensor_preview,
    nearest_step_count,
    sha256_file,
)


def _write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        + "\n",
        encoding="utf-8",
    )


def _git_state(repository: Path) -> tuple[str, bool]:
    commit = subprocess.run(
        ["git", "-C", str(repository), "rev-parse", "HEAD"],
        capture_output=True,
        text=True,
        check=False,
    )
    dirty = subprocess.run(
        ["git", "-C", str(repository), "status", "--porcelain"],
        capture_output=True,
        text=True,
        check=False,
    )
    return (
        commit.stdout.strip() if commit.returncode == 0 else "unknown",
        dirty.returncode != 0 or bool(dirty.stdout.strip()),
    )


def _source_hashes(repository: Path) -> dict[str, str]:
    paths = sorted((repository / "src" / "rby1_tray").rglob("*.py"))
    paths.extend(sorted((repository / "src" / "rby1_tray" / "web").glob("*.html")))
    paths.append(Path(__file__).resolve())
    return {
        path.relative_to(repository).as_posix(): sha256_file(path)
        for path in sorted(set(paths))
    }


class _VideoEncoder:
    def __init__(self, output: Path, ffmpeg: str):
        command = [
            ffmpeg,
            "-hide_banner",
            "-loglevel",
            "error",
            "-nostdin",
            "-n",
            "-f",
            "rawvideo",
            "-pixel_format",
            "rgb24",
            "-video_size",
            f"{WIDTH}x{HEIGHT}",
            "-framerate",
            str(FPS),
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "19",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(output),
        ]
        self.output = output
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )

    def write(self, frame: Any) -> None:
        import numpy as np

        pixels = np.asarray(frame, dtype=np.uint8)
        if pixels.shape != (HEIGHT, WIDTH, 3):
            raise ValueError(
                f"renderer returned {pixels.shape}, expected {(HEIGHT, WIDTH, 3)}"
            )
        if float(np.std(pixels)) < 1.0:
            raise RuntimeError("renderer produced an empty playback frame")
        if self.process.stdin is None:
            raise RuntimeError("ffmpeg input pipe is unavailable")
        try:
            self.process.stdin.write(np.ascontiguousarray(pixels).tobytes())
        except BrokenPipeError as exc:
            raise RuntimeError(self._error_message("ffmpeg closed its input")) from exc

    def finish(self) -> None:
        if self.process.stdin is not None and not self.process.stdin.closed:
            self.process.stdin.close()
        stderr = self.process.stderr.read().decode("utf-8", errors="replace") if self.process.stderr else ""
        return_code = self.process.wait()
        if return_code:
            raise RuntimeError(
                f"ffmpeg failed for {self.output.name} with exit {return_code}: "
                f"{stderr.strip() or 'no diagnostic output'}"
            )
        if not self.output.is_file() or self.output.stat().st_size == 0:
            raise RuntimeError(f"ffmpeg did not create {self.output.name}")

    def terminate(self) -> None:
        if self.process.poll() is not None:
            return
        if self.process.stdin is not None and not self.process.stdin.closed:
            self.process.stdin.close()
        self.process.terminate()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5)

    def _error_message(self, prefix: str) -> str:
        return_code = self.process.poll()
        stderr = b""
        if return_code is not None and self.process.stderr is not None:
            stderr = self.process.stderr.read()
        detail = stderr.decode("utf-8", errors="replace").strip()
        return f"{prefix} for {self.output.name}: {detail or f'exit {return_code}'}"


def _verify_video(path: Path, ffprobe: str, expected_frames: int) -> None:
    completed = subprocess.run(
        [
            ffprobe,
            "-v",
            "error",
            "-count_frames",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=width,height,r_frame_rate,nb_read_frames,pix_fmt,codec_name",
            "-of",
            "json",
            str(path),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode:
        raise RuntimeError(
            f"ffprobe failed for {path.name}: {completed.stderr.strip() or 'no diagnostics'}"
        )
    try:
        streams = json.loads(completed.stdout)["streams"]
        stream = streams[0]
    except (KeyError, IndexError, TypeError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"ffprobe returned no video stream for {path.name}") from exc
    expected = {
        "width": WIDTH,
        "height": HEIGHT,
        "r_frame_rate": f"{FPS}/1",
        "nb_read_frames": str(expected_frames),
        "pix_fmt": "yuv420p",
        "codec_name": "h264",
    }
    mismatch = {
        key: {"expected": value, "actual": stream.get(key)}
        for key, value in expected.items()
        if stream.get(key) != value
    }
    if mismatch:
        raise RuntimeError(f"encoded video contract mismatch for {path.name}: {mismatch}")


def _write_sensor_previews(output: Path, frames: list[Any]) -> dict[str, Any]:
    if len(frames) < len(SENSOR_CAMERAS):
        raise RuntimeError(
            "playback requires head and both wrist RGB-D observations, "
            f"received {len(frames)} frames"
        )
    sensor_directory = output / "sensors"
    sensor_directory.mkdir()
    rows = []
    ordered_frames = sorted(frames, key=lambda frame: float(frame.timestamp_sim_s))
    for preview_index, frame in enumerate(ordered_frames):
        camera_name = str(frame.metadata.get("camera_name", "rgbd"))
        if camera_name not in SENSOR_CAMERAS:
            raise RuntimeError(f"unsupported recorded RGB-D camera: {camera_name!r}")
        safe_camera = "".join(character if character.isalnum() or character in "-_" else "_" for character in camera_name)
        prefix = f"{preview_index:02d}_{safe_camera}"
        encoded = {}
        preview_metadata = {}
        for kind in ("rgb", "depth"):
            payload, metadata = encode_sensor_preview(frame, kind)
            relative = Path("sensors") / f"{prefix}_{kind}.png"
            (output / relative).write_bytes(payload)
            encoded[kind] = relative.as_posix()
            preview_metadata[kind] = metadata
        rows.append(
            {
                "preview_index": preview_index,
                "source_frame_index": preview_index,
                "camera_name": camera_name,
                "timestamp_sim_s": float(frame.timestamp_sim_s),
                "rgb_path": encoded["rgb"],
                "depth_path": encoded["depth"],
                "preview": preview_metadata,
                "recorded_frame_provenance": {
                    "sensor_profile": frame.metadata.get("sensor_profile"),
                    "data_source": frame.metadata.get("data_source"),
                    "observation_role": frame.metadata.get("observation_role", "controller"),
                    "capture_batch_id": frame.metadata.get("capture_batch_id"),
                    "depth_range_m": frame.metadata.get("depth_range_m"),
                    "calibration_source": frame.metadata.get("calibration_source"),
                    "clock_domain": frame.metadata.get("clock_domain"),
                },
            }
        )
    coverage = {row["camera_name"] for row in rows}
    if coverage != SENSOR_CAMERAS:
        raise RuntimeError(
            "playback sensor previews must cover head and both wrist RGB-D cameras"
        )
    document = {
        "schema_version": 1,
        "selection": "all controller and monitor RGB-D frames in chronological order",
        "preview_only": True,
        "metric_depth_location": "controller-data observations in the validated mission evidence, not these PNG previews",
        "frames": rows,
    }
    _write_json(output / "sensors.json", document)
    return document


def build_playback(config: dict[str, Any], output: str | Path) -> dict[str, Any]:
    """Run one mission and encode three synchronized camera views."""

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg is required to build playback videos")
    ffprobe = shutil.which("ffprobe")
    if ffprobe is None:
        raise RuntimeError("ffprobe is required to validate playback videos")
    source_commit, source_dirty = _git_state(REPOSITORY)
    source_sha256 = _source_hashes(REPOSITORY)

    from rby1_tray.mission import MobileSession

    published_destination = Path(output)
    staged_output = StagedOutput(published_destination)
    destination = staged_output.create()
    session = None
    encoders: dict[str, _VideoEncoder] = {}
    states: list[dict[str, Any]] = []
    started = time.monotonic()
    try:
        _write_json(destination / "config.json", config)
        session = MobileSession(config)
        for view in VIEWS:
            encoders[view] = _VideoEncoder(destination / f"{view}.mp4", ffmpeg)

        def capture(target_time_s: float, terminal_frame: bool = False) -> None:
            snapshot = session.snapshot()
            actual_time_s = float(snapshot["time_s"])
            row = {
                "frame_index": len(states),
                "target_time_s": float(target_time_s),
                "actual_time_s": actual_time_s,
                "terminal_frame": bool(terminal_frame),
                "state": snapshot,
            }
            states.append(row)
            for view in VIEWS:
                frame = session.render(
                    width=WIDTH,
                    height=HEIGHT,
                    azimuth=105,
                    elevation=-20,
                    view=view,
                )
                encoders[view].write(frame)

        capture(0.0)
        physics_dt_s = float(config["physics_dt_s"])
        frame_interval_sim_s = BASE_SPEED / FPS
        next_frame_index = 1
        while not session.finished:
            target_time_s = next_frame_index * frame_interval_sim_s
            steps = nearest_step_count(
                float(session.data.time), target_time_s, physics_dt_s
            )
            session.step(steps)
            capture(target_time_s, terminal_frame=bool(session.finished))
            next_frame_index += 1
            if next_frame_index > 100_000:
                raise RuntimeError("playback rollout exceeded the frame safety limit")

        final_state = states[-1]["state"]
        final_result = final_state.get("result", {})
        if not final_result.get("terminal") or not final_result.get("success"):
            raise RuntimeError(
                "playback rollout did not finish successfully: "
                + json.dumps(final_result, ensure_ascii=False, allow_nan=False)
            )
        for encoder in encoders.values():
            encoder.finish()
        for view in VIEWS:
            _verify_video(destination / f"{view}.mp4", ffprobe, len(states))

        with (destination / "states.json.gz").open("wb") as raw_stream:
            with gzip.GzipFile(fileobj=raw_stream, mode="wb", mtime=0) as compressed:
                compressed.write(
                    json.dumps(
                        states,
                        ensure_ascii=False,
                        separators=(",", ":"),
                        allow_nan=False,
                    ).encode("utf-8")
                )
        _write_sensor_previews(
            destination,
            [
                *list(session.sensor_frames),
                *list(getattr(session, "monitor_frames", ())),
            ],
        )
        result = {
            **final_result,
            "simulation_time_s": float(final_state["time_s"]),
            "frame_count": len(states),
            "wall_time_s": time.monotonic() - started,
            "rendering": {
                "base_speed": BASE_SPEED,
                "fps": FPS,
                "views": list(VIEWS),
                "width": WIDTH,
                "height": HEIGHT,
                "state_source": "MobileSession.snapshot after unmodified MuJoCo stepping",
            },
        }
        _write_json(destination / "result.json", result)
        artifact_hashes = {
            path.relative_to(destination).as_posix(): sha256_file(path)
            for path in sorted(destination.rglob("*"))
            if path.is_file() and path.name != "manifest.json"
        }
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "base_speed": BASE_SPEED,
            "fps": FPS,
            "frame_count": len(states),
            "duration_sim_s": float(final_state["time_s"]),
            "video_duration_s": len(states) / FPS,
            "width": WIDTH,
            "height": HEIGHT,
            "physics_dt_s": physics_dt_s,
            "sampling": "nearest 2 ms physics state to i/fps*base_speed; terminal success is always appended",
            "source_commit": source_commit,
            "source_dirty": source_dirty,
            "source_sha256": source_sha256,
            "config_sha256": canonical_config_sha256(config),
            "result": final_result,
            "artifact_sha256": artifact_hashes,
        }
        _write_json(destination / "manifest.json", manifest)
        closing_session, session = session, None
        closing_session.close()
        staged_output.commit_if_sources_match(
            source_sha256, _source_hashes(REPOSITORY)
        )
        return manifest
    except BaseException:
        for encoder in encoders.values():
            encoder.terminate()
        raise
    finally:
        try:
            if session is not None:
                session.close()
        finally:
            staged_output.cleanup()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Build synchronized, smooth 8x RB-Y1 playback videos"
    )
    parser.add_argument(
        "--config",
        default=str(REPOSITORY / "configs" / "mobile-native.json"),
    )
    parser.add_argument(
        "--output",
        default=str(REPOSITORY / "evidence" / "playback-native"),
    )
    args = parser.parse_args(argv)

    from rby1_tray.cli import rendering_environment

    rendering_environment()
    from rby1_tray.scene import load_config

    config = load_config(args.config)
    manifest = build_playback(config, args.output)
    print(json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
