"""Local simulation controls and synchronized recorded-physics playback.

The HTTP server deliberately handles one request at a time.  MuJoCo's OpenGL
context is created lazily by ``Session.render`` and must continue to be used on
the same thread.  Keeping rendering and stepping on the server thread also
makes browser play/pause commands deterministic: there is no background
simulation loop.
"""

from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, HTTPServer
from io import BytesIO
import json
import math
from pathlib import Path
import re
from typing import Any
from urllib.parse import parse_qs, urlsplit

from PIL import Image

from .playback import encode_sensor_preview


_INDEX_PATH = Path(__file__).with_name("web") / "index.html"


class _UnavailableStructuralScene:
    """Expose a safe configured-but-invalid status without serving any bytes."""

    def info(self) -> dict[str, Any]:
        return {
            "enabled": False,
            "configured": True,
            "error": "동기 구조 응력 데이터가 무결성 검증을 통과하지 못했습니다.",
        }

    def asset(self, name: str) -> None:
        return None


def _json_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
        default=_json_default,
    ).encode("utf-8")


def _json_default(value: Any) -> Any:
    """Convert common NumPy values without making NumPy a viewer dependency."""

    if hasattr(value, "item"):
        return value.item()
    if hasattr(value, "tolist"):
        return value.tolist()
    raise TypeError(f"{type(value).__name__} is not JSON serializable")


def _camera_value(query: dict[str, list[str]], name: str, default: float) -> float:
    try:
        value = float(query.get(name, [str(default)])[0])
    except (TypeError, ValueError):
        return default
    return max(-360.0, min(360.0, value))


def _view_value(query: dict[str, list[str]]) -> str:
    """Return one of the camera compositions implemented by ``Session``."""

    value = query.get("view", ["overview"])[0]
    return value if value in {"overview", "pickup", "placement"} else "overview"


_SENSOR_CAMERAS = {"head_rgbd", "left_wrist_rgbd", "right_wrist_rgbd"}
_SENSOR_KINDS = {"rgb", "depth"}


def _latest_sensor_frame(session: Any, camera_name: str) -> Any | None:
    controller_frames = list(getattr(session, "sensor_frames", ()))
    monitor_frames = list(getattr(session, "monitor_frames", ()))
    batches: dict[str, list[Any]] = {}
    for frame in monitor_frames:
        batch_id = frame.metadata.get("capture_batch_id")
        if isinstance(batch_id, str) and batch_id:
            batches.setdefault(batch_id, []).append(frame)
    complete_batches = [
        batch
        for batch in batches.values()
        if {frame.metadata.get("camera_name") for frame in batch} == _SENSOR_CAMERAS
        and len(batch) == len(_SENSOR_CAMERAS)
        and max(float(frame.timestamp_sim_s) for frame in batch)
        - min(float(frame.timestamp_sim_s) for frame in batch)
        <= 1e-9
    ]
    if complete_batches:
        batch = max(
            complete_batches,
            key=lambda frames: max(float(frame.timestamp_sim_s) for frame in frames),
        )
        return next(
            frame
            for frame in batch
            if frame.metadata.get("camera_name") == camera_name
        )
    return max(
        (
            frame
            for frame in [*controller_frames, *monitor_frames]
            if frame.metadata.get("camera_name") == camera_name
        ),
        key=lambda frame: float(frame.timestamp_sim_s),
        default=None,
    )


def _sensor_png(
    session: Any, camera_name: str, kind: str
) -> tuple[bytes, float, dict[str, Any]] | None:
    """Encode the latest stored sensor frame without capturing a new frame.

    Depth PNGs are display-only grayscale previews.  Metric depth remains in
    the session frame and exported NPZ dataset.
    """

    frame = _latest_sensor_frame(session, camera_name)
    if frame is None:
        return None
    payload, _preview_metadata = encode_sensor_preview(frame, kind)
    return payload, float(frame.timestamp_sim_s), dict(frame.metadata)


def _sensor_headers(
    camera_name: str, timestamp_sim_s: float, metadata: dict[str, Any]
) -> dict[str, str]:
    depth_range = metadata.get("depth_range_m", {})
    headers = {
        "X-Sensor-Timestamp-S": f"{timestamp_sim_s:.9g}",
        "X-Sensor-Camera": camera_name,
        "X-Sensor-Profile": str(metadata.get("sensor_profile", "unspecified")),
        "X-Sensor-Observation-Role": str(metadata.get("observation_role", "controller")),
        "X-Sensor-Clock-Domain": str(metadata.get("clock_domain", "unspecified")),
        "X-Sensor-Calibration-Source": str(metadata.get("calibration_source", "unspecified")),
    }
    batch_id = metadata.get("capture_batch_id")
    if isinstance(batch_id, str) and batch_id:
        headers["X-Sensor-Capture-Batch-Id"] = batch_id
    if isinstance(depth_range, dict):
        minimum, maximum = depth_range.get("minimum"), depth_range.get("maximum")
        if isinstance(minimum, (int, float)) and math.isfinite(minimum):
            headers["X-Sensor-Depth-Min-M"] = f"{float(minimum):.9g}"
        if isinstance(maximum, (int, float)) and math.isfinite(maximum):
            headers["X-Sensor-Depth-Max-M"] = f"{float(maximum):.9g}"
    return headers


def _byte_range(header: str | None, size: int) -> tuple[int, int] | None:
    """Resolve one HTTP byte range, including suffix and open-ended requests."""
    if header is None:
        return None
    match = re.fullmatch(r"bytes=(\d*)-(\d*)", header.strip())
    if not match or size <= 0 or not any(match.groups()):
        raise ValueError("Unsatisfiable byte range")
    first, last = match.groups()
    if not first:
        suffix = int(last)
        if suffix <= 0:
            raise ValueError("Unsatisfiable suffix range")
        return max(0, size - suffix), size - 1
    start = int(first)
    end = min(int(last), size - 1) if last else size - 1
    if start >= size or start > end:
        raise ValueError("Unsatisfiable byte range")
    return start, end


def _make_handler(
    session: Any,
    playback: Any = None,
    structural: Any = None,
    structural_scene: Any = None,
) -> type[BaseHTTPRequestHandler]:
    class ViewerHandler(BaseHTTPRequestHandler):
        server_version = "RBY1TrayViewer/0.1"

        def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
            request = urlsplit(self.path)
            if request.path in {"/", "/index.html"}:
                self._send(HTTPStatus.OK, _INDEX_PATH.read_bytes(), "text/html; charset=utf-8")
                return
            if request.path == "/api/state":
                self._send_json(HTTPStatus.OK, session.snapshot())
                return
            if request.path == "/api/playback":
                self._send_json(HTTPStatus.OK, playback.info() if playback else {"enabled": False})
                return
            if request.path == "/api/structural":
                self._send_json(
                    HTTPStatus.OK,
                    structural.info() if structural else {"enabled": False},
                )
                return
            if request.path == "/api/structural-scene":
                self._send_json(
                    HTTPStatus.OK,
                    structural_scene.info()
                    if structural_scene
                    else {"enabled": False, "configured": False},
                )
                return
            if request.path in {"/structural-scene.js", "/structural-diagnostics.js"}:
                self._send(
                    HTTPStatus.OK,
                    (Path(__file__).with_name("web") / request.path[1:]).read_bytes(),
                    "text/javascript; charset=utf-8",
                )
                return
            if request.path.startswith("/structural-scene/"):
                self._serve_structural_scene(request.path)
                return
            if request.path.startswith("/playback/"):
                self._serve_playback(request.path)
                return
            if request.path.startswith("/structural/"):
                self._serve_structural(request.path)
                return
            if request.path == "/frame.png":
                query = parse_qs(request.query)
                azimuth = _camera_value(query, "azimuth", 105.0)
                elevation = _camera_value(query, "elevation", -20.0)
                view = _view_value(query)
                frame = session.render(
                    width=960,
                    height=720,
                    azimuth=azimuth,
                    elevation=elevation,
                    view=view,
                )
                image = Image.fromarray(frame)
                payload = BytesIO()
                image.save(payload, format="PNG", optimize=True)
                self._send(HTTPStatus.OK, payload.getvalue(), "image/png")
                return
            if request.path == "/sensor.png":
                query = parse_qs(request.query)
                camera_name = query.get("camera", [""])[0]
                kind = query.get("kind", [""])[0]
                if camera_name not in _SENSOR_CAMERAS or kind not in _SENSOR_KINDS:
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {"error": "camera 또는 kind 값이 올바르지 않습니다."},
                    )
                    return
                if playback and "time_s" in query:
                    try:
                        time_s = float(query["time_s"][0])
                        if not math.isfinite(time_s) or time_s < 0:
                            raise ValueError("Invalid playback time")
                        encoded = playback.sensor_png(camera_name, kind, time_s)
                    except (ValueError, TypeError):
                        self._send_json(HTTPStatus.BAD_REQUEST, {"error": "센서 재생 시각이 올바르지 않습니다."})
                        return
                else:
                    encoded = _sensor_png(session, camera_name, kind)
                if encoded is None:
                    self._send_json(
                        HTTPStatus.NOT_FOUND,
                        {"error": "아직 저장된 센서 프레임이 없습니다."},
                    )
                    return
                body, timestamp_sim_s, metadata = encoded
                self._send(
                    HTTPStatus.OK,
                    body,
                    "image/png",
                    _sensor_headers(camera_name, timestamp_sim_s, metadata),
                )
                return
            self._send_json(HTTPStatus.NOT_FOUND, {"error": "요청한 경로를 찾을 수 없습니다."})

        def do_HEAD(self) -> None:  # noqa: N802
            request = urlsplit(self.path)
            if request.path.startswith("/playback/"):
                self._serve_playback(request.path, head_only=True)
            elif request.path.startswith("/structural-scene/"):
                self._serve_structural_scene(request.path, head_only=True)
            elif request.path in {"/structural-scene.js", "/structural-diagnostics.js"}:
                body = (Path(__file__).with_name("web") / request.path[1:]).read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/javascript; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("X-Content-Type-Options", "nosniff")
                self.end_headers()
            elif request.path.startswith("/structural/"):
                self._serve_structural(request.path, head_only=True)
            else:
                self.send_error(HTTPStatus.NOT_FOUND)

        def _serve_structural_scene(
            self, request_path: str, head_only: bool = False
        ) -> None:
            name = request_path.removeprefix("/structural-scene/")
            body = structural_scene.asset(name) if structural_scene else None
            if body is None:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "private, max-age=0, must-revalidate")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            if not head_only:
                self.wfile.write(body)

        def _serve_structural(
            self, request_path: str, head_only: bool = False
        ) -> None:
            asset = (
                structural.asset(request_path.removeprefix("/structural/"))
                if structural
                else None
            )
            if asset is None:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            body = asset
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            if not head_only:
                self.wfile.write(body)

        def _serve_playback(self, request_path: str, head_only: bool = False) -> None:
            asset = playback.asset(request_path.removeprefix("/playback/")) if playback else None
            if asset is None:
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            path, content_type, encoding = asset
            size = path.stat().st_size
            try:
                selected = _byte_range(self.headers.get("Range"), size)
            except ValueError:
                self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                self.send_header("Content-Range", f"bytes */{size}")
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            start, end = selected if selected else (0, size - 1)
            self.send_response(HTTPStatus.PARTIAL_CONTENT if selected else HTTPStatus.OK)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(max(0, end - start + 1)))
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Cache-Control", "private, max-age=0, must-revalidate")
            self.send_header("X-Content-Type-Options", "nosniff")
            if encoding:
                self.send_header("Content-Encoding", encoding)
            if selected:
                self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
            self.end_headers()
            if head_only:
                return
            try:
                with path.open("rb") as stream:
                    stream.seek(start)
                    remaining = end - start + 1
                    while remaining > 0:
                        chunk = stream.read(min(64 * 1024, remaining))
                        if not chunk:
                            break
                        self.wfile.write(chunk)
                        remaining -= len(chunk)
            except (BrokenPipeError, ConnectionResetError):
                # Native video seeking cancels the previous byte-range request.
                return

        def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
            request = urlsplit(self.path)
            if request.path == "/api/reset":
                session.reset()
                self._send_json(HTTPStatus.OK, session.snapshot())
                return
            if request.path == "/api/replay":
                session.reset()
                self._send_json(HTTPStatus.OK, session.snapshot())
                return
            if request.path == "/api/step":
                body = self._read_json()
                if body is None:
                    return
                try:
                    requested = int(body.get("steps", 1))
                except (TypeError, ValueError):
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {"error": "steps는 정수여야 합니다."},
                    )
                    return
                steps = max(1, min(500, requested))
                session.step(n_steps=steps)
                self._send_json(HTTPStatus.OK, session.snapshot())
                return
            self._send_json(HTTPStatus.NOT_FOUND, {"error": "요청한 경로를 찾을 수 없습니다."})

        def _read_json(self) -> dict[str, Any] | None:
            try:
                length = max(0, min(int(self.headers.get("Content-Length", "0")), 16_384))
                raw = self.rfile.read(length) if length else b"{}"
                value = json.loads(raw)
                if not isinstance(value, dict):
                    raise ValueError("JSON object required")
                return value
            except (ValueError, json.JSONDecodeError, UnicodeDecodeError):
                self._send_json(
                    HTTPStatus.BAD_REQUEST,
                    {"error": "올바른 JSON 객체가 필요합니다."},
                )
                return None

        def _send_json(self, status: HTTPStatus, value: Any) -> None:
            try:
                body = _json_bytes(value)
            except (TypeError, ValueError) as exc:
                body = _json_bytes({"error": f"상태를 JSON으로 변환하지 못했습니다: {exc}"})
                status = HTTPStatus.INTERNAL_SERVER_ERROR
            self._send(status, body, "application/json; charset=utf-8")

        def _send(
            self,
            status: HTTPStatus,
            body: bytes,
            content_type: str,
            headers: dict[str, str] | None = None,
        ) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'unsafe-inline'; img-src 'self' blob:; media-src 'self'")
            for name, value in (headers or {}).items():
                self.send_header(name, value)
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format: str, *args: Any) -> None:
            print(f"[viewer] {self.address_string()} {format % args}")

    return ViewerHandler


def serve(
    session: Any,
    host: str = "127.0.0.1",
    port: int = 8786,
    structural_directory: str | Path | None = None,
    structural_scene_directory: str | Path | None = None,
) -> None:
    """Serve the local viewer until interrupted.

    The caller owns process lifetime; this function does not start daemon
    threads or simulation loops.  The supplied session is only stepped or
    reset in response to explicit HTTP requests.
    """

    if not _INDEX_PATH.is_file():
        raise FileNotFoundError(f"Viewer page is missing: {_INDEX_PATH}")
    from .playback import PlaybackAssets
    playback = None
    directory = Path(__file__).resolve().parents[2] / "evidence" / "playback-native"
    try:
        playback = PlaybackAssets(directory, session.config)
    except (FileNotFoundError, ValueError) as exc:
        print(f"[viewer] Recorded playback unavailable; live controls remain available: {exc}")
    structural = None
    if structural_directory is not None:
        from .structural_view import StructuralAssets
        try:
            structural = StructuralAssets(structural_directory)
        except (OSError, ValueError) as exc:
            print(
                "[viewer] Structural result unavailable; primary playback remains "
                f"available: {exc}"
            )
    structural_scene = None
    if structural_scene_directory is not None:
        from .structural_scene import StructuralSceneAssets
        try:
            structural_scene = StructuralSceneAssets(structural_scene_directory, playback)
        except (OSError, ValueError) as exc:
            print(
                "[viewer] Synchronized structural scene unavailable; other viewer "
                f"modes remain available: {exc}"
            )
            structural_scene = _UnavailableStructuralScene()
    server = HTTPServer(
        (host, port), _make_handler(session, playback, structural, structural_scene)
    )
    print(f"RB-Y1 Tray Lab: http://{host}:{server.server_port}")
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


__all__ = ["serve"]
