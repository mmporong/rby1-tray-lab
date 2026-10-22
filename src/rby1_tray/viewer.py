"""Local, read-only web controls for an :class:`rby1_tray.session.Session`.

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
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlsplit

import numpy as np
from PIL import Image


_INDEX_PATH = Path(__file__).with_name("web") / "index.html"


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


def _sensor_png(session: Any, camera_name: str, kind: str) -> tuple[bytes, float] | None:
    """Encode the latest stored sensor frame without capturing a new frame.

    Depth PNGs are display-only grayscale previews.  Metric depth remains in
    the session frame and exported NPZ dataset.
    """

    frames = getattr(session, "sensor_frames", ())
    frame = next(
        (
            candidate
            for candidate in reversed(frames)
            if candidate.metadata.get("camera_name") == camera_name
        ),
        None,
    )
    if frame is None:
        return None
    if kind == "rgb":
        image = Image.fromarray(np.asarray(frame.rgb, dtype=np.uint8), mode="RGB")
    else:
        depth = np.asarray(frame.depth_m, dtype=float)
        valid = np.isfinite(depth) & (depth > 0)
        preview = np.zeros(depth.shape, dtype=np.uint8)
        if np.any(valid):
            low = float(np.min(depth[valid]))
            high = float(np.max(depth[valid]))
            if high > low:
                preview[valid] = np.rint(
                    1.0 + 254.0 * (high - depth[valid]) / (high - low)
                ).astype(np.uint8)
            else:
                preview[valid] = 255
        image = Image.fromarray(preview, mode="L")
    payload = BytesIO()
    image.save(payload, format="PNG", optimize=True)
    return payload.getvalue(), float(frame.timestamp_sim_s)


def _make_handler(session: Any) -> type[BaseHTTPRequestHandler]:
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
                encoded = _sensor_png(session, camera_name, kind)
                if encoded is None:
                    self._send_json(
                        HTTPStatus.NOT_FOUND,
                        {"error": "아직 저장된 센서 프레임이 없습니다."},
                    )
                    return
                body, timestamp_sim_s = encoded
                self._send(
                    HTTPStatus.OK,
                    body,
                    "image/png",
                    {"X-Sensor-Timestamp-S": f"{timestamp_sim_s:.9g}"},
                )
                return
            self._send_json(HTTPStatus.NOT_FOUND, {"error": "요청한 경로를 찾을 수 없습니다."})

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
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; img-src 'self' blob:")
            for name, value in (headers or {}).items():
                self.send_header(name, value)
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format: str, *args: Any) -> None:
            print(f"[viewer] {self.address_string()} {format % args}")

    return ViewerHandler


def serve(session: Any, host: str = "127.0.0.1", port: int = 8786) -> None:
    """Serve the local viewer until interrupted.

    The caller owns process lifetime; this function does not start daemon
    threads or simulation loops.  The supplied session is only stepped or
    reset in response to explicit HTTP requests.
    """

    if not _INDEX_PATH.is_file():
        raise FileNotFoundError(f"Viewer page is missing: {_INDEX_PATH}")
    server = HTTPServer((host, port), _make_handler(session))
    print(f"RB-Y1 Tray Lab: http://{host}:{server.server_port}")
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


__all__ = ["serve"]
