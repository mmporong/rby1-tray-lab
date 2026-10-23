"""Validated, read-only structural-analysis assets for the local viewer."""

from __future__ import annotations

import hashlib
from io import BytesIO
import json
import math
from pathlib import Path
from typing import Any
import warnings

from PIL import Image


_IMAGE_NAMES = ("surface.png", "cutaway.png")
_METRIC_NAMES = (
    "max_displacement_m",
    "peak_von_mises_pa",
    "volume_p95_von_mises_pa",
    "force_balance_relative",
    "moment_balance_relative",
)
_MAX_REPORT_BYTES = 1_000_000
_MAX_IMAGE_BYTES = 50_000_000
_MAX_IMAGE_DIMENSION = 8_192
_MAX_IMAGE_PIXELS = 32_000_000


def _finite_number(value: Any, name: str, *, minimum: float = 0.0) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    number = float(value)
    if not math.isfinite(number) or number < minimum:
        raise ValueError(f"{name} must be finite and at least {minimum}")
    return number


def _regular_file(root: Path, name: str, *, maximum_bytes: int) -> Path:
    path = root / name
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"{name} must be a regular file")
    size = path.stat().st_size
    if size <= 0 or size > maximum_bytes:
        raise ValueError(f"{name} has an invalid size")
    if path.resolve().parent != root.resolve():
        raise ValueError(f"{name} escapes the structural directory")
    return path


class StructuralAssets:
    """Load a fixed report plus two hash-bound PNGs from one local directory."""

    def __init__(self, directory: str | Path):
        self.directory = Path(directory)
        if self.directory.is_symlink() or not self.directory.is_dir():
            raise ValueError("structural directory must be a regular directory")

        report_path = _regular_file(
            self.directory, "report.json", maximum_bytes=_MAX_REPORT_BYTES
        )
        try:
            report = json.loads(report_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError(f"report.json is not valid UTF-8 JSON: {exc}") from exc
        self._summary = self._validate_report(report)

        self._images: dict[str, bytes] = {}
        for name in _IMAGE_NAMES:
            path = _regular_file(
                self.directory, name, maximum_bytes=_MAX_IMAGE_BYTES
            )
            payload = path.read_bytes()
            digest = hashlib.sha256(payload).hexdigest()
            if digest != report["artifact_sha256"][name]:
                raise ValueError(f"{name} SHA-256 does not match report.json")
            try:
                with warnings.catch_warnings():
                    warnings.simplefilter("error", Image.DecompressionBombWarning)
                    image = Image.open(BytesIO(payload))
                with image:
                    if image.format != "PNG":
                        raise ValueError(f"{name} is not a PNG image")
                    width, height = image.size
                    if (
                        width <= 0
                        or height <= 0
                        or width > _MAX_IMAGE_DIMENSION
                        or height > _MAX_IMAGE_DIMENSION
                        or width * height > _MAX_IMAGE_PIXELS
                    ):
                        raise ValueError(
                            f"{name} dimensions exceed the 8192 px / 32 MP limit"
                        )
                    image.verify()
            except (
                Image.DecompressionBombError,
                Image.DecompressionBombWarning,
                OSError,
                ValueError,
            ) as exc:
                raise ValueError(f"{name} is not a valid PNG image: {exc}") from exc
            self._images[name] = payload

    @staticmethod
    def _validate_report(report: Any) -> dict[str, Any]:
        if not isinstance(report, dict) or report.get("schema_version") != 1:
            raise ValueError("report.json schema_version must be 1")

        title = report.get("title")
        phase = report.get("phase")
        if not isinstance(title, str) or not title.strip():
            raise ValueError("title must be a non-empty string")
        if not isinstance(phase, str) or not phase.strip():
            raise ValueError("phase must be a non-empty string")
        snapshot_time_s = _finite_number(report.get("snapshot_time_s"), "snapshot_time_s")

        material = report.get("material")
        if not isinstance(material, dict) or material.get("source") != "assumed":
            raise ValueError("material source must be 'assumed'")
        youngs = _finite_number(
            material.get("youngs_modulus_pa"), "material.youngs_modulus_pa", minimum=1.0
        )
        poisson = _finite_number(
            material.get("poisson_ratio"), "material.poisson_ratio"
        )
        if not 0.0 <= poisson < 0.5:
            raise ValueError("material.poisson_ratio must be in [0, 0.5)")
        if youngs != 69e9 or poisson != 0.33:
            raise ValueError("material must use the declared 69 GPa, 0.33 assumption")

        metrics = report.get("metrics")
        if not isinstance(metrics, dict):
            raise ValueError("metrics must be an object")
        clean_metrics = {
            name: _finite_number(metrics.get(name), f"metrics.{name}")
            for name in _METRIC_NAMES
        }
        mesh_converged = metrics.get("mesh_converged")
        if not isinstance(mesh_converged, bool):
            raise ValueError("metrics.mesh_converged must be boolean")
        clean_metrics["mesh_converged"] = mesh_converged

        assumptions = report.get("assumptions")
        if (
            not isinstance(assumptions, list)
            or not assumptions
            or any(not isinstance(item, str) or not item.strip() for item in assumptions)
        ):
            raise ValueError("assumptions must be a non-empty list of strings")

        hashes = report.get("artifact_sha256")
        if not isinstance(hashes, dict) or set(hashes) != set(_IMAGE_NAMES):
            raise ValueError("artifact_sha256 must declare only the two structural PNGs")
        for name, digest in hashes.items():
            if (
                not isinstance(digest, str)
                or len(digest) != 64
                or any(character not in "0123456789abcdef" for character in digest)
            ):
                raise ValueError(f"artifact_sha256.{name} must be lowercase SHA-256")

        return {
            "title": title.strip(),
            "snapshot_time_s": snapshot_time_s,
            "phase": phase.strip(),
            "material": {
                "youngs_modulus_pa": youngs,
                "poisson_ratio": poisson,
                "source": "assumed",
            },
            "metrics": clean_metrics,
            "assumptions": [item.strip() for item in assumptions],
        }

    def info(self) -> dict[str, Any]:
        return {
            "enabled": True,
            "summary": self._summary,
            "image_urls": {
                "surface": "/structural/surface.png",
                "cutaway": "/structural/cutaway.png",
            },
        }

    def asset(self, name: str) -> bytes | None:
        return self._images.get(name)


__all__ = ["StructuralAssets"]
