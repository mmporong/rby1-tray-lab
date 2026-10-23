"""Validated synchronized structural-scene assets for the local viewer."""

from __future__ import annotations

from array import array
import hashlib
import json
import math
from pathlib import Path
import re
import sys
from typing import Any


ASSET_NAMES = (
    "geometry.bin",
    "frames.bin",
    "fem-surface.bin",
    "fem-cutaway.bin",
)
_ASSET_LIMIT_BYTES = 256 * 1024 * 1024
_TOTAL_LIMIT_BYTES = 384 * 1024 * 1024
_MANIFEST_LIMIT_BYTES = 2 * 1024 * 1024
_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}\Z")


def _finite_number(value: Any, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0):
        raise ValueError(f"{name} must be finite{' and positive' if positive else ''}")
    return result


def _integer(value: Any, name: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}")
    return value


def _vector(value: Any, length: int, name: str) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{name} must contain {length} numbers")
    return [_finite_number(item, name) for item in value]


def _text_list(value: Any, name: str) -> list[str]:
    if not isinstance(value, list) or not value:
        raise ValueError(f"{name} must be a non-empty string list")
    if any(not isinstance(item, str) or not item.strip() or len(item) > 500 for item in value):
        raise ValueError(f"{name} contains an invalid string")
    return list(value)


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _float_buffer(payload: bytes, expected_floats: int, name: str) -> array:
    if len(payload) != expected_floats * 4:
        raise ValueError(f"{name} byte length does not match its declared shape")
    values = array("f")
    values.frombytes(payload)
    if sys.byteorder != "little":
        values.byteswap()
    if any(not math.isfinite(value) for value in values):
        raise ValueError(f"{name} contains a non-finite float")
    return values


def _validate_rotation(rotation: list[float] | array, name: str) -> None:
    rows = [rotation[index * 3 : index * 3 + 3] for index in range(3)]
    for row in rows:
        norm = math.sqrt(sum(component * component for component in row))
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=2e-4):
            raise ValueError(f"{name} is not orthonormal")
    for first in range(3):
        for second in range(first + 1, 3):
            dot = sum(rows[first][axis] * rows[second][axis] for axis in range(3))
            if not math.isclose(dot, 0.0, rel_tol=0.0, abs_tol=2e-4):
                raise ValueError(f"{name} is not orthonormal")
    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    if not math.isclose(determinant, 1.0, rel_tol=0.0, abs_tol=4e-4):
        raise ValueError(f"{name} is not a right-handed rotation")


def _validate_basis(value: Any, name: str) -> list[float]:
    basis = _vector(value, 9, name)
    columns = ([basis[row * 3 + column] for row in range(3)] for column in range(3))
    columns = list(columns)
    for index, column in enumerate(columns):
        norm = math.sqrt(sum(component * component for component in column))
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=2e-4):
            raise ValueError(f"{name} column {index} is not unit length")
    for first in range(3):
        for second in range(first + 1, 3):
            dot = sum(columns[first][axis] * columns[second][axis] for axis in range(3))
            if not math.isclose(dot, 0.0, rel_tol=0.0, abs_tol=2e-4):
                raise ValueError(f"{name} columns are not orthogonal")
    _validate_rotation(basis, name)
    return basis


def _load_metadata(value: Any, parts: list[dict], frame_count: int) -> dict:
    if not isinstance(value, dict) or value.get("asset") != "diagnostics.bin":
        raise ValueError("load_diagnostics must identify diagnostics.bin")
    if value.get("part_stride_floats") != 18 or value.get("frame_stride_floats") != len(parts) * 18:
        raise ValueError("load diagnostics strides do not match the scene")
    raw_parts = value.get("parts")
    if not isinstance(raw_parts, list) or len(raw_parts) != len(parts):
        raise ValueError("load diagnostics part count does not match the scene")
    clean = []
    for raw, part in zip(raw_parts, parts):
        if not isinstance(raw, dict) or raw.get("id") != part["id"]:
            raise ValueError("load diagnostics part order does not match the scene")
        item = {"id": part["id"]}
        for field in ("parent_label", "source_body_label"):
            text = raw.get(field)
            if not isinstance(text, str) or not text.strip() or len(text) > 150:
                raise ValueError(f"invalid load diagnostics {field}")
            item[field] = text
        action = raw.get("action")
        varying_base = part["id"] == "base"
        expected_action = "child_on_parent" if varying_base or part["kind"] == "fem" else "parent_on_child"
        if action != expected_action:
            raise ValueError("load diagnostics action disagrees with the recorded cut convention")
        item["action"] = action
        item["origin_body_m"] = _vector(raw.get("origin_body_m"), 3, "load origin")
        item["basis_body"] = _validate_basis(raw.get("basis_body"), "load basis")
        if part["kind"] == "section":
            for field in ("origin_body_m", "basis_body"):
                if any(not math.isclose(a, b, rel_tol=0.0, abs_tol=1e-10) for a, b in zip(item[field], part["section"][field])):
                    raise ValueError("load diagnostics frame differs from the section frame")
        mass = raw.get("subtree_mass_kg")
        if mass is not None:
            mass = _finite_number(mass, "subtree_mass_kg")
            if mass < 0:
                raise ValueError("subtree mass cannot be negative")
        item["subtree_mass_kg"] = mass
        sources = raw.get("source_body_per_frame")
        if varying_base:
            if sources is None or mass is not None:
                raise ValueError("base load metadata requires varying sources and no scalar mass")
        elif sources is not None or mass is None:
            raise ValueError("fixed load cuts require scalar mass and cannot vary source bodies")
        if sources is not None:
            masses, labels = raw.get("subtree_mass_kg_by_source"), raw.get("source_labels")
            if not isinstance(sources, list) or len(sources) != frame_count or not all(isinstance(x, str) and _ID.fullmatch(x) for x in sources):
                raise ValueError("invalid load source_body_per_frame")
            if not isinstance(masses, dict) or not isinstance(labels, dict) or set(masses) != set(sources) or set(labels) != set(sources):
                raise ValueError("load source lookup maps must cover exactly the recorded sources")
            clean_masses = {key: _finite_number(number, "source mass") for key, number in masses.items()}
            if any(number < 0 for number in clean_masses.values()) or any(not isinstance(label, str) or not label.strip() or len(label) > 150 for label in labels.values()):
                raise ValueError("invalid load source labels or mass")
            item.update(source_body_per_frame=list(sources), subtree_mass_kg_by_source=clean_masses, source_labels=dict(labels))
        elif mass is None:
            raise ValueError("a varying load subtree mass requires per-frame sources")
        clean.append(item)
    return {"asset_url": "/structural-scene/diagnostics.bin", "part_stride_floats": 18, "frame_stride_floats": len(parts) * 18, "parts": clean}


class StructuralSceneAssets:
    """Load one immutable whole-robot structural bundle after strict validation."""

    def __init__(self, directory: str | Path, playback: Any):
        self.directory = Path(directory)
        if self.directory.is_symlink() or not self.directory.is_dir():
            raise ValueError("structural scene directory must be a regular directory")
        manifest_path = self.directory / "manifest.json"
        if manifest_path.is_symlink() or not manifest_path.is_file():
            raise FileNotFoundError(manifest_path)
        if manifest_path.stat().st_size > _MANIFEST_LIMIT_BYTES:
            raise ValueError("structural scene manifest is too large")
        try:
            manifest = json.loads(
                manifest_path.read_text(encoding="utf-8"),
                parse_constant=lambda value: (_ for _ in ()).throw(
                    ValueError(f"invalid JSON constant: {value}")
                ),
            )
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("structural scene manifest is not valid JSON") from exc
        if not isinstance(manifest, dict) or manifest.get("schema_version") != 1:
            raise ValueError("unsupported structural scene manifest schema")
        if playback is None:
            raise ValueError("structural scene requires verified recorded playback")
        playback_info = playback.info()
        frame_count = _integer(manifest.get("frame_count"), "frame_count", minimum=1)
        fps = _finite_number(manifest.get("fps"), "fps", positive=True)
        base_speed = _finite_number(manifest.get("base_speed"), "base_speed", positive=True)
        if frame_count != playback_info.get("frame_count"):
            raise ValueError("structural scene frame_count does not match playback")
        if fps != float(playback_info.get("fps", -1)):
            raise ValueError("structural scene fps does not match playback")
        if base_speed != float(playback_info.get("base_speed", -1)):
            raise ValueError("structural scene base_speed does not match playback")
        reference_hash = manifest.get("reference_states_sha256")
        playback_hash = playback.manifest.get("artifact_sha256", {}).get("states.json.gz")
        if not isinstance(reference_hash, str) or reference_hash != playback_hash:
            raise ValueError("structural scene reference states do not match playback")

        times = manifest.get("frame_times_s")
        if not isinstance(times, list) or len(times) != frame_count:
            raise ValueError("frame_times_s length does not match frame_count")
        previous = -math.inf
        for value in times:
            current = _finite_number(value, "frame_times_s")
            if current < 0 or current <= previous:
                raise ValueError("frame_times_s must be non-negative and strictly increasing")
            previous = current
        if not math.isclose(times[0], 0.0, rel_tol=0.0, abs_tol=1e-7):
            raise ValueError("frame_times_s must begin at zero")

        frame_stride = _integer(manifest.get("frame_stride_floats"), "frame_stride_floats", minimum=1)
        part_stride = _integer(manifest.get("part_stride_floats"), "part_stride_floats", minimum=1)
        geometry_stride = _integer(manifest.get("geometry_stride_floats"), "geometry_stride_floats", minimum=1)
        fem_stride = _integer(manifest.get("fem_stride_floats"), "fem_stride_floats", minimum=1)
        if part_stride != 19 or geometry_stride != 6 or fem_stride != 42:
            raise ValueError("structural scene binary strides are unsupported")

        parts = manifest.get("parts")
        if not isinstance(parts, list) or not parts:
            raise ValueError("structural scene requires at least one part")
        clean_parts = []
        identifiers: set[str] = set()
        total_geometry_vertices = 0
        fem_ids = []
        for index, raw in enumerate(parts):
            if not isinstance(raw, dict):
                raise ValueError("each structural part must be an object")
            identifier = raw.get("id")
            label = raw.get("label")
            kind = raw.get("kind")
            analysis_status = raw.get("analysis_status", "section_assumption")
            if not isinstance(identifier, str) or not _ID.fullmatch(identifier) or identifier in identifiers:
                raise ValueError("structural part ids must be unique safe identifiers")
            if not isinstance(label, str) or not label.strip() or len(label) > 100:
                raise ValueError(f"parts[{index}].label is invalid")
            if kind not in {"section", "fem"}:
                raise ValueError(f"parts[{index}].kind is unsupported")
            if analysis_status not in {"section_assumption", "visual_only"}:
                raise ValueError(f"parts[{index}].analysis_status is unsupported")
            if kind == "fem" and analysis_status == "visual_only":
                raise ValueError("the FEM part cannot be visual_only")
            identifiers.add(identifier)
            offset = _integer(raw.get("vertex_offset"), f"parts[{index}].vertex_offset")
            count = _integer(raw.get("vertex_count"), f"parts[{index}].vertex_count")
            assumptions = _text_list(raw.get("assumptions"), f"parts[{index}].assumptions")
            item = {
                "id": identifier,
                "label": label,
                "kind": kind,
                "analysis_status": analysis_status,
                "vertex_offset": offset,
                "vertex_count": count,
                "assumptions": assumptions,
            }
            if kind == "section":
                if count <= 0 or count % 3 or offset != total_geometry_vertices:
                    raise ValueError("section geometry ranges must be contiguous triangles")
                section = raw.get("section")
                if not isinstance(section, dict):
                    raise ValueError("section part is missing section properties")
                clean_section = {
                    "origin_body_m": _vector(section.get("origin_body_m"), 3, "origin_body_m"),
                    "basis_body": _validate_basis(section.get("basis_body"), "basis_body"),
                    "area_m2": _finite_number(section.get("area_m2"), "area_m2", positive=True),
                    "I_m4": _finite_number(section.get("I_m4"), "I_m4", positive=True),
                    "J_m4": _finite_number(section.get("J_m4"), "J_m4", positive=True),
                }
                item["section"] = clean_section
                total_geometry_vertices += count
            else:
                if count != 0:
                    raise ValueError("FEM geometry is stored only in the dedicated FEM buffers")
                fem_ids.append(identifier)
            clean_parts.append(item)
        if frame_stride != len(clean_parts) * part_stride:
            raise ValueError("frame_stride_floats does not match part count")
        fem_part_id = manifest.get("fem_part_id")
        if len(fem_ids) != 1 or fem_part_id != fem_ids[0]:
            raise ValueError("fem_part_id must identify the single FEM part")
        fem_part = next(part for part in clean_parts if part["id"] == fem_part_id)
        if fem_part["vertex_offset"] != total_geometry_vertices:
            raise ValueError("FEM vertex_offset must equal the section geometry vertex count")
        fem_surface_count = _integer(
            manifest.get("fem_surface_vertex_count"), "fem_surface_vertex_count", minimum=3
        )
        fem_cutaway_count = _integer(
            manifest.get("fem_cutaway_vertex_count"), "fem_cutaway_vertex_count", minimum=3
        )
        if fem_surface_count % 3 or fem_cutaway_count % 3:
            raise ValueError("FEM buffers must contain expanded triangles")
        max_stress = _finite_number(
            manifest.get("max_von_mises_pa"), "max_von_mises_pa", positive=True
        )
        scope = manifest.get("scope")
        if not isinstance(scope, str) or not scope.strip() or len(scope) > 1000:
            raise ValueError("scope must be a non-empty string")
        assumptions = _text_list(manifest.get("assumptions"), "assumptions")

        load_raw = manifest.get("load_diagnostics")
        load_info = _load_metadata(load_raw, clean_parts, frame_count) if load_raw is not None else None
        allowed_assets = (*ASSET_NAMES, "diagnostics.bin") if load_info else ASSET_NAMES
        hashes = manifest.get("assets")
        if not isinstance(hashes, dict) or set(hashes) != set(allowed_assets):
            raise ValueError("structural scene must declare exactly the supported binary assets")
        if load_info:
            assert isinstance(load_raw, dict)
            if load_raw.get("sha256") != hashes["diagnostics.bin"]:
                raise ValueError("load diagnostics hash declarations do not match")
        expected_floats = {
            "geometry.bin": total_geometry_vertices * geometry_stride,
            "frames.bin": frame_count * frame_stride,
            "fem-surface.bin": fem_surface_count * fem_stride,
            "fem-cutaway.bin": fem_cutaway_count * fem_stride,
        }
        if load_info:
            expected_floats["diagnostics.bin"] = frame_count * len(clean_parts) * 18
        self._assets: dict[str, bytes] = {}
        frame_values: array | None = None
        load_values: array | None = None
        total_bytes = 0
        for name in allowed_assets:
            expected_hash = hashes.get(name)
            if not isinstance(expected_hash, str) or not re.fullmatch(r"[0-9a-f]{64}", expected_hash):
                raise ValueError(f"invalid SHA-256 declaration for {name}")
            path = self.directory / name
            if path.is_symlink() or not path.is_file():
                raise ValueError(f"structural scene asset must be a regular file: {name}")
            try:
                path.resolve(strict=True).relative_to(self.directory.resolve(strict=True))
            except (OSError, ValueError) as exc:
                raise ValueError(f"structural scene asset escapes its bundle: {name}") from exc
            size = path.stat().st_size
            if size > _ASSET_LIMIT_BYTES:
                raise ValueError(f"structural scene asset is too large: {name}")
            total_bytes += size
            if total_bytes > _TOTAL_LIMIT_BYTES:
                raise ValueError("structural scene assets exceed the total size limit")
            payload = path.read_bytes()
            if _sha256(payload) != expected_hash:
                raise ValueError(f"structural scene asset hash mismatch: {name}")
            values = _float_buffer(payload, expected_floats[name], name)
            if name == "frames.bin":
                frame_values = values
            elif name == "diagnostics.bin":
                load_values = values
            self._assets[name] = payload
        assert frame_values is not None
        registered_maximum = 0.0
        for frame in range(frame_count):
            for part in range(len(clean_parts)):
                start = frame * frame_stride + part * part_stride + 3
                _validate_rotation(
                    frame_values[start : start + 9],
                    f"frames.bin pose at frame {frame}, part {part}",
                )
                peak = float(frame_values[start + 15])
                if load_values is not None:
                    load_start = frame * len(clean_parts) * 18 + part * 18
                    for axis in range(6):
                        total = float(frame_values[start + 9 + axis])
                        summed = sum(float(load_values[load_start + term * 6 + axis]) for term in range(3))
                        tolerance = .01 if axis < 3 else .002
                        if not math.isclose(total, summed, rel_tol=1e-5, abs_tol=tolerance):
                            raise ValueError("load diagnostics do not reconstruct the recorded wrench")
                if peak < 0:
                    raise ValueError(
                        f"frames.bin peak stress is negative at frame {frame}, part {part}"
                    )
                if clean_parts[part]["analysis_status"] != "visual_only":
                    registered_maximum = max(registered_maximum, peak)
        if not math.isclose(
            registered_maximum, max_stress, rel_tol=2e-6, abs_tol=1e-3
        ):
            raise ValueError("max_von_mises_pa does not match recorded part peaks")

        self._info = {
            "enabled": True,
            "schema_version": 1,
            "fps": fps,
            "frame_count": frame_count,
            "base_speed": base_speed,
            "frame_times_s": times,
            "frame_stride_floats": frame_stride,
            "part_stride_floats": part_stride,
            "geometry_stride_floats": geometry_stride,
            "fem_stride_floats": fem_stride,
            "max_von_mises_pa": max_stress,
            "parts": clean_parts,
            "fem_part_id": fem_part_id,
            "fem_surface_vertex_count": fem_surface_count,
            "fem_cutaway_vertex_count": fem_cutaway_count,
            "scope": scope,
            "assumptions": assumptions,
            "asset_urls": {name: f"/structural-scene/{name}" for name in allowed_assets},
        }
        if load_info:
            self._info["load_diagnostics"] = load_info

    def info(self) -> dict[str, Any]:
        return self._info

    def asset(self, name: str) -> bytes | None:
        return self._assets.get(name)


__all__ = ["ASSET_NAMES", "StructuralSceneAssets"]
