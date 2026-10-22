"""Pinned upstream assets stay outside this repository."""
import hashlib
import math
import os
from pathlib import Path
import subprocess

REVISION = "cda5236f98383f6fbaa4820c6b18dba892cd087f"
REPOSITORY = "https://github.com/google-deepmind/mujoco_menagerie.git"
MODEL_DIRECTORY = "rainbow_robotics_rby1"
MODEL_FILE = "rby1a_1.2.xml"
FINGER_PROXY_CENTER_M = (0.0, 0.0, -0.027)
FINGER_PROXY_HALF_EXTENTS_M = (0.003, 0.014, 0.025)


def _git(checkout, *args):
    return subprocess.check_output(
        ["git", "-C", str(checkout), *args], text=True, stderr=subprocess.STDOUT
    ).strip()


def _normalized_repository(value):
    return value.removesuffix(".git").rstrip("/")


def validate_checkout(checkout, expected_revision=REVISION, expected_repository=REPOSITORY):
    """Reject an asset checkout whose identity or worktree no longer matches the pin."""
    checkout = Path(checkout)
    if not (checkout / ".git").exists():
        raise ValueError(f"Asset checkout is not a Git worktree: {checkout}")
    revision = _git(checkout, "rev-parse", "HEAD")
    if revision != expected_revision:
        raise ValueError(f"Asset revision mismatch: {revision}")
    repository = _git(checkout, "remote", "get-url", "origin")
    if _normalized_repository(repository) != _normalized_repository(expected_repository):
        raise ValueError(f"Asset repository mismatch: {repository}")
    dirty = _git(checkout, "status", "--porcelain=v1", "--untracked-files=all")
    if dirty:
        first_entry = dirty.splitlines()[0]
        raise ValueError(f"Asset checkout has local changes: {first_entry}")
    return {"repository": repository, "revision": revision, "clean": True}


def _obj_bounds(path):
    vertices = []
    for line in Path(path).read_text().splitlines():
        if line.startswith("v "):
            vertex = tuple(float(value) for value in line.split()[1:4])
            if len(vertex) != 3 or not all(math.isfinite(value) for value in vertex):
                raise ValueError(f"Invalid OBJ vertex in {path}")
            vertices.append(vertex)
    if not vertices:
        raise ValueError(f"OBJ contains no vertices: {path}")
    minimum = tuple(min(vertex[axis] for vertex in vertices) for axis in range(3))
    maximum = tuple(max(vertex[axis] for vertex in vertices) for axis in range(3))
    return minimum, maximum


def finger_proxy_comparison(path):
    """Return local-frame AABB measurements for the synthetic contact proxy."""
    path = Path(path)
    components = {}
    for name in ("EE_FINGER_0.obj", "EE_FINGER_1.obj"):
        minimum, maximum = _obj_bounds(path / "assets" / name)
        components[name] = {
            "minimum_m": minimum,
            "maximum_m": maximum,
            "extent_m": tuple(high - low for low, high in zip(minimum, maximum)),
        }
    center = FINGER_PROXY_CENTER_M
    half = FINGER_PROXY_HALF_EXTENTS_M
    proxy_minimum = tuple(round(value - radius, 9) for value, radius in zip(center, half))
    proxy_maximum = tuple(round(value + radius, 9) for value, radius in zip(center, half))
    proxy_extent = tuple(round(2 * radius, 9) for radius in half)
    contact_mesh = components["EE_FINGER_0.obj"]
    contact_center = tuple(
        (low + high) / 2
        for low, high in zip(contact_mesh["minimum_m"], contact_mesh["maximum_m"])
    )
    return {
        "frame": "finger body local coordinates",
        "method": "axis-aligned bounding-box comparison; not a surface-fit metric",
        "proxy": {
            "center_m": center,
            "half_extents_m": half,
            "minimum_m": proxy_minimum,
            "maximum_m": proxy_maximum,
            "extent_m": proxy_extent,
        },
        "source_meshes": components,
        "contact_mesh_reference": "EE_FINGER_0.obj",
        "contact_mesh_extent_ratio": tuple(
            round(proxy / source, 9)
            for proxy, source in zip(proxy_extent, contact_mesh["extent_m"])
        ),
        "contact_mesh_center_offset_m": tuple(
            round(proxy - source, 9) for proxy, source in zip(center, contact_center)
        ),
    }


def asset_directory(fetch=False):
    cache = Path(os.environ.get("RBY1_CACHE", Path.home() / ".cache/rby1-tray-lab"))
    checkout = cache / "menagerie"
    if not checkout.exists():
        if not fetch:
            raise FileNotFoundError("Run python -m rby1_tray.cli fetch-assets first")
        cache.mkdir(parents=True, exist_ok=True)
        subprocess.run(["git", "clone", "--filter=blob:none", "--no-checkout", REPOSITORY, str(checkout)], check=True)
        subprocess.run(["git", "-C", str(checkout), "sparse-checkout", "set", MODEL_DIRECTORY], check=True)
        subprocess.run(["git", "-C", str(checkout), "checkout", REVISION], check=True)
    validate_checkout(checkout)
    path = checkout / MODEL_DIRECTORY
    if not (path / MODEL_FILE).is_file():
        raise FileNotFoundError("Pinned RB-Y1 model is absent")
    return path


def provenance(path):
    path = Path(path)
    checkout = path.parent
    identity = validate_checkout(checkout)
    license_text = (path / "LICENSE").read_text()
    if "Apache License" not in license_text or "Version 2.0" not in license_text:
        raise ValueError("Upstream LICENSE is not recognizable as Apache-2.0")
    files = [path / MODEL_FILE, path / "LICENSE", *sorted((path / "assets").glob("*"))]
    return {
        "repository": identity["repository"],
        "revision": identity["revision"],
        "worktree_clean": identity["clean"],
        "license": "Apache-2.0",
        "model": "RBY1-A 1.2",
        "files": {
            str(file.relative_to(path)): hashlib.sha256(file.read_bytes()).hexdigest()
            for file in files if file.is_file()
        },
        "finger_proxy_comparison": finger_proxy_comparison(path),
    }
