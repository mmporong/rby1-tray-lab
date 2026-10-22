"""RGB-D tray pose estimation without simulator state access.

The estimator consumes only camera observations and calibration.  It is
therefore usable with a hardware RGB-D driver that produces the same contract.
MuJoCo object names, geometry ids, qpos, and ground-truth poses are deliberately
absent from this module.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math

import numpy as np
from scipy import ndimage
from scipy.spatial import ConvexHull, QhullError


class PerceptionError(ValueError):
    """Raised when an observation cannot support a defensible tray estimate."""


@dataclass(frozen=True)
class TrayCAD:
    """Replaceable tray dimensions, expressed in metres."""

    half_extents_m: tuple[float, float, float] = (0.18, 0.27, 0.012)
    rim_width_m: float = 0.006
    rim_height_m: float = 0.05
    grasp_height_m: float = 0.055

    def __post_init__(self) -> None:
        if len(self.half_extents_m) != 3 or not all(
            np.isfinite(value) and value > 0 for value in self.half_extents_m
        ):
            raise ValueError("tray half extents must contain three positive finite values")
        if not np.isfinite(self.rim_width_m) or self.rim_width_m <= 0:
            raise ValueError("rim_width_m must be positive and finite")
        if not np.isfinite(self.rim_height_m) or self.rim_height_m <= 0:
            raise ValueError("rim_height_m must be positive and finite")
        if not 0 < self.grasp_height_m <= self.half_extents_m[2] + self.rim_height_m:
            raise ValueError("grasp height must lie on the tray rim")

    @property
    def outer_dimensions_m(self) -> np.ndarray:
        hx, hy, _ = self.half_extents_m
        return np.array([2 * (hx + self.rim_width_m),
                         2 * (hy + self.rim_width_m)], dtype=float)


@dataclass(frozen=True)
class BlueMaskConfig:
    """Replaceable colour-segmentation parameters for a measured camera."""

    min_blue: int = 70
    min_blue_red_delta: int = 35
    min_blue_green_delta: int = 18
    min_saturation: float = 0.45
    morphology_kernel_px: int = 3

    def __post_init__(self) -> None:
        values = (self.min_blue, self.min_blue_red_delta, self.min_blue_green_delta)
        if not all(isinstance(value, int) and 0 <= value <= 255 for value in values):
            raise ValueError("blue-mask thresholds must be integer values in [0, 255]")
        if not np.isfinite(self.min_saturation) or not 0 <= self.min_saturation <= 1:
            raise ValueError("min_saturation must be finite and in [0, 1]")
        if self.morphology_kernel_px < 1 or self.morphology_kernel_px % 2 == 0:
            raise ValueError("morphology_kernel_px must be a positive odd integer")


@dataclass(frozen=True)
class GraspEndpoint:
    side: str
    position_world_m: tuple[float, float, float]
    yaw_world_rad: float
    contact_surface: str = "native_tray_rim"


@dataclass(frozen=True)
class TrayEstimate:
    center_world_m: tuple[float, float, float]
    yaw_rad: float
    dimensions_m: tuple[float, float]
    covariance_xyzyaw: tuple[tuple[float, ...], ...]
    plane_normal_world: tuple[float, float, float]
    pixel_count: int
    quality: dict[str, float | bool | str]
    grasps: tuple[GraspEndpoint, GraspEndpoint]
    frame_convention: str = (
        "world pose; camera optical frame is +X right, +Y down, +Z forward"
    )
    yaw_ambiguity: str = (
        "rectangle appearance is invariant under yaw+pi; yaw is canonicalized "
        "so the estimated short axis points toward world +X (then +Y at tie)"
    )

    def to_dict(self) -> dict:
        return asdict(self)


def validate_calibration(K: np.ndarray, T_world_optical: np.ndarray) -> None:
    K = np.asarray(K, dtype=float)
    transform = np.asarray(T_world_optical, dtype=float)
    if K.shape != (3, 3) or not np.all(np.isfinite(K)):
        raise PerceptionError("K must be a finite 3x3 intrinsic matrix")
    if K[0, 0] <= 0 or K[1, 1] <= 0 or not np.isclose(K[2, 2], 1):
        raise PerceptionError("K must have positive focal lengths and K[2,2]=1")
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise PerceptionError("T_world_optical must be a finite 4x4 transform")
    if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8):
        raise PerceptionError("T_world_optical has an invalid homogeneous row")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(
        np.linalg.det(rotation), 1, atol=1e-6
    ):
        raise PerceptionError("T_world_optical rotation must be proper and orthonormal")


def backproject_pixels(
    uv: np.ndarray,
    axial_depth_m: np.ndarray,
    K: np.ndarray,
    T_world_optical: np.ndarray,
) -> np.ndarray:
    """Back-project pixels using optical-axis Z depth, then transform to world."""

    validate_calibration(K, T_world_optical)
    uv = np.asarray(uv, dtype=float)
    depth = np.asarray(axial_depth_m, dtype=float)
    if uv.ndim != 2 or uv.shape[1] != 2 or depth.shape != (len(uv),):
        raise PerceptionError("uv must be Nx2 and axial_depth_m must be length N")
    if not np.all(np.isfinite(uv)) or not np.all(np.isfinite(depth)) or np.any(depth <= 0):
        raise PerceptionError("pixels and depth must be finite; depth must be positive")
    inverse = np.linalg.inv(np.asarray(K, dtype=float))
    rays = (inverse @ np.column_stack([uv, np.ones(len(uv))]).T).T
    optical = rays * depth[:, None]
    homogeneous = np.column_stack([optical, np.ones(len(optical))])
    return (np.asarray(T_world_optical, dtype=float) @ homogeneous.T).T[:, :3]


def _blue_mask(rgb: np.ndarray, config: BlueMaskConfig) -> np.ndarray:
    values = rgb.astype(np.int16)
    red, green, blue = values[..., 0], values[..., 1], values[..., 2]
    maximum = values.max(axis=2)
    minimum = values.min(axis=2)
    saturation = np.divide(
        maximum - minimum,
        maximum,
        out=np.zeros_like(maximum, dtype=float),
        where=maximum > 0,
    )
    raw = (
        (blue >= config.min_blue)
        & (blue - red >= config.min_blue_red_delta)
        & (blue - green >= config.min_blue_green_delta)
        & (saturation >= config.min_saturation)
    )
    # Remove isolated antialiasing/noise while retaining holes made by payloads.
    structure = np.ones(
        (config.morphology_kernel_px, config.morphology_kernel_px), dtype=bool
    )
    clean = ndimage.binary_opening(raw, structure=structure)
    clean = ndimage.binary_closing(clean, structure=structure)
    return clean


def _candidate_components(mask: np.ndarray, min_pixels: int) -> list[np.ndarray]:
    labels, count = ndimage.label(mask)
    if count == 0:
        raise PerceptionError("no blue tray candidate was detected")
    sizes = np.bincount(labels.ravel())
    sizes[0] = 0
    candidates = [index for index in np.argsort(sizes)[::-1] if sizes[index] >= min_pixels]
    if not candidates:
        largest = int(sizes.max()) if len(sizes) else 0
        raise PerceptionError(
            f"blue candidate has {largest} pixels; minimum is {min_pixels}"
        )
    return [labels == int(index) for index in candidates]


def _normalise_yaw_pi(yaw: float) -> float:
    return float((yaw + math.pi / 2) % math.pi - math.pi / 2)


def _robust_floor_plane(
    points: np.ndarray, *, inlier_threshold_m: float = 0.004
) -> tuple[np.ndarray, np.ndarray, float]:
    """Fit the dominant horizontal surface while rejecting raised rim points."""

    if len(points) < 12:
        raise PerceptionError("insufficient points for robust plane fit")
    z = points[:, 2]
    # Deterministic one-point RANSAC under the known horizontal-floor prior.
    # Candidate heights are quantiles, avoiding random-state sensitivity.
    hypotheses = np.unique(np.quantile(z, np.linspace(0.02, 0.98, 65)))
    counts = np.array([np.count_nonzero(np.abs(z - value) <= inlier_threshold_m)
                       for value in hypotheses])
    seed_z = float(hypotheses[int(np.argmax(counts))])
    inliers = np.abs(z - seed_z) <= inlier_threshold_m
    if np.count_nonzero(inliers) < max(12, int(0.30 * len(points))):
        raise PerceptionError("dominant tray-floor plane has insufficient inlier coverage")

    floor = points[inliers]
    center = np.mean(floor, axis=0)
    _, _, vh = np.linalg.svd(floor - center, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    if normal[2] < math.cos(math.radians(18)):
        raise PerceptionError("tray candidate plane is not horizontal")
    residual = (points - center) @ normal
    refined = np.abs(residual) <= inlier_threshold_m
    floor = points[refined]
    center = np.mean(floor, axis=0)
    _, _, vh = np.linalg.svd(floor - center, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    residual = (floor - center) @ normal
    rms = float(np.sqrt(np.mean(residual**2)))
    return normal, refined, rms


def _fit_cad_footprint(
    xy: np.ndarray, expected: np.ndarray, *, step_degrees: float = 0.25
) -> dict[str, np.ndarray | float]:
    """Fit rectangle yaw by CAD dimensions rather than payload-biased moments."""

    try:
        hull_vertices = xy[ConvexHull(xy).vertices]
    except QhullError as error:
        raise PerceptionError("tray footprint does not form a valid 2D hull") from error
    angles = np.deg2rad(np.arange(-90.0, 90.0, step_degrees))
    best = None
    for yaw in angles:
        short = np.array([math.cos(yaw), math.sin(yaw)])
        long = np.array([-short[1], short[0]])
        axes = np.column_stack([short, long])
        projected = xy @ axes
        lower, upper = np.percentile(projected, [0.5, 99.5], axis=0)
        dimensions = upper - lower
        relative = (dimensions - expected) / expected
        area_error = (np.prod(dimensions) - np.prod(expected)) / np.prod(expected)
        clipped_hull = np.clip(hull_vertices @ axes, lower, upper)
        try:
            hull_area = float(ConvexHull(clipped_hull).volume)
        except QhullError:
            continue
        rectangle_area = float(np.prod(dimensions))
        hull_coverage = hull_area / rectangle_area if rectangle_area > 0 else 0.0
        score = float(
            relative @ relative
            + 0.25 * area_error**2
            + 10.0 * (1.0 - min(hull_coverage, 1.0)) ** 2
        )
        if best is None or score < best["score"]:
            best = {
                "yaw": float(yaw), "axes": axes, "lower": lower, "upper": upper,
                "dimensions": dimensions, "score": score,
                "hull_coverage": hull_coverage,
            }
    if best is None:
        raise PerceptionError("tray footprint sweep did not produce a valid rectangle")
    return best


def estimate_tray_pose(
    rgb: np.ndarray,
    depth_m: np.ndarray,
    K: np.ndarray,
    T_world_optical: np.ndarray,
    *,
    cad: TrayCAD = TrayCAD(),
    blue_mask: BlueMaskConfig = BlueMaskConfig(),
    min_pixels: int = 180,
    dimension_relative_tolerance: float = 0.15,
    max_plane_rms_m: float = 0.02,
    minimum_hull_coverage: float = 0.60,
) -> TrayEstimate:
    """Estimate a horizontal blue tray pose and its native-rim grasp endpoints.

    The known CAD dimensions are used only as a candidate-validation gate and
    to define grasp locations.  The pose itself comes from RGB-D observations;
    there is no ground-truth fallback.
    """

    rgb = np.asarray(rgb)
    depth = np.asarray(depth_m, dtype=float)
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise PerceptionError("rgb must be an HxWx3 uint8 array")
    if depth.shape != rgb.shape[:2]:
        raise PerceptionError("depth_m must match the RGB image height and width")
    validate_calibration(K, T_world_optical)
    if min_pixels < 12:
        raise PerceptionError("min_pixels must be at least 12")
    valid_depth = np.isfinite(depth) & (depth > 0)
    expected = cad.outer_dimensions_m
    fit_candidates = []
    rejection_reasons = []
    components = _candidate_components(_blue_mask(rgb, blue_mask) & valid_depth, min_pixels)
    for component in components:
        rows, cols = np.nonzero(component)
        if rows.min() <= 1 or cols.min() <= 1 or rows.max() >= rgb.shape[0] - 2 or cols.max() >= rgb.shape[1] - 2:
            rejection_reasons.append("candidate touches image boundary")
            continue
        points = backproject_pixels(
            np.column_stack([cols.astype(float), rows.astype(float)]),
            depth[rows, cols], K, T_world_optical,
        )
        try:
            normal, floor_inliers, plane_rms = _robust_floor_plane(points)
            footprint = _fit_cad_footprint(points[:, :2], expected)
        except PerceptionError as error:
            rejection_reasons.append(str(error))
            continue
        dimensions = np.asarray(footprint["dimensions"])
        relative_error = np.abs(dimensions - expected) / expected
        hull_coverage = float(footprint["hull_coverage"])
        if plane_rms > max_plane_rms_m:
            rejection_reasons.append(f"plane RMS {plane_rms:.4f} m exceeds gate")
            continue
        if np.any(relative_error > dimension_relative_tolerance):
            rejection_reasons.append(
                f"dimensions {dimensions.tolist()} m failed CAD gate"
            )
            continue
        if hull_coverage < minimum_hull_coverage:
            rejection_reasons.append(
                f"footprint hull coverage {hull_coverage:.3f} is incomplete"
            )
            continue
        fit_candidates.append((float(footprint["score"]), component, rows, cols,
                               points, normal, floor_inliers, plane_rms,
                               footprint, relative_error))
    if not fit_candidates:
        detail = "; ".join(rejection_reasons[:3]) or "no valid component"
        raise PerceptionError(f"no complete tray candidate passed geometry gates: {detail}")
    (_, component, rows, cols, points, normal, floor_inliers, plane_rms,
     footprint, relative_error) = min(fit_candidates, key=lambda item: item[0])
    axes = np.asarray(footprint["axes"])
    dimensions = np.asarray(footprint["dimensions"])
    center_xy = axes @ ((np.asarray(footprint["lower"]) + np.asarray(footprint["upper"])) / 2)
    hull_coverage = float(footprint["hull_coverage"])
    floor_surface_z = float(np.median(points[floor_inliers, 2]))
    center_z = floor_surface_z - cad.half_extents_m[2]
    yaw = _normalise_yaw_pi(float(footprint["yaw"]))

    pixel_scale = float(np.median(depth[rows, cols]) / max(K[0, 0], K[1, 1]))
    count = len(points)
    coverage_penalty = max(0.0, 1.0 - hull_coverage)
    sigma_xy = max(pixel_scale, np.max(np.abs(dimensions - expected)) / math.sqrt(12),
                   coverage_penalty * 0.01)
    sigma_z = max(plane_rms / math.sqrt(count), 0.0005)
    sigma_yaw = max(math.radians(0.25), float(np.max(relative_error)) * 0.08,
                    coverage_penalty * 0.05)
    # This matrix is a conservative fit-quality proxy, not a calibrated sensor
    # covariance.  Its sources and exclusions are exported alongside it below.
    covariance = np.diag([sigma_xy**2, sigma_xy**2, sigma_z**2, sigma_yaw**2])
    quality_score = float(np.clip(
        1.0
        - 0.5 * np.max(relative_error) / dimension_relative_tolerance
        - 0.5 * plane_rms / max_plane_rms_m,
        0.0,
        1.0,
    ))

    rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
    grasp_yaw = _normalise_yaw_pi(yaw + math.pi / 2)
    endpoints = []
    for side, sign in (("left", 1), ("right", -1)):
        offset_xy = rotation @ np.array([0.0, sign * cad.half_extents_m[1]])
        position = (center_xy[0] + offset_xy[0], center_xy[1] + offset_xy[1],
                    center_z + cad.grasp_height_m)
        endpoints.append(GraspEndpoint(side, tuple(float(v) for v in position), grasp_yaw))

    return TrayEstimate(
        center_world_m=tuple(float(v) for v in (*center_xy, center_z)),
        yaw_rad=yaw,
        dimensions_m=tuple(float(v) for v in dimensions),
        covariance_xyzyaw=tuple(tuple(float(v) for v in row) for row in covariance),
        plane_normal_world=tuple(float(v) for v in normal),
        pixel_count=count,
        quality={
            "valid": True,
            "source": "rgbd_observation",
            "mask": "blue_contrast_connected_component",
            "mask_parameters": (
                f"B>={blue_mask.min_blue},B-R>={blue_mask.min_blue_red_delta},"
                f"B-G>={blue_mask.min_blue_green_delta},"
                f"S>={blue_mask.min_saturation:.3f},"
                f"kernel={blue_mask.morphology_kernel_px}px"
            ),
            "plane_fit": "deterministic horizontal RANSAC plus PCA refinement",
            "plane_inlier_ratio": float(np.mean(floor_inliers)),
            "footprint_fit": "CAD dimension sweep at 0.25 degree resolution",
            "footprint_hull_coverage": hull_coverage,
            "candidate_count": len(components),
            "geometry_fit_score": float(footprint["score"]),
            "plane_rms_m": plane_rms,
            "max_dimension_relative_error": float(np.max(relative_error)),
            "pixel_scale_m": pixel_scale,
            "score_0_to_1": quality_score,
            "uncertainty_semantics": "heuristic_uncertainty_proxy_not_calibrated_covariance",
            "uncertainty_sources": (
                "pixel_scale,CAD_dimension_residual,plane_fit_residual,"
                "footprint_hull_coverage,yaw_sweep_resolution"
            ),
            "uncertainty_excludes": (
                "camera_intrinsic_bias,camera_extrinsic_bias,depth_scale_bias,"
                "RGB_depth_registration_bias,clock_synchronization_error,"
                "model_to_hardware_systematic_error"
            ),
        },
        grasps=tuple(endpoints),
    )
