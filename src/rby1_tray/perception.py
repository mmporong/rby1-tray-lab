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
    quality: dict[str, float | bool | str | None]
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


def _estimate_from_components(
    depth: np.ndarray,
    K: np.ndarray,
    T_world_optical: np.ndarray,
    components: list[np.ndarray],
    *,
    cad: TrayCAD,
    dimension_relative_tolerance: float,
    max_plane_rms_m: float,
    minimum_hull_coverage: float,
    source: str,
    mask: str,
    mask_parameters: str,
) -> TrayEstimate:
    """Apply common plane, CAD, uncertainty, and grasp gates to masks."""

    expected = cad.outer_dimensions_m
    fit_candidates = []
    rejection_reasons = []
    for component in components:
        rows, cols = np.nonzero(component)
        if (
            rows.min() <= 1
            or cols.min() <= 1
            or rows.max() >= depth.shape[0] - 2
            or cols.max() >= depth.shape[1] - 2
        ):
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
        image_bbox_area = int(rows.max() - rows.min() + 1) * int(
            cols.max() - cols.min() + 1
        )
        image_fill_ratio = len(rows) / image_bbox_area
        fill_penalty = 0.0
        if source == "depth_cad_observation":
            # A thin raised rim can have the correct convex hull while not
            # supporting the tray floor. Prefer the filled CAD surface when
            # several height slabs describe the same outer rectangle.
            fill_penalty = 0.5 * max(0.0, 1.0 - image_fill_ratio / 0.50) ** 2
        fit_candidates.append((
            float(footprint["score"]) + fill_penalty,
            rows, cols, points, normal, floor_inliers, plane_rms, footprint,
            relative_error, image_fill_ratio,
        ))
    if not fit_candidates:
        detail = "; ".join(rejection_reasons[:3]) or "no valid component"
        raise PerceptionError(f"no complete tray candidate passed geometry gates: {detail}")

    candidate_centers = []
    for candidate in fit_candidates:
        candidate_footprint = candidate[7]
        candidate_axes = np.asarray(candidate_footprint["axes"])
        candidate_center = candidate_axes @ (
            (
                np.asarray(candidate_footprint["lower"])
                + np.asarray(candidate_footprint["upper"])
            )
            / 2
        )
        candidate_centers.append(candidate_center)
    merge_distance_m = 0.25 * float(np.min(expected))
    unassigned = set(range(len(fit_candidates)))
    spatial_hypotheses: list[list[int]] = []
    while unassigned:
        seed = unassigned.pop()
        hypothesis = {seed}
        frontier = [seed]
        while frontier:
            current = frontier.pop()
            neighbours = {
                index
                for index in unassigned
                if np.linalg.norm(
                    candidate_centers[current] - candidate_centers[index]
                )
                <= merge_distance_m
            }
            hypothesis.update(neighbours)
            frontier.extend(neighbours)
            unassigned.difference_update(neighbours)
        spatial_hypotheses.append(sorted(hypothesis))

    hypothesis_best = [
        min((fit_candidates[index] for index in hypothesis), key=lambda item: item[0])
        for hypothesis in spatial_hypotheses
    ]
    hypothesis_best.sort(key=lambda item: item[0])
    if len(hypothesis_best) > 1:
        score_margin = float(hypothesis_best[1][0] - hypothesis_best[0][0])
        raise PerceptionError(
            "multiple spatial tray hypotheses passed geometry gates: "
            f"count={len(hypothesis_best)}, score_margin={score_margin:.6f}; "
            "selection_prior=none_ambiguity_reject"
        )

    (_, rows, cols, points, normal, floor_inliers, plane_rms,
     footprint, relative_error, image_fill_ratio) = hypothesis_best[0]
    axes = np.asarray(footprint["axes"])
    dimensions = np.asarray(footprint["dimensions"])
    center_xy = axes @ (
        (np.asarray(footprint["lower"]) + np.asarray(footprint["upper"])) / 2
    )
    hull_coverage = float(footprint["hull_coverage"])
    floor_surface_z = float(np.median(points[floor_inliers, 2]))
    center_z = floor_surface_z - cad.half_extents_m[2]
    yaw = _normalise_yaw_pi(float(footprint["yaw"]))

    pixel_scale = float(np.median(depth[rows, cols]) / max(K[0, 0], K[1, 1]))
    count = len(points)
    coverage_penalty = max(0.0, 1.0 - hull_coverage)
    sigma_xy = max(
        pixel_scale,
        np.max(np.abs(dimensions - expected)) / math.sqrt(12),
        coverage_penalty * 0.01,
    )
    sigma_z = max(plane_rms / math.sqrt(count), 0.0005)
    sigma_yaw = max(
        math.radians(0.25),
        float(np.max(relative_error)) * 0.08,
        coverage_penalty * 0.05,
    )
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
        position = (
            center_xy[0] + offset_xy[0],
            center_xy[1] + offset_xy[1],
            center_z + cad.grasp_height_m,
        )
        endpoints.append(
            GraspEndpoint(side, tuple(float(v) for v in position), grasp_yaw)
        )

    return TrayEstimate(
        center_world_m=tuple(float(v) for v in (*center_xy, center_z)),
        yaw_rad=yaw,
        dimensions_m=tuple(float(v) for v in dimensions),
        covariance_xyzyaw=tuple(
            tuple(float(v) for v in row) for row in covariance
        ),
        plane_normal_world=tuple(float(v) for v in normal),
        pixel_count=count,
        quality={
            "valid": True,
            "source": source,
            "mask": mask,
            "mask_parameters": mask_parameters,
            "plane_fit": "deterministic horizontal RANSAC plus PCA refinement",
            "plane_inlier_ratio": float(np.mean(floor_inliers)),
            "footprint_fit": "CAD dimension sweep at 0.25 degree resolution",
            "footprint_hull_coverage": hull_coverage,
            "image_component_fill_ratio": float(image_fill_ratio),
            "candidate_count": len(components),
            "hypothesis_count": len(hypothesis_best),
            "score_margin": None,
            "selection_prior": "none_ambiguity_reject",
            "hypothesis_merge_distance_m": merge_distance_m,
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


def _depth_geometry_components(
    depth: np.ndarray,
    K: np.ndarray,
    T_world_optical: np.ndarray,
    *,
    min_pixels: int,
    height_bin_m: float,
    height_band_m: float,
) -> list[np.ndarray]:
    """Find bounded horizontal depth regions without using RGB labels."""

    valid = np.isfinite(depth) & (depth > 0)
    rows, cols = np.nonzero(valid)
    if len(rows) < min_pixels:
        raise PerceptionError("insufficient valid depth pixels for tray detection")
    points = backproject_pixels(
        np.column_stack([cols.astype(float), rows.astype(float)]),
        depth[rows, cols], K, T_world_optical,
    )
    world_z = np.full(depth.shape, np.nan, dtype=float)
    world_z[rows, cols] = points[:, 2]
    finite_z = points[:, 2]
    lower, upper = np.quantile(finite_z, [0.002, 0.998])
    if upper - lower < height_bin_m:
        centers = np.array([float(np.median(finite_z))])
    else:
        edges = np.arange(lower, upper + 2 * height_bin_m, height_bin_m)
        counts, edges = np.histogram(finite_z, bins=edges)
        previous_counts = np.concatenate(([0], counts[:-1]))
        next_counts = np.concatenate((counts[1:], [0]))
        peak_indices = np.flatnonzero(
            (counts >= min_pixels)
            & (counts >= previous_counts)
            & (counts >= next_counts)
        )
        order = peak_indices[np.argsort(counts[peak_indices])[::-1]]
        all_centers = (edges[:-1] + edges[1:]) / 2
        centers = all_centers[order]

    candidates: list[np.ndarray] = []
    for center in centers:
        slab = valid & (np.abs(world_z - center) <= height_band_m)
        labels, count = ndimage.label(slab)
        sizes = np.bincount(labels.ravel())
        for index in range(1, count + 1):
            if sizes[index] < min_pixels:
                continue
            component = labels == index
            if any(
                np.count_nonzero(component & existing)
                / min(np.count_nonzero(component), np.count_nonzero(existing))
                > 0.80
                for existing in candidates
            ):
                continue
            candidates.append(component)
    if not candidates:
        raise PerceptionError("no connected horizontal depth candidate was detected")
    return candidates


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
    components = _candidate_components(_blue_mask(rgb, blue_mask) & valid_depth, min_pixels)
    return _estimate_from_components(
        depth,
        K,
        T_world_optical,
        components,
        cad=cad,
        dimension_relative_tolerance=dimension_relative_tolerance,
        max_plane_rms_m=max_plane_rms_m,
        minimum_hull_coverage=minimum_hull_coverage,
        source="rgbd_observation",
        mask="blue_contrast_connected_component",
        mask_parameters=(
            f"B>={blue_mask.min_blue},B-R>={blue_mask.min_blue_red_delta},"
            f"B-G>={blue_mask.min_blue_green_delta},"
            f"S>={blue_mask.min_saturation:.3f},"
            f"kernel={blue_mask.morphology_kernel_px}px"
        ),
    )


def estimate_tray_pose_from_depth(
    depth_m: np.ndarray,
    K: np.ndarray,
    T_world_optical: np.ndarray,
    *,
    cad: TrayCAD = TrayCAD(),
    min_pixels: int = 180,
    height_bin_m: float = 0.004,
    height_band_m: float = 0.006,
    dimension_relative_tolerance: float = 0.15,
    max_plane_rms_m: float = 0.02,
    minimum_hull_coverage: float = 0.60,
) -> TrayEstimate:
    """Estimate tray pose from registered depth geometry and known CAD only.

    Horizontal world-height slabs generate candidates. Connected components
    still have to pass complete-footprint, plane, and CAD-dimension gates; the
    observation is rejected when depth does not support those gates.
    """

    depth = np.asarray(depth_m, dtype=float)
    if depth.ndim != 2:
        raise PerceptionError("depth_m must be a 2D array")
    validate_calibration(K, T_world_optical)
    if min_pixels < 12:
        raise PerceptionError("min_pixels must be at least 12")
    if not np.isfinite(height_bin_m) or height_bin_m <= 0:
        raise PerceptionError("height_bin_m must be positive and finite")
    if not np.isfinite(height_band_m) or height_band_m < height_bin_m:
        raise PerceptionError("height_band_m must be finite and at least height_bin_m")
    components = _depth_geometry_components(
        depth,
        K,
        T_world_optical,
        min_pixels=min_pixels,
        height_bin_m=height_bin_m,
        height_band_m=height_band_m,
    )
    return _estimate_from_components(
        depth,
        K,
        T_world_optical,
        components,
        cad=cad,
        dimension_relative_tolerance=dimension_relative_tolerance,
        max_plane_rms_m=max_plane_rms_m,
        minimum_hull_coverage=minimum_hull_coverage,
        source="depth_cad_observation",
        mask="world_height_connected_component",
        mask_parameters=(
            f"height_bin={height_bin_m:.4f}m,height_band={height_band_m:.4f}m"
        ),
    )


def validate_tray_estimate_update(
    previous: TrayEstimate,
    candidate: TrayEstimate,
    *,
    max_position_innovation_m: float = 0.04,
    max_yaw_innovation_rad: float = math.radians(6.0),
) -> dict[str, float | bool]:
    """Reject a re-observation that jumps beyond physical innovation gates."""

    if not np.isfinite(max_position_innovation_m) or max_position_innovation_m <= 0:
        raise PerceptionError("max_position_innovation_m must be positive and finite")
    if not np.isfinite(max_yaw_innovation_rad) or max_yaw_innovation_rad <= 0:
        raise PerceptionError("max_yaw_innovation_rad must be positive and finite")
    previous_center = np.asarray(previous.center_world_m, dtype=float)
    candidate_center = np.asarray(candidate.center_world_m, dtype=float)
    if (
        previous_center.shape != (3,)
        or candidate_center.shape != (3,)
        or not np.all(np.isfinite(previous_center))
        or not np.all(np.isfinite(candidate_center))
        or not np.isfinite(previous.yaw_rad)
        or not np.isfinite(candidate.yaw_rad)
    ):
        raise PerceptionError("tray estimates contain non-finite pose values")
    position_innovation = float(np.linalg.norm(candidate_center - previous_center))
    yaw_innovation = abs(_normalise_yaw_pi(candidate.yaw_rad - previous.yaw_rad))
    if position_innovation > max_position_innovation_m:
        raise PerceptionError(
            f"tray position innovation {position_innovation:.4f} m exceeds "
            f"{max_position_innovation_m:.4f} m gate"
        )
    if yaw_innovation > max_yaw_innovation_rad:
        raise PerceptionError(
            f"tray yaw innovation {math.degrees(yaw_innovation):.2f} deg exceeds "
            f"{math.degrees(max_yaw_innovation_rad):.2f} deg gate"
        )
    return {
        "accepted": True,
        "position_innovation_m": position_innovation,
        "yaw_innovation_rad": yaw_innovation,
        "max_position_innovation_m": float(max_position_innovation_m),
        "max_yaw_innovation_rad": float(max_yaw_innovation_rad),
    }


def confirm_grasp_rim_from_depth(
    depth_m: np.ndarray,
    K: np.ndarray,
    T_world_optical: np.ndarray,
    expected_grasp_world_m: np.ndarray,
    tray_center_world_m: np.ndarray,
    tray_yaw_rad: float,
    *,
    cad: TrayCAD = TrayCAD(),
    roi_radius_px: int = 24,
    rim_plane_tolerance_m: float = 0.012,
    rim_height_tolerance_m: float = 0.020,
    tangent_half_span_m: float = 0.040,
    minimum_valid_depth_fraction: float = 0.50,
    minimum_geometry_support_fraction: float = 0.15,
    minimum_geometry_support_pixels: int = 20,
    minimum_tangent_span_m: float = 0.008,
    minimum_height_span_m: float = 0.006,
) -> dict[str, float | int | bool | str | list[float] | None]:
    """Confirm local native-rim geometry in a narrow wrist depth image.

    The whole tray need not be visible. The expected grasp point is projected
    into the image and local depth returns must support the known long-side
    plane, grasp height, and short tangent/vertical spans around that point.
    No RGB label, simulator geometry id, or object pose fallback is used.
    """

    depth = np.asarray(depth_m, dtype=float)
    if depth.ndim != 2:
        raise PerceptionError("depth_m must be a 2D array")
    validate_calibration(K, T_world_optical)
    expected = np.asarray(expected_grasp_world_m, dtype=float)
    center = np.asarray(tray_center_world_m, dtype=float)
    if expected.shape != (3,) or center.shape != (3,):
        raise PerceptionError("expected grasp and tray center must be 3D points")
    if not np.all(np.isfinite(expected)) or not np.all(np.isfinite(center)):
        raise PerceptionError("expected grasp and tray center must be finite")
    if not np.isfinite(tray_yaw_rad):
        raise PerceptionError("tray_yaw_rad must be finite")
    if not isinstance(roi_radius_px, int) or roi_radius_px < 1:
        raise PerceptionError("roi_radius_px must be a positive integer")
    positive_thresholds = (
        rim_plane_tolerance_m,
        rim_height_tolerance_m,
        tangent_half_span_m,
        minimum_tangent_span_m,
        minimum_height_span_m,
    )
    if not all(np.isfinite(value) and value > 0 for value in positive_thresholds):
        raise PerceptionError("rim geometry thresholds must be positive and finite")
    if not 0 < minimum_valid_depth_fraction <= 1:
        raise PerceptionError("minimum_valid_depth_fraction must lie in (0, 1]")
    if not 0 < minimum_geometry_support_fraction <= 1:
        raise PerceptionError("minimum_geometry_support_fraction must lie in (0, 1]")
    if minimum_geometry_support_pixels < 3:
        raise PerceptionError("minimum_geometry_support_pixels must be at least 3")

    c, s = math.cos(tray_yaw_rad), math.sin(tray_yaw_rad)
    world_to_tray_xy = np.array([[c, s], [-s, c]])
    expected_local_xy = world_to_tray_xy @ (expected[:2] - center[:2])
    expected_local_z = float(expected[2] - center[2])
    side_sign = 1.0 if expected_local_xy[1] >= 0 else -1.0
    rim_side_y = side_sign * cad.half_extents_m[1]
    expected_on_known_rim = bool(
        abs(expected_local_xy[0])
        <= cad.half_extents_m[0] + cad.rim_width_m + tangent_half_span_m
        and abs(expected_local_xy[1] - rim_side_y) <= rim_plane_tolerance_m
        and abs(expected_local_z - cad.grasp_height_m) <= rim_height_tolerance_m
    )

    optical = np.linalg.inv(np.asarray(T_world_optical, dtype=float)) @ np.append(
        expected, 1.0
    )
    if optical[2] > 0:
        homogeneous_pixel = np.asarray(K, dtype=float) @ optical[:3]
        uv = homogeneous_pixel[:2] / homogeneous_pixel[2]
    else:
        uv = np.array([np.nan, np.nan])
    height, width = depth.shape
    in_frame = bool(
        np.all(np.isfinite(uv))
        and 0 <= uv[0] < width
        and 0 <= uv[1] < height
    )
    result: dict[str, float | int | bool | str | list[float] | None] = {
        "passed": False,
        "expected_grasp_in_frame": in_frame,
        "expected_grasp_on_known_rim": expected_on_known_rim,
        "expected_grasp_pixel_uv": [float(value) for value in uv],
        "roi_valid_depth_fraction": 0.0,
        "rim_geometry_support_fraction": 0.0,
        "rim_geometry_support_pixels": 0,
        "rim_plane_rms_m": None,
        "rim_height_median_residual_m": None,
        "rim_tangent_span_m": 0.0,
        "rim_height_span_m": 0.0,
        "source": "wrist_depth_expected_native_rim_geometry",
        "ambiguity": (
            "a single depth view cannot distinguish a perfectly coplanar "
            "occluder that reproduces the expected CAD rim patch"
        ),
        "rejection_reason": "",
    }
    if not expected_on_known_rim:
        result["rejection_reason"] = "expected grasp is not on the known CAD rim"
        return result
    if not in_frame:
        result["rejection_reason"] = "expected grasp projects outside the depth image"
        return result

    col, row = np.rint(uv).astype(int)
    row_min, row_max = max(0, row - roi_radius_px), min(height, row + roi_radius_px + 1)
    col_min, col_max = max(0, col - roi_radius_px), min(width, col + roi_radius_px + 1)
    roi = depth[row_min:row_max, col_min:col_max]
    valid = np.isfinite(roi) & (roi > 0)
    valid_fraction = float(np.mean(valid)) if valid.size else 0.0
    result["roi_valid_depth_fraction"] = valid_fraction
    if valid_fraction < minimum_valid_depth_fraction or not np.any(valid):
        result["rejection_reason"] = "insufficient valid depth near expected rim"
        return result

    roi_rows, roi_cols = np.nonzero(valid)
    image_rows = roi_rows + row_min
    image_cols = roi_cols + col_min
    points = backproject_pixels(
        np.column_stack([image_cols.astype(float), image_rows.astype(float)]),
        roi[roi_rows, roi_cols],
        K,
        T_world_optical,
    )
    local_xy = (points[:, :2] - center[:2]) @ world_to_tray_xy.T
    local_z = points[:, 2] - center[2]
    tangent_window = (
        np.abs(local_xy[:, 0] - expected_local_xy[0]) <= tangent_half_span_m
    )
    height_window = np.abs(local_z - cad.grasp_height_m) <= rim_height_tolerance_m
    plane_residual = np.abs(local_xy[:, 1] - rim_side_y)
    support = tangent_window & height_window & (plane_residual <= rim_plane_tolerance_m)
    support_count = int(np.count_nonzero(support))
    support_fraction = float(support_count / len(points))
    result["rim_geometry_support_pixels"] = support_count
    result["rim_geometry_support_fraction"] = support_fraction
    if support_count:
        supported_x = local_xy[support, 0]
        supported_z = local_z[support]
        result["rim_plane_rms_m"] = float(
            np.sqrt(np.mean((local_xy[support, 1] - rim_side_y) ** 2))
        )
        result["rim_height_median_residual_m"] = float(
            np.median(np.abs(supported_z - cad.grasp_height_m))
        )
        if support_count >= 3:
            result["rim_tangent_span_m"] = float(
                np.percentile(supported_x, 95) - np.percentile(supported_x, 5)
            )
            result["rim_height_span_m"] = float(
                np.percentile(supported_z, 95) - np.percentile(supported_z, 5)
            )

    checks = (
        support_count >= minimum_geometry_support_pixels,
        support_fraction >= minimum_geometry_support_fraction,
        float(result["rim_tangent_span_m"]) >= minimum_tangent_span_m,
        float(result["rim_height_span_m"]) >= minimum_height_span_m,
    )
    result["passed"] = bool(all(checks))
    if not result["passed"]:
        result["rejection_reason"] = "depth does not support the expected CAD rim patch"
    return result
