#!/usr/bin/env python3
"""Clean an LIO-SAM PCD map for PCT tomography.

The filtering stages only remove input points.  The final voxel-center sampling
stage emits one representative point at the geometric center of each occupied
voxel, so that stage intentionally regularizes point positions in space.
"""

from __future__ import annotations

import argparse
import copy
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple

import numpy as np
import open3d as o3d
import yaml


DEFAULT_CONFIG: Dict[str, Any] = {
    "input_pcd": None,
    "output_pcd": None,
    "sor": {
        "enable": True,
        "nb_neighbors": 30,
        "std_ratio": 2.5,
    },
    "ror": {
        "enable": True,
        "nb_points": 2,
        "radius": 0.15,
    },
    "floating_cluster": {
        "enable": True,
        "eps": 0.15,
        "min_points": 3,
        "remove_if_point_count_below": 30,
        "max_bbox_x": 0.30,
        "max_bbox_y": 0.30,
        "max_bbox_z": 0.30,
        "save_removed_clusters": True,
    },
    "uniform_sampling": {
        "enable": True,
        "spacing": 0.05,
        "sampling_mode": "voxel_center",
    },
    "debug": {
        "save_intermediate": True,
        "output_directory": "./pct_map_cleaner_debug",
    },
}


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """Merge a YAML mapping over defaults without mutating either mapping."""

    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_config(config_path: str) -> Dict[str, Any]:
    """Load and lightly validate the YAML configuration."""

    path = Path(config_path)
    if not path.is_file():
        raise FileNotFoundError(f"configuration file does not exist: {path}")

    with path.open("r", encoding="utf-8") as config_file:
        loaded = yaml.safe_load(config_file) or {}

    if not isinstance(loaded, dict):
        raise ValueError("configuration root must be a YAML mapping")

    return _deep_merge(DEFAULT_CONFIG, loaded)


def _as_int(config: Dict[str, Any], key: str, section: str, minimum: int) -> int:
    value = config[section].get(key)
    if isinstance(value, bool):
        raise ValueError(f"{section}.{key} must be an integer")
    try:
        integer = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{section}.{key} must be an integer") from exc
    if integer < minimum:
        raise ValueError(f"{section}.{key} must be >= {minimum}")
    return integer


def _as_positive_float(config: Dict[str, Any], key: str, section: str) -> float:
    value = config[section].get(key)
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{section}.{key} must be a positive number") from exc
    if not np.isfinite(number) or number <= 0.0:
        raise ValueError(f"{section}.{key} must be a positive finite number")
    return number


def validate_config(config: Dict[str, Any]) -> None:
    """Validate parameters that are used by enabled processing stages."""

    for section in ("sor", "ror", "floating_cluster", "uniform_sampling", "debug"):
        if not isinstance(config.get(section), dict):
            raise ValueError(f"{section} must be a YAML mapping")

    if config["sor"].get("enable", True):
        _as_int(config, "nb_neighbors", "sor", 1)
        _as_positive_float(config, "std_ratio", "sor")

    if config["ror"].get("enable", True):
        _as_int(config, "nb_points", "ror", 1)
        _as_positive_float(config, "radius", "ror")

    if config["floating_cluster"].get("enable", True):
        _as_positive_float(config, "eps", "floating_cluster")
        _as_int(config, "min_points", "floating_cluster", 1)
        _as_int(config, "remove_if_point_count_below", "floating_cluster", 1)
        for key in ("max_bbox_x", "max_bbox_y", "max_bbox_z"):
            _as_positive_float(config, key, "floating_cluster")

    if config["uniform_sampling"].get("enable", True):
        _as_positive_float(config, "spacing", "uniform_sampling")
        sampling_mode = config["uniform_sampling"].get("sampling_mode")
        if sampling_mode != "voxel_center":
            raise ValueError(
                "uniform_sampling.sampling_mode must be 'voxel_center'"
            )


def load_point_cloud(path: str) -> o3d.geometry.PointCloud:
    """Read a PCD with Open3D and reject missing, unreadable, or empty input."""

    input_path = Path(path)
    if not input_path.is_file():
        raise FileNotFoundError(f"input PCD does not exist: {input_path}")

    cloud = o3d.io.read_point_cloud(str(input_path))
    if cloud is None:
        raise RuntimeError(f"Open3D failed to read point cloud: {input_path}")

    points = np.asarray(cloud.points)
    if points.ndim != 2 or points.shape[1] != 3:
        raise RuntimeError(f"input PCD does not contain an XYZ point array: {input_path}")
    if points.shape[0] == 0:
        raise ValueError(f"input PCD is empty: {input_path}")

    return cloud


def _copy_point_attributes(
    source_cloud: o3d.geometry.PointCloud,
    destination_cloud: o3d.geometry.PointCloud,
    indices: np.ndarray,
) -> None:
    """Copy Open3D-supported attributes using the same point indices."""

    source_count = len(source_cloud.points)
    if len(source_cloud.colors) == source_count:
        destination_cloud.colors = o3d.utility.Vector3dVector(
            np.asarray(source_cloud.colors)[indices]
        )
    if len(source_cloud.normals) == source_count:
        destination_cloud.normals = o3d.utility.Vector3dVector(
            np.asarray(source_cloud.normals)[indices]
        )


def indices_to_cloud(
    source_cloud: o3d.geometry.PointCloud, indices: Iterable[int]
) -> o3d.geometry.PointCloud:
    """Build a point cloud from existing source points and synchronized attributes."""

    selected = np.asarray(indices, dtype=np.int64)
    if selected.ndim != 1:
        selected = selected.reshape(-1)

    source_points = np.asarray(source_cloud.points)
    destination = o3d.geometry.PointCloud()
    destination.points = o3d.utility.Vector3dVector(source_points[selected])
    _copy_point_attributes(source_cloud, destination, selected)
    return destination


def points_to_cloud_with_attributes(
    source_cloud: o3d.geometry.PointCloud,
    points: np.ndarray,
    attribute_indices: Iterable[int],
) -> o3d.geometry.PointCloud:
    """Build a cloud at explicit XYZ positions with source attributes.

    ``attribute_indices`` identifies the input point whose colors/normals are
    copied.  It may differ geometrically from ``points`` for voxel-center
    sampling.
    """

    selected = np.asarray(attribute_indices, dtype=np.int64).reshape(-1)
    output_points = np.asarray(points, dtype=np.float64)
    if output_points.shape != (selected.size, 3):
        raise ValueError("explicit point positions and attribute indices disagree")

    destination = o3d.geometry.PointCloud()
    destination.points = o3d.utility.Vector3dVector(output_points)
    _copy_point_attributes(source_cloud, destination, selected)
    return destination


def remove_invalid_points(
    cloud: o3d.geometry.PointCloud,
) -> Tuple[o3d.geometry.PointCloud, np.ndarray, o3d.geometry.PointCloud, np.ndarray]:
    """Remove non-finite XYZ values and return valid/removed local index mappings."""

    points = np.asarray(cloud.points)
    valid_mask = np.isfinite(points).all(axis=1)
    valid_indices = np.flatnonzero(valid_mask).astype(np.int64, copy=False)
    invalid_indices = np.flatnonzero(~valid_mask).astype(np.int64, copy=False)
    return (
        indices_to_cloud(cloud, valid_indices),
        valid_indices,
        indices_to_cloud(cloud, invalid_indices),
        invalid_indices,
    )


def _empty_like(cloud: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    return indices_to_cloud(cloud, np.empty(0, dtype=np.int64))


def statistical_outlier_filter(
    cloud: o3d.geometry.PointCloud,
    point_indices: np.ndarray,
    config: Dict[str, Any],
) -> Tuple[o3d.geometry.PointCloud, np.ndarray, o3d.geometry.PointCloud, np.ndarray, float]:
    """Apply conservative Open3D statistical outlier filtering."""

    start = time.perf_counter()
    before = len(cloud.points)
    section = config["sor"]
    if not section.get("enable", True):
        return cloud, point_indices, _empty_like(cloud), np.empty(0, dtype=np.int64), time.perf_counter() - start

    nb_neighbors = _as_int(config, "nb_neighbors", "sor", 1)
    std_ratio = _as_positive_float(config, "std_ratio", "sor")
    if before <= nb_neighbors:
        print(f"[SOR] skipped: {before} points are insufficient for nb_neighbors={nb_neighbors}")
        return cloud, point_indices, _empty_like(cloud), np.empty(0, dtype=np.int64), time.perf_counter() - start

    filtered, inlier_indices = cloud.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio,
    )
    del filtered
    keep_local = np.asarray(inlier_indices, dtype=np.int64)
    keep_local = np.sort(keep_local)
    keep_mask = np.ones(before, dtype=bool)
    keep_mask[keep_local] = False
    removed_local = np.flatnonzero(keep_mask).astype(np.int64, copy=False)
    return (
        indices_to_cloud(cloud, keep_local),
        point_indices[keep_local],
        indices_to_cloud(cloud, removed_local),
        point_indices[removed_local],
        time.perf_counter() - start,
    )


def radius_outlier_filter(
    cloud: o3d.geometry.PointCloud,
    point_indices: np.ndarray,
    config: Dict[str, Any],
) -> Tuple[o3d.geometry.PointCloud, np.ndarray, o3d.geometry.PointCloud, np.ndarray, float]:
    """Apply conservative Open3D radius outlier filtering."""

    start = time.perf_counter()
    before = len(cloud.points)
    section = config["ror"]
    if not section.get("enable", True):
        return cloud, point_indices, _empty_like(cloud), np.empty(0, dtype=np.int64), time.perf_counter() - start

    nb_points = _as_int(config, "nb_points", "ror", 1)
    radius = _as_positive_float(config, "radius", "ror")
    if before == 0:
        return cloud, point_indices, _empty_like(cloud), np.empty(0, dtype=np.int64), time.perf_counter() - start

    filtered, inlier_indices = cloud.remove_radius_outlier(
        nb_points=nb_points,
        radius=radius,
    )
    del filtered
    keep_local = np.asarray(inlier_indices, dtype=np.int64)
    keep_local = np.sort(keep_local)
    keep_mask = np.ones(before, dtype=bool)
    keep_mask[keep_local] = False
    removed_local = np.flatnonzero(keep_mask).astype(np.int64, copy=False)
    return (
        indices_to_cloud(cloud, keep_local),
        point_indices[keep_local],
        indices_to_cloud(cloud, removed_local),
        point_indices[removed_local],
        time.perf_counter() - start,
    )


def remove_small_floating_clusters(
    cloud: o3d.geometry.PointCloud,
    point_indices: np.ndarray,
    config: Dict[str, Any],
) -> Tuple[
    o3d.geometry.PointCloud,
    np.ndarray,
    o3d.geometry.PointCloud,
    np.ndarray,
    int,
    int,
    int,
    float,
]:
    """Remove only small dense DBSCAN clusters with a small 3D bounding box."""

    start = time.perf_counter()
    before = len(cloud.points)
    section = config["floating_cluster"]
    if not section.get("enable", True) or before == 0:
        return (
            cloud,
            point_indices,
            _empty_like(cloud),
            np.empty(0, dtype=np.int64),
            0,
            0,
            0,
            time.perf_counter() - start,
        )

    eps = _as_positive_float(config, "eps", "floating_cluster")
    min_points = _as_int(config, "min_points", "floating_cluster", 1)
    point_limit = _as_int(config, "remove_if_point_count_below", "floating_cluster", 1)
    bbox_limits = np.array(
        [
            _as_positive_float(config, "max_bbox_x", "floating_cluster"),
            _as_positive_float(config, "max_bbox_y", "floating_cluster"),
            _as_positive_float(config, "max_bbox_z", "floating_cluster"),
        ],
        dtype=np.float64,
    )

    labels = np.asarray(
        cloud.cluster_dbscan(
            eps=eps,
            min_points=min_points,
            print_progress=True,
        ),
        dtype=np.int64,
    )
    if labels.shape != (before,):
        raise RuntimeError("Open3D DBSCAN returned an unexpected label array")

    cluster_labels = np.unique(labels[labels >= 0])
    remove_mask = np.zeros(before, dtype=bool)
    points = np.asarray(cloud.points)
    removed_cluster_count = 0
    for label in cluster_labels:
        cluster_indices = np.flatnonzero(labels == label)
        cluster_points = points[cluster_indices]
        bbox_size = cluster_points.max(axis=0) - cluster_points.min(axis=0)
        if len(cluster_indices) < point_limit and np.all(bbox_size < bbox_limits):
            remove_mask[cluster_indices] = True
            removed_cluster_count += 1

    removed_local = np.flatnonzero(remove_mask).astype(np.int64, copy=False)
    keep_local = np.flatnonzero(~remove_mask).astype(np.int64, copy=False)
    noise_count = int(np.count_nonzero(labels == -1))
    return (
        indices_to_cloud(cloud, keep_local),
        point_indices[keep_local],
        indices_to_cloud(cloud, removed_local),
        point_indices[removed_local],
        int(len(cluster_labels)),
        removed_cluster_count,
        noise_count,
        time.perf_counter() - start,
    )


def spatial_uniform_sampling(
    cloud: o3d.geometry.PointCloud,
    spacing: float,
    sampling_mode: str = "voxel_center",
) -> Tuple[o3d.geometry.PointCloud, np.ndarray, float, float, float]:
    """Place one representative point at each occupied voxel center.

    Voxel grouping and representative selection are vectorized with NumPy.
    The source index returned for each output point identifies the input point
    whose colors/normals are copied; its XYZ position is intentionally replaced
    by the occupied voxel's geometric center.
    """

    start = time.perf_counter()
    if sampling_mode != "voxel_center":
        raise ValueError("only voxel_center sampling_mode is supported")
    if not np.isfinite(spacing) or spacing <= 0.0:
        raise ValueError("uniform sampling spacing must be a positive finite number")

    points = np.asarray(cloud.points)
    count = points.shape[0]
    if count == 0:
        return (
            cloud,
            np.empty(0, dtype=np.int64),
            time.perf_counter() - start,
            0.0,
            0.0,
        )

    origin = points.min(axis=0)
    voxel_indices = np.floor((points - origin) / spacing).astype(np.int64)

    # np.lexsort uses the last key as the primary key: x, then y, then z.
    order = np.lexsort(
        (voxel_indices[:, 2], voxel_indices[:, 1], voxel_indices[:, 0])
    )
    sorted_voxels = voxel_indices[order]
    sorted_points = points[order]

    starts_mask = np.empty(count, dtype=bool)
    starts_mask[0] = True
    starts_mask[1:] = np.any(sorted_voxels[1:] != sorted_voxels[:-1], axis=1)
    starts = np.flatnonzero(starts_mask)
    ends = np.empty_like(starts)
    ends[:-1] = starts[1:]
    ends[-1] = count
    counts = ends - starts

    unique_voxels = sorted_voxels[starts]
    centers = origin + (unique_voxels.astype(np.float64) + 0.5) * spacing
    group_ids = np.repeat(np.arange(len(starts), dtype=np.int64), counts)
    distances = np.sum((sorted_points - centers[group_ids]) ** 2, axis=1)

    # For every group, reduce the first position whose distance equals the
    # group minimum. This remains fully vectorized and deterministic.
    group_minimum = np.minimum.reduceat(distances, starts)
    candidate_positions = np.arange(count, dtype=np.int64)
    candidate_positions[distances != group_minimum[group_ids]] = count
    best_sorted_positions = np.minimum.reduceat(candidate_positions, starts)
    representative_local = order[best_sorted_positions]
    representative_centers = centers

    # Preserve deterministic input-order output while keeping the matching
    # voxel center and source attribute index aligned.
    output_order = np.argsort(representative_local, kind="stable")
    selected_local = representative_local[output_order]
    selected_centers = representative_centers[output_order]
    displacements = np.sqrt(group_minimum[output_order])

    return (
        points_to_cloud_with_attributes(cloud, selected_centers, selected_local),
        selected_local,
        time.perf_counter() - start,
        float(displacements.max()) if displacements.size else 0.0,
        float(displacements.mean()) if displacements.size else 0.0,
    )


def save_cloud(cloud: o3d.geometry.PointCloud, path: str) -> None:
    """Write a point cloud and fail loudly if Open3D reports an error."""

    output_path = Path(path)
    os.makedirs(output_path.parent, exist_ok=True)
    if len(cloud.points) == 0:
        # Open3D 0.19 refuses to emit a header for an empty cloud.  A valid
        # zero-point PCD is still useful for recording that a stage removed
        # nothing, and contains no generated geometry.
        output_path.write_text(
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            "WIDTH 0\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            "POINTS 0\n"
            "DATA ascii\n",
            encoding="ascii",
        )
        return
    success = o3d.io.write_point_cloud(str(output_path), cloud)
    if not success:
        raise RuntimeError(f"Open3D failed to write point cloud: {output_path}")


def save_removed_points(cloud: o3d.geometry.PointCloud, path: str) -> None:
    """Save a removed-point cloud for inspection in CloudCompare/Open3D."""

    save_cloud(cloud, path)


def compute_bbox(cloud: o3d.geometry.PointCloud) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    points = np.asarray(cloud.points)
    if points.shape[0] == 0:
        return None
    return points.min(axis=0), points.max(axis=0)


def print_cloud_statistics(label: str, cloud: o3d.geometry.PointCloud) -> None:
    """Print point count and XYZ extent for a cloud."""

    print(f"{label}: points: {len(cloud.points)}")
    bbox = compute_bbox(cloud)
    if bbox is None:
        print("  bbox: empty")
        return
    bbox_min, bbox_max = bbox
    print(f"  bbox: x=[{bbox_min[0]:.6f}, {bbox_max[0]:.6f}]")
    print(f"         y=[{bbox_min[1]:.6f}, {bbox_max[1]:.6f}]")
    print(f"         z=[{bbox_min[2]:.6f}, {bbox_max[2]:.6f}]")


def print_stage_report(
    name: str,
    before: int,
    after: int,
    elapsed: float,
    warn_threshold: Optional[float] = 0.20,
) -> None:
    removed = before - after
    ratio = removed / before if before else 0.0
    print(f"[{name}]")
    print(f"  before: {before}")
    print(f"  after: {after}")
    print(f"  removed: {removed}")
    print(f"  removed ratio: {ratio:.6f}")
    print(f"  time: {elapsed:.2f} s")
    if warn_threshold is not None and ratio > warn_threshold:
        print(
            f"  WARNING: removed ratio {ratio:.2%} exceeds "
            f"{warn_threshold:.0%}"
        )


def _resolve_artifact_directory(config: Dict[str, Any], output_path: Path) -> Path:
    debug = config["debug"]
    if debug.get("save_intermediate", False):
        directory = Path(str(debug.get("output_directory", "./pct_map_cleaner_debug")))
    else:
        directory = output_path.parent
    os.makedirs(directory, exist_ok=True)
    return directory


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Clean an LIO-SAM PCD map using conservative outlier removal, "
            "small-cluster removal, and voxel-center spatial sampling."
        )
    )
    parser.add_argument(
        "--config",
        required=True,
        help="YAML configuration file containing processing parameters and paths.",
    )
    parser.add_argument(
        "--input",
        dest="input_pcd",
        help="Override input_pcd from YAML.",
    )
    parser.add_argument(
        "--output",
        dest="output_pcd",
        help="Override output_pcd from YAML.",
    )
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = _build_argument_parser()
    args = parser.parse_args(argv)
    total_start = time.perf_counter()

    try:
        config = load_config(args.config)
        if args.input_pcd:
            config["input_pcd"] = args.input_pcd
        if args.output_pcd:
            config["output_pcd"] = args.output_pcd
        if not config.get("input_pcd"):
            raise ValueError("input_pcd is missing from YAML and --input was not provided")
        if not config.get("output_pcd"):
            raise ValueError("output_pcd is missing from YAML and --output was not provided")
        validate_config(config)

        input_path = Path(str(config["input_pcd"]))
        output_path = Path(str(config["output_pcd"]))
        artifact_directory = _resolve_artifact_directory(config, output_path)

        input_cloud = load_point_cloud(str(input_path))
        raw_count = len(input_cloud.points)
        valid_cloud, valid_indices, _, invalid_indices = remove_invalid_points(input_cloud)
        valid_count = len(valid_cloud.points)
        if valid_count == 0:
            raise ValueError("input PCD contains no finite XYZ points")

        print("========== PCT Map Cleaner Report ==========")
        print("Input:")
        print(f"  path: {input_path}")
        print(f"  points: {raw_count}")
        print(f"  valid: {valid_count}")
        print(f"  invalid: {len(invalid_indices)}")
        print_cloud_statistics("  valid cloud", valid_cloud)

        debug_enabled = bool(config["debug"].get("save_intermediate", False))
        if debug_enabled:
            save_cloud(valid_cloud, str(artifact_directory / "00_input_valid.pcd"))

        current_cloud = valid_cloud
        current_indices = valid_indices

        before = len(current_cloud.points)
        (
            current_cloud,
            current_indices,
            removed_sor,
            _,
            sor_time,
        ) = statistical_outlier_filter(current_cloud, current_indices, config)
        print_stage_report("SOR", before, len(current_cloud.points), sor_time)
        save_removed_points(removed_sor, str(artifact_directory / "removed_sor.pcd"))
        if debug_enabled:
            save_cloud(current_cloud, str(artifact_directory / "01_sor.pcd"))

        before = len(current_cloud.points)
        (
            current_cloud,
            current_indices,
            removed_ror,
            _,
            ror_time,
        ) = radius_outlier_filter(current_cloud, current_indices, config)
        print_stage_report("ROR", before, len(current_cloud.points), ror_time)
        save_removed_points(removed_ror, str(artifact_directory / "removed_ror.pcd"))
        if debug_enabled:
            save_cloud(current_cloud, str(artifact_directory / "02_ror.pcd"))

        before = len(current_cloud.points)
        (
            current_cloud,
            current_indices,
            removed_clusters,
            _,
            cluster_count,
            removed_cluster_count,
            noise_count,
            cluster_time,
        ) = remove_small_floating_clusters(current_cloud, current_indices, config)
        print("[Floating Cluster Filter]")
        print(f"  DBSCAN clusters: {cluster_count}")
        print(f"  small clusters removed: {removed_cluster_count}")
        print(f"  points removed: {before - len(current_cloud.points)}")
        print(f"  noise points label=-1: {noise_count}")
        print(f"  time: {cluster_time:.2f} s")
        if config["floating_cluster"].get("save_removed_clusters", True):
            save_removed_points(
                removed_clusters,
                str(artifact_directory / "removed_floating_clusters.pcd"),
            )
        if debug_enabled:
            save_cloud(current_cloud, str(artifact_directory / "03_cluster_clean.pcd"))

        before = len(current_cloud.points)
        if config["uniform_sampling"].get("enable", True):
            spacing = _as_positive_float(config, "spacing", "uniform_sampling")
            sampling_mode = config["uniform_sampling"].get(
                "sampling_mode", "voxel_center"
            )
            (
                current_cloud,
                selected_local,
                uniform_time,
                max_displacement,
                mean_displacement,
            ) = spatial_uniform_sampling(
                current_cloud,
                spacing,
                sampling_mode,
            )
            current_indices = current_indices[selected_local]
        else:
            spacing = _as_positive_float(config, "spacing", "uniform_sampling")
            uniform_time = 0.0
            max_displacement = 0.0
            mean_displacement = 0.0
        after = len(current_cloud.points)
        reduction = (before - after) / before if before else 0.0
        print("[Uniform Sampling]")
        print(f"  spacing: {spacing:.6f} m")
        print(f"  before: {before}")
        print(f"  after: {after}")
        print(f"  reduction: {before - after}")
        print(f"  reduction ratio: {reduction:.6f}")
        print(f"  occupied voxels: {after}")
        print(f"  max displacement: {max_displacement:.6f} m")
        print(f"  mean displacement: {mean_displacement:.6f} m")
        print(f"  time: {uniform_time:.2f} s")
        if debug_enabled:
            save_cloud(current_cloud, str(artifact_directory / "04_uniform.pcd"))

        save_cloud(current_cloud, str(output_path))

        print("\nFinal:")
        print_cloud_statistics("  final", current_cloud)
        print("Output:")
        print(f"  {output_path}")
        print("Total time:")
        print(f"  {time.perf_counter() - total_start:.2f} s")
        print("============================================")
        return 0
    except Exception as exc:  # Keep command-line errors concise and actionable.
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
