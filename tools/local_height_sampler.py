#!/usr/bin/env python3
"""Local height sampler for a global 2D heightmap.

The online ROS2 node is implemented in C++ for real-time use. This Python class
keeps the same sampling convention for offline checks and tests.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Optional, Tuple

import numpy as np
import yaml


class LocalHeightSampler:
    """Query a global heightmap around the robot in yaw-aligned local points."""

    VALID_FORMULAS = {"terrain_minus_base", "base_minus_terrain", "legged_gym"}

    def __init__(
        self,
        heightmap_path: str | Path,
        metadata_path: str | Path,
        x_points: Optional[Iterable[float]] = None,
        y_points: Optional[Iterable[float]] = None,
        default_height: float = 0.0,
        height_formula: str = "legged_gym",
        measured_height_offset: float = 0.3,
    ) -> None:
        self.heightmap_path = Path(heightmap_path)
        self.metadata_path = Path(metadata_path)
        self.height_map = np.load(self.heightmap_path).astype(np.float32, copy=False)
        if self.height_map.ndim != 2:
            raise ValueError(f"height map must be 2D, got shape {self.height_map.shape}")

        with self.metadata_path.open("r", encoding="utf-8") as f:
            metadata = yaml.safe_load(f) or {}

        self.resolution = float(metadata["resolution"])
        self.origin_x = float(metadata["origin_x"])
        self.origin_y = float(metadata["origin_y"])
        self.width = int(metadata.get("width", self.height_map.shape[1]))
        self.height = int(metadata.get("height", self.height_map.shape[0]))
        self.frame_id = str(metadata.get("frame_id", "map"))
        self.z_mode = str(metadata.get("z_mode", "unknown"))

        if self.resolution <= 0.0:
            raise ValueError("resolution must be positive")
        if (self.height, self.width) != self.height_map.shape:
            raise ValueError(
                "metadata width/height do not match height map shape: "
                f"metadata=({self.height}, {self.width}), map={self.height_map.shape}"
            )

        self.default_height = float(default_height)
        self.height_formula = self._validate_formula(height_formula)
        self.measured_height_offset = float(measured_height_offset)

        self.x_points = (
            np.asarray(list(x_points), dtype=np.float64)
            if x_points is not None
            else np.asarray(
                [-0.45, -0.3, -0.15, 0.0, 0.15, 0.3, 0.45, 0.6, 0.75, 0.9, 1.05, 1.2],
                dtype=np.float64,
            )
        )
        self.y_points = (
            np.asarray(list(y_points), dtype=np.float64)
            if y_points is not None
            else np.asarray(
                [-0.75, -0.6, -0.45, -0.3, -0.15, 0.0, 0.15, 0.3, 0.45, 0.6, 0.75],
                dtype=np.float64,
            )
        )
        if self.x_points.ndim != 1 or self.y_points.ndim != 1:
            raise ValueError("x_points and y_points must be 1D")

        xx, yy = np.meshgrid(self.x_points, self.y_points, indexing="ij")
        self.local_points = np.stack([xx.reshape(-1), yy.reshape(-1)], axis=1)

    @staticmethod
    def _validate_formula(height_formula: str) -> str:
        if height_formula not in LocalHeightSampler.VALID_FORMULAS:
            valid = ", ".join(sorted(LocalHeightSampler.VALID_FORMULAS))
            raise ValueError(f"unsupported height_formula {height_formula!r}; valid: {valid}")
        return height_formula

    def world_to_grid(self, world_x, world_y) -> Tuple[np.ndarray, np.ndarray]:
        """Convert world coordinates to integer row/col indices using floor."""
        wx = np.asarray(world_x, dtype=np.float64)
        wy = np.asarray(world_y, dtype=np.float64)
        col = np.floor((wx - self.origin_x) / self.resolution).astype(np.int64)
        row = np.floor((wy - self.origin_y) / self.resolution).astype(np.int64)
        return row, col

    def grid_to_world(self, row, col) -> Tuple[np.ndarray, np.ndarray]:
        """Convert row/col indices to the world coordinate represented by a cell."""
        r = np.asarray(row, dtype=np.float64)
        c = np.asarray(col, dtype=np.float64)
        world_x = self.origin_x + c * self.resolution
        world_y = self.origin_y + r * self.resolution
        return world_x, world_y

    def _valid_indices(self, row: np.ndarray, col: np.ndarray) -> np.ndarray:
        return (row >= 0) & (row < self.height) & (col >= 0) & (col < self.width)

    def query_nearest(self, world_x, world_y):
        """Return terrain z at the nearest/floor grid cell, or default_height."""
        row, col = self.world_to_grid(world_x, world_y)
        scalar = row.ndim == 0
        row_1d = np.atleast_1d(row)
        col_1d = np.atleast_1d(col)
        result = np.full(row_1d.shape, self.default_height, dtype=np.float32)
        valid = self._valid_indices(row_1d, col_1d)
        if np.any(valid):
            values = self.height_map[row_1d[valid], col_1d[valid]]
            finite = np.isfinite(values)
            valid_positions = np.flatnonzero(valid)
            result[valid_positions[finite]] = values[finite]
        return float(result[0]) if scalar else result

    def query_bilinear(self, world_x, world_y):
        """Bilinear terrain query with nearest fallback for invalid neighborhoods."""
        wx = np.asarray(world_x, dtype=np.float64)
        wy = np.asarray(world_y, dtype=np.float64)
        scalar = wx.ndim == 0 and wy.ndim == 0

        gx = (wx - self.origin_x) / self.resolution
        gy = (wy - self.origin_y) / self.resolution
        col0 = np.floor(gx).astype(np.int64)
        row0 = np.floor(gy).astype(np.int64)
        col1 = col0 + 1
        row1 = row0 + 1

        col0_1d = np.atleast_1d(col0)
        row0_1d = np.atleast_1d(row0)
        col1_1d = np.atleast_1d(col1)
        row1_1d = np.atleast_1d(row1)
        gx_1d = np.atleast_1d(gx)
        gy_1d = np.atleast_1d(gy)

        result = np.asarray(self.query_nearest(wx, wy), dtype=np.float32).reshape(-1)
        valid = (
            self._valid_indices(row0_1d, col0_1d)
            & self._valid_indices(row0_1d, col1_1d)
            & self._valid_indices(row1_1d, col0_1d)
            & self._valid_indices(row1_1d, col1_1d)
        )
        if np.any(valid):
            q00 = self.height_map[row0_1d[valid], col0_1d[valid]]
            q10 = self.height_map[row0_1d[valid], col1_1d[valid]]
            q01 = self.height_map[row1_1d[valid], col0_1d[valid]]
            q11 = self.height_map[row1_1d[valid], col1_1d[valid]]
            finite = np.isfinite(q00) & np.isfinite(q10) & np.isfinite(q01) & np.isfinite(q11)
            if np.any(finite):
                dx = gx_1d[valid][finite] - col0_1d[valid][finite]
                dy = gy_1d[valid][finite] - row0_1d[valid][finite]
                values = (
                    q00[finite] * (1.0 - dx) * (1.0 - dy)
                    + q10[finite] * dx * (1.0 - dy)
                    + q01[finite] * (1.0 - dx) * dy
                    + q11[finite] * dx * dy
                )
                valid_positions = np.flatnonzero(valid)
                result[valid_positions[finite]] = values.astype(np.float32)

        return float(result[0]) if scalar else result

    def local_points_to_world(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        local_points: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Transform local horizontal sample points to world xy using yaw only."""
        points = self.local_points if local_points is None else np.asarray(local_points, dtype=np.float64)
        c = np.cos(robot_yaw)
        s = np.sin(robot_yaw)
        world_x = robot_x + c * points[:, 0] - s * points[:, 1]
        world_y = robot_y + s * points[:, 0] + c * points[:, 1]
        return np.stack([world_x, world_y], axis=1)

    def _apply_formula(
        self,
        terrain_z: np.ndarray,
        robot_z: float,
        height_formula: str,
        measured_height_offset: float,
    ) -> np.ndarray:
        if height_formula == "terrain_minus_base":
            return terrain_z - robot_z
        if height_formula == "base_minus_terrain":
            return robot_z - terrain_z
        if height_formula == "legged_gym":
            return robot_z - measured_height_offset - terrain_z
        raise AssertionError(f"validated formula unexpectedly unsupported: {height_formula}")

    def sample(
        self,
        robot_x: float,
        robot_y: float,
        robot_z: float,
        robot_yaw: float,
        *,
        clip_min: Optional[float] = -1.0,
        clip_max: Optional[float] = 1.0,
        use_bilinear: bool = True,
        sample_as_grid: bool = False,
        default_height: Optional[float] = None,
        height_formula: Optional[str] = None,
        measured_height_offset: Optional[float] = None,
    ) -> np.ndarray:
        """Sample local height observations around the robot."""
        old_default = self.default_height
        if default_height is not None:
            self.default_height = float(default_height)
        try:
            world_xy = self.local_points_to_world(robot_x, robot_y, robot_yaw)
            terrain_z = (
                self.query_bilinear(world_xy[:, 0], world_xy[:, 1])
                if use_bilinear
                else self.query_nearest(world_xy[:, 0], world_xy[:, 1])
            )
        finally:
            self.default_height = old_default

        formula = self._validate_formula(height_formula or self.height_formula)
        offset = self.measured_height_offset if measured_height_offset is None else float(measured_height_offset)
        heights = self._apply_formula(np.asarray(terrain_z, dtype=np.float32), float(robot_z), formula, offset)

        if clip_min is not None or clip_max is not None:
            lo = -np.inf if clip_min is None else float(clip_min)
            hi = np.inf if clip_max is None else float(clip_max)
            heights = np.clip(heights, lo, hi)

        heights = heights.astype(np.float32, copy=False)
        if sample_as_grid:
            return heights.reshape(len(self.x_points), len(self.y_points))
        return heights
