import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from local_height_sampler import LocalHeightSampler


class LocalHeightSamplerTest(unittest.TestCase):
    def make_sampler(self):
        tmp = tempfile.TemporaryDirectory()
        tmp_path = Path(tmp.name)
        height_map = np.zeros((120, 120), dtype=np.float32)
        for row in range(height_map.shape[0]):
            for col in range(height_map.shape[1]):
                height_map[row, col] = 0.01 * row + 0.001 * col
        heightmap_path = tmp_path / "global_heightmap_filled.npy"
        metadata_path = tmp_path / "metadata.yaml"
        np.save(heightmap_path, height_map)
        metadata = {
            "resolution": 0.1,
            "origin_x": -6.0,
            "origin_y": -6.0,
            "width": int(height_map.shape[1]),
            "height": int(height_map.shape[0]),
            "frame_id": "map",
            "z_mode": "min",
        }
        metadata_path.write_text(yaml.safe_dump(metadata), encoding="utf-8")
        sampler = LocalHeightSampler(heightmap_path, metadata_path)
        return tmp, sampler

    def test_default_points_shape(self):
        tmp, sampler = self.make_sampler()
        self.addCleanup(tmp.cleanup)
        self.assertEqual(len(sampler.x_points), 12)
        self.assertEqual(len(sampler.y_points), 11)
        self.assertEqual(sampler.local_points.shape, (132, 2))
        np.testing.assert_allclose(
            sampler.x_points,
            [-0.45, -0.3, -0.15, 0.0, 0.15, 0.3, 0.45, 0.6, 0.75, 0.9, 1.05, 1.2],
        )
        np.testing.assert_allclose(
            sampler.y_points,
            [-0.75, -0.6, -0.45, -0.3, -0.15, 0.0, 0.15, 0.3, 0.45, 0.6, 0.75],
        )

    def test_world_grid_round_trip(self):
        tmp, sampler = self.make_sampler()
        self.addCleanup(tmp.cleanup)
        row = 42
        col = 37
        world_x, world_y = sampler.grid_to_world(row, col)
        out_row, out_col = sampler.world_to_grid(world_x, world_y)
        self.assertEqual(int(out_row), row)
        self.assertEqual(int(out_col), col)

    def test_sample_shapes(self):
        tmp, sampler = self.make_sampler()
        self.addCleanup(tmp.cleanup)
        flat = sampler.sample(0.0, 0.0, 0.5, 0.0)
        grid = sampler.sample(0.0, 0.0, 0.5, 0.0, sample_as_grid=True)
        self.assertEqual(flat.shape, (132,))
        self.assertEqual(grid.shape, (12, 11))
        self.assertEqual(flat.dtype, np.float32)
        self.assertEqual(grid.dtype, np.float32)

    def test_yaw_zero_local_axes_match_world_axes(self):
        tmp, sampler = self.make_sampler()
        self.addCleanup(tmp.cleanup)
        local = np.array([[0.1, 0.0], [0.0, 0.1]], dtype=np.float64)
        world = sampler.local_points_to_world(1.0, 2.0, 0.0, local)
        np.testing.assert_allclose(world[0], [1.1, 2.0], atol=1e-9)
        np.testing.assert_allclose(world[1], [1.0, 2.1], atol=1e-9)

    def test_yaw_pi_over_two_rotates_forward_to_world_y(self):
        tmp, sampler = self.make_sampler()
        self.addCleanup(tmp.cleanup)
        local = np.array([[0.1, 0.0]], dtype=np.float64)
        world = sampler.local_points_to_world(1.0, 2.0, math.pi / 2.0, local)
        np.testing.assert_allclose(world[0], [1.0, 2.1], atol=1e-9)


if __name__ == "__main__":
    unittest.main()
