"""CPU and (when available) CUDA tomogram backend regression tests."""

import unittest
from types import SimpleNamespace

import numpy as np

from tomogram_cpu import CpuTomogram


def make_config():
    return SimpleNamespace(
        map=SimpleNamespace(resolution=1.0, slice_dh=1.0),
        trav=SimpleNamespace(
            kernel_size=3,
            interval_min=0.5,
            interval_free=0.65,
            slope_max=0.4,
            step_max=0.3,
            standable_ratio=0.2,
            cost_barrier=50.0,
            safe_margin=1.0,
            inflation=0.2,
        ),
    )


def sample_points():
    points = []
    for x in range(-3, 4):
        for y in range(-3, 4):
            points.append((x, y, 0.0))
            points.append((x, y, 2.0))
    points.append((np.nan, 0.0, 0.0))
    return np.asarray(points, dtype=np.float32)


def build(backend_class):
    tomogram = backend_class(make_config())
    tomogram.initMappingEnv(np.zeros(2, dtype=np.float32), 9, 9, 3, 0.5)
    return tomogram.point2map(sample_points())


class CpuTomogramTest(unittest.TestCase):
    def test_cpu_output_contract(self):
        result = build(CpuTomogram)
        layers_t, grad_x, grad_y, layers_g, layers_c, timings = result
        self.assertEqual(layers_t.shape, grad_x.shape)
        self.assertEqual(layers_t.shape, grad_y.shape)
        self.assertEqual(layers_t.shape, layers_g.shape)
        self.assertEqual(layers_t.shape, layers_c.shape)
        self.assertGreater(np.isfinite(layers_g).sum(), 0)
        self.assertGreater(np.isnan(layers_g).sum(), 0)
        self.assertTrue(np.all(layers_t >= 0.0))
        self.assertEqual(set(timings), {'t_map', 't_trav', 't_simp'})
        self.assertTrue(all(value >= 0.0 for value in timings.values()))

    def test_cuda_style_half_rounding_and_nan_input(self):
        tomogram = CpuTomogram(make_config())
        tomogram.initMappingEnv(np.zeros(2, dtype=np.float32), 5, 5, 1, 0.5)
        result = tomogram.point2map(np.asarray([
            (0.5, -0.5, 0.0),
            (-0.5, 0.5, 0.0),
            (np.nan, 0.0, 0.0),
        ], dtype=np.float32))
        ground = result[3][0]
        self.assertEqual(ground[3, 1], 0.0)
        self.assertEqual(ground[1, 3], 0.0)

    def test_threaded_inflation_matches_single_thread(self):
        single = CpuTomogram(make_config())
        single.initMappingEnv(np.zeros(2, dtype=np.float32), 9, 9, 3, 0.5)
        single.cpu_workers = 1
        single_result = single.point2map(sample_points())

        threaded = CpuTomogram(make_config())
        threaded.initMappingEnv(np.zeros(2, dtype=np.float32), 9, 9, 3, 0.5)
        threaded_result = threaded.point2map(sample_points())

        for single_array, threaded_array in zip(
            single_result[:5], threaded_result[:5]
        ):
            np.testing.assert_array_equal(single_array, threaded_array)


@unittest.skipUnless(
    __import__('importlib').util.find_spec('cupy') is not None,
    'CuPy is not installed',
)
class CudaParityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import cupy as cp
        try:
            device_count = cp.cuda.runtime.getDeviceCount()
        except cp.cuda.runtime.CUDARuntimeError as error:
            raise unittest.SkipTest('CUDA runtime is unavailable: {}'.format(error))
        if device_count < 1:
            raise unittest.SkipTest('No CUDA device is available')
        from tomogram import Tomogram
        cls.cuda_class = Tomogram

    def test_cpu_and_cuda_match(self):
        cpu_result = build(CpuTomogram)
        cuda_result = build(self.cuda_class)
        for cpu_array, cuda_array in zip(cpu_result[:5], cuda_result[:5]):
            np.testing.assert_allclose(
                cpu_array, cuda_array, rtol=1e-4, atol=1e-4, equal_nan=True
            )


if __name__ == '__main__':
    unittest.main()
