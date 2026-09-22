# Native planner sources

This directory contains the C++ source copy used by `global_pct_planner`:

- `src/a_star`: native A* search;
- `src/map_manager`: dense elevation map;
- `src/common`: common data types and OSQP-backed smoothing;
- `src/trajectory_optimization`: GPMP/GTSAM factors and optimizers;
- `src/ele_planner`: the offline elevation planner facade.

The copy is intentionally kept independent from the old `pct_planner` build
targets.  GTSAM and the OSQP install are external configurable dependencies;
see the package-level CMake options.
