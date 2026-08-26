#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR=$(cd "$(dirname "$0")"; pwd)
PCT_PYTHON_EXECUTABLE="${PCT_PYTHON_EXECUTABLE:-/usr/bin/python3}"
if [[ ! -x "${PCT_PYTHON_EXECUTABLE}" ]]; then
  PCT_PYTHON_EXECUTABLE=$(command -v python3)
fi
# echo "ROOT_DIR: ${ROOT_DIR}"

cd "${ROOT_DIR}/lib"

# Keep only modules compatible with the interpreter selected for this build.
rm -f a_star*.so traj_opt*.so ele_planner*.so py_map_manager*.so
rm -f build/src/a_star/a_star*.so \
  build/src/trajectory_optimization/traj_opt*.so \
  build/src/ele_planner/ele_planner*.so \
  build/src/map_manager/py_map_manager*.so

# rm -rf build
mkdir -p build

cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE="${PCT_PYTHON_EXECUTABLE}"
make -j6
cp ./src/a_star/a_star*.so ../
cp ./src/trajectory_optimization/traj_opt*.so ../
cp ./src/ele_planner/ele_planner*.so ../
cp ./src/map_manager/py_map_manager*.so ../
cp ./src/common/smoothing/libcommon_smoothing.so ../
cd ..

# # optional
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${ROOT_DIR}/lib/3rdparty/gtsam-4.1.1/install/lib"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ROOT_DIR}/lib/build/src/common/smoothing"
export PYTHONPATH="${PYTHONPATH:-}:${ROOT_DIR}/lib"
# pybind11-stubgen -o ./ a_star
# pybind11-stubgen -o ./ traj_opt
# pybind11-stubgen -o ./ ele_planner
# pybind11-stubgen -o ./ py_map_manager
# cp ./a_star-stubs/__init__.pyi ./a_star.pyi
# cp ./traj_opt-stubs/__init__.pyi ./traj_opt.pyi
# cp ./ele_planner-stubs/__init__.pyi ./ele_planner.pyi
# cp ./py_map_manager-stubs/__init__.pyi ./py_map_manager.pyi
