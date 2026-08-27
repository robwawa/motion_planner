#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)"
acados_root="${workspace_root}/third_party/acados"
acados_tag="v0.5.4"

if [[ ! -d "${acados_root}/.git" ]]; then
  mkdir -p "$(dirname "${acados_root}")"
  git clone --branch "${acados_tag}" --depth 1 --recurse-submodules https://github.com/acados/acados.git "${acados_root}"
else
  git -C "${acados_root}" checkout "${acados_tag}"
  git -C "${acados_root}" submodule update --init --recursive
fi

cmake -S "${acados_root}" -B "${acados_root}/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DACADOS_WITH_QPOASES=ON \
  -DACADOS_EXAMPLES=OFF \
  -DACADOS_UNIT_TESTS=OFF \
  -DACADOS_INSTALL_DIR="${acados_root}/install"
cmake --build "${acados_root}/build" -j"$(nproc)"
cmake --install "${acados_root}/build"

python3 -m pip install --user 'setuptools<70' 'packaging<24'
python3 -m pip install --user -e "${acados_root}/interfaces/acados_template"
ACADOS_SOURCE_DIR="${acados_root}" python3 - <<'PY'
from acados_template.utils import get_tera
get_tera(tera_version='0.0.34', force_download=True)
PY

ACADOS_SOURCE_DIR="${acados_root}" python3 "${workspace_root}/src/scan_planner/planner/plan_manage/scripts/generate_mpc_solver.py"
echo "acados ${acados_tag} is ready"
