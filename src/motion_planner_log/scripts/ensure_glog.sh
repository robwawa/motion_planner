#!/usr/bin/env bash
set -euo pipefail

if pkg-config --exists libglog; then
  version="$(pkg-config --modversion libglog)"
  if [[ "${version}" == "0.4.0" ]]; then
    echo "System Google glog ${version} is available; no download is needed."
    exit 0
  fi
  echo "System Google glog ${version} is available, but this project pins 0.4.0." >&2
  exit 1
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
target="${repo_root}/third_party/glog"
if [[ -e "${target}" ]]; then
  echo "Fallback directory exists but is incomplete: ${target}" >&2
  exit 1
fi

mkdir -p "$(dirname "${target}")"
git clone --depth 1 --branch v0.4.0 https://github.com/google/glog.git "${target}"
echo "Downloaded Google glog 0.4.0 to ${target}. Re-run catkin_make."
