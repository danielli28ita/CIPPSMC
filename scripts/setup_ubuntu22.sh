#!/usr/bin/env bash
# Explicitly invoked host dependency installation; never starts PX4 or a vehicle.
set -euo pipefail
RELEASE_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ -n "${1:-}" && "${1:-}" != --gazebo ]]; then
  echo 'Usage: bash scripts/setup_ubuntu22.sh [--gazebo]' >&2
  exit 1
fi
. /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 ]]; then
  echo 'This installer is tested for Ubuntu 22.04 only.' >&2
  exit 1
fi
sudo apt-get update
sudo apt-get install -y build-essential git cmake ninja-build ccache pkg-config \
  python3 python3-venv python3-dev curl gnupg lsb-release \
  gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi \
  libstdc++-arm-none-eabi-newlib genromfs libimage-exiftool-perl zip unzip
if [[ "${1:-}" == --gazebo ]]; then
  RELEASE_KEY=$(mktemp)
  trap 'rm -f "$RELEASE_KEY"' EXIT
  curl --fail --location https://packages.osrfoundation.org/gazebo.gpg --output "$RELEASE_KEY"
  sudo install -m 0644 "$RELEASE_KEY" /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
  sudo apt-get update
  sudo apt-get install -y gz-harmonic libgz-sim8-dev libgz-transport13-dev libgz-msgs10-dev
fi
mkdir -p "$RELEASE_ROOT/workspace"
python3 -m venv "$RELEASE_ROOT/workspace/venv"
"$RELEASE_ROOT/workspace/venv/bin/python" -m pip install -r "$RELEASE_ROOT/environments/requirements.lock.txt"
echo 'Installed dependencies. No firmware was flashed and no simulator was started.'
