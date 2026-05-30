#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_DIR="${ENV_DIR:-$ROOT_DIR/env}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
TORCH_INDEX_URL="${TORCH_INDEX_URL:-https://download.pytorch.org/whl/cu128}"

"$PYTHON_BIN" -m venv "$ENV_DIR"
source "$ENV_DIR/bin/activate"
python -m pip install --upgrade pip setuptools wheel
python -m pip install torch torchvision --index-url "$TORCH_INDEX_URL"
python -m pip install \
  numpy scipy opencv-python-headless matplotlib imageio tqdm pyyaml \
  scikit-learn tensorboardX imgaug torchgeometry torchsummary coloredlogs \
  einops kornia h5py pandas evo
python -m pip install -e "$ROOT_DIR/thirdparty/thermal-kd-superpoint"

cat <<MSG

Ubuntu 24.04 Python environment is ready.
Activate it with:
  source "$ENV_DIR/bin/activate"
MSG
