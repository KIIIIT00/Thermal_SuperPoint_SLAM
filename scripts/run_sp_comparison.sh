#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'MSG'
Usage:
  scripts/run_sp_comparison.sh --images IMAGE_DIR --times TIMES_TXT --config CONFIG_YAML --out OUT_DIR [options]

Options:
  --classic-model PATH   pytorch-superpoint checkpoint
                         default: trained_networks/superpoint_thermal/thermal.pth.tar
  --kdsp-ckpt PATH       thermal-kd-superpoint checkpoint
                         default: trained_networks/superpoint_kd_thermal/newloss_vivid/best.pth
  --vocab PATH           SuperPoint vocabulary
                         default: vocabularies/superpt_thermal.yml.gz
  --max-features N       default: 100000
  --resize W H           optional resize passed to feature generation
MSG
}

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="$ROOT_DIR/env/bin/python3"
CLASSIC_MODEL="$ROOT_DIR/trained_networks/superpoint_thermal/thermal.pth.tar"
KDSP_CKPT="$ROOT_DIR/trained_networks/superpoint_kd_thermal/newloss_vivid/best.pth"
VOCAB="$ROOT_DIR/vocabularies/superpt_thermal.yml.gz"
MAX_FEATURES=100000
RESIZE_ARGS=()

IMAGES=""
TIMES=""
CONFIG=""
OUT=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --images) IMAGES="$2"; shift 2 ;;
    --times) TIMES="$2"; shift 2 ;;
    --config) CONFIG="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --classic-model) CLASSIC_MODEL="$2"; shift 2 ;;
    --kdsp-ckpt) KDSP_CKPT="$2"; shift 2 ;;
    --vocab) VOCAB="$2"; shift 2 ;;
    --max-features) MAX_FEATURES="$2"; shift 2 ;;
    --resize) RESIZE_ARGS=(--resize "$2" "$3"); shift 3 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "$IMAGES" || -z "$TIMES" || -z "$CONFIG" || -z "$OUT" ]]; then
  usage >&2
  exit 2
fi

mkdir -p "$OUT"
CLASSIC_FEATURES="$OUT/features_pytorch_superpoint"
KDSP_FEATURES="$OUT/features_thermal_kd_superpoint"
SLAM_BIN="$ROOT_DIR/thirdparty/SuperPoint_SLAM/Examples/Monocular/mono_euroc"

"$PYTHON" "$ROOT_DIR/utils/generate_keypts_and_desc.py" \
  --superpoint-model "$CLASSIC_MODEL" \
  --image-dir "$IMAGES" \
  --out-dir "$CLASSIC_FEATURES" \
  --max-features "$MAX_FEATURES" \
  "${RESIZE_ARGS[@]}"

"$PYTHON" "$ROOT_DIR/utils/generate_keypts_and_desc.py" \
  --kdsp-ckpt "$KDSP_CKPT" \
  --image-dir "$IMAGES" \
  --out-dir "$KDSP_FEATURES" \
  --max-features "$MAX_FEATURES" \
  "${RESIZE_ARGS[@]}"

run_slam() {
  local name="$1"
  local features="$2"
  local run_dir="$OUT/$name"
  mkdir -p "$run_dir"
  (cd "$run_dir" && "$SLAM_BIN" "$VOCAB" "$CONFIG" "$IMAGES" "$TIMES" "$features")
}

run_slam pytorch_superpoint "$CLASSIC_FEATURES"
run_slam thermal_kd_superpoint "$KDSP_FEATURES"

cat <<MSG

Finished.
Trajectories:
  $OUT/pytorch_superpoint/KeyFrameTrajectory.txt
  $OUT/thermal_kd_superpoint/KeyFrameTrajectory.txt

If ground truth is available, compare with evo, for example:
  source "$ROOT_DIR/env/bin/activate"
  evo_ape tum GROUNDTRUTH_TUM.txt "$OUT/pytorch_superpoint/KeyFrameTrajectory.txt" -a
  evo_ape tum GROUNDTRUTH_TUM.txt "$OUT/thermal_kd_superpoint/KeyFrameTrajectory.txt" -a
MSG
