#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'MSG'
Usage:
  scripts/run_sp_slam_and_eval.sh --images IMAGE_DIR --times TIMES_TXT --config CONFIG_YAML --out OUT_DIR [options]

Purpose:
  Generate features, run Thermal_SuperPoint_SLAM, and evaluate tracking stability for:
    1. thirdparty/pytorch-superpoint baseline
    2. thermal-kd-superpoint model

Required:
  --images IMAGE_DIR       EuRoC-style image directory. mono_euroc expects TIMESTAMP.png.
  --times TIMES_TXT        Timestamp file used by mono_euroc.
  --config CONFIG_YAML     ORB_SLAM2/SuperPoint_SLAM camera config.
  --out OUT_DIR            Output directory.

Options:
  --classic-model PATH     pytorch-superpoint checkpoint.
                           default: trained_networks/superpoint_thermal/thermal.pth.tar
  --kdsp-ckpt PATH         thermal-kd-superpoint checkpoint.
                           default: trained_networks/superpoint_kd_thermal/newloss_vivid/best.pth
  --classic-vocab PATH     vocabulary for pytorch-superpoint baseline.
                           default: vocabularies/superpt_thermal.yml.gz
  --kdsp-vocab PATH        vocabulary for thermal-kd-superpoint.
                           default: vocabularies/superpt_kd_thermal.yml.gz if present, else classic vocab
  --max-features N         default: 100000
  --resize W H             optional resize passed to feature generation.
  --detection-threshold X  default: 0.015
  --nms-dist N             default: 4
  --gt-tum PATH            optional TUM-format GT trajectory. Runs evo_ape/evo_rpe if evo is available.
  --dataset-name NAME      metadata for evaluation JSON.
  --sequence NAME          metadata for evaluation JSON.
  --only baseline|kdsp|both
                           default: both
  --skip-feature-gen       reuse existing feature folders under OUT.
  --reuse-features-from DIR reuse feature folders from another output directory.
                           Uses DIR/features_pytorch_superpoint and
                           DIR/features_thermal_kd_superpoint.
  --classic-features DIR   existing pytorch-superpoint feature directory.
                           Skips baseline feature generation.
  --kdsp-features DIR      existing thermal-kd-superpoint feature directory.
                           Skips thermal-kd-superpoint feature generation.
  --no-slam                only evaluate existing OUT/*/KeyFrameTrajectory.txt and slam.log.
  --no-viewer              pass --no-viewer to mono_euroc. default: enabled in this wrapper.
  --viewer                 do not pass --no-viewer.
  -h, --help               show this help.

Outputs:
  OUT/features_pytorch_superpoint/
  OUT/features_thermal_kd_superpoint/
  OUT/pytorch_superpoint/slam.log
  OUT/pytorch_superpoint/KeyFrameTrajectory.txt
  OUT/pytorch_superpoint/tracking_stability/tracking_stability_summary.json
  OUT/thermal_kd_superpoint/slam.log
  OUT/thermal_kd_superpoint/KeyFrameTrajectory.txt
  OUT/thermal_kd_superpoint/tracking_stability/tracking_stability_summary.json
MSG
}

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="$ROOT_DIR/env/bin/python3"
EVO_APE="$ROOT_DIR/env/bin/evo_ape"
EVO_RPE="$ROOT_DIR/env/bin/evo_rpe"
SLAM_BIN="$ROOT_DIR/thirdparty/SuperPoint_SLAM/Examples/Monocular/mono_euroc"
EVAL_SCRIPT="$ROOT_DIR/evaluation/evaluate_slam_tracking_stability.py"

CLASSIC_MODEL="$ROOT_DIR/trained_networks/superpoint_thermal/thermal.pth.tar"
KDSP_CKPT="$ROOT_DIR/trained_networks/superpoint_kd_thermal/newloss_vivid/best.pth"
CLASSIC_VOCAB="$ROOT_DIR/vocabularies/superpt_thermal.yml.gz"
KDSP_VOCAB="$ROOT_DIR/vocabularies/superpt_kd_thermal.yml.gz"
MAX_FEATURES=100000
DETECTION_THRESHOLD=0.015
NMS_DIST=4
RESIZE_ARGS=()
GT_TUM=""
DATASET_NAME=""
SEQUENCE=""
ONLY="both"
SKIP_FEATURE_GEN=0
NO_SLAM=0
VIEWER_ARG=(--no-viewer)

IMAGES=""
TIMES=""
CONFIG=""
OUT=""
REUSE_FEATURES_FROM=""
CLASSIC_FEATURES_OVERRIDE=""
KDSP_FEATURES_OVERRIDE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --images) IMAGES="$2"; shift 2 ;;
    --times) TIMES="$2"; shift 2 ;;
    --config) CONFIG="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --classic-model) CLASSIC_MODEL="$2"; shift 2 ;;
    --kdsp-ckpt) KDSP_CKPT="$2"; shift 2 ;;
    --classic-vocab|--vocab) CLASSIC_VOCAB="$2"; shift 2 ;;
    --kdsp-vocab) KDSP_VOCAB="$2"; shift 2 ;;
    --max-features) MAX_FEATURES="$2"; shift 2 ;;
    --resize) RESIZE_ARGS=(--resize "$2" "$3"); shift 3 ;;
    --detection-threshold) DETECTION_THRESHOLD="$2"; shift 2 ;;
    --nms-dist) NMS_DIST="$2"; shift 2 ;;
    --gt-tum) GT_TUM="$2"; shift 2 ;;
    --dataset-name) DATASET_NAME="$2"; shift 2 ;;
    --sequence) SEQUENCE="$2"; shift 2 ;;
    --only) ONLY="$2"; shift 2 ;;
    --skip-feature-gen) SKIP_FEATURE_GEN=1; shift ;;
    --reuse-features-from) REUSE_FEATURES_FROM="$2"; shift 2 ;;
    --classic-features) CLASSIC_FEATURES_OVERRIDE="$2"; shift 2 ;;
    --kdsp-features) KDSP_FEATURES_OVERRIDE="$2"; shift 2 ;;
    --no-slam) NO_SLAM=1; shift ;;
    --no-viewer) VIEWER_ARG=(--no-viewer); shift ;;
    --viewer) VIEWER_ARG=(); shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "$IMAGES" || -z "$TIMES" || -z "$CONFIG" || -z "$OUT" ]]; then
  usage >&2
  exit 2
fi

if [[ ! -x "$PYTHON" ]]; then
  echo "ERROR: env python not found: $PYTHON" >&2
  exit 1
fi
if [[ ! -x "$SLAM_BIN" && "$NO_SLAM" -eq 0 ]]; then
  echo "ERROR: mono_euroc not found or not executable: $SLAM_BIN" >&2
  echo "Build it first: scripts/build_superpoint_slam_ubuntu2404.sh" >&2
  exit 1
fi

IMAGES="$(readlink -f "$IMAGES")"
TIMES="$(readlink -f "$TIMES")"
CONFIG="$(readlink -f "$CONFIG")"
CLASSIC_MODEL="$(readlink -f "$CLASSIC_MODEL")"
KDSP_CKPT="$(readlink -f "$KDSP_CKPT")"
CLASSIC_VOCAB="$(readlink -f "$CLASSIC_VOCAB")"
if [[ -f "$KDSP_VOCAB" ]]; then
  KDSP_VOCAB="$(readlink -f "$KDSP_VOCAB")"
else
  KDSP_VOCAB="$CLASSIC_VOCAB"
fi
if [[ -n "$GT_TUM" ]]; then
  GT_TUM="$(readlink -f "$GT_TUM")"
fi
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
FEATURES_CLASSIC="$OUT/features_pytorch_superpoint"
FEATURES_KDSP="$OUT/features_thermal_kd_superpoint"

feature_dir_has_data() {
  local dir="$1"
  [[ -d "$dir" && -f "${dir%/}/1.yaml" ]]
}

auto_resolve_feature_dir() {
  local method_dir_name="$1"
  local current_dir="$2"

  if feature_dir_has_data "$current_dir"; then
    printf '%s\n' "$current_dir"
    return
  fi
  if [[ "$SKIP_FEATURE_GEN" -ne 1 || -n "$REUSE_FEATURES_FROM" ]]; then
    printf '%s\n' "$current_dir"
    return
  fi

  local out_parent out_base candidate_base candidate_dir
  out_parent="$(dirname "$OUT")"
  out_base="$(basename "$OUT")"

  case "$out_base" in
    *_viewer) candidate_base="${out_base%_viewer}" ;;
    *_with_viewer) candidate_base="${out_base%_with_viewer}" ;;
    *) candidate_base="" ;;
  esac

  if [[ -n "$candidate_base" ]]; then
    candidate_dir="$out_parent/$candidate_base/$method_dir_name"
    if feature_dir_has_data "$candidate_dir"; then
      echo "[features] auto reuse $method_dir_name from $candidate_dir" >&2
      printf '%s\n' "$candidate_dir"
      return
    fi
  fi

  printf '%s\n' "$current_dir"
}

if [[ -n "$REUSE_FEATURES_FROM" ]]; then
  REUSE_FEATURES_FROM="$(readlink -f "$REUSE_FEATURES_FROM")"
  FEATURES_CLASSIC="$REUSE_FEATURES_FROM/features_pytorch_superpoint"
  FEATURES_KDSP="$REUSE_FEATURES_FROM/features_thermal_kd_superpoint"
fi
if [[ -n "$CLASSIC_FEATURES_OVERRIDE" ]]; then
  FEATURES_CLASSIC="$(readlink -f "$CLASSIC_FEATURES_OVERRIDE")"
fi
if [[ -n "$KDSP_FEATURES_OVERRIDE" ]]; then
  FEATURES_KDSP="$(readlink -f "$KDSP_FEATURES_OVERRIDE")"
fi
if [[ -z "$CLASSIC_FEATURES_OVERRIDE" ]]; then
  FEATURES_CLASSIC="$(auto_resolve_feature_dir features_pytorch_superpoint "$FEATURES_CLASSIC")"
fi
if [[ -z "$KDSP_FEATURES_OVERRIDE" ]]; then
  FEATURES_KDSP="$(auto_resolve_feature_dir features_thermal_kd_superpoint "$FEATURES_KDSP")"
fi
SUMMARY_TSV="$OUT/summary.tsv"

need_baseline=0
need_kdsp=0
case "$ONLY" in
  baseline|pytorch_superpoint) need_baseline=1 ;;
  kdsp|thermal_kd_superpoint) need_kdsp=1 ;;
  both) need_baseline=1; need_kdsp=1 ;;
  *) echo "ERROR: --only must be baseline, kdsp, or both" >&2; exit 2 ;;
esac

run_feature_generation() {
  if [[ "$SKIP_FEATURE_GEN" -eq 1 ]]; then
    echo "[features] skip feature generation"
    return
  fi
  if [[ "$need_baseline" -eq 1 && -z "$REUSE_FEATURES_FROM" && -z "$CLASSIC_FEATURES_OVERRIDE" ]]; then
    echo "[features] pytorch-superpoint baseline -> $FEATURES_CLASSIC"
    "$PYTHON" "$ROOT_DIR/utils/generate_keypts_and_desc.py" \
      --superpoint-model "$CLASSIC_MODEL" \
      --image-dir "$IMAGES" \
      --out-dir "$FEATURES_CLASSIC" \
      --max-features "$MAX_FEATURES" \
      --detection-threshold "$DETECTION_THRESHOLD" \
      --nms-dist "$NMS_DIST" \
      "${RESIZE_ARGS[@]}"
  elif [[ "$need_baseline" -eq 1 ]]; then
    echo "[features] pytorch-superpoint baseline: reuse $FEATURES_CLASSIC"
  fi
  if [[ "$need_kdsp" -eq 1 && -z "$REUSE_FEATURES_FROM" && -z "$KDSP_FEATURES_OVERRIDE" ]]; then
    echo "[features] thermal-kd-superpoint -> $FEATURES_KDSP"
    "$PYTHON" "$ROOT_DIR/utils/generate_keypts_and_desc.py" \
      --kdsp-ckpt "$KDSP_CKPT" \
      --image-dir "$IMAGES" \
      --out-dir "$FEATURES_KDSP" \
      --max-features "$MAX_FEATURES" \
      --detection-threshold "$DETECTION_THRESHOLD" \
      --nms-dist "$NMS_DIST" \
      "${RESIZE_ARGS[@]}"
  elif [[ "$need_kdsp" -eq 1 ]]; then
    echo "[features] thermal-kd-superpoint: reuse $FEATURES_KDSP"
  fi
}

validate_features() {
  local name="$1"
  local features="$2"
  local option="--${name}-features"
  if [[ "$name" == "pytorch_superpoint" ]]; then
    option="--classic-features"
  elif [[ "$name" == "thermal_kd_superpoint" ]]; then
    option="--kdsp-features"
  fi
  if [[ ! -d "$features" ]]; then
    echo "ERROR: feature directory for $name does not exist: $features" >&2
    echo "Generate features first, or pass --reuse-features-from or $option with the correct path." >&2
    exit 1
  fi
  if [[ ! -f "${features%/}/1.yaml" ]]; then
    echo "ERROR: feature directory for $name does not contain 1.yaml: $features" >&2
    echo "mono_euroc reads sequential YAML files starting from 1.yaml." >&2
    exit 1
  fi
}

run_one_slam() {
  local name="$1"
  local features="$2"
  local vocab="$3"
  local run_dir="$OUT/$name"
  mkdir -p "$run_dir"
  if [[ "$NO_SLAM" -eq 1 ]]; then
    echo "[slam] skip $name"
    return
  fi
  validate_features "$name" "$features"
  echo "[slam] $name"
  set +e
  (
    cd "$run_dir"
    "$SLAM_BIN" "$vocab" "$CONFIG" "$IMAGES" "$TIMES" "${features%/}/" "${VIEWER_ARG[@]}"
  ) > "$run_dir/slam.log" 2>&1
  local code=$?
  set -e
  echo "$code" > "$run_dir/exit_code.txt"
  if [[ "$code" -ne 0 ]]; then
    echo "[slam][WARN] $name exited with code $code. Evaluation will still run if trajectory exists. See $run_dir/slam.log" >&2
  fi
}

run_eval() {
  local name="$1"
  local run_dir="$OUT/$name"
  local traj="$run_dir/KeyFrameTrajectory.txt"
  local log="$run_dir/slam.log"
  local eval_dir="$run_dir/tracking_stability"
  mkdir -p "$eval_dir"
  echo "[eval] tracking stability: $name"
  local gt_args=()
  if [[ -n "$GT_TUM" ]]; then
    gt_args=(--gt-trajectory "$GT_TUM")
  fi
  "$PYTHON" "$EVAL_SCRIPT" \
    --trajectory "$traj" \
    --frame-list "$TIMES" \
    --slam-log "$log" \
    --dataset-name "$DATASET_NAME" \
    --sequence "$SEQUENCE" \
    --out-dir "$eval_dir" \
    "${gt_args[@]}"

  if [[ -n "$GT_TUM" && -s "$traj" && -x "$EVO_APE" ]]; then
    echo "[eval] evo APE/RPE: $name"
    set +e
    "$EVO_APE" tum "$GT_TUM" "$traj" -a --save_results "$run_dir/evo_ape.zip" > "$run_dir/evo_ape.log" 2>&1
    echo "$?" > "$run_dir/evo_ape_exit_code.txt"
    "$EVO_RPE" tum "$GT_TUM" "$traj" -a --save_results "$run_dir/evo_rpe.zip" > "$run_dir/evo_rpe.log" 2>&1
    echo "$?" > "$run_dir/evo_rpe_exit_code.txt"
    set -e
  fi
}

write_summary() {
  echo -e "method\ttracking_coverage\tcompletion_rate\ttrack_break_count\tlongest_contiguous_track_ratio\tnum_log_lost_or_reset_events\ttrajectory" > "$SUMMARY_TSV"
  for name in pytorch_superpoint thermal_kd_superpoint; do
    local json_path="$OUT/$name/tracking_stability/tracking_stability_summary.json"
    if [[ -f "$json_path" ]]; then
      "$PYTHON" - "$json_path" "$name" >> "$SUMMARY_TSV" <<'PYSUMMARY'
import json, sys
p, name = sys.argv[1], sys.argv[2]
d = json.load(open(p, 'r', encoding='utf-8'))
vals = [
    name,
    d.get('tracking_coverage'),
    d.get('completion_rate'),
    d.get('track_break_count'),
    d.get('longest_contiguous_track_ratio'),
    d.get('num_log_lost_or_reset_events'),
    d.get('trajectory'),
]
print('\t'.join('' if v is None else str(v) for v in vals))
PYSUMMARY
    fi
  done
}

run_feature_generation
if [[ "$need_baseline" -eq 1 ]]; then
  run_one_slam pytorch_superpoint "$FEATURES_CLASSIC" "$CLASSIC_VOCAB"
  run_eval pytorch_superpoint
fi
if [[ "$need_kdsp" -eq 1 ]]; then
  run_one_slam thermal_kd_superpoint "$FEATURES_KDSP" "$KDSP_VOCAB"
  run_eval thermal_kd_superpoint
fi
write_summary

cat <<MSG

Finished.
Summary:
  $SUMMARY_TSV

Tracking stability JSON:
  $OUT/pytorch_superpoint/tracking_stability/tracking_stability_summary.json
  $OUT/thermal_kd_superpoint/tracking_stability/tracking_stability_summary.json

Logs:
  $OUT/pytorch_superpoint/slam.log
  $OUT/thermal_kd_superpoint/slam.log
MSG
