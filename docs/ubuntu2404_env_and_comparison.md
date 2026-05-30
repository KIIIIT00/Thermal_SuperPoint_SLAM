# Ubuntu 24.04 env setup and SuperPoint comparison

このメモは、リポジトリ直下の `env` 仮想環境で Python 側を動かし、同じ画像列に対して `pytorch-superpoint` と `thermal-kd-superpoint` の特徴量を作り、同じ `SuperPoint_SLAM` で結果を比較するための手順です。

## 1. OSパッケージ

Ubuntu 24.04では、まずC++ビルドに必要なパッケージを入れます。

```bash
sudo apt update
sudo apt install -y   build-essential cmake git pkg-config   libeigen3-dev libopencv-dev   libglew-dev libgl1-mesa-dev libxkbcommon-dev   libwayland-dev wayland-protocols libegl1-mesa-dev   libpython3-dev python3-venv
```

Pangolinが未インストールの場合は、同梱ソースからローカルビルドできます。

```bash
scripts/build_pangolin_local.sh
```

その後、表示された `Pangolin_DIR` または `CMAKE_PREFIX_PATH` を指定してください。

## 2. Python仮想環境

既存の `env` を使う場合:

```bash
source env/bin/activate
python -c "import torch, cv2, evo; print(torch.__version__, cv2.__version__)"
```

作り直す場合:

```bash
scripts/setup_ubuntu2404_env.sh
source env/bin/activate
```

CUDA wheelを変えたい場合は例のように指定します。

```bash
TORCH_INDEX_URL=https://download.pytorch.org/whl/cu124 scripts/setup_ubuntu2404_env.sh
```

## 3. SuperPoint_SLAMのビルド

```bash
scripts/build_superpoint_slam_ubuntu2404.sh
```

Pangolinをローカルに入れている場合、または `scripts/build_pangolin_local.sh` を使った場合:

```bash
Pangolin_DIR=/path/to/Pangolin/install/lib/cmake/Pangolin CMAKE_PREFIX_PATH=/path/to/Pangolin/install scripts/build_superpoint_slam_ubuntu2404.sh
```

実行ファイルは以下にできます。

```bash
thirdparty/SuperPoint_SLAM/Examples/Monocular/mono_euroc
```

## 4. 比較実行

入力はEuRoC形式に合わせます。

- 画像ディレクトリ: `TIMESTAMP.png` が並ぶディレクトリ
- タイムスタンプ: 各行が画像ファイル名のベースになるtimestampのtxt
- カメラ設定: `configs/ORB_SLAM2/*.yaml`

例:

```bash
scripts/run_sp_comparison.sh   --images /path/to/images   --times /path/to/timestamps.txt   --config configs/ORB_SLAM2/ViViD_Thermal.yaml   --out results/vivid_compare
```

デフォルトのモデルは以下です。

- `pytorch-superpoint`: `trained_networks/superpoint_thermal/thermal.pth.tar`
- `thermal-kd-superpoint`: `trained_networks/superpoint_kd_thermal/newloss_vivid/best.pth`
- vocabulary: `vocabularies/superpt_thermal.yml.gz`

別モデルを使う場合:

```bash
scripts/run_sp_comparison.sh   --images /path/to/images   --times /path/to/timestamps.txt   --config configs/ORB_SLAM2/ViViD_Thermal.yaml   --out results/vivid_compare   --classic-model trained_networks/superpoint_thermal/thermal.pth.tar   --kdsp-ckpt trained_networks/superpoint_kd_thermal/oldloss/best.pth   --vocab vocabularies/superpt_thermal.yml.gz
```

出力は以下です。

```text
results/vivid_compare/features_pytorch_superpoint/
results/vivid_compare/features_thermal_kd_superpoint/
results/vivid_compare/pytorch_superpoint/KeyFrameTrajectory.txt
results/vivid_compare/thermal_kd_superpoint/KeyFrameTrajectory.txt
```

## 5. 軌跡評価

Ground truthがTUM形式である場合:

```bash
source env/bin/activate
evo_ape tum groundtruth.txt results/vivid_compare/pytorch_superpoint/KeyFrameTrajectory.txt -a
evo_ape tum groundtruth.txt results/vivid_compare/thermal_kd_superpoint/KeyFrameTrajectory.txt -a
```

プロットも残す場合:

```bash
evo_ape tum groundtruth.txt results/vivid_compare/pytorch_superpoint/KeyFrameTrajectory.txt -a --plot --save_results results/vivid_compare/pytorch_superpoint.zip
evo_ape tum groundtruth.txt results/vivid_compare/thermal_kd_superpoint/KeyFrameTrajectory.txt -a --plot --save_results results/vivid_compare/thermal_kd_superpoint.zip
```


## 6. `double free` の原因切り分けプロトコル

`double free or corruption (out)` は、実際にはその直前の不正メモリアクセスが後で検知されるケースが多いです。
以下の順で同じ入力条件を回すと、原因候補を分離できます。

### 6.1 GBAをスキップして前段の整合性確認

`CreateInitialMapMonocular` の Step7 だけをスキップします。

```bash
SP_SLAM_SKIP_GBA=1 ./thirdparty/SuperPoint_SLAM/Examples/Monocular/mono_euroc   vocabularies/superpt_kd_thermal.yml.gz   configs/ORB_SLAM2/ViViD_Thermal.yaml   data/VIVID/extracted_data/driving_full/campus_day1/img/thermal_fieldscale_clahe   data/VIVID/extracted_data/driving_full/campus_day1/thermal_timestamps.txt   data/VIVID/extracted_data/driving_full/campus_day1/img/features_kdsp/ --no-viewer
```

- ここで落ちるなら、GBA以前（対応付けやMapPoint生成）に問題
- ここで通るなら、GBA区間が主な原因候補

### 6.2 GBA入力の破損観測チェック

`Optimizer.cc` で観測インデックスとoctave境界をチェックし、問題がある観測を
`[GBA][WARN]` として出すようにしてあります。

- `Invalid observation index`
- `Invalid octave`
- `has no valid edges`

これらが出た場合、BA入力データ破損が一次原因です。

### 6.3 ASan/UBSanで最初の破壊点を特定

```bash
scripts/build_superpoint_slam_asan.sh
```

ASanビルド後は、`build_asan` 側の実行ファイルを使います。

```bash
ASAN_OPTIONS=abort_on_error=1:detect_leaks=0 UBSAN_OPTIONS=print_stacktrace=1 ./thirdparty/SuperPoint_SLAM/build_asan/Examples/Monocular/mono_euroc   vocabularies/superpt_kd_thermal.yml.gz   configs/ORB_SLAM2/ViViD_Thermal.yaml   data/VIVID/extracted_data/driving_full/campus_day1/img/thermal_fieldscale_clahe   data/VIVID/extracted_data/driving_full/campus_day1/thermal_timestamps.txt   data/VIVID/extracted_data/driving_full/campus_day1/img/features_kdsp/ --no-viewer
```

ASanが出す「最初の不正アクセス箇所」を根拠に修正するのが最短です。


## 7. SLAM実行から追跡安定性評価まで一括で行う

`scripts/run_sp_slam_and_eval.sh` を追加しました。これはリポジトリ直下の `env` を必ず使い、同じ入力画像列に対して以下をまとめて実行します。

- `thirdparty/pytorch-superpoint` baseline の特徴量生成
- `thermal-kd-superpoint` の特徴量生成
- `thirdparty/SuperPoint_SLAM/Examples/Monocular/mono_euroc` によるSLAM実行
- `KeyFrameTrajectory.txt` と `slam.log` に基づく追跡安定性評価
- GT軌跡がある場合の `evo_ape` / `evo_rpe`

基本コマンド:

```bash
cd /home/ais-lab/app/kawahara/app/Thermal_SuperPoint_SLAM

scripts/run_sp_slam_and_eval.sh \
  --images data/VIVID/extracted_data/driving_full/campus_day1/img/thermal_fieldscale_clahe \
  --times data/VIVID/extracted_data/driving_full/campus_day1/thermal_timestamps.txt \
  --config configs/ORB_SLAM2/ViViD_Thermal.yaml \
  --out outputs/slam_eval/vivid_campus_day1 \
  --dataset-name VIVID \
  --sequence driving_full/campus_day1
```

baselineだけを評価する場合:

```bash
scripts/run_sp_slam_and_eval.sh \
  --images /path/to/images \
  --times /path/to/timestamps.txt \
  --config configs/ORB_SLAM2/ViViD_Thermal.yaml \
  --out outputs/slam_eval/baseline_only \
  --only baseline
```

thermal-kd-superpointだけを評価する場合:

```bash
scripts/run_sp_slam_and_eval.sh \
  --images /path/to/images \
  --times /path/to/timestamps.txt \
  --config configs/ORB_SLAM2/ViViD_Thermal.yaml \
  --out outputs/slam_eval/kdsp_only \
  --only kdsp \
  --kdsp-ckpt trained_networks/superpoint_kd_thermal/newloss_vivid/best.pth \
  --kdsp-vocab vocabularies/superpt_kd_thermal.yml.gz
```

すでに特徴量とSLAM結果がある場合は、評価だけを再実行できます。

```bash
scripts/run_sp_slam_and_eval.sh \
  --images /path/to/images \
  --times /path/to/timestamps.txt \
  --config configs/ORB_SLAM2/ViViD_Thermal.yaml \
  --out outputs/slam_eval/vivid_campus_day1 \
  --skip-feature-gen \
  --no-slam
```

GT軌跡がTUM形式である場合は `--gt-tum` を渡します。

```bash
scripts/run_sp_slam_and_eval.sh \
  --images /path/to/images \
  --times /path/to/timestamps.txt \
  --config configs/ORB_SLAM2/ViViD_Thermal.yaml \
  --out outputs/slam_eval/sthereo_seq \
  --gt-tum /path/to/groundtruth_tum.txt
```

出力の主要ファイル:

```text
outputs/slam_eval/<run>/summary.tsv
outputs/slam_eval/<run>/pytorch_superpoint/slam.log
outputs/slam_eval/<run>/pytorch_superpoint/KeyFrameTrajectory.txt
outputs/slam_eval/<run>/pytorch_superpoint/tracking_stability/tracking_stability_summary.json
outputs/slam_eval/<run>/thermal_kd_superpoint/slam.log
outputs/slam_eval/<run>/thermal_kd_superpoint/KeyFrameTrajectory.txt
outputs/slam_eval/<run>/thermal_kd_superpoint/tracking_stability/tracking_stability_summary.json
```

追跡安定性の主指標:

- `completion_rate`: 最後の保存姿勢が入力シーケンスのどこまで到達したか
- `track_break_count`: 軌跡時間ギャップと追跡区間から推定した中断回数
- `longest_contiguous_track_ratio`: 最長連続追跡区間の割合
- `num_log_lost_or_reset_events`: SLAMログ中の lost/reset/relocalization 系イベント数
- `tracking_coverage`: 入力フレームに対応する保存姿勢の割合。ただし現状の monocular 実装では `KeyFrameTrajectory.txt` ベースなので、キーフレーム密度の影響を受ける保守的なproxyです。

研究評価では、pairwise matching や PoseAUC だけでなく、最終的にこの追跡安定性指標が改善しているかを主張の根拠にしてください。
