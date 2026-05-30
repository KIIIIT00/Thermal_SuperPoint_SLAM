# Thermal SuperPoint SLAM

**Note (March 2026):** If you're working on thermal SLAM, check out my latest project: [TRGS-SLAM](https://github.com/umautobots/trgs_slam). It addresses thermal image degradation (motion blur, rolling shutter, and fixed pattern noise) using a 3DGS map, a continuous-time trajectory, and thermal-aware rendering.

Thermal SuperPoint SLAM is a project completed for ROB 530 at the University of Michigan in the winter 2021 semester. This project aimed to create an indirect SLAM algorithm that can successfully perform on thermal imagery. Specifically, we trained a SuperPoint feature detection and description network on thermal imagery and integrated the network with ORB_SLAM2 in place of the ORB feature detector and descriptor. Our combined algorithm runs offline on precomputed keypoints and descriptors. See our [video](https://youtu.be/TwUVYOlQn44) and [report](https://github.com/specarmi/Thermal_SuperPoint_SLAM/blob/master/ROB_530_Final_Report_Thermal_SuperPoint_SLAM.pdf) for the details of the process and our results. Although the project was completed with thermal imagery in mind, the steps described here can be followed with any set of images to yield a SuperPoint network and corresponding vocabulary and use them within a modified version of ORB_SLAM2.

This project utilizes four existing codebases:
- SuperPoint Training: https://github.com/eric-yyjau/pytorch-superpoint
- Vocabulary Training: https://github.com/dorian3d/DBoW2
- Integration of SuperPoint and ORB_SLAM2: https://github.com/KinglittleQ/SuperPoint_SLAM
- Original ORB_SLAM2 (for comparison): https://github.com/raulmur/ORB_SLAM2

Each codebase required modifications and the modified forks are included in the *thirdparty* folder as submodules.

# 1. Setup

This library was tested on **Ubuntu 18.04**. After downloading the submodules the setup is divided into three task specific sections: SuperPoint training, vocabulary creation, and SLAM. Each task can be done independent of the others. Finally there are requirements for our preprocessing and evaluation scripts.

## Downloading Submodules

This repository uses submodules, after cloning download the submodules by running:
```
cd Thermal_SuperPoint_SLAM
git submodule update --init --recursive
```

## SuperPoint Training (pytorch-superpoint)

The pytorch-superpoint repository provides a requirements file for installing dependencies. An example of using it to create an environment with Anaconda is as follows:
```
cd thirdparty/pytorch-superpoint/
conda create --name py36-sp python=3.6
conda activate py36-sp
pip install -r requirements.txt
pip install -r requirements_torch.txt
```

## Vocabulary Generation (DBoW2)

For training a SuperPoint vocabulary DBoW2 only requires [OpenCV](http://opencv.org) (C++). Download and install instructions can be found at: http://opencv.org.

After installing OpenCV the DBoW2 code can be built by running the provided shell script:
```
chmod +x build_vocab_code.sh
./build_vocab_code.sh
```

## SLAM (ORB_SLAM2 and SuperPoint_SLAM)

To run the modified versions of ORB_SLAM2 and SuperPoint_SLAM the following is required:

- C++11 or C++0x Compiler
- [Pangolin](https://github.com/stevenlovegrove/Pangolin): used for visualization and user interface. Download and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.
- [OpenCV](http://opencv.org): used to manipulate images and features. Download and install instructions can be found at: http://opencv.org. **Required at least 2.4.3.**.
- Eigen: required by g2o (an included third party optimization library). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

After installing all of the above the ORB_SLAM2 and SuperPoint_SLAM code can be built by running the provided shell script:
```
chmod +x build_slam_code.sh
./build_slam_code.sh
```

## Preprocessing and Evaluation (*utils/* and *evaluation/*)

Any of these scripts that do not work with ROS bags can be run using the pytorch-superpoint environment (see the section SuperPoint Training above). The scripts that work with ROS bags require ROS to be installed (and were tested with [ROS melodic](http://wiki.ros.org/melodic/Installation)) and we have also found that these scripts do not work within conda environments. The scripts that create GIFs additionally require PIL which can be installed with `pip install Pillow`.

# 2. Image Directory Preprocessing

This section explains how to apply contrast limited adaptive histogram equalization (CLAHE) to an image directory containing 16 bit thermal images. This is a step we took for training our thermal SuperPoint network but it is not necessary for training on RGB images. See our [video](https://youtu.be/TwUVYOlQn44) and [report](https://github.com/specarmi/Thermal_SuperPoint_SLAM/blob/master/ROB_530_Final_Report_Thermal_SuperPoint_SLAM.pdf) for more details.

The script `utils/image_directory_preprocessor.py` is provided to apply CLAHE to an image directory and write the results to a new directory. See the script's help message for the full details.

An example of how to use this script is as follows:
```
python image_directory_preprocessor.py ../../datasets/FLIR_ADAS/train/Data/ Data_CLAHE
```
This will apply CLAHE to each image in the source directory and output them to *../../datasets/FLIR_ADAS/train/Data_CLAHE/* as PNG images with the same filenames.

# 3. ROS Bag Preprocessing

This section explains how to preprocess image messages in a ROS bag into a format suitable for ORB_SLAM2 and SuperPoint_SLAM. The end result is a text file of timestamps and a folder of images with filenames corresponding to the timestamps. This is the same format ORB_SLAM2 uses for the EuRoC dataset.

The script `utils/rosbag_preprocessor.py` is provided for this purpose. See the script's help message for the full details.

An example of how to use this script is as follows:
```
python rosbag_preprocessor.py ../../datasets/vivid/outdoor_robust_day1.bag /thermal/image_raw outdoor_thermal --apply-clahe
```
This will output all images under the topic `/thermal/image_raw` to the directory *../../datasets/vivid/outdoor_thermal/images_30hz_tstart_0_tstop_inf* and will output a text file containing the timestamps of each image to *../../datasets/vivid/outdoor_thermal/timestamps/timestamps_30hz_tstart_0_tstop_inf.txt*. Note that the framerate, start time, and stop time are denoted in the image folder name and the timestamp filename (in this example the original framerate has been assumed to be 30 Hz). Note also the `--apply-clahe` flag used here. This flag indicates that the input messages are 16 bit images, that CLAHE should be applied, and the result should be stored as an 8 bit image. This is unnecessary for RGB images.

# 4. SuperPoint Training

Training SuperPoint requires three steps: 1) train a MagicPoint network on synthetic shapes 2) generate pseudo-groundtruth keypoint labels using the trained MagicPoint network through Homographic Adaptation 3) train a SuperPoint network with the pseudo-groundtruth keypoint labels.

We used an existing trained MagicPoint network included in the original pytorch-superpoint repository instead of training one ourselves; see that repository for details on training a MagicPoint network. The MagicPoint network we used is now located at `trained_networks/magicpoint/magicpoint.pth.tar`.

Generating pseudo-groundtruth interest points can be done by making some modifications and running:
```
python thirdparty/pytorch-superpoint/export.py export_detector_homoAdapt configs/training/magicpoint_flir_export.yaml magicpoint_synth
```

Specifically `DATA_PATH` in [settings.py](https://github.com/specarmi/pytorch-superpoint/blob/master/settings.py) and [this](https://github.com/specarmi/pytorch-superpoint/blob/2aae5572ff8066f464d917eeb1884983af3ea7ae/datasets/FLIR_ADAS.py#L57) line in `FLIR_ADAS.py` need to be modified for images to be imported. Note that in `FLIR_ADAS.py` the input `task` will be set to the `export_folder` given in the config file `configs/training/magicpoint_flir_export.yaml`. The parameter `export_folder` can be set to either 'train' or 'val' and the image dataset used must be split into training and validation sets that are imported according to the corresponding setting for `export_folder`. The above command must be run twice, once with `export_folder` set to 'train' and once with it set to 'val'. The resulting pseudo-groundtruth keypoints will be stored in *logs/magicpoint_synth/predictions/train* and *logs/magicpoint_synth/predictions/val*. The config file includes many other parameters that can optionally be changed. Note that resized dimensions of the input images should be divisible by eight.

After generating the pseudo-groundtruth keypoints the SuperPoint network can be trained by running:
```
python thirdparty/pytorch-superpoint/train4.py train_joint configs/training superpoint_flir_train_heatmap.yaml superpoint
```
The result is a series of checkpoints of the network saved to *logs/superpoint/checkpoints/*. Once again the config file includes many parameters that can optionally be changed.

`trained_networks/superpoint_thermal/thermal.pth.tar` is our thermal SuperPoint network trained on the [FLIR ADAS dataset](https://www.flir.com/oem/adas/adas-dataset-form/).

# 5. SuperPoint Keypoint and Descriptor Generation

Applying the SuperPoint network trained using pytorch-superpoint to an image in C++ would require porting over a significant amount of python code. Due to time constraints we avoid this by precomputing the SuperPoint keypoints and descriptors using Python code, storing the results, and importing the results when needed in C++.

The script `utils/generate_keypts_and_desc.py` is provided to apply a SuperPoint network to an image directory and output the resulting keypoints and descriptors in sequentially named YAML files. See the script's help message for the full details.

An example of how to use this script with the thermal SuperPoint network we trained is as follows:
```
python generate_keypts_and_desc.py ../trained_networks/superpoint_thermal/thermal.pth.tar ../../datasets/vivid/outdoor_thermal/images_clahe_10hz_tstart_0_tstop_inf/ features
```
The result is a folder *../../datasets/vivid/outdoor_thermal/features/* with sequentially named YAML files containing the SuperPoint features.

# 6. Vocabulary Generation

To generate a SuperPoint vocabulary using precomputed SuperPoint keypoints and descriptors run:
```
./thirdparty/DBoW2/build/build_superpt_vocab <PATH_TO_SUPERPOINT_FEATURES>
```
The result will be a file `superpt_voc.yml.gz`.

Note that a hardcoded kmeans iteration limit of 100 was added [here](https://github.com/specarmi/DBoW2/blob/master/include/DBoW2/TemplatedVocabulary.h#L686). Previously DBoW2 only progressed to the next node once all descriptors remain in the same clusters for two iterations. In our experience, this would frequently not occur and instead the percentage of descriptors switching clusters each iteration would oscillate. Note also that the completion percentage printed during training is only a loose approximation as it uses an upper bound for the number of possible nodes to be processed.

`vocabularies/superpt_thermal.yml.gz` is our thermal SuperPoint vocabulary trained on the [FLIR ADAS dataset](https://www.flir.com/oem/adas/adas-dataset-form/).

# 7. Running SuperPoint SLAM

Our modified version of SuperPoint SLAM runs offline on precomputed keypoints and descriptors. The original SuperPoint SLAM could be run online but utilized the pretrained SuperPoint network provided by the original SuperPoint authors [here](https://github.com/magicleap/SuperPointPretrainedNetwork). The third party implementation we use for training (pytorch-superpoint) employs different layers in the network and our trained networks are incompatible with the original SuperPoint SLAM as a result. As was done in training the vocabulary, our quick fix is to generate keypoints and descriptors offline and import them into SuperPoint SLAM at runtime.

Assuming the data is in the format described in the ROS Bag Preprocessing section it can be imported using the EuRoC example. The executable can be run with the following arguments:
```
./thirdparty/SuperPoint_SLAM/Examples/Monocular/mono_euroc <PATH_TO_VOCABULARY> <PATH_TO_CONFIG> <PATH_TO_IMAGE_FOLDER> <PATH_TO_TIMESTAMP_FILE> <PATH_TO_SUPERPOINT_FEATURES>
```
For example:
```
./thirdparty/SuperPoint_SLAM/Examples/Monocular/mono_euroc vocabularies/superpt_thermal.yml.gz configs/ORB_SLAM2/ViViD_Thermal.yaml ../datasets/vivid/outdoor_thermal/images_clahe_10hz_tstart_0_tstop_inf/ ../datasets/vivid/outdoor_thermal/timestamps/timestamps_10hz_tstart_0_tstop_inf.txt ../datasets/vivid/outdoor_thermal/features/
```

# 8. Evaluation

This section gives the commands used to generate the results shown in our [video](https://youtu.be/TwUVYOlQn44) and [report](https://github.com/specarmi/Thermal_SuperPoint_SLAM/blob/master/ROB_530_Final_Report_Thermal_SuperPoint_SLAM.pdf).

## Comparing Contrast Enhancement Techniques
Contrast enhancement comparison figure:
```
python compare_contrast_enhancement.py ../../datasets/fcav/cadata_sequence.bag /ubol/image_raw -f 50
```
CLAHE GIF:
```
python generate_clahe_gif.py ../../datasets/fcav/cadata_sequence.bag /ubol/image_raw clahe --frame-rate-divisor 10 --time-start 20 --time-stop 30
```

## Feature Matching

Feature tracking GIF:
```
python generate_tracking_gif.py ../trained_networks/superpoint_thermal/thermal.pth.tar ../trained_networks/superpoint_rgb/rgb.pth.tar ../../datasets/fcav/uncooled_seq_1/images_clahe_10hz_tstart_90_tstop_110/ tracking 10
```
## Vocabulary Image Similarity Scoring
Image similarity scores using thermal SuperPoint features and the thermal SuperPoint vocabulary:
```
./thirdparty/DBoW2/build/test_vocab thirdparty/DBoW2/test_vocab_data/SuperPoint_Thermal_Keypts_and_Desc/ vocabularies/superpt_thermal.yml.gz
```
Image similarity scores using thermal SuperPoint features and the RGB SuperPoint vocabulary:
```
./thirdparty/DBoW2/build/test_vocab thirdparty/DBoW2/test_vocab_data/SuperPoint_Thermal_Keypts_and_Desc/ vocabularies/superpoint_rgb.yml.gz
```
Image similarity scores using RGB SuperPoint features and the RGB SuperPoint vocabulary:
```
./thirdparty/DBoW2/build/test_vocab thirdparty/DBoW2/test_vocab_data/SuperPoint_RGB_Keypts_and_Desc/ vocabularies/superpoint_rgb.yml.gz
```
Image similarity scores using RGB SuperPoint features and the Thermal SuperPoint vocabulary:
```
./thirdparty/DBoW2/build/test_vocab thirdparty/DBoW2/test_vocab_data/SuperPoint_RGB_Keypts_and_Desc/ vocabularies/superpt_thermal.yml.gz
```
Image similarity scores using ORB features and the ORB vocabulary:
```
./thirdparty/ORB_SLAM2/Examples/Monocular/test_vocab thirdparty/DBoW2/test_vocab_data/ORB_Keypts_and_Desc/ vocabularies/ORBvoc.txt
```
## SLAM Recordings
RGB SuperPoint SLAM run on KITTI sequence 03:
```
./thirdparty/SuperPoint_SLAM/Examples/Monocular/mono_kitti vocabularies/superpoint_rgb.yml.gz thirdparty/SuperPoint_SLAM/Examples/Monocular/KITTI03.yaml ../datasets/kitti/data_odometry_gray/dataset/sequences/03/ ../datasets/kitti/data_odometry_gray/dataset/sequences/03/RGB_Feat_and_Descriptors/
```
ORB_SLAM2 run on KITTI sequence 03:
```
./thirdparty/ORB_SLAM2/Examples/Monocular/mono_kitti vocabularies/ORBvoc.txt thirdparty/ORB_SLAM2/Examples/Monocular/KITTI03.yaml ../datasets/kitti/data_odometry_gray/dataset/sequences/03/
```
Thermal SuperPoint SLAM run on thermal images:
```
./thirdparty/SuperPoint_SLAM/Examples/Monocular/mono_euroc vocabularies/superpt_thermal.yml.gz configs/ORB_SLAM2/X8500.yaml ../datasets/fcav/cooled/images_clahe_10hz_tstart_108_tstop_inf/ ../datasets/fcav/cooled/timestamps/timestamps_10hz_tstart_108_tstop_inf.txt ../datasets/fcav/cooled/features/
```
ORB_SLAM2 run on thermal images:
```
./thirdparty/ORB_SLAM2/Examples/Monocular/mono_euroc vocabularies/ORBvoc.txt configs/ORB_SLAM2/X8500.yaml ../datasets/fcav/cooled/images_clahe_10hz_tstart_108_tstop_inf/ ../datasets/fcav/cooled/timestamps/timestamps_10hz_tstart_108_tstop_inf.txt
```

# 9. ThermalKDSuperPoint による特徴抽出と SLAM

このセクションでは `thirdparty/thermal-kd-superpoint` の学習済みモデルを使って特徴量を抽出し、SuperPoint_SLAM を実行するまでの手順を示します。

## 9.1 前提条件

### Python 環境のセットアップ (Ubuntu 24.04)

プロジェクトルートに仮想環境を作成し、必要なパッケージをインストールします。

```bash
# 仮想環境の作成と依存関係のインストール
scripts/setup_ubuntu2404_env.sh

# 仮想環境を有効化
source env/bin/activate
```

このスクリプトは `env/` 以下に仮想環境を作成し、PyTorch・OpenCV・evo などの依存関係と
`thirdparty/thermal-kd-superpoint` を editable インストールします。

### SuperPoint_SLAM のビルド (Ubuntu 24.04)

Pangolin を事前にインストールしたうえで以下を実行します。Pangolin のビルド手順は
`scripts/build_pangolin_local.sh` を参照してください。

```bash
# Pangolin をローカルにビルドする場合
scripts/build_pangolin_local.sh

# SuperPoint_SLAM 本体のビルド
scripts/build_superpoint_slam_ubuntu2404.sh
```

Pangolin をカスタムパスにインストールした場合は環境変数で指定します。

```bash
export Pangolin_DIR=/path/to/Pangolin/install/lib/cmake/Pangolin
export CMAKE_PREFIX_PATH=/path/to/Pangolin/install
scripts/build_superpoint_slam_ubuntu2404.sh
```

ビルドが完了すると `thirdparty/SuperPoint_SLAM/Examples/Monocular/mono_euroc` が生成されます。

## 9.2 データの準備

入力データは ROS Bag Preprocessing（セクション 3）で作成した EuRoC 形式のディレクトリを想定しています。

```
/path/to/dataset/
  images/           # 画像ファイル (TIMESTAMP.png)
  timestamps.txt    # タイムスタンプファイル（1行1タイムスタンプ）
```

サーマル画像が 16 bit の場合は事前に CLAHE を適用してください（セクション 2・3 参照）。

## 9.3 ThermalKDSuperPoint による特徴抽出

学習済みチェックポイントは以下の 2 種類が用意されています。

| チェックポイント | 説明 |
|---|---|
| `trained_networks/superpoint_kd_thermal/newloss_vivid/best.pth` | VIVID データセットで新ロスにより学習（推奨） |
| `trained_networks/superpoint_kd_thermal/oldloss/best.pth` | 旧ロスで学習したモデル |

`--kdsp-ckpt` フラグを指定して `utils/generate_keypts_and_desc.py` を実行します。

```bash
# 仮想環境を有効化していない場合は先に実行
source env/bin/activate

python utils/generate_keypts_and_desc.py \
  --kdsp-ckpt trained_networks/superpoint_kd_thermal/newloss_vivid/best.pth \
  --image-dir /path/to/dataset/images/ \
  --out-dir /path/to/dataset/features_kdsp/
```

主なオプション:

| オプション | デフォルト | 説明 |
|---|---|---|
| `--detection-threshold` | `0.015` | キーポイント検出閾値 |
| `--nms-dist` | `4` | NMS 距離（ピクセル） |
| `--max-features` | `100000` | 1 画像あたりの最大特徴点数 |
| `--resize W H` | なし | 画像をリサイズしてから推論 |

実行結果は `--out-dir` 以下に `1.yaml`, `2.yaml`, ... と連番で保存されます。

## 9.4 ボキャブラリの確認

KD SuperPoint 用の学習済みボキャブラリが `vocabularies/` に含まれています。

| ファイル | 説明 |
|---|---|
| `vocabularies/superpt_kd_thermal.yml.gz` | VIVID サーマル画像で学習した KD SuperPoint ボキャブラリ（推奨） |
| `vocabularies/superpt_kd_stage4_max700.yml.gz` | Stage 4 / max700 設定ボキャブラリ（DBoW2 形式） |
| `vocabularies/superpt_kd_stage4_max700_dbow3.yml.gz` | Stage 4 / max700 設定ボキャブラリ（DBoW3 形式） |

既存のボキャブラリを使用する場合は 9.5 に進んでください。新たにボキャブラリを作成する場合は
セクション 6 の手順に従い `thirdparty/DBoW2/build/build_superpt_vocab` に特徴量ディレクトリを渡してください。

## 9.5 ThermalKDSuperPoint SLAM の実行

特徴量ディレクトリ・ボキャブラリ・カメラ設定ファイル・タイムスタンプファイルを揃えたうえで
`mono_euroc` を実行します。

```bash
./thirdparty/SuperPoint_SLAM/Examples/Monocular/mono_euroc \
  vocabularies/superpt_kd_thermal.yml.gz \
  configs/ORB_SLAM2/ViViD_Thermal.yaml \
  /path/to/dataset/images/ \
  /path/to/dataset/timestamps.txt \
  /path/to/dataset/features_kdsp/
```

- **`vocabularies/superpt_kd_thermal.yml.gz`**: 9.4 で選択したボキャブラリ
- **`configs/ORB_SLAM2/ViViD_Thermal.yaml`**: カメラキャリブレーション設定。カメラが異なる場合は `configs/ORB_SLAM2/` 以下の別のファイルを使用するか、新たに作成してください。
- **`/path/to/dataset/images/`**: EuRoC 形式の画像ディレクトリ
- **`/path/to/dataset/timestamps.txt`**: タイムスタンプファイル
- **`/path/to/dataset/features_kdsp/`**: 9.3 で生成した特徴量ディレクトリ（末尾 `/` を付けること）

SLAM が正常に動作すると `KeyFrameTrajectory.txt` が出力されます。

ビューアーを無効化して実行する場合は引数に `--no-viewer` を追加します。

```bash
./thirdparty/SuperPoint_SLAM/Examples/Monocular/mono_euroc \
  vocabularies/superpt_kd_thermal.yml.gz \
  configs/ORB_SLAM2/ViViD_Thermal.yaml \
  /path/to/dataset/images/ \
  /path/to/dataset/timestamps.txt \
  /path/to/dataset/features_kdsp/ \
  --no-viewer
```

## 9.6 一括実行スクリプト（run_sp_slam_and_eval.sh）

`scripts/run_sp_slam_and_eval.sh` を使うと、特徴量生成・SLAM 実行・追跡安定性評価を一括で行えます。
Baseline（pytorch-superpoint）と ThermalKDSuperPoint の両方を同時に実行して比較することも可能です。

```bash
scripts/run_sp_slam_and_eval.sh \
  --images /path/to/dataset/images/ \
  --times /path/to/dataset/timestamps.txt \
  --config configs/ORB_SLAM2/ViViD_Thermal.yaml \
  --out outputs/slam_eval/my_run \
  --only kdsp
```

ThermalKDSuperPoint のみを実行したい場合は `--only kdsp` を指定します。両モデルを比較する場合は
`--only both`（デフォルト）です。

主なオプション:

| オプション | デフォルト | 説明 |
|---|---|---|
| `--kdsp-ckpt PATH` | `trained_networks/superpoint_kd_thermal/newloss_vivid/best.pth` | KD モデルのチェックポイント |
| `--kdsp-vocab PATH` | `vocabularies/superpt_kd_thermal.yml.gz` | KD モデル用ボキャブラリ |
| `--classic-model PATH` | `trained_networks/superpoint_thermal/thermal.pth.tar` | Baseline モデル |
| `--only baseline\|kdsp\|both` | `both` | 実行するモデルの選択 |
| `--no-viewer` | — | SLAM ビューアーを無効化 |
| `--skip-feature-gen` | — | 特徴量生成をスキップして既存フォルダを再利用 |
| `--gt-tum PATH` | — | TUM 形式の GT 軌跡（evo による APE/RPE 評価を有効化） |

実行結果は `--out` で指定したディレクトリ以下に保存されます。

```
outputs/slam_eval/my_run/
  features_pytorch_superpoint/       # Baseline 特徴量
  features_thermal_kd_superpoint/    # KD SuperPoint 特徴量
  pytorch_superpoint/
    KeyFrameTrajectory.txt
    slam.log
    tracking_stability/
      tracking_stability_summary.json
  thermal_kd_superpoint/
    KeyFrameTrajectory.txt
    slam.log
    tracking_stability/
      tracking_stability_summary.json
  summary.tsv                        # 両手法の比較サマリー
```

## 9.7 VIVID データセットでの実行例

VIVID `campus_day1` シーケンスを使って KD SuperPoint SLAM を実行する例です。

```bash
# 仮想環境を有効化
source env/bin/activate

# Step 1: 特徴量の生成
python utils/generate_keypts_and_desc.py \
  --kdsp-ckpt trained_networks/superpoint_kd_thermal/newloss_vivid/best.pth \
  --image-dir /path/to/VIVID/campus_day1/img/thermal_fieldscale_clahe/ \
  --out-dir /path/to/VIVID/campus_day1/img/features_kdsp/

# Step 2: SLAM の実行
./thirdparty/SuperPoint_SLAM/Examples/Monocular/mono_euroc \
  vocabularies/superpt_kd_thermal.yml.gz \
  configs/ORB_SLAM2/ViViD_Thermal.yaml \
  /path/to/VIVID/campus_day1/img/thermal_fieldscale_clahe/ \
  /path/to/VIVID/campus_day1/thermal_timestamps.txt \
  /path/to/VIVID/campus_day1/img/features_kdsp/
```

または一括スクリプトを使った場合：

```bash
scripts/run_sp_slam_and_eval.sh \
  --images /path/to/VIVID/campus_day1/img/thermal_fieldscale_clahe/ \
  --times /path/to/VIVID/campus_day1/thermal_timestamps.txt \
  --config configs/ORB_SLAM2/ViViD_Thermal.yaml \
  --out outputs/slam_eval/vivid_campus_day1 \
  --only kdsp \
  --no-viewer
```
