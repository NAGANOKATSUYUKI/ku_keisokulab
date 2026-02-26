# Ubuntu 22.04 に Miniconda をインストールして仮想環境を作る手順

このREADMEは、Ubuntu 22.04でMinicondaを導入し、Python仮想環境を作成できるところまでを説明します。

## 1. 事前確認

```bash
lsb_release -a
uname -m
```

- `lsb_release -a` が `Ubuntu 22.04` であることを確認
- `uname -m` でCPUアーキテクチャを確認（`x86_64` か `aarch64`）

## 2. Miniconda インストーラをダウンロード

### x86_64 の場合

```bash
cd ~/Downloads
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
```

### aarch64 (ARM64) の場合

```bash
cd ~/Downloads
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh
```

## 3. Miniconda をインストール

### x86_64 の場合

```bash
bash Miniconda3-latest-Linux-x86_64.sh
```

### aarch64 の場合

```bash
bash Miniconda3-latest-Linux-aarch64.sh
```

インストール中のポイント:
- ライセンス確認で `yes`
- インストール先は通常 `~/miniconda3` のままでOK
- `Do you wish the installer to initialize Miniconda3 by running conda init?` は `yes`

## 4. シェル設定を反映

```bash
source ~/.bashrc
conda --version
```

`conda 23.x` や `conda 24.x` のようにバージョンが表示されれば成功です。

## 5. 仮想環境を作成

例として `py310` という名前でPython 3.10環境を作成:

```bash
conda create -n py310 python=3.10 -y
```

## 6. 仮想環境を有効化/無効化

```bash
conda activate py310
python --version
which python
```

無効化:

```bash
conda deactivate
```

## 7. よく使うコマンド

環境一覧:

```bash
conda env list
```

環境削除:

```bash
conda remove -n py310 --all -y
```

## 8. うまくいかないとき

- `conda: command not found` が出る  
  `source ~/.bashrc` を実行し、再度確認
- それでも認識されない  
  `~/miniconda3/bin/conda --version` で存在確認し、必要なら再インストール

## 9. ROS1 (Noetic) を使うための環境設定（Ubuntu 22.04 + Miniconda）

Ubuntu 22.04 では、`apt` で ROS1 Noetic をそのまま入れる構成は基本的に非推奨です。  
この手順では Miniconda 環境に ROS1 Noetic を入れる方法（RoboStack）を使います。

### 9-1. ROS用の仮想環境を作成

```bash
conda create -n ros1_noetic python=3.10 -y
conda activate ros1_noetic
```

### 9-2. RoboStack チャネルを追加

```bash
conda config --env --add channels conda-forge
conda config --env --add channels robostack-noetic
conda config --env --set channel_priority strict
```

### 9-3. ROS1 の基本パッケージをインストール

```bash
conda install ros-noetic-desktop catkin_tools -y
```

最小構成でよければ `ros-noetic-ros-base` でも可。

### 9-4. ROS環境を読み込む

```bash
source "$CONDA_PREFIX/setup.bash"
echo $ROS_DISTRO
```

`noetic` と表示されればOKです。

### 9-5. この catkin_ws をビルド

ワークスペース直下（`~/catkin_ws`）で:

```bash
cd ~/catkin_ws
catkin config --extend "$CONDA_PREFIX" --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
```

### 9-6. 毎回の起動手順（最小）

```bash
conda activate ros1_noetic
source "$CONDA_PREFIX/setup.bash"
cd ~/catkin_ws
source devel/setup.bash
```

### 9-7. 動作確認

ターミナルA:

```bash
roscore
```

ターミナルB:

```bash
conda activate ros1_noetic
source "$CONDA_PREFIX/setup.bash"
source ~/catkin_ws/devel/setup.bash
rostopic list
```

### 9-8. ROS1 関連で詰まりやすい点

- `catkin: command not found`  
  `conda activate ros1_noetic` ができているか確認
- `ROS_DISTRO` が空  
  `source "$CONDA_PREFIX/setup.bash"` を実行
- ビルド時に依存不足エラー  
  不足したROSパッケージを `conda install ros-noetic-<package名>` で追加
