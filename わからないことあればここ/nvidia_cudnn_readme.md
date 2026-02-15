# NVIDIA / CUDA / cuDNN セットアップ README

このドキュメントは `nvidia_cudnn` の内容をもとに、Ubuntu環境での NVIDIA ドライバ・CUDA・cuDNN の導入手順を整理したものです。

## CUDAとGPUの関係（初学者向け）

- `GPU` は計算を行うハードウェアです。
- `CUDA` は NVIDIA製GPUを使って計算するためのソフトウェア基盤です。
- `cuDNN` は深層学習向けに最適化された計算ライブラリで、CUDAの上で動きます。

イメージ:
- GPU = 計算する機械本体
- CUDA = その機械を使うための共通ルール
- cuDNN = 深層学習専用の高速機能

## インストールすると何が変わるか

1. AI処理が高速化する  
   CPUのみより、学習・推論・画像処理が大幅に速くなります。
2. PyTorch/TensorFlowでGPU計算が使える  
   `torch.cuda.is_available()` が `True` になる状態を目指します。
3. 重いモデルを扱いやすくなる  
   より大きなモデルやデータを実用的な時間で処理しやすくなります。

## 初学者がつまずきやすい点

- ドライバ / CUDA / cuDNN / PyTorch のバージョン対応が必要です。
- 組み合わせが合わないと、GPUが認識されない・実行時エラーになることがあります。
- インストール後は必ず `nvidia-smi`、`nvcc --version`、PyTorch の確認を実施してください。

## 0. 事前注意

- 既存環境を削除するコマンドは強力です。実行前に必要な環境のバックアップを取ってください。
- GPU 型番ごとに対応ドライバや CUDA バージョンが異なります。
- PyTorch などのライブラリは、CUDA との対応バージョンを必ず確認してください。

## 1. 古いバージョンの削除（必要時のみ）

```bash
sudo apt-get --purge remove nvidia*
sudo apt-get --purge remove cuda*
sudo apt-get --purge remove cudnn*
sudo apt-get --purge remove libnvidia*
sudo apt-get --purge remove libcuda*
sudo apt-get --purge remove libcudnn*
sudo apt-get autoremove
sudo apt-get autoclean
sudo apt-get update
sudo rm -rf /usr/local/cuda*
```

## 2. インストール順

1. NVIDIA ドライバ
2. CUDA
3. cuDNN
4. Python/PyTorch 側でGPU認識確認

補足: 環境によっては CUDA インストール時に NVIDIA ドライバも自動で導入されます。

## 3. NVIDIA ドライバ

参考:
- https://www.kkaneko.jp/tools/ubuntu/ubuntu_cudnn.html

メモ:
- 上記ページは「ドライバー確認」までは参考になるが、それ以降は環境によって差異あり。

## 4. CUDA インストール

公式:
- https://developer.nvidia.com/cuda-downloads?target_os=Linux

手順:
1. 自分のOS/アーキテクチャに合う項目を選択
2. 表示されるコマンドに従ってインストール
3. `~/.bashrc` にパスを追加

```bash
export PATH="/usr/local/cuda/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH"
```

注意:
- 古い CUDA のパス設定が残っている場合は削除またはコメントアウトする。

確認:

```bash
nvcc -V
```

## 5. cuDNN インストール

公式:
- https://developer.nvidia.com/rdp/cudnn-download
- https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html#download

手順メモ:
1. NVIDIA アカウントでログインしてダウンロード
2. インストールガイドの Linux 手順を確認
3. インストール後に動作確認

## 6. 動作確認コマンド

### ドライバ / CUDA 確認

```bash
nvidia-smi
nvcc --version
```

### PyTorch でGPU利用確認

```python
import torch
print(torch.__version__)
print(torch.cuda.is_available())
print(torch.cuda.device_count())
print(torch.cuda.get_device_name(0))
```

## 7. 実環境メモ（2024年12月16日時点）

- `nvidia-smi`: Driver `535.104.05`, CUDA `12.2`
- `nvcc --version`: CUDA `12.2` (`V12.2.140`)
- Python: `3.8.10`
- PyTorch: `2.4.1+cu121`
- GPU認識: `True`（`NVIDIA GeForce RTX 3070 Ti Laptop GPU`）

## 8. 参考リンク

- NVIDIA ドライバ / CUDA / cuDNN 対応メモ  
  https://scrapbox.io/programming-notes/NVidia%E3%83%89%E3%83%A9%E3%82%A4%E3%83%90%E3%83%BBCUDA%E3%83%BBcuDNN%E3%81%AE%E3%83%90%E3%83%BC%E3%82%B8%E3%83%A7%E3%83%B3%E5%AF%BE%E5%BF%9C_+_tensorflow%E3%81%AE%E5%AF%BE%E5%BF%9C
