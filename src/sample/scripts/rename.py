#!/usr/bin/env python3
import os
from natsort import natsorted

def rename_files_in_directory(directory_path, extension="png"):
    # ディレクトリ内のファイルリストを取得
    files = [f for f in os.listdir(directory_path) if f.endswith(f".{extension}")]
    
    # ファイルをソート
    # natsorted(files)
    # print(natsorted(files, key=lambda y: y.lower()))
    files =natsorted(files, key=lambda y: y.lower())
    # print(files)
    # ゼロパディングの幅を計算
    padding_width = len(str(len(files)))
    # print(padding_width)

    # ファイルを連番でリネーム
    for index, old_filename in enumerate(files):
        new_filename = f"{str(index + 1).zfill(padding_width)}.{extension}"
        old_filepath = os.path.join(directory_path, old_filename)
        new_filepath = os.path.join(directory_path, new_filename)
        os.rename(old_filepath, new_filepath)
        print(f"Renamed: {old_filename} -> {new_filename}")

# 使用例
directory_path = "/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data7/edge/color"

rename_files_in_directory(directory_path)
