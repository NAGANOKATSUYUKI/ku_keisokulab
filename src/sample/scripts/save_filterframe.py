#!/usr/bin/env python3

import os
import shutil
from natsort import natsorted

def save_every_fifth_frame(source_dir, destination_dir):
    # ソースディレクトリからファイルリストを取得
    files = [f for f in os.listdir(source_dir)]
    
    # ファイルをソート
    files =natsorted(files, key=lambda y: y.lower())
    # フレーム番号をトラッキングするカウンター
    frame_number = 1

    # ディレクトリが存在しない場合は作成
    if not os.path.exists(destination_dir):
        os.makedirs(destination_dir)

    # 各ファイルについて処理
    for file_name in files:
        # フルパスを取得
        file_path = os.path.join(source_dir, file_name)

        # ファイルが画像の場合のみ処理
        if os.path.isfile(file_path) and file_name.lower().endswith(('.png', '.jpg', '.jpeg')):
            # フレーム番号がxの倍数のときに保存
            if frame_number % 6 == 0:
                # destination_path = os.path.join(destination_dir, file_name)
                # shutil.copy(file_path, destination_path)
                # print(f"Saved {file_name} to {destination_dir}")
                pass
            else :
                destination_path = os.path.join(destination_dir, file_name)
                shutil.copy(file_path, destination_path)
                print(f"Saved {file_name} to {destination_dir}")
                pass
            

            # フレーム番号をインクリメント
            frame_number += 1
            if frame_number == 4118 :
                break

# 使用例
source_directory = '/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data7/20241210_tukigaoka画像加工済/infra'
destination_directory = '/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data7/edge/infra'
save_every_fifth_frame(source_directory, destination_directory)
