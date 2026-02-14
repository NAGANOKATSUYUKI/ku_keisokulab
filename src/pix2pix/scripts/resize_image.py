#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ディレクトリ内画像を指定サイズにリサイズして保存する。"""

import os
import cv2

# ===== 設定（最初に編集する場所）=====
INPUT_FOLDER = "/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/20251105_train_data4/背景なし/color_filtering"
OUTPUT_FOLDER = INPUT_FOLDER
TARGET_SIZE = (512, 512)  # (width, height)
SUPPORTED_EXTENSIONS = (".png", ".jpg", ".jpeg")
INTERPOLATION = cv2.INTER_NEAREST_EXACT


def resize_and_save_images(input_folder, output_folder, target_size):
    """入力フォルダの画像をリサイズして出力フォルダに保存する。"""
    for filename in os.listdir(input_folder):
        if not filename.lower().endswith(SUPPORTED_EXTENSIONS):
            continue

        input_image_path = os.path.join(input_folder, filename)
        output_image_path = os.path.join(output_folder, filename)
        input_image = cv2.imread(input_image_path)

        if input_image is None:
            print(f"スキップ: 読み込み失敗 '{input_image_path}'")
            continue

        resized_image = cv2.resize(input_image, target_size, interpolation=INTERPOLATION)
        cv2.imwrite(output_image_path, resized_image)
        height, width = resized_image.shape[:2]
        print(f"Resized Image Size - Width: {width}, Height: {height}")


def main():
    os.makedirs(OUTPUT_FOLDER, exist_ok=True)
    resize_and_save_images(INPUT_FOLDER, OUTPUT_FOLDER, TARGET_SIZE)


if __name__ == "__main__":
    main()
