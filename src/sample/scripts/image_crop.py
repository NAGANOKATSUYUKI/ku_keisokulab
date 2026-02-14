#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ディレクトリ内画像を指定矩形でクロップして保存する。"""

import cv2
import os

# ===== 設定（最初に編集する場所）=====
INPUT_FOLDER = "/home/keisoku/20250724_col_ir_image/ir"
OUTPUT_FOLDER = "/home/keisoku/20250724_col_ir_image/ir_resize"
SUPPORTED_EXTENSIONS = (".png", ".jpg", ".jpeg")

CROP_X_START = 100
CROP_Y_START = 50
CROP_WIDTH = 480
CROP_HEIGHT = 343

WINDOW_NAME = "frame_comb"

def crop_image(image):
    """設定値に基づいて画像をクロップする。"""
    return image[
        CROP_Y_START : CROP_Y_START + CROP_HEIGHT,
        CROP_X_START : CROP_X_START + CROP_WIDTH,
    ]

def crop_and_save_images(input_folder, output_folder):
    """フォルダ内の画像をクロップして保存する。"""
    for filename in os.listdir(input_folder):
        if not filename.lower().endswith(SUPPORTED_EXTENSIONS):
            continue

        input_image_path = os.path.join(input_folder, filename)
        output_image_path = os.path.join(output_folder, filename)
        input_image = cv2.imread(input_image_path)

        if input_image is None:
            print(f"スキップ: 読み込み失敗 '{input_image_path}'")
            continue

        cropped_image = crop_image(input_image)
        cv2.imwrite(output_image_path, cropped_image)
        cv2.imshow(WINDOW_NAME, cropped_image)
        cv2.waitKey(1)

def main():
    os.makedirs(OUTPUT_FOLDER, exist_ok=True)
    crop_and_save_images(INPUT_FOLDER, OUTPUT_FOLDER)

if __name__ == "__main__":
    main()