#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ディレクトリ内の画像に CLAHE を適用して保存し、結果を表示する。"""

import cv2
import os

# ===== 設定（最初に編集する場所）=====
INPUT_FOLDER = "/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data5/Data/infra"
OUTPUT_FOLDER = "/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data5/clahe/1"
SUPPORTED_EXTENSIONS = (".png", ".jpg")
CLIP_LIMIT = 2
TILE_GRID_SIZE = 4
WINDOW_NAME = "Image"

def apply_clahe_to_directory(input_folder: str, output_folder: str) -> None:
    """入力ディレクトリの画像に CLAHE を適用し、出力ディレクトリへ保存する。"""
    if not os.path.isdir(input_folder):
        raise NotADirectoryError(f"Input folder not found: {input_folder}")

    os.makedirs(output_folder, exist_ok=True)
    clahe = cv2.createCLAHE(
        clipLimit=CLIP_LIMIT, tileGridSize=(TILE_GRID_SIZE, TILE_GRID_SIZE)
    )

    processed_count = 0
    for filename in os.listdir(input_folder):
        if not filename.lower().endswith(SUPPORTED_EXTENSIONS):
            continue

        input_path = os.path.join(input_folder, filename)
        output_path = os.path.join(output_folder, filename)

        image = cv2.imread(input_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            print(f"スキップ: 読み込み失敗 '{input_path}'")
            continue

        enhanced = clahe.apply(image)
        cv2.imwrite(output_path, enhanced)

        preview = cv2.cvtColor(enhanced, cv2.COLOR_GRAY2BGR)
        cv2.imshow(WINDOW_NAME, preview)
        cv2.waitKey(1)
        processed_count += 1

    cv2.destroyAllWindows()
    print(f"全画像のコントラストと明度を統一: {processed_count} files")


if __name__ == "__main__":
    apply_clahe_to_directory(INPUT_FOLDER, OUTPUT_FOLDER)
