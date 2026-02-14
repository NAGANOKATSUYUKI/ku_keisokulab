#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""2つの画像ディレクトリを対応付け、黒画素をグレースケール値で置換して保存する。"""

import os
import cv2
import numpy as np

# ===== 設定（最初に編集する場所）=====
DIR1_PATH = "/home/keisoku/20251031_近距離物体/crop/color_crop"
DIR2_PATH = "/home/keisoku/20251031_近距離物体/crop/ir_homo_crop"
OUTPUT_DIR = "/home/keisoku/20251031_近距離物体/crop/combi_crop"
SUPPORTED_EXTENSIONS = (".png", ".jpg", ".jpeg", ".bmp", ".tiff")

def list_image_files(directory_path: str) -> list[str]:
    """指定ディレクトリ内の画像ファイル名一覧をソートして返す。"""
    return sorted(
        filename
        for filename in os.listdir(directory_path)
        if filename.lower().endswith(SUPPORTED_EXTENSIONS)
    )

def replace_black_with_gray_values(
    color_image: np.ndarray, gray_image: np.ndarray
) -> np.ndarray:
    """color_image の (0,0,0) 画素を gray_image の値で置換する。"""
    zero_rgb_locations = np.all(color_image == [0, 0, 0], axis=-1)
    modified = color_image.copy()
    modified[zero_rgb_locations] = np.stack([gray_image[zero_rgb_locations]] * 3, axis=-1)
    return modified

def process_images_in_directories(dir1_path: str, dir2_path: str, output_dir: str) -> None:
    """2ディレクトリの画像を順番対応で処理し、結果を output_dir に保存する。"""
    os.makedirs(output_dir, exist_ok=True)

    images1 = list_image_files(dir1_path)
    images2 = list_image_files(dir2_path)

    if not images1:
        print(f"エラー: {dir1_path} に画像が見つかりませんでした。")
        return
    if not images2:
        print(f"エラー: {dir2_path} に画像が見つかりませんでした。")
        return
    if len(images1) != len(images2):
        print("警告: ディレクトリ1とディレクトリ2の画像の枚数が異なります。")

    for index, filename1 in enumerate(images1):
        if index >= len(images2):
            print(f"警告: {filename1} に対応する画像がディレクトリ2にありません。スキップします。")
            continue

        filename2 = images2[index]
        image1_path = os.path.join(dir1_path, filename1)
        image2_path = os.path.join(dir2_path, filename2)
        output_image_path = os.path.join(output_dir, filename1)

        print(f"Processing: {filename1} and {filename2}...")

        image1 = cv2.imread(image1_path, cv2.IMREAD_COLOR)
        image2 = cv2.imread(image2_path, cv2.IMREAD_GRAYSCALE)

        if image1 is None:
            print(f"エラー: {image1_path} を読み込めませんでした。スキップします。")
            continue
        if image2 is None:
            print(f"エラー: {image2_path} を読み込めませんでした。スキップします。")
            continue
        if image1.shape[:2] != image2.shape[:2]:
            print(f"エラー: {filename1} と {filename2} のサイズが異なります。スキップします。")
            continue

        modified_image = replace_black_with_gray_values(image1, image2)

        if cv2.imwrite(output_image_path, modified_image):
            print(f"保存済み: {output_image_path}")
        else:
            print(f"エラー: {output_image_path} の保存に失敗しました。")

def main() -> None:
    process_images_in_directories(DIR1_PATH, DIR2_PATH, OUTPUT_DIR)

if __name__ == "__main__":
    main()
