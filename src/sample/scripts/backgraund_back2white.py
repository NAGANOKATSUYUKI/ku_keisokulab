#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ディレクトリ内の画像で、純粋な黒 (0,0,0) のみを白に置換する。"""

import os
import cv2
import numpy as np

# ===== 設定（最初に編集する場所）=====
INPUT_DIRECTORY = "/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/20251107_train_data5/データ拡張_枚数減/0/infra"
OUTPUT_DIRECTORY = "/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/20251107_train_data5/データ拡張_枚数減/0/infra"
SUPPORTED_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp")
PURE_BLACK = np.array([0, 0, 0], dtype=np.uint8)
PURE_WHITE_VALUE = 255


def replace_pure_black_with_white(image: np.ndarray) -> np.ndarray:
    """画像内の純粋な黒ピクセルのみを白に変換する。"""
    if len(image.shape) == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    pure_black_mask = cv2.inRange(image, PURE_BLACK, PURE_BLACK)
    white_image = np.full(image.shape, PURE_WHITE_VALUE, dtype=np.uint8)
    return np.where(pure_black_mask[:, :, np.newaxis] == 255, white_image, image)


def convert_black_to_white_pure_black_in_directory(input_dir, output_dir):
    """
    指定されたディレクトリ内のすべての画像を処理し、
    ピクセル値が(0,0,0)の純粋な黒の部分のみを白に変換します。

    Args:
        input_dir (str): 入力画像が格納されているディレクトリのパス。
        output_dir (str): 変換された画像を保存するディレクトリのパス。
    """
    if not os.path.exists(input_dir):
        print(f"エラー: 入力ディレクトリ '{input_dir}' が見つかりません。")
        return

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"出力ディレクトリ '{output_dir}' を作成しました。")

    print(f"ディレクトリ '{input_dir}' 内の画像を処理中...")
    processed_count = 0

    for filename in os.listdir(input_dir):
        if not filename.lower().endswith(SUPPORTED_EXTENSIONS):
            continue

        input_image_path = os.path.join(input_dir, filename)
        output_image_path = os.path.join(output_dir, filename)

        try:
            img = cv2.imread(input_image_path)
            if img is None:
                print(f"  スキップ: 画像 '{filename}' を読み込めませんでした。")
                continue

            result_img = replace_pure_black_with_white(img)
            cv2.imwrite(output_image_path, result_img)
            print(f"  変換完了: '{filename}' -> '{output_image_path}'")
            processed_count += 1
        except Exception as error:
            print(f"  エラー: '{filename}' の処理中に問題が発生しました: {error}")

    print(f"\n処理が完了しました。合計 {processed_count} 個の画像を変換しました。")


if __name__ == "__main__":
    convert_black_to_white_pure_black_in_directory(INPUT_DIRECTORY, OUTPUT_DIRECTORY)
