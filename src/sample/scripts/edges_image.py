#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ディレクトリ内の画像に Canny エッジを重ねて保存する。"""

import glob
import os

import cv2
import numpy as np
from tqdm import tqdm

# ===== 設定（最初に編集する場所）=====
INPUT_GLOB_PATTERN = "/home/keisoku/sam2/6_Sam2/ir_filtering_homo_masked/*"
OUTPUT_DIR = "/home/keisoku/sam2/6_Sam2/ir_filtering_homo_masked_edge"
PADDING_WIDTH = 3
CANNY_THRESHOLD_1 = 200
CANNY_THRESHOLD_2 = 200
WINDOW_NAME = "frame_comb"
QUIT_KEY = "q"


def draw_white_edges(image: np.ndarray) -> np.ndarray:
    """画像に Canny エッジを白で上書きした結果を返す。"""
    frame = image.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, CANNY_THRESHOLD_1, CANNY_THRESHOLD_2)
    y_coords, x_coords = np.where(edges > 0)
    frame[y_coords, x_coords] = [255, 255, 255]
    return frame


def main() -> None:
    image_paths = sorted(glob.glob(INPUT_GLOB_PATTERN))
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    try:
        for index, image_path in enumerate(tqdm(image_paths, desc="Now Edgesing"), start=1):
            image = cv2.imread(image_path)
            if image is None:
                print(f"Error: Could not load image {image_path}")
                continue

            frame = draw_white_edges(image)
            output_path = os.path.join(OUTPUT_DIR, f"{str(index).zfill(PADDING_WIDTH)}.png")
            cv2.imwrite(output_path, frame)
            cv2.imshow(WINDOW_NAME, frame)

            if cv2.waitKey(10) & 0xFF == ord(QUIT_KEY):
                break

        print("全ての画像にエッジ検出・上書きしました！")
    except Exception as error:
        print(f"An error occurred: {error}")
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
