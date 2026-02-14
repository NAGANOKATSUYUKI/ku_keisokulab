#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""画像にSobelを適用し、エッジ強度平均を算出して保存する。"""

import cv2 as cv
import numpy as np

# ===== 設定（最初に編集する場所）=====
INPUT_IMAGE_PATH = "/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data2/Data_0705/infra/0001.png"
OUTPUT_IMAGE_PATH = "/home/keisoku/output.png"
SOBEL_KERNEL_SIZE = 3


def compute_sobel_magnitude(image):
    """画像からSobelエッジ強度（勾配ノルム）を計算する。"""
    gray = cv.cvtColor(image, cv.COLOR_RGB2GRAY)
    gray_x = cv.Sobel(gray, cv.CV_32F, 1, 0, ksize=SOBEL_KERNEL_SIZE)
    gray_y = cv.Sobel(gray, cv.CV_32F, 0, 1, ksize=SOBEL_KERNEL_SIZE)
    return np.sqrt(gray_x ** 2 + gray_y ** 2)


def main():
    img = cv.imread(INPUT_IMAGE_PATH)
    if img is None:
        raise FileNotFoundError(f"Input image not found: {INPUT_IMAGE_PATH}")

    dst = compute_sobel_magnitude(img)
    average_edge_strength = np.mean(dst)
    print("Average Edge Strength:", average_edge_strength)
    cv.imwrite(OUTPUT_IMAGE_PATH, dst)


if __name__ == "__main__":
    main()
