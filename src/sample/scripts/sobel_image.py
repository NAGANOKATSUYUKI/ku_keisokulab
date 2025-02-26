#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import cv2 as cv
import numpy as np

# 入力画像を読み込み
img = cv.imread("/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data2/Data_0705/infra/0001.png")

# グレースケール変換
gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY)

# 方法3: Sobelフィルタでエッジ検出
gray_x = cv.Sobel(gray, cv.CV_32F, 1, 0, ksize=3)
gray_y = cv.Sobel(gray, cv.CV_32F, 0, 1, ksize=3)
dst = np.sqrt(gray_x ** 2 + gray_y ** 2)

# エッジ強度の平均値を計算
average_edge_strength = np.mean(dst)

# 平均値を出力
print("Average Edge Strength:", average_edge_strength)

# 結果を出力
cv.imwrite("/home/keisoku/output.png", dst)
