#!/usr/bin/env python3
# import cv2
# import numpy as np

# # 読み込み
# image = cv2.imread('/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data4/Data_edges/Infra/Infra_frame_80.png')

# # コントラストと明るさの変更
# alpha = 1.5  # コントラストの倍率（1より大きい値でコントラストが上がる）
# beta = 10  # 明るさの調整値（正の値で明るくなる）
# adjusted_image1 = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)

# alpha2 = 2.0
# beta2 = 1
# adjusted_image2 = cv2.convertScaleAbs(image, alpha=alpha2, beta=beta2)

# alpha3 = 2.5  
# beta3 = 5
# adjusted_image3 = cv2.convertScaleAbs(image, alpha=alpha3, beta=beta3)

# alpha4 = 2.0  
# beta4 = 5
# adjusted_image4 = cv2.convertScaleAbs(image, alpha=alpha4, beta=beta4)

# alpha5 = 2.0
# beta5 = 5 
# adjusted_image5 = cv2.convertScaleAbs(image, alpha=alpha5, beta=beta5)

# # 変更後の画像の表示
# edges = cv2.Canny(image, 27, 27)
# edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
# edges_image = cv2.addWeighted(image, 0.8, edges_color, 0.2, 0)
# cv2.imshow("Original Image", edges_image)
# # cv2.imshow('convertScaleAbs1', adjusted_image1)
# # cv2.imshow('convertScaleAbs2', adjusted_image2)
# # cv2.imshow('convertScaleAbs3', adjusted_image3)
# # cv2.imshow('convertScaleAbs4', adjusted_image4)
# # cv2.imshow('convertScaleAbs5', adjusted_image5)

# #トーンカーブとエッジ
# min_val = 50
# max_val = 200
# table = np.arange(256, dtype=np.uint8)
# for i in range(0, min_val):
#     table[i] = 0
# for i in range(min_val, max_val):
#     table[i] = 255 * (i - min_val) / (max_val - min_val)
# for i in range(max_val, 256):
#     table[i] = 255
# img_dst = cv2.LUT(image, table)

# edges = cv2.Canny(img_dst, 27, 27)
# edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
# img_dst = cv2.addWeighted(img_dst, 0.8, edges_color, 0.2, 0)

# cv2.imshow('image', img_dst)

# min_val = 70
# max_val = 200
# table = np.arange(256, dtype=np.uint8)
# for i in range(0, min_val):
#     table[i] = 0
# for i in range(min_val, max_val):
#     table[i] = 255 * (i - min_val) / (max_val - min_val)
# for i in range(max_val, 256):
#     table[i] = 255
# img_dst = cv2.LUT(image, table)

# edges = cv2.Canny(img_dst, 27, 27)
# edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
# img_dst = cv2.addWeighted(img_dst, 0.8, edges_color, 0.2, 0)

# cv2.imshow('image2', img_dst)

# #二値化
# thresh = 110
# ret, img_thresh = cv2.threshold(image, thresh, 255, cv2.THRESH_BINARY)
# # cv2.imshow('thresh', img_thresh)



# cv2.waitKey(0)
# cv2.destroyAllWindows()

import cv2
import os

# 画像が保存されているディレクトリのパスを指定
input_dir = "/home/keisoku/ROBOT_data/Robot_data0718/fake/infra/edge_resize"  # ここに入力ディレクトリのパスを指定
output_dir = "/home/keisoku/gomi3/fake"  # ここに出力ディレクトリのパスを指定

# 出力ディレクトリが存在しない場合は作成
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# ディレクトリ内の全てのファイルを処理
for filename in os.listdir(input_dir):
    # 画像ファイルのみを処理
    if filename.endswith((".jpg", ".png", ".jpeg")):  # 必要に応じて拡張子を追加
        img_path = os.path.join(input_dir, filename)
        img = cv2.imread(img_path)
        
        if img is not None:
            # サンプル1の切り出し、保存
            img1 = img[0 : 350, 0 : 640]
            output_path1 = os.path.join(output_dir, f"sample1_{filename}")
            cv2.imwrite(output_path1, img1)
            
        else:
            print(f"Failed to load image: {filename}")
