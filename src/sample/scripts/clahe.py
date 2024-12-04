#!/usr/bin/env python3
import cv2
import os


input_folder = '/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data5/Data/infra'
output_folder = '/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data5/clahe/1'

# 適用的ヒストグラム平坦化
clipLimit = 2
size = 4
clahe = cv2.createCLAHE(clipLimit= clipLimit, tileGridSize=(size, size))

for filename in os.listdir(input_folder):
    if filename.endswith('.png') or filename.endswith('.jpg'):
        img_path = os.path.join(input_folder, filename)
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        cl1 = clahe.apply(img)  

        output_path = os.path.join(output_folder, filename)
        cv2.imwrite(output_path, cl1)

        cl1 = cv2.cvtColor(cl1, cv2.COLOR_GRAY2BGR)
        cv2.imshow('Image', cl1)
        cv2.waitKey(1)

cv2.destroyAllWindows()

print("全画像のコントラストと明度を統一")
