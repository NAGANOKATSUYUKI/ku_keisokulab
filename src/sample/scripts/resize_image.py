#!/usr/bin/env python3
import os
import cv2

def resize_and_save_images(input_folder, output_folder, target_size=(640, 480)):
    # フォルダ内のファイルに対して処理
    for filename in os.listdir(input_folder):
        if filename.endswith(('.png', '.jpg', '.jpeg')):
            # パスを構築
            input_image_path = os.path.join(input_folder, filename)
            output_image_path = os.path.join(output_folder, filename)
            
            input_image = cv2.imread(input_image_path)
            resized_image = cv2.resize(input_image, target_size, interpolation=cv2.INTER_NEAREST_EXACT)
            cv2.imwrite(output_image_path, resized_image)
            height, width = resized_image.shape[:2]
            print(f"Resized Image Size - Width: {width}, Height: {height}")

if __name__ == '__main__':
    # フォルダ指定
    input_folder =  '/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data7/20241210_tukigaoka画像加工済/color'
    output_folder = input_folder
    # output_folder = '/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data6/20241203_tukigaoka画像加工/infra'
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
        
    target_size = (512, 512)
    resize_and_save_images(input_folder, output_folder, target_size)
