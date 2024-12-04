#!/usr/bin/env python3

import os
import cv2

def crop_and_save_images(input_folder, output_folder):
    # フォルダ内のファイルに対して処理
    for filename in os.listdir(input_folder):
        if filename.endswith(('.png', '.jpg', '.jpeg')):
            # パスを構築
            input_image_path = os.path.join(input_folder, filename)
            output_image_path = os.path.join(output_folder, filename)
            input_image = cv2.imread(input_image_path)
            
            crop_x_start = 165
            crop_y_start = 62
            crop_width   = 760
            crop_height  = 570
            
            cropped_image = input_image[crop_y_start:crop_y_start+crop_height,
                                        crop_x_start:crop_x_start+crop_width]
            
            
            cv2.imwrite(output_image_path, cropped_image)
            cv2.imshow("frame_comb", cropped_image)
            cv2.waitKey(1)
            

if __name__ == '__main__':
    # フォルダ指定
    input_folder =  '/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data5/edge/infra'
    output_folder = '/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data5/edge/infra'
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
        
    crop_and_save_images(input_folder, output_folder)