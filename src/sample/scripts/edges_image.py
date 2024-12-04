#!/usr/bin/env python3

import cv2
import os
import glob

# 画像ファイルのパスのリストを取得
# image_paths = sorted(glob.glob("/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data4/Data_edges/infra/*"))
image_paths = sorted(glob.glob("/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data5/clahe/B/test/*"))

# 保存先ディレクトリ
output_dir = "/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data5/clahe/B/test"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

padding_width = 3

try:
    for i, image_path in enumerate(image_paths):
        image = cv2.imread(image_path)
        if image is None:
            print(f"Error: Could not load image {image_path}")
            continue

        frame = image.copy()
        print(f"{image_path}")
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        edges = cv2.Canny(gray, 70, 70)

        edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        comb_images = cv2.addWeighted(frame, 0.8, edges_color, 0.2, 0)

        comb_output_path = os.path.join(output_dir, f"{str(i + 1).zfill(padding_width)}.png")
        
        cv2.imwrite(comb_output_path, comb_images)
        cv2.imshow("frame_comb", comb_images)

        # 'q' キーが押されたらループを抜ける
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f"An error occurred: {e}")
finally:
    cv2.destroyAllWindows()
