#!/usr/bin/env python3

import cv2
import os
import glob

# 画像ファイルのパスのリストを取得
image_paths = sorted(glob.glob("/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data7/edge/Test_image/infra/*"))

# 保存先ディレクトリ
output_dir = "/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data7/edge/Test_image/origin_edge"
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

        # Cannyエッジ検出
        edges = cv2.Canny(gray, 30, 50)

        # エッジ線を膨張させて太くする
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))  # カーネルサイズを変更するとエッジ線の太さが調整できる
        edges_dilated = cv2.dilate(edges, kernel, iterations=1)

        # 色付きのエッジ画像
        edges_color = cv2.cvtColor(edges_dilated, cv2.COLOR_GRAY2BGR)

        # 元の画像とエッジ画像を合成
        comb_images = cv2.addWeighted(frame, 0.8, edges_color, 0.2, 0)

        # 結果の保存
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


# #depth_anythingを使った時エッジの合成
# import cv2
# import os
# import glob

# # 入力画像のパスリストを取得
# input_dir = "/home/keisoku/20241203_gomi/depth_any/*"
# corresponding_infra_dir = "/home/keisoku/20241203_gomi/"
# image_paths = sorted(glob.glob(input_dir))

# # 画像ファイルが格納されているディレクトリのパス
# conb_dir = "/home/keisoku/20241203_gomi/a/*"  # ここがディレクトリであることに注意
# conb_paths = sorted(glob.glob(conb_dir))

# print(f"Number of images in 'depth_any': {len(image_paths)}")
# print(f"Number of images in 'a': {len(conb_paths)}")

# # 画像ファイルの数が一致していることを確認
# if len(image_paths) != len(conb_paths):
#     print("Error: The number of images in 'depth_any' and 'a' directories do not match.")
# else:
#     # 複数の画像に対して処理
#     for img_path, conb_path in zip(image_paths, conb_paths):
#         # 画像を読み込む
#         image = cv2.imread(img_path)
#         if image is None:
#             print(f"Error: Could not load image {img_path}")
#             continue
        
#         # conb画像を読み込む
#         conb_image = cv2.imread(conb_path)
#         if conb_image is None:
#             print(f"Error: Could not load image {conb_path}")
#             continue
        
#         # 対応する infra ファイルのパスを計算
#         file_name = os.path.basename(img_path)
#         infra_output_path = os.path.join(corresponding_infra_dir, file_name)
        
#         # 元画像処理
#         frame = image.copy()
#         print(f"Processing: {img_path} -> {infra_output_path}")

#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         edges = cv2.Canny(gray, 50, 50)
#         edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

#         # 合成のため、conb_image をグレースケールに変換
#         conbi = cv2.cvtColor(conb_image, cv2.COLOR_BGR2GRAY)

#         # サイズを揃える
#         conbi_resized = cv2.resize(conbi, (edges_color.shape[1], edges_color.shape[0]))

#         # グレースケール画像をカラーに変換
#         conbi_resized_color = cv2.cvtColor(conbi_resized, cv2.COLOR_GRAY2BGR)

#         # 画像を合成
#         comb_images = cv2.addWeighted(conbi_resized_color, 0.8, edges_color, 0.2, 0)

#         # 対応する infra ファイルに上書き保存
#         cv2.imwrite(infra_output_path, comb_images)
#         # cv2.imshow("frame_comb", comb_images)

#         cv2.waitKey(0)
#         cv2.destroyAllWindows()
