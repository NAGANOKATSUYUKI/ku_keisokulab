#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""2枚の画像をクリックし、同じ座標の画素値を比較表示する。"""

import cv2

# ===== 設定（最初に編集する場所）=====
IMAGE_PATH_1 = "/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/20251107_train_data5/データ拡張_枚数減/0/color/0007.png"
IMAGE_PATH_2 = "/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/20251107_train_data5/データ拡張_枚数減/0/infra/0007.png"
WINDOW_NAME_1 = "Image 1"
WINDOW_NAME_2 = "Image 2"
QUIT_KEY = "q"


def get_pixel_value(img, x, y, window_name):
    """指定画像の (x, y) の画素値を文字列で返す。"""
    if y < 0 or y >= img.shape[0] or x < 0 or x >= img.shape[1]:
        return f"[{window_name}] (x={x}, y={y}): 範囲外"

    if len(img.shape) == 3:
        b, g, r = img[y, x]
        return f"[{window_name}] (x={x}, y={y}): RGB = ({r}, {g}, {b})"

    gray_value = img[y, x]
    return f"[{window_name}] (x={x}, y={y}): Gray = {gray_value}"


def mouse_callback(event, x, y, flags, param):
    """左クリック時に、2画像の同一座標画素値を表示する。"""
    del flags  # OpenCV のコールバック引数仕様上必要だが未使用
    img1 = param["img1"]
    img2 = param["img2"]
    clicked_window_name = param["clicked_window_name"]

    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"\n--- {clicked_window_name} でクリックされました ---")
        print(get_pixel_value(img1, x, y, WINDOW_NAME_1))
        print(get_pixel_value(img2, x, y, WINDOW_NAME_2))


def main():
    img1 = cv2.imread(IMAGE_PATH_1, cv2.IMREAD_COLOR)
    img2 = cv2.imread(IMAGE_PATH_2, cv2.IMREAD_COLOR)

    if img1 is None:
        print(f"エラー: 画像1 '{IMAGE_PATH_1}' を読み込めませんでした。")
        return
    if img2 is None:
        print(f"エラー: 画像2 '{IMAGE_PATH_2}' を読み込めませんでした。")
        return

    cv2.namedWindow(WINDOW_NAME_1)
    cv2.setMouseCallback(
        WINDOW_NAME_1,
        mouse_callback,
        {"img1": img1, "img2": img2, "clicked_window_name": WINDOW_NAME_1},
    )

    cv2.namedWindow(WINDOW_NAME_2)
    cv2.setMouseCallback(
        WINDOW_NAME_2,
        mouse_callback,
        {"img1": img1, "img2": img2, "clicked_window_name": WINDOW_NAME_2},
    )

    cv2.imshow(WINDOW_NAME_1, img1)
    cv2.imshow(WINDOW_NAME_2, img2)

    print(
        f"いずれかの画像ウィンドウをクリックしてください。"
        f" '{QUIT_KEY}' キーで終了します。"
    )

    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord(QUIT_KEY):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
