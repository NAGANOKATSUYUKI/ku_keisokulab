#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ホモグラフィ関連の複数機能を、起動時メニューで選んで実行する。"""

import os
import cv2
import numpy as np

# ===== 設定（最初に編集する場所）=====
MODE_AUTO_H = "1"  # 自動推定モード
MODE_MANUAL_H = "2"  # 手動4点指定モード
MODE_FIXED_TEST = "3"  # 固定行列で1枚テスト
MODE_BATCH_WARP = "4"  # 固定行列で一括変換
MODE_MATCH_TEST = "5"  # マッチング誤差確認

# 1) 自動で射影変換行列を推定
AUTO_IR_PATH = "/home/keisoku/ピクチャ/1.png"  # 自動推定で使うIR入力画像
AUTO_RGB_PATH = "/home/keisoku/ピクチャ/2.png"  # 自動推定で使うRGB入力画像
AUTO_MATCH_TOP_N = 50
AUTO_RANSAC_THRESH = 5.0
AUTO_MATRIX_SAVE_PATH = "/home/keisoku/Mfixed.npy"  # 自動推定行列の保存先
AUTO_WARPED_SAVE_PATH = "/home/keisoku/warped_ir_image.png"  # 自動推定後IR画像の保存先
AUTO_PREVIEW_MS = 6000

# 2) 手動で4点対応から射影変換行列を推定
MANUAL_IR_PATH = "/home/keisoku/20251031_近距離物体/ir/00001_ir.png"  # 手動点選択で使うIR入力画像
MANUAL_RGB_PATH = "/home/keisoku/20251031_近距離物体/color/00001_color.png"  # 手動点選択で使うRGB入力画像
MANUAL_MATRIX_SAVE_PATH = "/home/keisoku/Mfixed.npy"  # 手動推定行列の保存先
MANUAL_WARPED_SAVE_PATH = "/home/keisoku/warped_ir_image.png"  # 手動推定後IR画像の保存先
MANUAL_PREVIEW_MS = 6000

# 3) 固定行列で1枚テスト
FIXED_IR_PATH = "/home/keisoku/infra.png"  # 固定行列テストで使うIR入力画像
FIXED_RGB_PATH = "/home/keisoku/color.png"  # 固定行列テストで使うRGB入力画像
FIXED_MATRIX_PATH = "/home/keisoku/Mfixed.npy"  # 固定行列の読み込み先
FIXED_WARPED_SAVE_PATH = "/home/keisoku/warped_ir_image.png"  # 固定行列テスト結果の保存先
FIXED_PREVIEW_MS = 6000
FIXED_DEFAULT_MATRIX = np.array(
    [
        [1.3131, -0.0349, -128.0381],
        [0.0170, 1.3393, -47.1509],
        [0.000049, -0.000128, 1.0],
    ]
)

# 4) 複数画像を固定行列で変換
BATCH_IR_DIR = "/home/keisoku/20251031_近距離物体/ir"  # 一括処理のIR入力フォルダ
BATCH_RGB_DIR = "/home/keisoku/20251031_近距離物体/color"  # 一括処理のRGB入力フォルダ
BATCH_OUTPUT_DIR = "/home/keisoku/20251031_近距離物体/ir_homo"  # 一括処理結果の出力フォルダ
BATCH_MATRIX_PATH = "/home/keisoku/20251031_近距離物体/Mfixed.npy"  # 一括処理で使う行列ファイル

# 5) 射影変換後画像のマッチング誤差確認
MATCH_RGB_PATH = "/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/20251107_train_data5/color/0007.png"  # 誤差評価で使うRGB入力画像
MATCH_WARPED_IR_PATH = "/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/20251107_train_data5/infra/0007.png"  # 誤差評価で使う射影後IR画像
MATCH_TOP_N = 1000
MATCH_PREVIEW_MS = 6000


def ensure_image_loaded(image, path):
    if image is None:
        raise FileNotFoundError(f"画像を読み込めませんでした: {path}")


def load_matrix_or_default(matrix_path, default_matrix):
    if os.path.exists(matrix_path):
        matrix = np.load(matrix_path)
        print(f"ファイルから射影変換行列を読み込みました: {matrix_path}")
        return matrix
    print("行列ファイルがないため既定行列を使用します。")
    return default_matrix


def auto_estimate_homography():
    ir = cv2.imread(AUTO_IR_PATH, cv2.IMREAD_GRAYSCALE)
    rgb = cv2.imread(AUTO_RGB_PATH)
    ensure_image_loaded(ir, AUTO_IR_PATH)
    ensure_image_loaded(rgb, AUTO_RGB_PATH)

    rgb_gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create(nfeatures=1000)
    kp1, des1 = orb.detectAndCompute(ir, None)
    kp2, des2 = orb.detectAndCompute(rgb_gray, None)
    if des1 is None or des2 is None:
        raise RuntimeError("特徴点が不足しているため行列を推定できません。")

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = sorted(bf.match(des1, des2), key=lambda m: m.distance)
    if len(matches) < 4:
        raise RuntimeError("マッチ数が不足しています（4点未満）。")

    src_pts = np.float32([kp1[m.queryIdx].pt for m in matches[:AUTO_MATCH_TOP_N]]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches[:AUTO_MATCH_TOP_N]]).reshape(-1, 1, 2)
    matrix, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, AUTO_RANSAC_THRESH)
    if matrix is None:
        raise RuntimeError("ホモグラフィ推定に失敗しました。")

    print("射影変換行列:\n", matrix)
    warped_ir = cv2.warpPerspective(ir, matrix, (rgb.shape[1], rgb.shape[0]))
    warped_ir_bgr = cv2.cvtColor(warped_ir, cv2.COLOR_GRAY2BGR)
    overlay = cv2.addWeighted(rgb, 0.5, warped_ir_bgr, 0.5, 0)
    cv2.imshow("Overlay: RGB + Warped IR", overlay)
    cv2.waitKey(AUTO_PREVIEW_MS)
    cv2.destroyAllWindows()

    np.save(AUTO_MATRIX_SAVE_PATH, matrix)
    cv2.imwrite(AUTO_WARPED_SAVE_PATH, warped_ir_bgr)
    print(f"保存しました: {AUTO_MATRIX_SAVE_PATH}")
    print(f"保存しました: {AUTO_WARPED_SAVE_PATH}")


def manual_estimate_homography():
    ir_points = []
    rgb_points = []
    state = {"collecting_ir": True}

    def mouse_callback(event, x, y, flags, param):
        del flags, param
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if state["collecting_ir"]:
            ir_points.append((x, y))
            print(f"IR点: {x}, {y}")
        else:
            rgb_points.append((x, y))
            print(f"RGB点: {x}, {y}")

    ir = cv2.imread(MANUAL_IR_PATH, cv2.IMREAD_GRAYSCALE)
    rgb = cv2.imread(MANUAL_RGB_PATH)
    ensure_image_loaded(ir, MANUAL_IR_PATH)
    ensure_image_loaded(rgb, MANUAL_RGB_PATH)

    if ir.shape != rgb.shape[:2]:
        ir = cv2.resize(ir, (rgb.shape[1], rgb.shape[0]))

    cv2.namedWindow("Click IR Points")
    cv2.setMouseCallback("Click IR Points", mouse_callback)
    print("IR画像上で4点クリックしてください")
    while len(ir_points) < 4:
        disp = cv2.cvtColor(ir.copy(), cv2.COLOR_GRAY2BGR)
        for pt in ir_points:
            cv2.circle(disp, pt, 5, (0, 0, 255), -1)
        cv2.imshow("Click IR Points", disp)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            return

    cv2.destroyWindow("Click IR Points")
    state["collecting_ir"] = False

    cv2.namedWindow("Click RGB Points")
    cv2.setMouseCallback("Click RGB Points", mouse_callback)
    print("RGB画像上で4点クリックしてください（IRと対応する位置）")
    while len(rgb_points) < 4:
        disp = rgb.copy()
        for pt in rgb_points:
            cv2.circle(disp, pt, 3, (255, 0, 0), -1)
        cv2.imshow("Click RGB Points", disp)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            return

    cv2.destroyAllWindows()
    pts_ir = np.array(ir_points, dtype=np.float32)
    pts_rgb = np.array(rgb_points, dtype=np.float32)
    matrix, _ = cv2.findHomography(pts_ir, pts_rgb, cv2.RANSAC)
    if matrix is None:
        raise RuntimeError("ホモグラフィ推定に失敗しました。")

    print("射影変換行列 M:\n", matrix)
    warped_ir = cv2.warpPerspective(ir, matrix, (rgb.shape[1], rgb.shape[0]))
    warped_ir_bgr = cv2.cvtColor(warped_ir, cv2.COLOR_GRAY2BGR)
    overlay = cv2.addWeighted(rgb, 0.5, warped_ir_bgr, 0.5, 0)
    cv2.imshow("Overlay: RGB + Warped IR", overlay)
    cv2.waitKey(MANUAL_PREVIEW_MS)
    cv2.destroyAllWindows()

    np.save(MANUAL_MATRIX_SAVE_PATH, matrix)
    cv2.imwrite(MANUAL_WARPED_SAVE_PATH, warped_ir_bgr)
    print(f"保存しました: {MANUAL_MATRIX_SAVE_PATH}")
    print(f"保存しました: {MANUAL_WARPED_SAVE_PATH}")


def fixed_matrix_single_test():
    ir = cv2.imread(FIXED_IR_PATH, cv2.IMREAD_GRAYSCALE)
    rgb = cv2.imread(FIXED_RGB_PATH)
    ensure_image_loaded(ir, FIXED_IR_PATH)
    ensure_image_loaded(rgb, FIXED_RGB_PATH)

    matrix = load_matrix_or_default(FIXED_MATRIX_PATH, FIXED_DEFAULT_MATRIX)
    print("使用する射影変換行列:\n", matrix)

    warped_ir = cv2.warpPerspective(ir, matrix, (rgb.shape[1], rgb.shape[0]))
    warped_ir_bgr = cv2.cvtColor(warped_ir, cv2.COLOR_GRAY2BGR)
    overlay = cv2.addWeighted(rgb, 0.2, warped_ir_bgr, 0.8, 0)
    cv2.imshow("Overlay IR on RGB", overlay)
    cv2.waitKey(FIXED_PREVIEW_MS)
    cv2.destroyAllWindows()
    cv2.imwrite(FIXED_WARPED_SAVE_PATH, warped_ir_bgr)
    print(f"保存しました: {FIXED_WARPED_SAVE_PATH}")


def batch_warp_images():
    os.makedirs(BATCH_OUTPUT_DIR, exist_ok=True)
    matrix = load_matrix_or_default(BATCH_MATRIX_PATH, FIXED_DEFAULT_MATRIX)
    print("使用する射影変換行列:\n", matrix)

    extensions = (".png", ".jpg", ".jpeg", ".bmp")
    ir_files = sorted(f for f in os.listdir(BATCH_IR_DIR) if f.lower().endswith(extensions))
    rgb_files = sorted(f for f in os.listdir(BATCH_RGB_DIR) if f.lower().endswith(extensions))
    if len(ir_files) != len(rgb_files):
        print("エラー: IR と RGB の画像数が一致しません。処理を中断します。")
        print(f"IR: {len(ir_files)}, RGB: {len(rgb_files)}")
        return

    for ir_name, rgb_name in zip(ir_files, rgb_files):
        ir_path = os.path.join(BATCH_IR_DIR, ir_name)
        rgb_path = os.path.join(BATCH_RGB_DIR, rgb_name)
        print(f"処理中: IR={ir_name}, RGB={rgb_name}")

        ir = cv2.imread(ir_path, cv2.IMREAD_GRAYSCALE)
        rgb = cv2.imread(rgb_path)
        if ir is None or rgb is None:
            print("読み込み失敗のためスキップ")
            continue

        warped_ir = cv2.warpPerspective(ir, matrix, (rgb.shape[1], rgb.shape[0]))
        warped_ir_bgr = cv2.cvtColor(warped_ir, cv2.COLOR_GRAY2BGR)
        out_name = f"overlay_{os.path.splitext(ir_name)[0]}.png"
        out_path = os.path.join(BATCH_OUTPUT_DIR, out_name)
        cv2.imwrite(out_path, warped_ir_bgr)
        print(f"保存: {out_path}")


def matching_error_test():
    rgb = cv2.imread(MATCH_RGB_PATH)
    warped_ir = cv2.imread(MATCH_WARPED_IR_PATH)
    ensure_image_loaded(rgb, MATCH_RGB_PATH)
    ensure_image_loaded(warped_ir, MATCH_WARPED_IR_PATH)

    if rgb.shape != warped_ir.shape:
        warped_ir = cv2.resize(warped_ir, (rgb.shape[1], rgb.shape[0]))

    gray_rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    gray_ir = cv2.cvtColor(warped_ir, cv2.COLOR_BGR2GRAY)

    orb = cv2.ORB_create(nfeatures=1000)
    kp1, des1 = orb.detectAndCompute(gray_rgb, None)
    kp2, des2 = orb.detectAndCompute(gray_ir, None)
    if des1 is None or des2 is None:
        raise RuntimeError("特徴点が不足しているためマッチングできません。")

    matches = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(des1, des2)
    matches = sorted(matches, key=lambda x: x.distance)
    if not matches:
        raise RuntimeError("マッチング結果が0件です。")

    errors = []
    for m in matches[:MATCH_TOP_N]:
        pt1 = kp1[m.queryIdx].pt
        pt2 = kp2[m.trainIdx].pt
        errors.append(np.linalg.norm(np.array(pt1) - np.array(pt2)))

    print("平均誤差（ピクセル）:", np.mean(errors))
    print("最大誤差:", np.max(errors))
    print("最小誤差:", np.min(errors))

    comb = cv2.addWeighted(gray_rgb, 0.8, gray_ir, 0.2, 0)
    cv2.imshow("Matched Keypoints", comb)
    cv2.waitKey(MATCH_PREVIEW_MS)
    cv2.destroyAllWindows()


def select_mode():
    print("\n=== HomographyProjecterTf: 機能選択 ===")
    print(f"{MODE_AUTO_H}: 自動で射影変換行列を推定")
    print(f"{MODE_MANUAL_H}: 手動4点で射影変換行列を推定")
    print(f"{MODE_FIXED_TEST}: 固定行列で1枚テスト")
    print(f"{MODE_BATCH_WARP}: 固定行列で複数画像を一括変換")
    print(f"{MODE_MATCH_TEST}: 射影後画像のマッチング誤差確認")
    return input("実行する機能番号を入力してください: ").strip()


def main():
    mode = select_mode()

    if mode == MODE_AUTO_H:
        auto_estimate_homography()
    elif mode == MODE_MANUAL_H:
        manual_estimate_homography()
    elif mode == MODE_FIXED_TEST:
        fixed_matrix_single_test()
    elif mode == MODE_BATCH_WARP:
        batch_warp_images()
    elif mode == MODE_MATCH_TEST:
        matching_error_test()
    else:
        print(f"未対応の入力です: {mode}")


if __name__ == "__main__":
    main()
