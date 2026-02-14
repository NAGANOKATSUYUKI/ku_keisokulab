#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""2点クリックで画像をクロップし、必要に応じて連動画像も同範囲で保存する。"""

import glob
import os
import re

import cv2

# ===== 設定（最初に編集する場所）=====
INPUT_DIR1 = "/home/keisoku/20251031_近距離物体/ir_homo"
INPUT_DIR2 = "/home/keisoku/20251031_近距離物体/color"
OUTPUT_DIR1 = "/home/keisoku/20251031_近距離物体/ir_homo_crop"
OUTPUT_DIR2 = "/home/keisoku/20251031_近距離物体/color_crop"
WINDOW_NAME = "Select Region"
SUPPORTED_PATTERN = "*.*"

KEY_SKIP = "q"
KEY_TOGGLE_LINK = "l"


def draw_helper_lines(img, x, y, is_top_left=True):
    """クリック位置に補助線を描画する。"""
    color = (0, 255, 0)
    thickness = 1
    offset = 150
    if is_top_left:
        cv2.line(img, (x, y), (x + offset, y), color, thickness)
        cv2.line(img, (x, y), (x, y + offset), color, thickness)
    else:
        cv2.line(img, (x, y), (x - offset, y), color, thickness)
        cv2.line(img, (x, y), (x, y - offset), color, thickness)


def crop_image(img, pt1, pt2):
    """2点で囲まれた矩形領域を切り抜く。"""
    x1, y1 = pt1
    x2, y2 = pt2
    x_min, x_max = sorted([x1, x2])
    y_min, y_max = sorted([y1, y2])
    return img[y_min:y_max, x_min:x_max]


def find_best_match(target_name, search_dir):
    """対象ファイル名に近い連動画像を search_dir から探す。"""
    base = os.path.splitext(os.path.basename(target_name))[0]
    num_match = re.search(r"\d+", base)
    num = num_match.group() if num_match else ""

    candidates = sorted(glob.glob(os.path.join(search_dir, SUPPORTED_PATTERN)))
    if not candidates:
        return None

    best_match = None
    best_score = -1
    for candidate in candidates:
        candidate_base = os.path.basename(candidate)
        score = len(os.path.commonprefix([base, candidate_base]))
        if num and num in candidate_base:
            score += 5
        if score > best_score:
            best_score = score
            best_match = candidate
    return best_match


def save_cropped_pair(state):
    """メイン画像を保存し、連動モード時は連動画像も保存する。"""
    base_name = os.path.basename(state["img_name"])
    pt1, pt2 = state["points"]

    cropped_main = crop_image(state["current_img"], pt1, pt2)
    save_path1 = os.path.join(OUTPUT_DIR1, base_name)
    cv2.imwrite(save_path1, cropped_main)
    print(f"保存: {save_path1}")

    if not state["link_mode"]:
        return

    other_path = find_best_match(base_name, INPUT_DIR2)
    if other_path is None:
        print(f"⚠ 連動画像が見つかりません: {base_name}")
        return

    other_img = cv2.imread(other_path)
    if other_img is None:
        print(f"⚠ 読み込み失敗: {other_path}")
        return

    if other_img.shape[:2] != state["current_img"].shape[:2]:
        print(
            f"⚠ サイズ不一致: {os.path.basename(other_path)} "
            f"({other_img.shape[:2]} vs {state['current_img'].shape[:2]})"
        )
        return

    cropped_other = crop_image(other_img, pt1, pt2)
    save_path2 = os.path.join(OUTPUT_DIR2, os.path.basename(other_path))
    cv2.imwrite(save_path2, cropped_other)
    print(f"連動保存: {save_path2}")


def mouse_callback(event, x, y, flags, state):
    """左クリック2回でクロップ範囲を決め、保存する。"""
    del flags
    if event != cv2.EVENT_LBUTTONDOWN:
        return

    state["temp_img"] = state["current_img"].copy()

    if len(state["points"]) == 0:
        state["points"].append((x, y))
        draw_helper_lines(state["temp_img"], x, y, is_top_left=True)
        cv2.circle(state["temp_img"], (x, y), 3, (0, 0, 255), -1)
        cv2.imshow(WINDOW_NAME, state["temp_img"])
        return

    if len(state["points"]) == 1:
        state["points"].append((x, y))
        draw_helper_lines(state["temp_img"], x, y, is_top_left=False)
        cv2.circle(state["temp_img"], (x, y), 3, (0, 0, 255), -1)
        cv2.rectangle(state["temp_img"], state["points"][0], state["points"][1], (255, 0, 0), 1)
        cv2.imshow(WINDOW_NAME, state["temp_img"])
        save_cropped_pair(state)
        state["done_flag"] = True


def main():
    os.makedirs(OUTPUT_DIR1, exist_ok=True)
    os.makedirs(OUTPUT_DIR2, exist_ok=True)

    state = {
        "points": [],
        "current_img": None,
        "temp_img": None,
        "img_name": "",
        "link_mode": True,
        "done_flag": False,
    }

    image_paths = sorted(glob.glob(os.path.join(INPUT_DIR1, SUPPORTED_PATTERN)))
    for path in image_paths:
        state["img_name"] = path
        state["points"] = []
        state["done_flag"] = False

        state["current_img"] = cv2.imread(path)
        if state["current_img"] is None:
            print(f"読み込み失敗: {path}")
            continue
        state["temp_img"] = state["current_img"].copy()

        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, mouse_callback, state)

        print(f"\n画像: {path}")
        print("-------クリック①: 左上, クリック②: 右下で自動保存-------")
        print(f"現在の連動モード: {'ON' if state['link_mode'] else 'OFF'}")
        print(f"キー操作 → [{KEY_TOGGLE_LINK}]:連動モード切替, [{KEY_SKIP}]:スキップ")

        while True:
            cv2.imshow(WINDOW_NAME, state["temp_img"])
            key = cv2.waitKey(100) & 0xFF

            if state["done_flag"]:
                cv2.destroyWindow(WINDOW_NAME)
                break

            if key == ord(KEY_SKIP):
                print("スキップします。")
                cv2.destroyWindow(WINDOW_NAME)
                break

            if key == ord(KEY_TOGGLE_LINK):
                state["link_mode"] = not state["link_mode"]
                print(f"連動モード: {'ON' if state['link_mode'] else 'OFF'}")

    cv2.destroyAllWindows()
    print("全画像処理完了！")


if __name__ == "__main__":
    main()
