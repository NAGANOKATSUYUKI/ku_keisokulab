#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Color と Infra の番号を照合し、一致した Infra だけを移動する。"""

import os
import shutil

# ===== 設定（最初に編集する場所）=====
COLOR_DIR = "/home/keisoku/20251031_近距離物体/crop/color_crop"
INFRA_DIR = "/home/keisoku/20251031_近距離物体/crop/ir_homo_crop"
OUTPUT_DIR = "/home/keisoku/20251031_近距離物体/crop/combi_crop"
COLOR_PREFIX = "Color_frame_"
INFRA_PREFIX = "Infra_frame_"

def collect_color_ids(color_dir: str) -> set[str]:
    """Color ファイル名からフレームID集合を作成する。"""
    return {
        os.path.splitext(filename)[0].replace(COLOR_PREFIX, "")
        for filename in os.listdir(color_dir)
        if filename.startswith(COLOR_PREFIX)
    }

def filter_and_move_infra_files(
    infra_dir: str, output_dir: str, valid_ids: set[str]
) -> int:
    """一致する Infra を移動し、不一致を削除する。"""
    moved_count = 0
    for filename in os.listdir(infra_dir):
        if not filename.startswith(INFRA_PREFIX):
            continue

        base_name = filename.replace(INFRA_PREFIX, "").split(".")[0]
        src_path = os.path.join(infra_dir, filename)

        if base_name in valid_ids:
            dst_path = os.path.join(output_dir, filename)
            shutil.move(src_path, dst_path)
            moved_count += 1
        else:
            os.remove(src_path)

    return moved_count

def main() -> None:
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    color_ids = collect_color_ids(COLOR_DIR)
    moved_count = filter_and_move_infra_files(INFRA_DIR, OUTPUT_DIR, color_ids)
    print(f"処理完了: {moved_count} 枚のインフラ画像を保存しました。")

if __name__ == "__main__":
    main()
