#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""source フォルダ内画像を train/test/val に一括振り分けコピーする。"""

import os
import shutil

from natsort import natsorted
from tqdm import tqdm

# ===== 設定（最初に編集する場所）=====
SOURCE_DIRECTORY = "/home/keisoku/20251222_crop_image/infra_crop"
SUPPORTED_EXTENSIONS = (".png", ".jpg", ".jpeg")
MAX_FRAMES = 10000  # 上限不要なら None に変更

# source と同階層に生成
DATASET_ROOT = os.path.join(SOURCE_DIRECTORY, "..")
OUTPUT_DIRS = {
    "train": os.path.join(DATASET_ROOT, "train"),
    "test": os.path.join(DATASET_ROOT, "test"),
    "val": os.path.join(DATASET_ROOT, "val"),
}


def list_source_files(source_dir):
    """自然順ソートされた対象画像一覧を返す。"""
    files = [
        filename
        for filename in os.listdir(source_dir)
        if filename.lower().endswith(SUPPORTED_EXTENSIONS)
    ]
    return natsorted(files, key=lambda name: name.lower())


def split_dataset(source_dir, out_dirs):
    """画像を train/test/val に振り分けてコピーする。"""
    files = list_source_files(source_dir)

    for path in out_dirs.values():
        os.makedirs(path, exist_ok=True)

    frame_number = 1
    val_toggle = True  # 偶数フレームを test / val に交互で割り当てる

    for file_name in tqdm(files, desc="Now Copying"):
        file_path = os.path.join(source_dir, file_name)

        if frame_number % 2 == 0:
            if val_toggle:
                dest_path = os.path.join(out_dirs["test"], file_name)
            else:
                dest_path = os.path.join(out_dirs["val"], file_name)
            val_toggle = not val_toggle
        else:
            dest_path = os.path.join(out_dirs["train"], file_name)

        shutil.copy(file_path, dest_path)
        frame_number += 1

        if MAX_FRAMES is not None and frame_number == MAX_FRAMES:
            break


def main():
    split_dataset(SOURCE_DIRECTORY, OUTPUT_DIRS)
    print("\n--- Completed ---")
    print(f"Train folder: {len(os.listdir(OUTPUT_DIRS['train']))} files")
    print(f"Test folder:  {len(os.listdir(OUTPUT_DIRS['test']))} files")
    print(f"Val folder:   {len(os.listdir(OUTPUT_DIRS['val']))} files")


if __name__ == "__main__":
    main()
