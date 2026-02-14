#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""指定ディレクトリ内の画像ファイル名を連番にリネームするスクリプト。"""

import os
from natsort import natsorted

# ===== 設定（最初に編集する場所）=====
DIRECTORY_PATH = "/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/20251126_train_data7/背景白黒混合/ir"
EXTENSION = "png"


def get_target_files(directory_path: str, extension: str) -> list[str]:
    """指定拡張子のファイル一覧を自然順で返す。"""
    normalized_ext = extension.lower().lstrip(".")
    files = [
        filename
        for filename in os.listdir(directory_path)
        if filename.lower().endswith(f".{normalized_ext}")
    ]
    return natsorted(files, key=lambda name: name.lower())


def build_rename_plan(files: list[str], extension: str) -> list[tuple[str, str]]:
    """連番リネーム計画（旧名, 新名）を作成する。"""
    padding_width = len(str(len(files)))
    normalized_ext = extension.lower().lstrip(".")

    plan: list[tuple[str, str]] = []
    for index, old_name in enumerate(files, start=1):
        new_name = f"{str(index).zfill(padding_width)}.{normalized_ext}"
        plan.append((old_name, new_name))
    return plan


def apply_rename_plan(directory_path: str, rename_plan: list[tuple[str, str]]) -> None:
    """衝突回避のため、2段階でリネームする。"""
    temp_pairs: list[tuple[str, str]] = []

    # 1段階目: 一時ファイル名に変更
    for index, (old_name, new_name) in enumerate(rename_plan, start=1):
        old_path = os.path.join(directory_path, old_name)
        temp_name = f"__tmp_rename_{index:08d}__"
        temp_path = os.path.join(directory_path, temp_name)
        os.rename(old_path, temp_path)
        temp_pairs.append((temp_name, new_name))

    # 2段階目: 最終ファイル名に変更
    for index, (temp_name, final_name) in enumerate(temp_pairs, start=1):
        temp_path = os.path.join(directory_path, temp_name)
        final_path = os.path.join(directory_path, final_name)
        os.rename(temp_path, final_path)

        if index % 100 == 0 or index == len(temp_pairs):
            print(f"{index} files processed: {temp_name} -> {final_name}")


def rename_files_in_directory(directory_path: str, extension: str = EXTENSION) -> None:
    """ディレクトリ内の画像を連番にリネームする。"""
    if not os.path.isdir(directory_path):
        raise NotADirectoryError(f"Directory not found: {directory_path}")

    files = get_target_files(directory_path, extension)
    if not files:
        print(f"No '*.{extension}' files found in: {directory_path}")
        return

    rename_plan = build_rename_plan(files, extension)
    apply_rename_plan(directory_path, rename_plan)
    print(f"Completed: {len(rename_plan)} files renamed in '{directory_path}'")


if __name__ == "__main__":
    rename_files_in_directory(DIRECTORY_PATH, EXTENSION)
