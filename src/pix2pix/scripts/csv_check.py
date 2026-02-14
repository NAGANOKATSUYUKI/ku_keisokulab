#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""CSVの画像番号更新と、必要に応じたクラス絞り込みを行う。"""

import pandas as pd

# ===== 設定（最初に編集する場所）=====
CSV_FILE = "/home/keisoku/ダウンロード/detic_edgena.csv"
IMAGE_FILENAME_COLUMN = "Image Filename"

# 任意機能: 指定クラス以外を削除したいときだけ True
ENABLE_CLASS_FILTER = False
CLASS_COLUMN = "Class"
ALLOWED_CLASSES = ["bottle", "cup", "bowl"]


def update_image_numbers(df: pd.DataFrame) -> pd.DataFrame:
    """Image Filename の並びを見て、先頭列の画像番号を再計算する。"""
    if df.empty:
        return df

    numbers = [1]
    for i in range(len(df) - 1):
        curr_val = df.loc[i, IMAGE_FILENAME_COLUMN]
        next_val = df.loc[i + 1, IMAGE_FILENAME_COLUMN]
        if curr_val >= next_val:
            numbers.append(numbers[-1] + 1)
        else:
            numbers.append(numbers[-1])

    df.iloc[:, 0] = numbers
    return df


def filter_classes(df: pd.DataFrame) -> pd.DataFrame:
    """指定クラスのみ残す。"""
    return df[df[CLASS_COLUMN].isin(ALLOWED_CLASSES)]


def main():
    df = pd.read_csv(CSV_FILE)
    df = update_image_numbers(df)

    if ENABLE_CLASS_FILTER:
        df = filter_classes(df)

    print(df)
    df.to_csv(CSV_FILE, index=False)


if __name__ == "__main__":
    main()
