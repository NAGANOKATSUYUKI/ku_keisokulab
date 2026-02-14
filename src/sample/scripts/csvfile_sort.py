#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""検出結果 CSV から不要列を削除して保存する。"""

import pandas as pd

# ===== 設定（最初に編集する場所）=====
BASE_DIR = "/home/keisoku/20250724_col_ir_image/yolo11/colorization_canny100_250_edgepix2"
INPUT_CSV = f"{BASE_DIR}/detection_results.csv"
OUTPUT_CSV = f"{BASE_DIR}/sorted_detection_results.csv"
DROP_COLUMNS = ["X1", "Y1", "X2", "Y2"]
SORT_BY_CLASS_NAME = False


def process_csv(input_csv: str, output_csv: str) -> pd.DataFrame:
    """CSV を読み込み、列削除（必要ならソート）して保存する。"""
    df = pd.read_csv(input_csv)
    df = df.drop(columns=DROP_COLUMNS)

    if SORT_BY_CLASS_NAME:
        df = df.sort_values(by="Class Name")

    df.to_csv(output_csv, index=False)
    return df


def main() -> None:
    df = process_csv(INPUT_CSV, OUTPUT_CSV)
    print(df)


if __name__ == "__main__":
    main()
