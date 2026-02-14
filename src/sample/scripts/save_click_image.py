#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ウィンドウクリックで4トピックの最新画像を保存する。"""

import os
from threading import Lock

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# ===== 設定（最初に編集する場所）=====
NODE_NAME = "multi_image_saver"
WINDOW_NAME = "Click to Save"
ESC_KEY = 27

TOPIC_COLOR = "/camera/color/image_raw"  # 生カラー画像の入力トピック
TOPIC_COLOR_FILTERING = "/z/color_image_filtered"  # フィルタ後カラー画像の入力トピック
TOPIC_IR_FILTERING = "/z/infra_image_masked"  # フィルタ後赤外画像の入力トピック
TOPIC_IR = "/camera/infra/image_raw"  # 生赤外画像の入力トピック

SAVE_DIR_COLOR = "/home/keisoku/save_image/color"
SAVE_DIR_COLOR_FILTERING = "/home/keisoku/save_image/color_filtering"
SAVE_DIR_IR_FILTERING = "/home/keisoku/save_image/ir_filtering"
SAVE_DIR_IR = "/home/keisoku/save_image/ir"

CANVAS_HEIGHT = 500
CANVAS_WIDTH = 700

class MultiImageSaver:
    """複数トピックの最新画像を管理し、クリック時に保存する。"""

    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=True)
        self.bridge = CvBridge()
        self.lock = Lock()
        self.image_data = {
            "color": None,
            "color_filtering": None,
            "ir_filtering": None,  # type: ignore[dict-item]
            "ir": None
        }
        self.counter = 1

        rospy.Subscriber(TOPIC_COLOR, Image, self.color_callback)
        rospy.Subscriber(TOPIC_COLOR_FILTERING, Image, self.color_filtering_callback)
        rospy.Subscriber(TOPIC_IR_FILTERING, Image, self.ir_filtering_callback)
        rospy.Subscriber(TOPIC_IR, Image, self.ir_callback)

        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)
        rospy.loginfo("画像保存ノード起動：ウィンドウをクリックすると保存します")
        self.run_ui_loop()

    def run_ui_loop(self):
        """終了キー入力まで案内ウィンドウを表示し続ける。"""
        while not rospy.is_shutdown():
            canvas = self.create_canvas()
            cv2.imshow(WINDOW_NAME, canvas)
            key = cv2.waitKey(30)
            if key == ESC_KEY:
                break

        cv2.destroyAllWindows()

    def color_callback(self, msg):
        self.update_image("color", msg, "bgr8")

    def color_filtering_callback(self, msg):
        self.update_image("color_filtering", msg, "bgr8")

    def ir_filtering_callback(self, msg):
        self.update_image("ir_filtering", msg)

    def ir_callback(self, msg):
        self.update_image("ir", msg, "mono8")

    def update_image(self, key, msg, encoding=None):
        """受信したROS画像をOpenCV画像に変換してキャッシュする。"""
        converted = self.bridge.imgmsg_to_cv2(msg, encoding) if encoding else self.bridge.imgmsg_to_cv2(msg)
        with self.lock:
            self.image_data[key] = converted

    def mouse_callback(self, event, x, y, flags, param):
        del x, y, flags, param
        if event == cv2.EVENT_LBUTTONDOWN:
            self.save_all_images()

    def save_all_images(self):
        """保存可能な最新画像をすべて保存する。"""
        for directory in [SAVE_DIR_COLOR, SAVE_DIR_COLOR_FILTERING, SAVE_DIR_IR_FILTERING, SAVE_DIR_IR]:
            os.makedirs(directory, exist_ok=True)

        with self.lock:
            base_name = f"{self.counter:04d}"
            missing_topics = [k for k, v in self.image_data.items() if v is None]

            if self.image_data["color"] is not None:
                cv2.imwrite(f"{SAVE_DIR_COLOR}/{base_name}_color.png", self.image_data["color"])
            if self.image_data["color_filtering"] is not None:
                cv2.imwrite(
                    f"{SAVE_DIR_COLOR_FILTERING}/{base_name}_color_filtering.png",
                    self.image_data["color_filtering"],
                )
            if self.image_data["ir_filtering"] is not None:
                cv2.imwrite(
                    f"{SAVE_DIR_IR_FILTERING}/{base_name}_ir_filtering.png",
                    self.image_data["ir_filtering"],
                )
            if self.image_data["ir"] is not None:
                cv2.imwrite(f"{SAVE_DIR_IR}/{base_name}_ir.png", self.image_data["ir"])

            if len(missing_topics) == len(self.image_data):
                rospy.logwarn("保存可能な画像がありません。スキップします。")
                return

            rospy.loginfo(
                "%s 番の画像を保存しました（不足トピック: %s）",
                base_name,
                ", ".join(missing_topics) if missing_topics else "なし",
            )
            self.counter += 1

    def create_canvas(self):
        """保存案内用キャンバスを作成する。"""
        h, w = CANVAS_HEIGHT, CANVAS_WIDTH
        canvas = 255 * np.ones((h, w, 3), dtype=np.uint8)
        cv2.putText(canvas, "Click window to save 4 images", (10, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        return canvas


if __name__ == "__main__":
    try:
        MultiImageSaver()
    except rospy.ROSInterruptException:
        pass
