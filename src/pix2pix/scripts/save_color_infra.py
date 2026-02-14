#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""4つの画像トピックを受信し、揃ったタイミングで1セットとして保存する。"""

import os
from threading import Lock

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# ===== 設定（最初に編集する場所）=====
NODE_NAME = "multi_image_saver"

# 「生カラー画像」の入力トピック（カメラの元画像）
TOPIC_COLOR_RAW = "/camera/color/image_raw"
# 「フィルタ後カラー画像」の入力トピック（前処理済み）
TOPIC_COLOR_FILTERED = "/z/color_image_filtered"
# 「フィルタ後赤外画像」の入力トピック（マスク済み想定）
TOPIC_IR_FILTERED = "/z/infra_image_masked"
# 「生赤外画像」の入力トピック（カメラの元画像）
TOPIC_IR_RAW = "/camera/infra/image_raw"

SAVE_DIRS = {
    "color": "/home/keisoku/save_image/color",
    "color_filtering": "/home/keisoku/save_image/color_filtering",
    "ir_filtering": "/home/keisoku/save_image/ir_filtering",
    "ir": "/home/keisoku/save_image/ir",
}

CONTROL_WINDOW = "Control Panel"
KEY_QUIT = "q"
KEY_PAUSE = " "
KEY_RESUME = 13  # Enter key


class MultiImageSaver:
    """受信画像を保持し、4種類そろったら連番で保存する。"""

    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=True)
        self.bridge = CvBridge()
        self.lock = Lock()
        self.counter = 1
        self.is_paused = False
        self.image_data = {
            "color": None,
            "color_filtering": None,
            "ir_filtering": None,
            "ir": None,
        }

        for directory in SAVE_DIRS.values():
            os.makedirs(directory, exist_ok=True)

        # トピック購読設定（上の TOPIC_* を変更すると入力元を切り替え可能）
        rospy.Subscriber(TOPIC_COLOR_RAW, Image, self.callback, "color")
        rospy.Subscriber(TOPIC_COLOR_FILTERED, Image, self.callback, "color_filtering")
        rospy.Subscriber(TOPIC_IR_FILTERED, Image, self.callback, ("ir_filtering", "mono8"))
        rospy.Subscriber(TOPIC_IR_RAW, Image, self.callback, ("ir", "mono8"))

        rospy.loginfo("リアルタイム画像保存ノードを起動しました。")

    def callback(self, msg, callback_args):
        with self.lock:
            if self.is_paused:
                return

            if isinstance(callback_args, tuple):
                image_type, encoding = callback_args
            else:
                image_type, encoding = callback_args, "bgr8"

            try:
                if encoding == "mono8":
                    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    self.image_data[image_type] = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
                else:
                    self.image_data[image_type] = self.bridge.imgmsg_to_cv2(
                        msg, desired_encoding="bgr8"
                    )
            except Exception as error:
                rospy.logerr("画像の変換に失敗: %s", error)
                return

            self.save_all_images()

    def save_all_images(self):
        if not all(value is not None for value in self.image_data.values()):
            return

        base_name = f"{self.counter:04d}"
        try:
            cv2.imwrite(
                os.path.join(SAVE_DIRS["color"], f"{base_name}_color.png"),
                self.image_data["color"],
            )
            cv2.imwrite(
                os.path.join(SAVE_DIRS["color_filtering"], f"{base_name}_color_filtering.png"),
                self.image_data["color_filtering"],
            )
            cv2.imwrite(
                os.path.join(SAVE_DIRS["ir_filtering"], f"{base_name}_ir_filtering.png"),
                self.image_data["ir_filtering"],
            )
            cv2.imwrite(
                os.path.join(SAVE_DIRS["ir"], f"{base_name}_ir.png"),
                self.image_data["ir"],
            )

            rospy.loginfo("セット %s の画像を保存しました", base_name)
            self.counter += 1
        except Exception as error:
            rospy.logerr("画像の書き込みに失敗: %s", error)

        for key in self.image_data:
            self.image_data[key] = None

    def create_status_canvas(self):
        h, w = 120, 400
        canvas = 255 * np.ones((h, w, 3), dtype=np.uint8)

        if self.is_paused:
            status_text = "Status: PAUSED"
            color = (0, 0, 255)
        else:
            status_text = "Status: SAVING"
            color = (0, 150, 0)

        cv2.putText(canvas, status_text, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(canvas, "'q': Quit", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        cv2.putText(
            canvas, "'space': Pause", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1
        )
        cv2.putText(
            canvas, "'enter': Resume", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1
        )
        return canvas

    def run(self):
        cv2.namedWindow(CONTROL_WINDOW)
        while not rospy.is_shutdown():
            canvas = self.create_status_canvas()
            cv2.imshow(CONTROL_WINDOW, canvas)
            key = cv2.waitKey(30) & 0xFF

            if key == ord(KEY_QUIT):
                rospy.loginfo("プログラムを終了します。")
                break
            if key == ord(KEY_PAUSE) and not self.is_paused:
                self.is_paused = True
                rospy.loginfo("保存を一時停止しました。")
            if key == KEY_RESUME and self.is_paused:
                self.is_paused = False
                rospy.loginfo("保存を再開しました。")

        cv2.destroyAllWindows()


def main():
    saver = MultiImageSaver()
    saver.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
