#!/usr/bin/env python3
import pygame
import random
from pynput import keyboard

# Pygameの初期化
pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=512)

# サウンドファイルのパス
sounds = {
    "normal"        : pygame.mixer.Sound("/home/keisoku/catkin_ws/src/multi_function_switch/sound_dir/vvv_countdown.wav"),
    "hit"       : pygame.mixer.Sound("/home/keisoku/catkin_ws/src/multi_function_switch/sound_dir/vvv_継続ストック獲得.wav"),
    "special_hit" : pygame.mixer.Sound("/home/keisoku/catkin_ws/src/multi_function_switch/sound_dir/vvv_ハラキリ.wav"),
    "human"         : pygame.mixer.Sound("/home/keisoku/catkin_ws/src/multi_function_switch/sound_dir/vvv_らいぞう.wav")
}

# 初期値
count = 0
next_is_super_special = False

# 当選確率
hit_probability = 2
special_probability = 2

# キー入力を監視する関数
def on_press(key):
    global count, next_is_super_special,special_probability,hit_probability

    if key == keyboard.Key.enter:
        count += 1
        if next_is_super_special:
            if random.randint(1, special_probability) == 1:
                print(f"{count} 回目   ハラキリドライブ！！", end = "\r")
                sounds["special_hit"].play()
            else:
                print(f"{count} 回目", end = "\r")
                sounds["human"].play()
            count = 0
            next_is_super_special = False
        else:
            if random.randint(1, hit_probability) == 1:
                print(f"{count} 回目   レア音！", end = "\r")
                sounds["hit"].play()
                next_is_super_special = True
            else:
                print(f"{count} 回目", end = "\r")
                sounds["normal"].play()

# ハラキリ
# ハラキリネガティブ

# 通常
# 継続


# メイン
try:
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()
except KeyboardInterrupt:
    print("中断します")
