# 赤外線画像の疑似カラー化方法

## 1画像の収集・カラー化データセット作成

1. リアルセンスカメラ起動  
カメラによってファイルを指定する。`~/realsense-ros/realsense2_camera/launch/` 以下に様々なファイルがある。  
・l515カメラの場合

```bash
roslaunch realsense2_camera rs_camera_l515.launch
```

2. カラー画像の背景除去用ファイル起動

```bash
rosrun pix2pix color_depth_filtering.py
```

3. 赤外線画像の背景除去用ファイル起動

```bash
rosrun pix2pix infra_depth_filtering.py
```

4. 画像の保存  
・クリックして保存する時（画像トピックが全てパブリッシュされないと保存できない）

```bash
rosrun pix2pix save_click_image.py
```

・クリックしないで保存したい時（画像トピックが全てパブされないと保存できない）

```bash
rosrun pix2pix save_color_infra.py
```

・保存したい画像がある場合は適時ファイル内のトピック名を編集する

## 2pix2pixで画像生成モデルの作成
### 画像の前処理
1. 画像を連番でrenameする

```bash
rosrun pix2pix rename.py 
```

2. サイズ変更  
・ファイル内の固定値を編集することでサイズ変更ができる

```bash
rosrun pix2pix resize_image.py 
```
### データセットの作成
3. 画像をtest,train,valに仕分ける

```bash
rosrun pix2pix save_filterframe.py 
```

4. 画像をA，Bフォルダの中のtest,train,valに各コピー  
Aがカラー画像用、Bが赤外線画像用として使う

5. カラー画像とinfra画像を結合する（`--fold_A`, `--fold_B`, `--fold_AB` の後にフォルダ名を指定）

```bash
python3 combine_A_and_B.py --fold_A /home/~ --fold_B /home/~ --fold_AB /home/~
```

6. 機械学習コマンド  
`$ python3 train.py --dataroot /学習させたい画像までのフォルダ指定 --name 作成モデル名 --model モデル形式 --direction BtoA --crop_size 512or256 --load_size 512or256`  
・画像サイズを512で学習、画像生成したい場合

```bash
python3 train.py --dataroot /home/~ --name ~ --model pix2pix --direction BtoA --display_id 0 --crop_size 512 --load_size 512
```
- 作成されたモデルは，checkpoints/に保存される．

### 疑似カラー化テスト

・1枚画像をカラー化する

```bash
python3 test_image.py --dataroot None --name Data_edges --model pix2pix --direction BtoA --crop_size 512or256 --load_size 512or256
```

・複数画像のカラー化

```bash
python3 test_dir.py --dataroot None --name Data_edges --model pix2pix --direction BtoA --crop_size 512or256 --load_size 512or256
```

※注意!!!  
`test_image.py-test_dir.py` と `test_realtime.py` でモデルが同じでも生成される画像が違うので注意（原因不明）

## 4HSRを使った暗闇の中での物体把持

・リアルセンスカメラ起動とrqtで動いているか確認

```bash
roslaunch realsense2_camera rs_camera_l515.launch 
```

・赤外線画像フィルターモード

```bash
rosrun pix2pix infra_depth_filtering.py
```

・カラー化システム起動  
1. edgeが必要な場合はエッジをつけて出力

```bash
rosrun pix2pix edges_realtime.py
```

・必要ないなら以下

```bash
rosrun pix2pix infra2rgb.py
```

2. カラー化---ディレクトリがpythoch_pix2pixの場所で動かす必要あり。モデル構造上  
`$ python3 test_realtime.py --dataroot None --name モデル名   --model pix2pix --direction BtoA --display_id 0 --crop_size 512or256 --load_size 512or256`

```bash
python3 test_realtime.py --dataroot None --name Data_edge --model pix2pix --direction BtoA --display_id 0 --crop_size 512or256 --load_size 512or256
```

3. 物体検出・距離計測_topic指定あり

```bash
roslaunch darknet_ros darknet_ros.launch
```

4. 把持処理  
・検出から距離をトピックで出力・tf出力

```bash
rosrun pix2pix topic_tf_point.py
```

・Realsense_cameraのtf発行、カメラの位置合わせ

```bash
roslaunch pix2pix realsense_tf.launch
```

・Rviz表示

```bash
rviz -d /home/keisoku/catkin_ws/src/hsrc/config/hsr_rviz.rviz
```

・ロボットによる把持,回収動作
HSRとPCの時刻同期
HSR,PC両方のターミナルで実行
```bash
sudo service chrony restart
```

動作制御の起動
```bash
rosrun pix2pix grasping_key.py
```
