# keisoku-lab
長野の知識より作られたワークスペース

## カラー化データセット作成
1.画像発信確認
2.画像の位置合わせ:
```bash
rosrun sample align_color_infra.py
```
3.画像保存
```bash	　
rosrun sample save_color_infra.py
```

## pix2pixカラー化モデル作成
1.サイズ変更
```bash
rosrun sample resize_image.py 
```
2.画像をtestとtrainに仕分ける
```bash
rosrun sample save_filterframe.py 
```
3.画像を連番でrenameする
```bash
rosrun sample rename.py 
```
4.画像をA，Bフォルダの中のtest,train,valに各コピー

5.rgb画像とinfra画像をくっつける（--fold_A-B-ABの後にフォルダ名を指定）
```bash
python3 combine_A_and_B.py --fold_A /home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data6/edge/A --fold_B /home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data6/edge/B --fold_AB /home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data6/edge
```
6.学習
$ python3 train.py --dataroot /学習させたい画像までのフォルダ指定 --name 作成モデル名 --model モデル形式 --direction BtoA
```bash
python3 train.py --dataroot /home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/Data6/edge --name tukigaoka_1 --model pix2pix --direction BtoA
```
・・・・・

## 暗闇の中での物体把持
・realsense起動とrqtで確認
```bash
roslaunch realsense2_camera rs_camera_l515.launch 
```
・カラー化システム起動
1.edgeが必要な場合はエッジをつけて出力
```bash
rosrun pix2pix edges_realtime.py
```
・必要ないなら以下
```bash
rosrun pix2pix infra2rgb.py
```
2.カラー化---ディレクトリがpythoch_pix2pixの場所で動かす必要あり。モデル構造上
```bash
python3 test_realtime.py --dataroot None --name Data_edge --model pix2pix --direction BtoA
python3 test_realtime.py --dataroot None --name モデル名   --model pix2pix --direction BtoA
```
3.物体検出・距離計測_topic指定あり
```bash
roslaunch darknet_ros darknet_ros.launch
```
4.把持処理
・検出から距離をトピックで出力・tf出力
```bash
rosrun pix2pix topic_tf_point.py
```
・Realsense_cameraのtf発行
```bash
roslaunch pix2pix realsense_tf,launch
```
・把持処理
```bash
rosrun pix2pix grasping_key.py
```

next__

	ｈｓｒが有線できどうできない。無線ならできる謎
	
## 画像処理・物体検出
	・depth anything
	・clip物体検出
	・

