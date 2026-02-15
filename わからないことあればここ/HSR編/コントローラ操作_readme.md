# HSR コントローラ操作 README

参考:
- ユーザマニュアル  
  `Docs » 6. HSRの使い方 » 6.3. ツールからのHSR操縦 » 6.3.2. DUALSHOCK®を使った操縦`

## 1. コントローラのペアリング手順（HSR側）

1. USBケーブルでコントローラをHSRに接続する。
2. HSRへログインする。

```bash
ssh administrator@hsrc.local
```

3. ペアリングを実行する。

```bash
sudo sixpair
```

4. `sixad` サービスを開始する。

```bash
sudo service sixad start
```

5. USBケーブルを抜いて、コントローラの `PS` ボタンを押す。  
   コントローラが振動し、赤ランプが1つ点灯すれば接続成功。
繋がらない時は以下のコマンドをHSRで実行
```bash
sudo hciconfig hci0 up piscan
sudo hciconfig hci0
```

## 2. sixad の自動起動設定（PC側）

必要に応じて実行します。

```bash
sudo sixad --boot-yes   # 自動起動を有効化
sudo sixad --boot-no    # 自動起動を無効化
```

## 3. 動作確認コマンド

### デバイス名確認

```bash
ls /dev/input/js*
```

### USB認識ログ確認

```bash
sudo tail -f /var/log/syslog
```

### ROSトピック確認

```bash
rostopic echo /hsrb/joy
```

### Bluetooth接続確認

```bash
sudo hciconfig hci0 up piscan
sudo hciconfig hci0
```

