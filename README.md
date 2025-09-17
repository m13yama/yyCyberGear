# yy_cyber_gear

## 概要

- CyberGear アクチュエータを Linux SocketCAN 経由で扱うための ROS 2 パッケージ群です。
- `yy_socket_can`, `yy_cybergear`, `yy_cybergear_app` の 3 つで構成し、ドライバ層から GUI までをカバーします。
- 例題バイナリと最小限のテストを含み、セットアップ直後から通信確認ができます。
- USB-CANアダプタは<https://amzn.asia/d/clTand9>を使用して動作確認しています。

## パッケージ構成

- `yy_socket_can`: SocketCAN の薄い C++17 ラッパー。ノンブロッキング/バッファ設定/ループバック切り替えを提供します。
- `yy_cybergear`: CyberGear 固有プロトコルの実装。操作コマンド生成、ステータス受信、パラメータ読み書き API を持ち、`exmp_*` の CLI サンプルを同梱しています。
- `yy_cybergear_app`: Qt5 Widgets ベースのシンプルな速度制御 GUI。接続設定、限界値の適用、ステータスのモニタとログ表示を 1 画面にまとめています。

## ビルド手順

1. ROS2 (Humble 以降) をインストールしておきます。
2. ワークスペース直下で`colcon build`を実行します。
3. 完了後に`source install/setup.bash`を読み込みます。

## 使い方

- 各コマンドはワークスペース直下で実行します。
- MCU ID の取得例:

  ```bash
  ./build/yy_cybergear/exmp_01_get_mcu_id --interface can0 --host-id 0x01 --motor-id 0x01
  ```

- ステータス監視:

  ```bash
  ./build/yy_cybergear/exmp_02_monitor_status --interface can0 --motor-id 1
  ```

- GUI 起動:

  ```bash
  ./build/yy_cybergear_app/cybergear_gui_app
  ```

## SocketCAN の準備

1. `sudo ip link set can0 down`
2. `sudo ip link set can0 up type can bitrate 1000000`
3. `ip -details link show can0`

## テスト

- プロトコルのエンコード/デコードテストは`./build/yy_cybergear/test_data_frame_codec`で実行できます。

## ライセンス

- Apache License 2.0
