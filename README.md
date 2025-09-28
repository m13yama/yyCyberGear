# yyCyberGear

🚧 **このソフトウェアは現在ベータ版です**
仕様やインターフェースは今後変更される可能性があります。

## 概要

- CyberGearアクチュエータをLinux SocketCAN経由で扱うためのROS2パッケージ群です。
- 例題バイナリと最小限のテストを含み、セットアップ直後から通信確認ができます。

## 環境

- OS: Ubuntu 20.04 (ROS2 humble)
- USB-CANアダプタ: <https://amzn.asia/d/clTand9>

## パッケージ構成

- `yy_socket_can`: SocketCANラッパー。ノンブロッキング/バッファ設定/ループバック切り替えを提供します。
- `yy_cybergear`: CyberGear 固有プロトコルの実装。操作コマンド生成、ステータス受信、パラメータ読み書き API を持ち、`exmp_*` の CLI サンプルを同梱しています。

## ビルド手順

1. ワークスペースを作成して、`[ワークスペース]/src`に本リポジトリをクローンします。
2. ワークスペース直下で`colcon build`を実行します。
3. 完了後に`source install/setup.bash`を読み込みます。

## 使い方

- SocketCANの準備

  ```bash
  sudo ip link set can0 down
  sudo ip link set can0 up type can bitrate 1000000
  ip -details link show can0
  ```

- サンプルプログラムの実行方法

  - ステータス監視:

    ```bash
    ./build/yy_cybergear/exmp_01_monitor_status --interface can0 --motor-id 1
    ```

## テスト

- プロトコルのエンコード/デコードテストは`./build/yy_cybergear/test_data_frame_handler`で実行できます。

## ライセンス

- Apache License 2.0
