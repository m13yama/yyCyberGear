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

## ツール

- `tools/up.bash`
  - CAN インターフェース (`can0` 既定) の初期化を一括実行するスクリプトです。
  - ビットレートや `txqueuelen` を設定し、インターフェースを UP 状態にします。既に UP の場合は安全に DOWN → 再設定 → UP を行います。
  - 実行例:

    ```bash
    ./tools/up.bash           # デフォルト設定 (1 Mbps, txqueuelen=1000)
    # ※ インターフェース名を変更したい場合は tools/up.bash 内の IFACE 変数を直接編集してください
    ```

## ビルド手順

1. ワークスペースを作成して、`[ワークスペース]/src`に本リポジトリをクローンします。
2. ワークスペース直下で`colcon build`を実行します。
3. 完了後に`source install/setup.bash`を読み込みます。

## 使い方

- CAN インターフェースの準備

  - `tools/up.bash` を使うと、ビットレート設定から状態確認までをまとめて実行できます。

    ```bash
    ./tools/up.bash
    ```

  - 手作業で行う場合:

    ```bash
    sudo ip link set can0 down
    sudo ip link set can0 up type can bitrate 1000000
    ip -details link show can0
    ```

- サンプルバイナリ

  - すべて `./build/yy_cybergear/<バイナリ名>` に生成されます。`--interface/-i` (既定 `can0`) と `--motor-id/-M` で対象デバイスを指定し、`Ctrl+C` で終了します。
  - 共通機能として、ほとんどのサンプルが以下を実装しています。
    - 起動時に `yy_socket_can::CanRuntime` を介して複数モーターを登録
    - 必要な CyberGear パラメータの ReadParam を送信し、取得完了まで待機
    - ステータス/警告/フォルトを検出して標準出力に整形表示

  - 各サンプルの詳細:
    - `exmp_00_zero_position`: 単体モーターを対象に、任意のパラメータ書き込み (速度/電流/トルク制限やゲインなど) → 機械ゼロ点の取得 → 指定秒数ゼロ位置保持を順番に実施します。`--speed-limit` や `--position-kp` などで書き込み値を指定できます。
    - `exmp_01_monitor_status`: 1 台以上のモーターに対してステータスフレーム (Type 2) を監視し続け、エラー／警告ビットを常時チェックします。`-M` を複数指定すれば集合監視が可能です。
    - `exmp_02_operation_sin_wave`: 複数モーターへ操作コマンド (Type 1) を周期送信し、位置・速度・トルクを正弦波で変化させるランコントロールの例です。`--amp`, `--freq`, `--kp`, `--kd` などで波形とゲインを設定します。
    - `exmp_03_position_sin_wave`: CyberGear の `POSITION_REFERENCE` を直接書き換え、正弦波位置指令を生成するポジション制御デモです。`--phase-step-deg` でモーターごとに位相をずらし、同期動作を試せます。
    - `exmp_04_speed_constant`: 各モーターに一定速度 (`SPEED_REFERENCE`) を与えるサンプルです。`-s/--speed` で正負を含む速度指令を設定でき、ウォームアップや通信テストに向きます。
    - `exmp_05_current_constant`: 電流モード (`IQ_REFERENCE`) で一定 q 軸電流を与え続ける例です。電流ループの検証や推力ゼロ点測定に利用できます。
    - `exmp_06_position_follow`: 最も ID が小さいモーターをマスターとし、他のモーターをその機械角に追従させるサンプルです。複数台の同期動作の確認に使用します。

## テスト

- プロトコルのエンコード/デコードテストは`./build/yy_cybergear/test_data_frame_handler`で実行できます。

## ライセンス

- Apache License 2.0
