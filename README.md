# Genesis Drones

マルチドローンシミュレーションのためのROS 2パッケージ
(ROS 2 package for multi-drone simulation)

## 概要

このパッケージは、Genesisエンジンを使用したマルチドローンシミュレーションを提供します。複数のドローンを同時にシミュレーションし、それぞれに異なる飛行ルートを設定することができます。また、カメラ画像の取得や録画機能も備えています。

## 機能

- 複数ドローンの同時シミュレーション
- 各ドローンに対する個別の飛行ルート設定
- ウェイポイントの視覚化（球体マーカー）
- カメラ画像の取得と表示
- カメラ映像の録画
- ROS 2トピックを介したドローン制御
- TF情報の発行

## コードの構成

コードは以下のように機能ごとにモジュール化されています：

- `multi_drone_simulation.py` - メインのシミュレーションクラス
- `drone_controller.py` - ドローン制御関連のクラスと関数（PID制御、飛行制御）
- `waypoint_utils.py` - ウェイポイント関連の機能（作成・描画・管理）
- `simulation_utils.py` - シミュレーション関連のユーティリティ（初期化、クリーンアップ）
- `visualization_utils.py` - 視覚化関連の機能（マーカー、カメラ表示）
- `camera_utils.py` - カメラ画像処理関連の関数
- `utils.py` - 一般的なユーティリティ関数
- `tf_utils.py` - TF関連の処理を行う関数

## 使用方法

### キーボード制御モード

ドローンを十字キーで操作することができます。シミュレーションとキーボード制御を別々のターミナルで実行することで、より柔軟な操作が可能です。

#### 別々のターミナルでの実行方法

**ターミナル1（シミュレーション）**:
```bash
# 速度制御モードでシミュレーションを起動
ros2 run genesis_drones multi_drone_simulation --ros-args -p velocity_control:=true -p num_drones:=1
```

**ターミナル2（キーボード制御）**:
```bash
# キーボード制御ノードを起動
ros2 run genesis_drones keyboard_control_node.py --ros-args -p linear_speed:=0.5 -p drone_id:=0
```

#### PIDゲインの最適化
```bash
python3 src/genesis_drones/scripts/velocity_tracking_plot.py --steps 100 --optimize --output
```

#### キーボード操作方法

- **矢印キー上**：前進（+X方向）
- **矢印キー下**：後退（-X方向）
- **矢印キー左**：左移動（+Y方向）
- **矢印キー右**：右移動（-Y方向）
- **スペースキー**：上昇（+Z方向）
- **Shiftキー**：下降（-Z方向）
- **Qキー**：左回転（+Yaw）
- **Eキー**：右回転（-Yaw）
- **Escキー**：停止（すべての速度をゼロに）

#### パラメータ設定

**シミュレーションのパラメータ**:
- `velocity_control:=true`：速度制御モードを有効にする（必須）
- `num_drones:=1`：シミュレーションするドローンの数
- `show_camera:=true`：カメラ画像を表示する

**キーボード制御のパラメータ**:
- `linear_speed:=0.5`：線形速度の大きさ [m/s]
- `angular_speed:=0.5`：角速度の大きさ [rad/s]
- `drone_id:=0`：制御対象のドローンID
```
