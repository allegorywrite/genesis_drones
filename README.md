# Genesis Drones

マルチドローンシミュレーションのためのROS 2パッケージ

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

### 基本的な実行方法

```bash
# 1つのドローンをシミュレーション
ros2 launch genesis_drones multi_drone_simulation.launch.py num_drones:=1

# 複数のドローンをシミュレーション
ros2 launch genesis_drones multi_drone_simulation.launch.py num_drones:=2

# カメラ画像を表示
ros2 launch genesis_drones multi_drone_simulation.launch.py num_drones:=1 show_camera:=true

# カメラ映像を録画
ros2 launch genesis_drones multi_drone_simulation.launch.py num_drones:=1 record:=true output:="output.mp4"

# 円形フォーメーションでのドローンシミュレーション
./scripts/circle_formation.py --num-drones 3 --radius 2.0 --height 1.0 --num-circles 2
```

### 飛行ルートの設定

#### 単一ドローンの飛行ルート

```bash
# 1つのドローンに飛行ルートを設定
ros2 launch genesis_drones multi_drone_simulation.launch.py num_drones:=1 route:="0,0,1;1,1,1" drone_id:=0
```

#### 複数ドローンの飛行ルート

```bash
# 複数のドローンに異なる飛行ルートを設定
ros2 launch genesis_drones multi_drone_simulation.launch.py routes:="0:0,0,1;1,1,1|1:1,0,1;0,1,1"
```

飛行ルートの形式：
- 単一ドローン: `"x1,y1,z1;x2,y2,z2;..."`
- 複数ドローン: `"drone_id1:x1,y1,z1;x2,y2,z2|drone_id2:x1,y1,z1;x2,y2,z2"`

## 実装の詳細

### URDFファイルの使用

ドローンのモデルには、Genesisの`gs.morphs.Drone`クラスを使用しています。URDFファイルのパスは絶対パスで指定する必要があります：

```python
drone = scene.add_entity(
    gs.morphs.Drone(
        file="/home/initial/lab_ws/Genesis/genesis/assets/urdf/drones/cf2x.urdf",
        pos=(i*1.0, 0, 0.5),
        fixed=False  # ベースリンクを固定しないように設定
    ),
)
```

### PIDコントローラ

ドローンの制御には、PIDコントローラを使用しています。PIDパラメータは以下のように設定されています：

```python
default_pid_params = [
    [2.5, 0.1, 0.5],  # pos_x
    [2.5, 0.1, 0.5],  # pos_y
    [3.0, 0.2, 1.0],  # pos_z
    [20.0, 0.5, 20.0],  # vel_x
    [20.0, 0.5, 20.0],  # vel_y
    [25.0, 1.0, 20.0],  # vel_z
    [10.0, 0.2, 1.0],  # att_roll
    [10.0, 0.2, 1.0],  # att_pitch
    [2.0, 0.1, 0.2],  # att_yaw
]
```

### カメラの設定

各ドローンにはカメラが設定されており、ドローンの姿勢に基づいて位置と向きが更新されます：

```python
# カメラの追加
camera = scene.add_camera(
    res=(320, 240),  # 解像度
    pos=(i*1.0 + 0.1, 0, 0.5),  # ドローンの少し前方に配置
    lookat=(i*1.0 + 2.0, 0, 0.5),  # より遠くを見る
    fov=60,  # 視野角
    GUI=False
)

# カメラの位置と向きの更新
def update_camera_position(camera, drone_pos, drone_quat, direction=None):
    # クォータニオンから回転行列を計算
    rot_matrix = quat_to_rot_matrix(drone_quat)
    
    # ドローンの前方ベクトル
    forward_vector = rot_matrix @ np.array([1.0, 0.0, 0.0])
    
    # カメラの位置と注視点を更新
    camera_pos = drone_pos + rot_matrix @ np.array([0.1, 0.0, 0.0])
    lookat_pos = drone_pos + forward_vector * 2.0
    
    camera.set_pose(pos=tuple(camera_pos), lookat=tuple(lookat_pos))
```

### ウェイポイントの視覚化

ウェイポイントは球体マーカーで視覚化されます。各ドローンの現在のウェイポイントのみが表示されます：

```python
# ウェイポイントマーカーの作成
marker = self.waypoint_manager.create_active_marker(
    drone_id=i,
    position=self.flight_controllers[i].current_target,
    size=0.15
)

# ウェイポイントが変更されたときにマーカーを更新
if waypoint_changed:
    self.waypoint_manager.create_active_marker(i, self.flight_controllers[i].current_target, size=0.15)
else:
    self.waypoint_manager.update_active_marker(i, self.flight_controllers[i].current_target)
```