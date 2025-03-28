#!/usr/bin/env python3

import genesis as gs
import math
import argparse
import numpy as np
import tempfile
import subprocess
import os
import cv2
from quadcopter_controller import DronePIDController
from genesis.engine.entities.drone_entity import DroneEntity
from genesis.vis.camera import Camera

# 定数
BASE_RPM = 14468.429183500699
MIN_RPM = 0.9 * BASE_RPM
MAX_RPM = 1.5 * BASE_RPM

def clamp(rpm):
    """RPMを最小値と最大値の間に制限する"""
    return max(MIN_RPM, min(int(rpm), MAX_RPM))

def process_xacro(xacro_path):
    """xacroファイルを処理してURDFファイルを作成"""
    if xacro_path.endswith('.xacro'):
        try:
            # 一時ファイルを作成
            fd, temp_urdf_path = tempfile.mkstemp(suffix='.urdf')
            os.close(fd)
            
            # xacroコマンドを実行
            cmd = ['ros2', 'run', 'xacro', 'xacro', xacro_path]
            with open(temp_urdf_path, 'w') as f:
                subprocess.run(cmd, stdout=f, check=True)
            
            # meshファイルのパスを絶対パスに変更
            with open(temp_urdf_path, 'r') as f:
                urdf_content = f.read()
            
            urdf_content = urdf_content.replace(
                '../meshes/crazyflie2.dae',
                '/home/initial/colcon_ws/src/genesis_drones/genesis_drones/meshes/crazyflie2.dae'
            )
            
            # 修正したURDFファイルを書き込む
            with open(temp_urdf_path, 'w') as f:
                f.write(urdf_content)
            
            print(f"Processed xacro file: {xacro_path} -> {temp_urdf_path}")
            return temp_urdf_path
        except Exception as e:
            print(f"Failed to process xacro file: {e}")
            return xacro_path
    
    return xacro_path

def hover(drone: DroneEntity):
    """ドローンをホバリングさせる"""
    drone.set_propellels_rpm([BASE_RPM, BASE_RPM, BASE_RPM, BASE_RPM])

def create_waypoint_marker(scene, position, color=(1.0, 0.5, 0.5), size=0.1):
    """
    ウェイポイントのマーカーを作成する
    
    Args:
        scene (gs.Scene): シーン
        position (tuple): マーカーの位置 (x, y, z)
        color (tuple): マーカーの色 (r, g, b)
        size (float): マーカーのサイズ
        
    Returns:
        Entity: 作成されたマーカーのエンティティ
    """
    try:
        # 球体のマーカーを追加
        marker = scene.add_entity(
            morph=gs.morphs.Sphere(
                pos=position,
                radius=size,
                fixed=True,
                collision=False
            ),
            surface=gs.surfaces.Rough(
                diffuse_texture=gs.textures.ColorTexture(
                    color=color
                )
            )
        )
        return marker
    except Exception as e:
        print(f"Failed to create waypoint marker: {e}")
        return None

def fly_to_point(target, controller: DronePIDController, scene: gs.Scene, cam: Camera, show_camera=False):
    """
    指定したポイントにドローンを飛行させる
    
    Args:
        target (tuple): 目標位置 (x, y, z)
        controller (DronePIDController): ドローンのPIDコントローラ
        scene (gs.Scene): シーン
        cam (gs.Camera): カメラ
        show_camera (bool): カメラ画像をウィンドウに表示するかどうか
    """
    drone = controller.drone
    step = 0
    x = target[0] - drone.get_pos()[0]
    y = target[1] - drone.get_pos()[1]
    z = target[2] - drone.get_pos()[2]

    distance = math.sqrt(x**2 + y**2 + z**2)

    while distance > 0.1 and step < 1000:
        [M1, M2, M3, M4] = controller.update(target)
        M1 = clamp(M1)
        M2 = clamp(M2)
        M3 = clamp(M3)
        M4 = clamp(M4)
        drone.set_propellels_rpm([M1, M2, M3, M4])
        scene.step()
        
        # カメラのレンダリング
        img = cam.render()
        
        # カメラ画像の表示
        if show_camera and img is not None:
            # 画像データの処理
            if isinstance(img, tuple) and len(img) == 4:
                # タプルの最初の要素が画像データ
                width, height = 320, 240  # カメラの解像度（小さくした）
                img_array = np.zeros((height, width, 3), dtype=np.uint8)
                
                try:
                    if img[0] is not None:
                        if isinstance(img[0], np.ndarray):
                            img_array = img[0]
                            if img_array.ndim == 3 and img_array.shape[2] > 3:
                                img_array = img_array[:, :, :3]  # RGBのみ使用
                        elif isinstance(img[0], list):
                            data = np.array(img[0])
                            if data.size > 0:
                                if data.size == width * height * 3:  # RGB
                                    img_array = data.reshape(height, width, 3)
                                elif data.size == width * height * 4:  # RGBA
                                    rgba = data.reshape(height, width, 4)
                                    img_array = rgba[:, :, :3]  # RGBのみ使用
                except Exception as e:
                    print(f"Error processing image data: {e}")
                
                img = img_array
            elif isinstance(img, list) and all(isinstance(x, list) for x in img):
                height, width = 240, 320  # カメラの解像度（小さくした）
                img_array = np.zeros((height, width, 3), dtype=np.uint8)
                
                for y in range(height):
                    for x in range(width):
                        if y*width + x < len(img):
                            pixel = img[y*width + x]
                            if len(pixel) >= 3:  # RGB値があることを確認
                                img_array[y, x] = pixel[:3]  # RGBのみ使用
                
                img = img_array
            elif isinstance(img, np.ndarray):
                if img.ndim == 3 and img.shape[2] >= 3:
                    img = img[:, :, :3]
            
            # OpenCVでBGR形式に変換
            if img.ndim == 3 and img.shape[2] == 3:
                bgr_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                bgr_img = img
                
            # カメラ画像をウィンドウに表示
            cv2.imshow('Drone Camera', bgr_img)
            cv2.waitKey(1)  # 1ミリ秒待機
        
        # ドローンの位置を取得
        drone_pos = drone.get_pos()
        drone_pos = drone_pos.cpu().numpy()
        x = drone_pos[0]
        y = drone_pos[1]
        z = drone_pos[2]
        
        # カメラの位置をドローンに合わせて更新（FPS視点）
        # ドローンの位置よりも前方に配置し、さらに前方を見る
        # ドローンの向きを取得
        drone_quat = drone.get_quat().cpu().numpy()
        
        # カメラをドローンの前方に配置し、さらに前方を見る
        cam.set_pose(
            pos=(x + 0.1, y, z),  # ドローンの少し前方に配置
            lookat=(x + 2.0, y, z)  # より遠くを見る
        )
        
        # 目標位置までの距離を計算
        x_diff = target[0] - x
        y_diff = target[1] - y
        z_diff = target[2] - z
        distance = math.sqrt(x_diff**2 + y_diff**2 + z_diff**2)
        step += 1
    
    print(f"Reached target {target} in {step} steps")

def main():
    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description='ドローンの飛行シミュレーション（カメラ付き、単一球マーカー付き）')
    parser.add_argument('--show-camera', action='store_true', help='カメラ画像をウィンドウに表示する')
    parser.add_argument('--route', type=str, default="1,1,2;-1,2,1;0,0,0.5", help='飛行ルート（例: "1,1,2;-1,2,1;0,0,0.5"）')
    parser.add_argument('--record', action='store_true', help='カメラ映像を録画する')
    parser.add_argument('--output', type=str, default="fly_route_camera.mp4", help='録画ファイルの出力先')
    parser.add_argument('--marker-size', type=float, default=0.1, help='ウェイポイントマーカーのサイズ')
    args = parser.parse_args()
    
    # Genesisの初期化
    gs.init(backend=gs.gpu)

    # シーンの作成
    scene = gs.Scene(show_viewer=True, sim_options=gs.options.SimOptions(dt=0.01))

    # 平面の追加
    plane = scene.add_entity(morph=gs.morphs.Plane())

    # xacroファイルを処理してURDFファイルを作成
    xacro_path = "/home/initial/colcon_ws/src/genesis_drones/genesis_drones/urdf/crazyflie_camera.urdf.xacro"
    urdf_path = process_xacro(xacro_path)
    
    # カメラ付きドローンの追加
    # URDFの代わりにDroneを使用
    drone = scene.add_entity(morph=gs.morphs.Drone(file="urdf/drones/cf2x.urdf", pos=(0, 0, 0.5)))

    # PIDコントローラのパラメータ
    pid_params = [
        [2.0, 0.0, 0.0],  # pos_x
        [2.0, 0.0, 0.0],  # pos_y
        [2.0, 0.0, 0.0],  # pos_z
        [20.0, 0.0, 20.0],  # vel_x
        [20.0, 0.0, 20.0],  # vel_y
        [25.0, 0.0, 20.0],  # vel_z
        [10.0, 0.0, 1.0],  # att_roll
        [10.0, 0.0, 1.0],  # att_pitch
        [2.0, 0.0, 0.2],  # att_yaw
    ]

    # PIDコントローラの初期化
    controller = DronePIDController(drone=drone, dt=0.01, base_rpm=BASE_RPM, pid_params=pid_params)

    # カメラの追加（FPS視点）
    cam = scene.add_camera(
        res=(320, 240),  # 解像度を小さくした
        pos=(0.1, 0, 0.5),  # ドローンの少し前方に配置
        lookat=(2.0, 0, 0.5),  # より遠くを見る
        fov=60,  # 狭い視野角（より望遠的な視点）
        GUI=False
    )

    # 飛行ルートのパース
    points = []
    for point_str in args.route.split(';'):
        coords = [float(x) for x in point_str.split(',')]
        if len(coords) == 3:
            points.append(tuple(coords))
    
    if not points:
        points = [(1, 1, 2), (-1, 2, 1), (0, 0, 0.5)]  # デフォルトのルート
    
    # 色の定義
    colors = [
        (1.0, 0.0, 0.0),  # 赤
        (0.0, 1.0, 0.0),  # 緑
        (0.0, 0.0, 1.0),  # 青
        (1.0, 1.0, 0.0),  # 黄
        (1.0, 0.0, 1.0),  # マゼンタ
        (0.0, 1.0, 1.0),  # シアン
    ]
    
    # 単一のウェイポイントマーカーを作成
    marker = create_waypoint_marker(scene, points[0], colors[0], args.marker_size)
    
    # シーンのビルド
    scene.build()

    # 録画の開始
    if args.record:
        cam.start_recording()
    
    # 各ポイントに飛行
    for i, point in enumerate(points):
        # マーカーを目標位置に移動
        try:
            marker.set_pos(point)
        except Exception as e:
            print(f"Failed to move marker: {e}")
        
        # ポイントに飛行
        fly_to_point(point, controller, scene, cam, show_camera=args.show_camera)

    # 録画の停止
    if args.record:
        cam.stop_recording(save_to_filename=args.output)
    
    # OpenCVのウィンドウを閉じる
    if args.show_camera:
        cv2.destroyAllWindows()
    
    # 一時ファイルの削除
    if urdf_path != xacro_path:
        try:
            os.remove(urdf_path)
            print(f"Removed temporary file: {urdf_path}")
        except Exception as e:
            print(f"Failed to remove temporary file: {e}")

if __name__ == "__main__":
    main()
