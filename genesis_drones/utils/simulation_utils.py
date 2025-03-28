#!/usr/bin/env python3

import os
import tempfile
import subprocess
import numpy as np
from typing import List, Tuple, Optional, Any

def process_xacro(xacro_path: str) -> str:
    """
    xacroファイルを処理してURDFファイルを作成
    
    Args:
        xacro_path (str): xacroファイルのパス
        
    Returns:
        str: 処理されたURDFファイルのパス
    """
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
            
            # パスを絶対パスに変更
            urdf_content = urdf_content.replace(
                '../meshes/crazyflie2.dae',
                '/home/initial/colcon_ws/src/genesis_drones/genesis_drones/meshes/crazyflie2.dae'
            )
            urdf_content = urdf_content.replace(
                '../meshes/crazyflie.dae',
                '/home/initial/colcon_ws/src/genesis_drones/genesis_drones/meshes/crazyflie.dae'
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

def cleanup_temp_files(temp_files: List[str]) -> None:
    """
    一時ファイルを削除
    
    Args:
        temp_files (list): 削除する一時ファイルのリスト
    """
    for file_path in temp_files:
        try:
            if os.path.exists(file_path):
                os.remove(file_path)
                print(f"Removed temporary file: {file_path}")
        except Exception as e:
            print(f"Failed to remove temporary file {file_path}: {e}")

def parse_route_string(route_str: str) -> List[Tuple[float, float, float]]:
    """
    ルート文字列をパース
    
    Args:
        route_str (str): ルート文字列（例: "1,1,2;-1,2,1;0,0,0.5"）
        
    Returns:
        list: ポイントのリスト [(x1, y1, z1), (x2, y2, z2), ...]
    """
    points = []
    
    try:
        # セミコロンで区切られたポイント文字列をパース
        for point_str in route_str.split(';'):
            if not point_str.strip():
                continue
                
            # カンマで区切られた座標をパース
            coords = [float(x) for x in point_str.split(',')]
            
            if len(coords) == 3:
                points.append(tuple(coords))
            else:
                print(f"Invalid point format: {point_str} (expected 3 coordinates)")
    except Exception as e:
        print(f"Error parsing route string: {e}")
    
    return points

def parse_multi_drone_routes(routes_str: str, num_drones: int) -> List[Tuple[int, List[Tuple[float, float, float]]]]:
    """
    複数ドローンのルート文字列をパース
    
    Args:
        routes_str (str): ルート文字列（例: "0:1,1,2;-1,2,1|1:0,0,1;1,1,1"）
        num_drones (int): ドローンの数
        
    Returns:
        list: (drone_id, points)のリスト
    """
    drone_routes = []
    
    try:
        # パイプで区切られたドローンごとのルート文字列をパース
        for drone_route in routes_str.split('|'):
            if not drone_route.strip():
                continue
                
            if ':' in drone_route:
                # ドローンIDとルート文字列を分離
                drone_id_str, route_str = drone_route.split(':', 1)
                
                try:
                    drone_id = int(drone_id_str)
                    
                    # ドローンIDが有効かチェック
                    if 0 <= drone_id < num_drones:
                        # ルート文字列をパース
                        points = parse_route_string(route_str)
                        
                        if points:
                            drone_routes.append((drone_id, points))
                    else:
                        print(f"Invalid drone ID: {drone_id} (must be between 0 and {num_drones-1})")
                except ValueError:
                    print(f"Invalid drone ID format: {drone_id_str}")
    except Exception as e:
        print(f"Error parsing multi-drone routes: {e}")
    
    return drone_routes

def initialize_genesis_scene(dt: float = 0.033, show_viewer: bool = True) -> Any:
    """
    Genesisシーンを初期化
    
    Args:
        dt (float): シミュレーションの時間ステップ
        show_viewer (bool): ビューアを表示するかどうか
        
    Returns:
        gs.Scene: 初期化されたシーン
    """
    import genesis as gs
    
    # Genesisの初期化
    gs.init(backend=gs.cpu)
    
    # シーンの作成
    scene = gs.Scene(
        show_viewer=show_viewer, 
        sim_options=gs.options.SimOptions(dt=dt)
    )
    
    # 平面の追加
    scene.add_entity(gs.morphs.Plane())
    
    return scene

def add_drone_to_scene(scene: Any, position: Tuple[float, float, float] = (0, 0, 0.5), 
                      urdf_path: Optional[str] = None) -> Any:
    """
    シーンにドローンを追加
    
    Args:
        scene (gs.Scene): シーン
        position (tuple): ドローンの初期位置
        urdf_path (str): URDFファイルのパス（指定しない場合はデフォルトのURDFを使用）
        
    Returns:
        DroneEntity: 追加されたドローン
    """
    import genesis as gs
    
    # URDFパスが指定されていない場合はデフォルトのURDFを使用
    if urdf_path is None:
        urdf_path = "/home/initial/lab_ws/Genesis/genesis/assets/urdf/drones/cf2x.urdf"
    
    # ドローンの追加
    drone = scene.add_entity(
        gs.morphs.Drone(
            file=urdf_path,
            pos=position,
            collision=False
        )
    )

    # print(f"KF, KM, : {drone._KF}, {drone._KM}")
    
    return drone

def add_camera_to_scene(scene: Any, position: Tuple[float, float, float] = (0.1, 0, 0.5),
                       lookat: Tuple[float, float, float] = (2.0, 0, 0.5),
                       resolution: Tuple[int, int] = (320, 240),
                       fov: float = 60.0) -> Any:
    """
    シーンにカメラを追加
    
    Args:
        scene (gs.Scene): シーン
        position (tuple): カメラの初期位置
        lookat (tuple): カメラの注視点
        resolution (tuple): カメラの解像度
        fov (float): 視野角
        
    Returns:
        Camera: 追加されたカメラ
    """
    # カメラの追加
    camera = scene.add_camera(
        res=resolution,
        pos=position,
        lookat=lookat,
        fov=fov,
        GUI=False
    )
    
    return camera

def update_camera_position(camera: Any, drone_pos: np.ndarray, drone_quat: np.ndarray, 
                          direction: Optional[np.ndarray] = None) -> None:
    """
    ドローンの位置と向きに基づいてカメラの位置を更新
    
    Args:
        camera (Camera): カメラ
        drone_pos (np.ndarray): ドローンの位置
        drone_quat (np.ndarray): ドローンの姿勢（クォータニオン）
        direction (np.ndarray): 進行方向ベクトル（使用しない）
    """
    # クォータニオンから回転行列を計算
    def quat_to_rot_matrix(quat):
        """クォータニオンから回転行列を計算"""
        w, x, y, z = quat
        
        # 回転行列の計算
        xx, xy, xz = x*x, x*y, x*z
        yy, yz, zz = y*y, y*z, z*z
        wx, wy, wz = w*x, w*y, w*z
        
        rot_matrix = np.array([
            [1 - 2*(yy + zz), 2*(xy - wz), 2*(xz + wy)],
            [2*(xy + wz), 1 - 2*(xx + zz), 2*(yz - wx)],
            [2*(xz - wy), 2*(yz + wx), 1 - 2*(xx + yy)]
        ])
        
        return rot_matrix
    
    # ドローンの回転行列を計算
    rot_matrix = quat_to_rot_matrix(drone_quat)
    
    # ドローンの前方ベクトル（X軸方向）
    forward_vector = rot_matrix @ np.array([1.0, 0.0, 0.0])
    
    # カメラをドローンに固定（ドローンの前方に配置）
    camera_offset = np.array([0.1, 0.0, 0.0])  # ドローンの座標系での位置
    
    # ドローンの座標系からワールド座標系への変換
    world_camera_offset = rot_matrix @ camera_offset
    
    # カメラの位置を計算
    camera_pos = drone_pos + world_camera_offset
    
    # カメラの注視点（ドローンの前方2.0単位）
    lookat_offset = forward_vector * 2.0
    lookat_pos = drone_pos + lookat_offset
    
    # カメラの位置と注視点を更新
    camera.set_pose(
        pos=tuple(camera_pos),
        lookat=tuple(lookat_pos)
    )
