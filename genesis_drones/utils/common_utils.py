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
