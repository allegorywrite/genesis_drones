#!/usr/bin/env python3

import math
import argparse
import subprocess
import numpy as np
from typing import List, Tuple

def generate_circle_points(
    radius: float,
    height: float,
    num_points: int,
    center: Tuple[float, float] = (0.0, 0.0)
) -> List[Tuple[float, float, float]]:
    """
    円周上の点を生成する
    
    Args:
        radius (float): 円の半径
        height (float): 飛行高度
        num_points (int): 生成する点の数
        center (tuple): 円の中心座標 (x, y)
        
    Returns:
        List[Tuple[float, float, float]]: 円周上の点のリスト [(x1, y1, z1), (x2, y2, z2), ...]
    """
    points = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        points.append((x, y, height))
    return points

def generate_circle_trajectory(
    radius: float,
    height: float,
    num_points_per_circle: int = 12,
    num_circles: int = 1,
    center: Tuple[float, float] = (0.0, 0.0)
) -> List[Tuple[float, float, float]]:
    """
    円軌道を生成する
    
    Args:
        radius (float): 円の半径
        height (float): 飛行高度
        num_points_per_circle (int): 1周あたりの点の数
        num_circles (int): 周回数
        center (tuple): 円の中心座標 (x, y)
        
    Returns:
        List[Tuple[float, float, float]]: 円軌道上の点のリスト [(x1, y1, z1), (x2, y2, z2), ...]
    """
    total_points = num_points_per_circle * num_circles
    points = []
    for i in range(total_points):
        angle = 2 * math.pi * i / num_points_per_circle
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        points.append((x, y, height))
    return points

def format_route_string(points: List[Tuple[float, float, float]]) -> str:
    """
    点のリストをルート文字列に変換する
    
    Args:
        points (List[Tuple[float, float, float]]): 点のリスト [(x1, y1, z1), (x2, y2, z2), ...]
        
    Returns:
        str: ルート文字列 "x1,y1,z1;x2,y2,z2;..."
    """
    return ";".join([f"{x},{y},{z}" for x, y, z in points])

def main():
    parser = argparse.ArgumentParser(description='円形フォーメーションでのドローンシミュレーション')
    parser.add_argument('--num-drones', type=int, default=3, help='ドローンの数')
    parser.add_argument('--radius', type=float, default=2.0, help='円の半径')
    parser.add_argument('--height', type=float, default=1.0, help='飛行高度')
    parser.add_argument('--center-x', type=float, default=0.0, help='円の中心のX座標')
    parser.add_argument('--center-y', type=float, default=0.0, help='円の中心のY座標')
    parser.add_argument('--points-per-circle', type=int, default=12, help='1周あたりの点の数')
    parser.add_argument('--num-circles', type=int, default=2, help='周回数')
    parser.add_argument('--show-camera', action='store_true', help='カメラ画像を表示する')
    parser.add_argument('--record', action='store_true', help='カメラ映像を録画する')
    parser.add_argument('--output', type=str, default="circle_formation.mp4", help='録画ファイルの出力先')
    args = parser.parse_args()
    
    # 円の中心
    center = (args.center_x, args.center_y)
    
    # 各ドローンの初期位置（円周上に均等に配置）
    initial_positions = generate_circle_points(
        radius=args.radius,
        height=args.height,
        num_points=args.num_drones,
        center=center
    )
    
    # 各ドローンの軌道を生成
    routes = []
    for i in range(args.num_drones):
        # 各ドローンの初期位置から開始する軌道を生成
        # 位相をずらすために、初期位置からの角度オフセットを計算
        phase_offset = 2 * math.pi * i / args.num_drones
        
        # 軌道の点を生成
        trajectory = []
        for j in range(args.points_per_circle * args.num_circles):
            angle = 2 * math.pi * j / args.points_per_circle + phase_offset
            x = center[0] + args.radius * math.cos(angle)
            y = center[1] + args.radius * math.sin(angle)
            trajectory.append((x, y, args.height))
        
        # 軌道を追加
        routes.append((i, trajectory))
    
    # ルート文字列を生成
    routes_str = "|".join([f"{drone_id}:{format_route_string(trajectory)}" for drone_id, trajectory in routes])
    
    # コマンドを構築
    cmd = [
        "python3", "-m", "genesis_drones.multi_drone_simulation",
        "--num-drones", str(args.num_drones),
        "--routes", routes_str
    ]
    
    # オプションを追加
    if args.show_camera:
        cmd.append("--show-camera")
    if args.record:
        cmd.append("--record")
        cmd.extend(["--output", args.output])
    
    # コマンドを実行
    print(f"実行コマンド: {' '.join(cmd)}")
    subprocess.run(cmd)

if __name__ == "__main__":
    main()
