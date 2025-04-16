"""
SE(3)ドローンシミュレータのメインスクリプト
"""
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
import os

# 親ディレクトリをパスに追加（絶対パスを使用）
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from scipy.spatial.transform import Rotation as R
from utils.se3 import SE3
from utils.drone import Drone, FeaturePoint
from utils.simulator import Simulator
from utils.visualization import Visualizer
from utils.solver import solve_direct_cbf_qp, solve_direct_qp, solve_distributed_ieq_pdmm_qp

# 分割したモジュールをインポート
from multi_drones.config import parse_arguments
from multi_drones.trajectory_generator import generate_target_trajectory
from multi_drones.utils import random_rotation_matrix, random_position
from multi_drones.setup import initialize_drones, setup_drone_initial_pose, add_feature_points, find_safe_pose_for_both_drones
from multi_drones.controller import create_drone_inputs_func


def main():
    """
    メイン関数
    """
    # コマンドライン引数の解析
    args = parse_arguments()
    
    # 視野角をラジアンに変換
    fov_angle_rad = np.radians(args.fov)
    
    # シミュレータの初期化
    dt = args.h
    simulator = Simulator(dt)

    # フレーム数の設定
    num_frames = 200
    
    # 目標軌道の生成（2機分）
    print(f"軌道タイプ: {args.trajectory_type}")
    target_trajectories = []
    for i in range(2):  # 2機分の軌道を生成
        trajectory = generate_target_trajectory(
            trajectory_type=args.trajectory_type,
            center=np.array([0.0, 0.0, 0.0]),
            radius=3.0,
            period=10.0,
            num_frames=num_frames,
            drone_idx=i,
            num_drones=2  # 常に2機分
        )
        target_trajectories.append(trajectory)
    
    # 特徴点を追加
    add_feature_points(simulator, args.trajectory_type)
    
    # 両方のドローンの安全な姿勢を同時に探索
    p_start1 = target_trajectories[0][0].copy()
    p_start2 = target_trajectories[1][0].copy()
    drone1, drone2, success = find_safe_pose_for_both_drones(
        simulator, p_start1, p_start2, fov_angle_rad, 
        min_barrier=args.min_barrier, 
        max_attempts=args.max_attempts,
        trajectory_type=args.trajectory_type,
        dynamics_model=args.dynamics_model
    )
    
    # 共有視野内の特徴点を表示
    cofov_indices = simulator.get_cofov_feature_points(0, 1)
    # print(f"共有視野内の特徴点数: {len(cofov_indices)}")
    if len(cofov_indices) > 0:
        # print("共有視野内の特徴点インデックス:", cofov_indices)
        # print("共有視野内の特徴点座標:")
        for idx in cofov_indices:
            fp = simulator.feature_points[idx]
            # print(f"  特徴点 {idx}: {fp.position}")
    
    # 可視化の初期化
    visualizer = Visualizer(
        simulator,
        keep_fov_history=args.keep_fov_history,
        fov_save_interval=args.fov_save_interval
    )
    
    # 目標軌道全体を可視化
    visualizer.target_trajectories = target_trajectories
    
    # ドローンの入力関数を作成
    drone_inputs_func = create_drone_inputs_func(simulator, target_trajectories, visualizer, args, num_frames)

    # アニメーションの作成と表示
    anim = visualizer.animate(num_frames, drone_inputs_func)
    
    # アニメーションオブジェクトを保持するためのグローバル変数
    global_anim = anim
    
    # アニメーションの表示
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
