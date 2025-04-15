"""
ドローンのコントローラー関連の関数を提供するモジュール
"""
import numpy as np
import sys
import os

# 親ディレクトリをパスに追加（絶対パスを使用）
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from utils.solver import solve_direct_cbf_qp, solve_direct_qp, solve_distributed_ieq_pdmm_qp
from utils.cbf_se3 import compute_position_tracking_control, compute_position_tracking_acceleration_control
from utils.drone import DynamicDrone

def create_drone_inputs_func(simulator, target_trajectories, visualizer, args, num_frames):
    """
    ドローンの入力関数を作成
    
    Parameters:
    -----------
    simulator : Simulator
        シミュレータ
    target_trajectories : list of ndarray
        各ドローンの目標軌道
    visualizer : Visualizer
        可視化オブジェクト
    args : argparse.Namespace
        コマンドライン引数
    num_frames : int
        フレーム数
        
    Returns:
    --------
    drone_inputs_func : callable
        ドローンの入力関数
    """
    # 目標位置の初期化
    target_positions = [trajectory[0] for trajectory in target_trajectories]
    visualizer.target_positions = target_positions
    
    # 軌道追従が完了したかどうかのフラグ
    trajectory_completed = False
    
    def drone_inputs_func(sim, frame):
        nonlocal trajectory_completed, target_positions
        
        # 軌道の終了判定（最後のフレームに達したら停止）
        if trajectory_completed:
            return None
        
        # 最後のフレームに近づいたら停止
        if frame >= num_frames-1:
            print(f"フレーム {frame} で軌道追従が完了しました。更新を停止します。")
            trajectory_completed = True
            return None
        
        # 現在のフレームに対応する目標位置を更新
        current_target_positions = []
        for i, trajectory in enumerate(target_trajectories):
            if frame < len(trajectory):
                current_target = trajectory[frame]
                current_target_positions.append(current_target)
            else:
                # フレーム数が軌道の長さを超えた場合は最後の位置を使用
                current_target_positions.append(trajectory[-1])
        
        # 目標位置を更新
        target_positions = current_target_positions
        visualizer.target_positions = target_positions
        
        # 目標位置を取得
        p1_des, p2_des = target_positions
        
        # 動力学モデルが2次系の場合
        if args.dynamics_model == 'dynamics':
            from utils.cbf_se3 import compute_position_tracking_acceleration_control
            from utils.drone import DynamicDrone
            
            # 2次系モデルの場合は加速度入力を計算
            if isinstance(sim.drones[0], DynamicDrone) and isinstance(sim.drones[1], DynamicDrone):
                # 単純なPD制御（CBF制約なし）
                u1 = compute_position_tracking_acceleration_control(sim.drones[0], p1_des)
                u2 = compute_position_tracking_acceleration_control(sim.drones[1], p2_des)
                
                # 現在の位置誤差を表示（10フレームごと）
                if frame % 10 == 0:
                    p1_error = np.linalg.norm(p1_des - sim.drones[0].T.p)
                    p2_error = np.linalg.norm(p2_des - sim.drones[1].T.p)
                    # print(f"フレーム {frame}: 2次系モデル（動力学モデル）")
                    # print(f"ドローン1: 位置誤差 = {p1_error:.4f}, 目標位置 = {p1_des}")
                    # print(f"ドローン2: 位置誤差 = {p2_error:.4f}, 目標位置 = {p2_des}")
                
                return [u1, u2]
        
        # 1次系モデルの場合（既存の実装）
        # Control Barrier Functionを使用する場合
        if args.use_cbf:
            # 最適化手法に基づいて適切な関数を呼び出す
            if args.optimization_method == 'distributed':
                # 分散型最適化（IEQ-PDMM）
                xi1, xi2, constraint_values = solve_distributed_ieq_pdmm_qp(
                    sim.drones[0], sim.drones[1], sim.feature_points, 
                    p1_des, p2_des, q=args.q, gamma0=args.gamma, 
                    c=args.c, max_iter=args.max_iter, h=args.h)
                
                # 現在の安全集合の値と位置誤差を表示（10フレームごと）
                if frame % 10 == 0:
                    p1_error = np.linalg.norm(p1_des - sim.drones[0].T.p)
                    p2_error = np.linalg.norm(p2_des - sim.drones[1].T.p)
                    # print(f"フレーム {frame}: 分散型最適化（IEQ-PDMM）")
                    # print(f"ドローン1: 位置誤差 = {p1_error:.4f}, 目標位置 = {p1_des}")
                    # print(f"ドローン2: 位置誤差 = {p2_error:.4f}, 目標位置 = {p2_des}")
            else:
                # 集中型最適化（既存の実装）
                xi1, xi2, constraint_values = solve_direct_cbf_qp(
                    sim.drones[0], sim.drones[1], sim.feature_points, 
                    p1_des, p2_des, q=args.q, gamma0=args.gamma, 
                    c1=args.c1, c2=args.c2, h=args.h)
                
                # 現在の安全集合の値と位置誤差を表示（10フレームごと）
                if frame % 10 == 0:
                    p1_error = np.linalg.norm(p1_des - sim.drones[0].T.p)
                    p2_error = np.linalg.norm(p2_des - sim.drones[1].T.p)
                    # print(f"フレーム {frame}: 集中型最適化")
                    # print(f"ドローン1: 位置誤差 = {p1_error:.4f}, 目標位置 = {p1_des}")
                    # print(f"ドローン2: 位置誤差 = {p2_error:.4f}, 目標位置 = {p2_des}")
            
            # 制約値を可視化に渡す
            if constraint_values is not None:
                visualizer.constraint_values = constraint_values
            
            return [xi1, xi2]
        else:
            # CBFを使用しない場合も制約なしQPを解く
            xi1, xi2, _ = solve_direct_qp(
                sim.drones[0], sim.drones[1], sim.feature_points, 
                p1_des, p2_des, h=args.h)
            
            # 現在の位置誤差を表示（10フレームごと）
            if frame % 10 == 0:
                p1_error = np.linalg.norm(p1_des - sim.drones[0].T.p)
                p2_error = np.linalg.norm(p2_des - sim.drones[1].T.p)
                print(f"フレーム {frame}: 制約なしQP")
                print(f"ドローン1: 位置誤差 = {p1_error:.4f}, 目標位置 = {p1_des}")
                print(f"ドローン2: 位置誤差 = {p2_error:.4f}, 目標位置 = {p2_des}")
            
            return [xi1, xi2]
    
    return drone_inputs_func
