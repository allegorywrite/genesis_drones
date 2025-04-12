"""
SE(3)ドローンシミュレータのメインスクリプト
"""
import numpy as np
import matplotlib.pyplot as plt
from utils.se3 import SE3
from utils.drone import Drone, FeaturePoint
from utils.simulator import Simulator
from utils.visualization import Visualizer
import random
import argparse
from scipy.spatial.transform import Rotation as R
from utils.cbf_se3 import sample_safe_configuration, compute_position_tracking_control
from utils.solver import solve_direct_cbf_qp, solve_direct_qp, solve_distributed_ieq_pdmm_qp

def random_rotation_matrix():
    """
    ランダムな回転行列を生成
    
    Returns:
    --------
    R : ndarray, shape (3, 3)
        ランダムな回転行列
    """
    # ランダムな四元数を生成
    quat = R.random().as_quat()
    # 四元数から回転行列に変換
    return R.from_quat(quat).as_matrix()

def random_position(min_val=-3.0, max_val=3.0):
    """
    ランダムな位置ベクトルを生成
    
    Parameters:
    -----------
    min_val : float
        最小値
    max_val : float
        最大値
        
    Returns:
    --------
    p : ndarray, shape (3,)
        ランダムな位置ベクトル
    """
    return np.array([
        random.uniform(min_val, max_val),
        random.uniform(min_val, max_val),
        random.uniform(min_val, max_val)
    ])

def parse_arguments():
    """
    コマンドライン引数を解析
    
    Returns:
    --------
    args : argparse.Namespace
        解析された引数
    """
    parser = argparse.ArgumentParser(description='SE(3)ドローンシミュレータ')
    parser.add_argument('--fov', type=float, default=30.0,
                        help='視野角（度）（デフォルト: 30.0）')
    parser.add_argument('--min-barrier', type=float, default=0.0,
                        help='最小安全集合値（デフォルト: 0.0）')
    parser.add_argument('--max-attempts', type=int, default=1000,
                        help='最大試行回数（デフォルト: 1000）')
    parser.add_argument('--use-cbf', action='store_true',
                        help='Control Barrier Functionを使用するかどうか')
    parser.add_argument('--q', type=float, default=0.5,
                        help='確率の閾値（デフォルト: 0.5）')
    parser.add_argument('--gamma', type=float, default=0.1,
                        help='CBFのゲイン（デフォルト: 0.1）')
    parser.add_argument('--c1', type=float, default=0.5,
                        help='CBF制約の重み1（デフォルト: 0.5）')
    parser.add_argument('--c2', type=float, default=0.5,
                        help='CBF制約の重み2（デフォルト: 0.5）')
    parser.add_argument('--h', type=float, default=0.1,
                        help='時間ステップ（デフォルト: 0.1）')
    parser.add_argument('--optimization-method', type=str, default='centralized',
                        choices=['centralized', 'distributed'],
                        help='最適化手法（centralized: 集中型, distributed: 分散型）')
    parser.add_argument('--c', type=float, default=1.0,
                        help='分散型最適化のペナルティパラメータ（デフォルト: 1.0）')
    parser.add_argument('--max-iter', type=int, default=10,
                        help='分散型最適化の最大反復回数（デフォルト: 10）')
    parser.add_argument('--keep-fov-history', action='store_true',
                        help='過去の視野錐台描画を残すかどうか')
    parser.add_argument('--fov-save-interval', type=int, default=10,
                        help='視野錐台を保存する間隔（フレーム数）（デフォルト: 10）')
    return parser.parse_args()

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

    # ドローン1: 原点、単位姿勢、指定された視野角
    drone1 = Drone(fov_angle=fov_angle_rad)
    simulator.add_drone(drone1)

    # 特徴点の追加（格子状）
    n_points = 5
    for x in np.linspace(-3, 3, n_points):
        for y in np.linspace(-3, 3, n_points):
            for z in np.linspace(-3, 3, n_points):
                if abs(x) > 1 or abs(y) > 1 or abs(z) > 1:  # 中心付近は除外
                    fp = FeaturePoint([x, y, z])
                    simulator.add_feature_point(fp)
    
    # ドローン2の位置と姿勢をランダムにサンプリングし、
    # 安全集合の値が指定値以上になるまで繰り返す
    success, attempts, barrier_value = sample_safe_configuration(
        simulator, fov_angle_rad, args.min_barrier, args.max_attempts, args.q)
    
    # 安全集合の値を表示
    if success:
        # 共有視野内の特徴点も表示
        cofov_indices = simulator.get_cofov_feature_points(0, 1)
        print(f"共有視野内の特徴点数: {len(cofov_indices)}")
        if len(cofov_indices) > 0:
            print("共有視野内の特徴点インデックス:", cofov_indices)
            print("共有視野内の特徴点座標:")
            for idx in cofov_indices:
                fp = simulator.feature_points[idx]
                print(f"  特徴点 {idx}: {fp.position}")
    
    # 固定目標位置
    target_positions = [
        np.array([4.0, -4.0, -4.0]),  # ドローン1の目標位置
        np.array([-4.0, 4.0, 4.0])  # ドローン2の目標位置
    ]
    
    # 可視化の初期化（目標位置を設定）
    visualizer = Visualizer(
        simulator,
        keep_fov_history=args.keep_fov_history,
        fov_save_interval=args.fov_save_interval
    )
    visualizer.target_positions = target_positions

    # ドローンの入力関数
    def drone_inputs_func(sim, frame):
        # 目標位置は固定
        p1_des, p2_des = target_positions
        
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
                    print(f"フレーム {frame}: 分散型最適化（IEQ-PDMM）")
                    print(f"ドローン1: 位置誤差 = {p1_error:.4f}, 目標位置 = {p1_des}")
                    print(f"ドローン2: 位置誤差 = {p2_error:.4f}, 目標位置 = {p2_des}")
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
                    print(f"フレーム {frame}: 集中型最適化")
                    print(f"ドローン1: 位置誤差 = {p1_error:.4f}, 目標位置 = {p1_des}")
                    print(f"ドローン2: 位置誤差 = {p2_error:.4f}, 目標位置 = {p2_des}")
            
            # 現在の安全集合の値と位置誤差を表示（10フレームごと）
            # if frame % 10 == 0:
            #     barrier_value = compute_barrier_function(
            #         sim.drones[0], sim.drones[1], sim.feature_points, args.q)
            #     p1_error = np.linalg.norm(p1_des - sim.drones[0].T.p)
            #     p2_error = np.linalg.norm(p2_des - sim.drones[1].T.p)
            #     # print(f"フレーム {frame}: 安全集合の値 = {barrier_value:.4f}")
            #     # print(f"ドローン1: 位置誤差 = {p1_error:.4f}, 目標位置 = {p1_des}")
            #     # print(f"ドローン2: 位置誤差 = {p2_error:.4f}, 目標位置 = {p2_des}")
                
            #     # 制約値を表示
            #     if constraint_values is not None:
            #         alpha_omega, beta_omega, alpha_v, beta_v, gamma_val, constraint_value = constraint_values
            #         print(f"CBF制約値: gamma = {gamma_val:.4f}")
            #         if constraint_value is not None:
            #             print(f"制約余裕: {constraint_value}")
            
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

    # if(args.use_cbf):
    #     print(f"CBF制約: gamma = {args.gamma:.4f}")
    # else:
    #     print("CBF制約: 使用しない")

    # アニメーションの作成と表示
    num_frames = 200
    anim = visualizer.animate(num_frames, drone_inputs_func)
    
    # アニメーションオブジェクトを保持するためのグローバル変数
    global_anim = anim
    
    # アニメーションの表示
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
