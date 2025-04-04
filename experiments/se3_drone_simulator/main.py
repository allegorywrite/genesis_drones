"""
SE(3)ドローンシミュレータのメインスクリプト
"""
import numpy as np
import matplotlib.pyplot as plt
from se3 import SE3
from drone import Drone, FeaturePoint
from simulator import Simulator
from visualization import Visualizer
import random
import argparse
from scipy.spatial.transform import Rotation as R
from cbf_se3 import compute_barrier_function, sample_safe_configuration, generate_safe_control_inputs, generate_position_tracking_control_inputs, compute_position_tracking_control


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
    dt = 0.1  # 時間ステップ
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
        simulator, fov_angle_rad, args.min_barrier, args.max_attempts)
    
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
        np.array([2.0, 2.0, 1.0]),  # ドローン1の目標位置
        np.array([-2.0, -2.0, 1.5])  # ドローン2の目標位置
    ]
    
    # 可視化の初期化（目標位置を設定）
    visualizer = Visualizer(simulator)
    visualizer.target_positions = target_positions

    # ドローンの入力関数
    def drone_inputs_func(sim, frame):
        # 目標位置は固定
        p1_des, p2_des = target_positions
        
        # Control Barrier Functionを使用する場合
        if args.use_cbf:
            # 目標位置追従のための安全な制御入力を生成
            xi1, xi2 = generate_position_tracking_control_inputs(
                sim, p1_des, p2_des, K_p=1.0, q=args.q, gamma=args.gamma)
            
            # 現在の安全集合の値と位置誤差を表示（10フレームごと）
            if frame % 10 == 0:
                barrier_value = compute_barrier_function(
                    sim.drones[0], sim.drones[1], sim.feature_points, args.q)
                p1_error = np.linalg.norm(p1_des - sim.drones[0].T.p)
                p2_error = np.linalg.norm(p2_des - sim.drones[1].T.p)
                print(f"フレーム {frame}: 安全集合の値 = {barrier_value:.4f}")
                print(f"ドローン1: 位置誤差 = {p1_error:.4f}, 目標位置 = {p1_des}")
                print(f"ドローン2: 位置誤差 = {p2_error:.4f}, 目標位置 = {p2_des}")
            
            return [xi1, xi2]
        else:
            # CBFを使用しない場合は単純な位置追従制御
            xi1 = compute_position_tracking_control(sim.drones[0], p1_des, K_p=1.0)
            xi2 = compute_position_tracking_control(sim.drones[1], p2_des, K_p=1.0)
            return [xi1, xi2]

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
