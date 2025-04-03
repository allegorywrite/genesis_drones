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
    parser.add_argument('--min-shared', type=int, default=3,
                        help='最小共有特徴点数（デフォルト: 3）')
    parser.add_argument('--max-attempts', type=int, default=1000,
                        help='最大試行回数（デフォルト: 1000）')
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
    # 共有視野内の特徴点が見つかるまで繰り返す
    max_attempts = args.max_attempts  # 最大試行回数
    min_shared_points = args.min_shared  # 最小共有特徴点数
    
    print(f"視野角: {args.fov}度（{fov_angle_rad:.4f}ラジアン）")
    print(f"ドローン2の位置と姿勢をランダムにサンプリングしています...")
    print(f"共有視野内に{min_shared_points}個以上の特徴点が見つかるまで繰り返します...")
    
    for attempt in range(max_attempts):
        # ドローン2をシミュレータから削除（2回目以降のループ用）
        if len(simulator.drones) > 1:
            simulator.drones.pop()
        
        # ドローン2: ランダムな位置と姿勢、指定された視野角
        rot_matrix = random_rotation_matrix()
        position = random_position(min_val=-3.0, max_val=3.0)
        
        # 位置が原点に近すぎる場合は調整（ドローン1と重ならないように）
        if np.linalg.norm(position) < 1.0:
            position = position / np.linalg.norm(position) * 1.0
        
        drone2 = Drone(SE3(rot_matrix, position), fov_angle=fov_angle_rad)
        simulator.add_drone(drone2)
        
        # 共有視野内の特徴点を確認
        cofov_indices = simulator.get_cofov_feature_points(0, 1)
        
        # 進捗表示（100回ごと）
        if attempt % 100 == 0 and attempt > 0:
            print(f"  {attempt}回試行: 共有特徴点数 = {len(cofov_indices)}")
        
        # 共有視野内の特徴点が一定数以上あれば終了
        if len(cofov_indices) >= min_shared_points:
            print(f"成功! {attempt + 1}回目の試行で共有視野内に{len(cofov_indices)}個の特徴点が見つかりました")
            print(f"ドローン2の位置: {position}")
            print(f"ドローン2の回転行列:\n{rot_matrix}")
            print("共有視野内の特徴点インデックス:", cofov_indices)
            print("共有視野内の特徴点座標:")
            for idx in cofov_indices:
                fp = simulator.feature_points[idx]
                print(f"  特徴点 {idx}: {fp.position}")
            break
    else:
        print(f"警告: {max_attempts}回の試行で共有視野内に{min_shared_points}個以上の特徴点が見つかりませんでした")
        print(f"最後の試行での共有特徴点数: {len(cofov_indices)}")
    
    # 可視化の初期化
    visualizer = Visualizer(simulator)

    # ドローンの入力関数（円運動）
    def drone_inputs_func(sim, frame):
        # ドローン1: x軸周りの回転と前進
        omega1 = np.array([0.2, 0.0, 0.0])  # x軸周りの回転
        v1 = np.array([0.1, 0.0, 0.0])      # x軸方向の前進
        xi1 = np.concatenate([omega1, v1])
        
        # ドローン2: x軸周りの回転と上昇
        omega2 = np.array([0.2, 0.0, 0.0])  # x軸周りの回転
        v2 = np.array([0.0, 0.0, 0.1])      # z軸方向の上昇
        xi2 = np.concatenate([omega2, v2])
        
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
