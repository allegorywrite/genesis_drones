"""
単一ドローンのシミュレーションデモ
視野内の点を強調表示
"""
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# 親ディレクトリをパスに追加
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.drone import Drone

# 同じディレクトリ内のモジュール
from .visualizer import SingleDroneVisualizer
from .controller import create_drone_input_func
from .utils import generate_feature_points, setup_drone_initial_pose


def main():
    """
    メイン関数
    """
    # コマンドライン引数の解析
    import argparse
    parser = argparse.ArgumentParser(description='単一ドローンのシミュレーションデモ')
    parser.add_argument('--use-cbf', action='store_true', help='CBF制約を使用する')
    parser.add_argument('--cbf-type', type=str, choices=['no-decomp', 'decomp'], default='no-decomp', 
                        help='CBF制約のタイプ: no-decomp (速度入力及び角速度入力について分解しない場合) または decomp (分解する場合)')
    parser.add_argument('--cbf-method', type=str, choices=['pcl', 'point'], default='pcl',
                        help='CBF制約の方法: pcl (複数特徴点に対するCBF) または point (単一特徴点に対するCBF)')
    parser.add_argument('--obstacles', type=int, default=0, help='障害物の数')
    args = parser.parse_args()
    
    # 特徴点の追加（より局在化された配置）
    feature_area_center = np.array([0.0, 3.0, 0.0])  # 目標位置付近
    feature_points = generate_feature_points(
        num_features=10,
        center=feature_area_center,
        radius=1.0,
        seed=42
    )
    
    # CBFメソッドが'point'の場合は、最初の特徴点のみを使用
    if args.use_cbf and args.cbf_method == 'point':
        # 最も中心に近い特徴点を選択
        closest_feature_idx = np.argmin([np.linalg.norm(fp.position - feature_area_center) for fp in feature_points])
        feature_points = [feature_points[closest_feature_idx]]
        print(f"単一特徴点モード: 特徴点位置 = {feature_points[0].position}")
    
    # ドローンの初期化（視野角を設定：π/6 = 30度）
    drone = Drone(fov_angle=np.pi/6)
    
    # 初期位置と姿勢を特徴点が視野内に入るように設定
    setup_drone_initial_pose(drone, feature_area_center)
    
    # 初期の目標位置
    target_position = np.array([3.0, 0.0, 0.0])
    
    # 可視化の初期化（目標位置を設定）
    visualizer = SingleDroneVisualizer(drone, feature_points, target_position=target_position)
    
    # CBF制約を使用するかどうかを表示
    if args.use_cbf:
        print("CBF制約を使用します")
    else:
        print("CBF制約を使用しません")
    
    # ドローンの入力関数を作成
    drone_input_func = create_drone_input_func(
        drone,
        feature_points,
        target_position,
        use_cbf=args.use_cbf,
        cbf_type=args.cbf_type,
        cbf_method=args.cbf_method
    )
    
    # アニメーションの作成と表示
    num_frames = 200
    
    # 入力関数をラップして、gamma値と制約余裕の値を可視化に渡す
    def wrapped_input_func(drone, frame):
        result = drone_input_func(drone, frame)
        return result  # (xi, gamma_val, constraint_margin)を返す
    
    anim = visualizer.animate(num_frames, wrapped_input_func)
    
    # アニメーションオブジェクトを保持するためのグローバル変数
    global global_anim
    global_anim = anim
    
    # アニメーションの表示
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
