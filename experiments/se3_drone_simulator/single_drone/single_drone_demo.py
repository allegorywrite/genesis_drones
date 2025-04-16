"""
単一ドローンのシミュレーションデモ
視野内の点を強調表示
目標軌道に追従
"""
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# 親ディレクトリをパスに追加
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.drone import Drone
from utils.constants import set_gravity

# 同じディレクトリ内のモジュール
from .visualizer import SingleDroneVisualizer
from .controller import create_drone_input_func
from .utils import generate_feature_points, setup_drone_initial_pose
from utils.drone import FeaturePoint
from .hocbf_visualizer import HOCBFVisualizer
from utils.drone import DynamicDrone
from utils.se3 import SE3

def generate_target_trajectory(trajectory_type='circle', center=np.array([0.0, 0.0, 0.0]), 
                              radius=3.0, period=10.0, num_frames=500):
    """
    目標軌道を生成する
    
    Parameters:
    -----------
    trajectory_type : str, optional
        軌道の種類（'circle', 'eight', 'lissajous', 'snake', 'snake_3d', 'step'）（デフォルトは'circle'）
    center : array_like, shape (3,), optional
        軌道の中心位置（デフォルトは[0.0, 0.0, 0.0]）
    radius : float, optional
        軌道の半径（デフォルトは3.0）
    period : float, optional
        1周期の時間（秒）（デフォルトは10.0）
    num_frames : int, optional
        フレーム数（デフォルトは200）
    
    Returns:
    --------
    trajectory : ndarray, shape (num_frames, 3)
        各フレームでの目標位置
    """
    # 時間パラメータ（0から1の範囲）
    t = np.linspace(0, 1, num_frames)
    
    # 軌道の初期化
    trajectory = np.zeros((num_frames, 3))
    
    if trajectory_type == 'circle':
        # 円軌道
        trajectory[:, 0] = center[0] + radius * np.cos(2 * np.pi * t)
        trajectory[:, 1] = center[1] + radius * np.sin(2 * np.pi * t)
        trajectory[:, 2] = center[2]
        
    elif trajectory_type == 'eight':
        # 8の字軌道
        trajectory[:, 0] = center[0] + radius * np.sin(2 * np.pi * t)
        trajectory[:, 1] = center[1] + radius * np.sin(4 * np.pi * t) / 2
        trajectory[:, 2] = center[2]
        
    elif trajectory_type == 'lissajous':
        # リサージュ曲線
        trajectory[:, 0] = center[0] + radius * np.sin(2 * np.pi * t)
        trajectory[:, 1] = center[1] + radius * np.sin(4 * np.pi * t)
        trajectory[:, 2] = center[2] + radius * np.sin(6 * np.pi * t) / 3
        
    elif trajectory_type == 'snake':
        # 点群に向かう進行方向に対して垂直に周波数成分を持つ軌道
        # 初期位置（ドローンの初期位置付近）
        start_point = np.array([0.0, -5.0, 0.0])
        # 目標位置（特徴点の中心付近）
        end_point = np.array([0.0, 3.0, 0.0])
        
        # 進行方向ベクトル
        direction = end_point - start_point
        direction = direction / np.linalg.norm(direction)  # 正規化
        
        # 進行方向に垂直なベクトル（外積を使用）
        # 進行方向は主にXY平面にあるので、Z軸との外積を取る
        z_axis = np.array([0.0, 0.0, 1.0])
        perpendicular = np.cross(direction, z_axis)
        perpendicular = perpendicular / np.linalg.norm(perpendicular)  # 正規化
        
        # 直線的に進む成分
        for i in range(3):
            trajectory[:, i] = start_point[i] + (end_point[i] - start_point[i]) * t
        
        # 進行方向に垂直な方向に周波数成分（サイン波）を追加
        amplitude = 3.0  # 振幅
        frequency = 2.0  # 周波数
        
        # サイン波の周波数成分
        sin_component = amplitude * np.sin(2 * np.pi * frequency * t)
        
        # 進行方向に垂直な方向にサイン波を適用
        for i in range(3):
            trajectory[:, i] += perpendicular[i] * sin_component

    elif trajectory_type == 'snake_3d':
        # 蛇行しながら点群に向かう軌道
        # 初期位置（ドローンの初期位置付近）
        start_point = np.array([0.0, -5.0, 0.0])
        # 目標位置（特徴点の中心付近）
        end_point = np.array([0.0, 3.0, 0.0])
        
        # 直線的に進む成分
        for i in range(3):
            trajectory[:, i] = start_point[i] + (end_point[i] - start_point[i]) * t
        
        # 蛇行成分を追加（美しいサイン波）
        # 蛇行の周波数（低くするとよりなめらかに）
        freq = 3.0
        
        # 蛇行の振幅
        amplitude = 3.0
    
        # y軸に絶対に蛇行せず固定値で進む
        trajectory[:, 1] += (end_point[1] - start_point[1]) / num_frames
        
        # x,z平面で回転運動
        trajectory[:, 0] += amplitude * np.sin(2 * np.pi * freq * t)
        trajectory[:, 2] += amplitude * np.cos(2 * np.pi * freq * t)    
        
    elif trajectory_type == 'step':
        # 原点から始まり、[0, 0, 3]に固定された目標位置
        fixed_target = np.array([0.0, 0.0, 3.0])
        
        # すべてのフレームで同じ目標位置
        for i in range(num_frames):
            trajectory[i] = fixed_target
        
        # 始点を原点に設定
        trajectory[0] = np.array([0.0, 0.0, 0.0])
        
    else:
        raise ValueError(f"不明な軌道タイプ: {trajectory_type}")
    
    return trajectory


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
    parser.add_argument('--keep-fov-history', action='store_true', help='過去の視野錐台描画を残す')
    parser.add_argument('--fov-save-interval', type=int, default=10, help='視野錐台を保存する間隔（フレーム数）')
    parser.add_argument('--trajectory-type', type=str, choices=['circle', 'eight', 'lissajous', 'snake', 'snake_3d', 'step'], default='circle',
                        help='目標軌道の種類: circle (円軌道), eight (8の字軌道), lissajous (リサージュ曲線), snake (蛇行軌道), snake_3d (3D蛇行軌道), step ([0,0,1]に固定)')
    parser.add_argument('--dynamics-model', type=str, choices=['kinematics', 'dynamics', 'holonomic_dynamics'], default='kinematics',
                        help='動力学モデル: kinematics (1次系), dynamics (2次系), または holonomic_dynamics (ホロノミック2次系)')
    parser.add_argument('--gravity', type=float, nargs=3, default=[0, 0, 9.81], help='重力加速度ベクトル [x, y, z]（デフォルトは[0, 0, 9.81]）')
    parser.add_argument('--dt', type=float, default=0.01, help='時間ステップ')
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
        # closest_feature_idx = np.argmin([np.linalg.norm(fp.position - feature_area_center) for fp in feature_points])
        # feature_points = [feature_points[closest_feature_idx]]
        feature_points = [FeaturePoint(position=np.array([0.0, 3.0, 0.0]))]
        print(f"単一特徴点モード: 特徴点位置 = {feature_points[0].position}")
    
    # 重力加速度を設定
    gravity = np.array(args.gravity)
    set_gravity(gravity)
    
    # フレーム数
    num_frames = 200
    
    # 目標軌道の生成
    target_trajectory = generate_target_trajectory(
        trajectory_type=args.trajectory_type,
        center=np.array([0.0, 0.0, 0.0]),
        radius=3.0,
        period=10.0,
        num_frames=num_frames
    )
    
    # 初期の目標位置（軌道の最初の点）
    target_position = target_trajectory[0]
    
    # 動力学モデルに応じてドローンを初期化
    if args.dynamics_model == 'dynamics' or args.dynamics_model == 'holonomic_dynamics':
        camera_direction = np.array([0, 1, 0])
        # ドローンの初期位置を軌道の始点に合わせる
        T = SE3(R=np.eye(3), p=target_position)
        drone = DynamicDrone(fov_angle=np.pi/6, T=T, camera_direction=camera_direction, dynamics_model=args.dynamics_model)
        print("2次系モデル（動力学モデル）を使用します")
    else:
        drone = Drone(fov_angle=np.pi/6)
        print("1次系モデル（運動学モデル）を使用します")
        # ドローンの初期位置を軌道の始点に設定
        drone.position = target_position.copy()
        # 初期位置と姿勢を特徴点が視野内に入るように設定（カメラ方向のみ）
        setup_drone_initial_pose(drone, feature_area_center)
    
    
    # 可視化の初期化（目標位置を設定）
    visualizer = SingleDroneVisualizer(
        drone,
        feature_points,
        target_position=target_position,
        keep_fov_history=args.keep_fov_history,
        fov_save_interval=args.fov_save_interval,
        dt=args.dt
    )
    
    # 目標軌道全体を可視化
    visualizer.set_target_trajectory(target_trajectory)
    
    # CBF制約を使用するかどうかを表示
    if args.use_cbf:
        print("CBF制約を使用します")
    else:
        print("CBF制約を使用しません")
    
    # HOCBFの可視化（2次系モデルの場合のみ）
    hocbf_visualizer = None
    if args.dynamics_model == 'dynamics' or args.dynamics_model == 'holonomic_dynamics':
        hocbf_visualizer = HOCBFVisualizer()
        hocbf_visualizer.show()
    
    # ドローンの入力関数を作成
    drone_input_func = create_drone_input_func(
        drone,
        feature_points,
        target_trajectory,  # 目標軌道を渡す
        use_cbf=args.use_cbf,
        cbf_type=args.cbf_type,
        cbf_method=args.cbf_method,
        dynamics_model=args.dynamics_model,
        hocbf_visualizer=hocbf_visualizer,
        dt=args.dt
    )
    
    # 軌道追従が完了したかどうかのフラグ
    trajectory_completed = False
    
    # 入力関数をラップして、gamma値と制約余裕の値を可視化に渡す
    def wrapped_input_func(drone, frame):
        nonlocal trajectory_completed
        
        # 軌道追従が既に完了している場合は停止状態を維持
        if trajectory_completed:
            return None
            # if args.dynamics_model == 'dynamics':
            #     return np.zeros(4), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0  # 停止状態の入力と値を返す（2次系）
            # elif args.dynamics_model == 'holonomic_dynamics':
            #     return np.zeros(6), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0  # 停止状態の入力と値を返す（ホロノミック2次系）
            # else:
            #     return np.zeros(6), 0.0, 0.0, 0.0  # 停止状態の入力と値を返す（1次系）
        
        # 軌道の終了判定（最後のフレームに達したら停止）
        if frame >= len(target_trajectory) - 20:
            print(f"フレーム {frame} で軌道追従が完了しました。更新を停止します。")
            trajectory_completed = True  # 軌道追従完了フラグを設定
            return None
            # if args.dynamics_model == 'dynamics':
            #     return np.zeros(4), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0  # 停止状態の入力と値を返す（2次系）
            # elif args.dynamics_model == 'holonomic_dynamics':
            #     return np.zeros(6), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0  # 停止状態の入力と値を返す（ホロノミック2次系）
            # else:
            #     return np.zeros(6), 0.0, 0.0, 0.0  # 停止状態の入力と値を返す（1次系）
        
        # 現在のフレームに対応する目標位置を設定
        current_target = target_trajectory[frame]
        visualizer.update_target_position(current_target)
        
        # 入力関数を呼び出し
        result = drone_input_func(drone, frame)
        
        # 2次系モデルの場合は、HOCBFの可視化に必要なデータを含む
        if (args.dynamics_model == 'dynamics' or args.dynamics_model == 'holonomic_dynamics') and len(result) >= 7:
            xi, gamma_val, constraint_margin, ax_value, h_val, h_dot_val, h_ddot_val = result
            return xi, gamma_val, constraint_margin, ax_value
        else:
            return result  # (xi, gamma_val, constraint_margin, ax_value)を返す
    
    # アニメーションの作成と表示（repeat=Falseで繰り返しを防止）
    anim = visualizer.animate(num_frames, wrapped_input_func)
    
    # アニメーションオブジェクトを保持するためのグローバル変数
    global global_anim
    global_anim = anim
    
    # アニメーションの表示
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
