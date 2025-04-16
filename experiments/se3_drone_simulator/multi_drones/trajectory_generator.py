"""
目標軌道を生成するモジュール
"""
import numpy as np

def generate_target_trajectory(trajectory_type='circle', center=np.array([0.0, 0.0, 0.0]), 
                              radius=3.0, period=10.0, num_frames=500, drone_idx=0, num_drones=1):
    """
    目標軌道を生成する
    
    Parameters:
    -----------
    trajectory_type : str, optional
        軌道の種類（'circle', 'eight', 'lissajous', 'snake', 'snake_3d', 'step', 'fixed'）（デフォルトは'circle'）
    center : array_like, shape (3,), optional
        軌道の中心位置（デフォルトは[0.0, 0.0, 0.0]）
    radius : float, optional
        軌道の半径（デフォルトは3.0）
    period : float, optional
        1周期の時間（秒）（デフォルトは10.0）
    num_frames : int, optional
        フレーム数（デフォルトは500）
    drone_idx : int, optional
        ドローンのインデックス（デフォルトは0）
    num_drones : int, optional
        ドローンの総数（デフォルトは1）
    
    Returns:
    --------
    trajectory : ndarray, shape (num_frames, 3)
        各フレームでの目標位置
    """
    # 時間パラメータ（0から1の範囲）
    t = np.linspace(0, 1, num_frames)
    
    # 軌道の初期化
    trajectory = np.zeros((num_frames, 3))
    
    # ドローンのオフセット（横並びに配置するため）
    # ドローンのインデックスに基づいて横方向にオフセット
    offset = np.zeros(3)
    if num_drones > 1:
        # X軸方向に均等に配置
        offset[0] = (drone_idx - (num_drones - 1) / 2) * 4.0
    
    if trajectory_type == 'circle':
        # 円軌道
        trajectory[:, 0] = center[0] + radius * np.cos(2 * np.pi * t) + offset[0]
        trajectory[:, 1] = center[1] + radius * np.sin(2 * np.pi * t) + offset[1]
        trajectory[:, 2] = center[2] + offset[2]
        
    elif trajectory_type == 'eight':
        # 8の字軌道
        trajectory[:, 0] = center[0] + radius * np.sin(2 * np.pi * t) + offset[0]
        trajectory[:, 1] = center[1] + radius * np.sin(4 * np.pi * t) / 2 + offset[1]
        trajectory[:, 2] = center[2] + offset[2]
        
    elif trajectory_type == 'lissajous':
        # リサージュ曲線
        trajectory[:, 0] = center[0] + radius * np.sin(2 * np.pi * t) + offset[0]
        trajectory[:, 1] = center[1] + radius * np.sin(4 * np.pi * t) + offset[1]
        trajectory[:, 2] = center[2] + radius * np.sin(6 * np.pi * t) / 3 + offset[2]
        
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
        amplitude = 1.5  # 振幅
        frequency = 2.0  # 周波数
        
        # サイン波の周波数成分
        sin_component = amplitude * np.sin(2 * np.pi * frequency * t)
        
        # 進行方向に垂直な方向にサイン波を適用
        for i in range(3):
            trajectory[:, i] += perpendicular[i] * sin_component
        
        # ドローンごとのオフセットを適用（横並びに配置）
        trajectory[:, 0] += offset[0]

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
        
        # ドローンごとのオフセットを適用（横並びに配置）
        trajectory[:, 0] += offset[0]
        
    elif trajectory_type == 'step':
        # [0, 0, 1]に固定された目標位置
        fixed_target = np.array([0.0, 0.0, 3.0])
        fixed_target += offset  # オフセットを適用
        
        # すべてのフレームで同じ目標位置
        for i in range(num_frames):
            trajectory[i] = fixed_target
    
    elif trajectory_type == 'fixed':
        # 現在の固定目標位置を使用
        if drone_idx == 0:
            fixed_target = np.array([4.0, -4.0, -4.0])  # ドローン1の目標位置
        else:
            fixed_target = np.array([-4.0, 4.0, 4.0])  # ドローン2の目標位置
        
        # すべてのフレームで同じ目標位置
        for i in range(num_frames):
            trajectory[i] = fixed_target
        
    else:
        raise ValueError(f"不明な軌道タイプ: {trajectory_type}")
    
    return trajectory
