"""
ドローンと特徴点の初期設定関連の関数を提供するモジュール
"""
import numpy as np
import random
import sys
import os
from scipy.spatial.transform import Rotation as R

# 親ディレクトリをパスに追加（絶対パスを使用）
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from utils.se3 import SE3
from utils.drone import Drone, FeaturePoint, DynamicDrone
from multi_drones.utils import random_rotation_matrix

def initialize_drones(simulator, fov_angle_rad, dynamics_model='kinematics'):
    """
    ドローンを初期化
    
    Parameters:
    -----------
    simulator : Simulator
        シミュレータ
    fov_angle_rad : float
        視野角（ラジアン）
    dynamics_model : str, optional
        動力学モデル（'kinematics'または'dynamics'）（デフォルトは'kinematics'）
        
    Returns:
    --------
    drone1 : Drone or DynamicDrone
        1つ目のドローン
    """
    # 動力学モデルに応じてドローンを初期化
    if dynamics_model == 'dynamics':
        # ドローン1: 原点、単位姿勢、指定された視野角
        drone1 = DynamicDrone(fov_angle=fov_angle_rad)
        print("2次系モデル（動力学モデル）を使用します")
    else:
        # ドローン1: 原点、単位姿勢、指定された視野角
        drone1 = Drone(fov_angle=fov_angle_rad)
        print("1次系モデル（運動学モデル）を使用します")
    
    simulator.add_drone(drone1)
    
    return drone1

def setup_drone_initial_pose(drone, trajectory_type, position):
    """
    ドローンの初期位置と姿勢を設定
    
    Parameters:
    -----------
    drone : Drone or DynamicDrone
        ドローン
    trajectory_type : str
        軌道タイプ
    position : array_like, shape (3,)
        初期位置
    """
    # ドローンの初期位置を設定
    drone.T.p = position.copy()
    
    # snake軌道の場合、Y軸方向を向くように姿勢を設定
    if trajectory_type == 'snake' or trajectory_type == 'snake_3d':
        # Y軸方向を向く回転行列
        R_y = np.array([
            [0, 0, 1],  # X軸 -> Z軸
            [1, 0, 0],  # Y軸 -> X軸
            [0, 1, 0]   # Z軸 -> Y軸
        ])
        drone.T.R = R_y

def add_feature_points(simulator, trajectory_type):
    """
    特徴点を追加
    
    Parameters:
    -----------
    simulator : Simulator
        シミュレータ
    trajectory_type : str
        軌道タイプ
    """
    # snake軌道の場合、Y軸方向に進むので、その周りに特徴点を配置
    if trajectory_type == 'snake' or trajectory_type == 'snake_3d':
        # 軌道の始点と終点
        start_point = np.array([0.0, -5.0, 0.0])
        end_point = np.array([0.0, 3.0, 0.0])
        
        # ドローンの初期位置を取得（2機分）
        if len(simulator.drones) >= 2:
            drone1_start = simulator.drones[0].T.p
            drone2_start = simulator.drones[1].T.p
            
            # 初期位置の中間点を計算
            mid_point = (drone1_start + drone2_start) / 2
        
        # 原点付近にNxNxNの格子状に配置
        n_points = 5
        radius = 3.0
        for x in np.linspace(-radius, radius, n_points):
            for y in np.linspace(-radius/3, radius/3, n_points):
                for z in np.linspace(-radius, radius, n_points):
                    fp = FeaturePoint([x, y, z])
                    simulator.add_feature_point(fp)
    else:
        # その他の軌道タイプの場合は格子状に配置
        n_points = 5
        for x in np.linspace(-3, 3, n_points):
            for y in np.linspace(-3, 3, n_points):
                for z in np.linspace(-3, 3, n_points):
                    if abs(x) > 1 or abs(y) > 1 or abs(z) > 1:  # 中心付近は除外
                        fp = FeaturePoint([x, y, z])
                        simulator.add_feature_point(fp)

def find_safe_pose_for_both_drones(simulator, p_start1, p_start2, fov_angle_rad, min_barrier=0.0, max_attempts=1000, trajectory_type='fixed', dynamics_model='kinematics'):
    """
    両方のドローンの安全な姿勢を同時に探索
    
    Parameters:
    -----------
    simulator : Simulator
        シミュレータ
    p_start1 : array_like, shape (3,)
        1つ目のドローンの初期位置
    p_start2 : array_like, shape (3,)
        2つ目のドローンの初期位置
    fov_angle_rad : float
        視野角（ラジアン）
    min_barrier : float, optional
        最小安全集合値（デフォルトは0.0）
    max_attempts : int, optional
        最大試行回数（デフォルトは1000）
    trajectory_type : str, optional
        軌道タイプ（デフォルトは'fixed'）
    dynamics_model : str, optional
        動力学モデル（'kinematics'または'dynamics'）（デフォルトは'kinematics'）
        
    Returns:
    --------
    drone1 : Drone or DynamicDrone
        1つ目のドローン（安全な姿勢に設定済み）
    drone2 : Drone or DynamicDrone
        2つ目のドローン（安全な姿勢に設定済み）
    success : bool
        安全な姿勢が見つかったかどうか
    """
    # 既存のドローンを削除（もし存在すれば）
    if len(simulator.drones) > 0:
        simulator.drones = []
    
    # 一時的にドローン1と2を追加
    if dynamics_model == 'dynamics':
        temp_drone1 = DynamicDrone(fov_angle=fov_angle_rad)
        temp_drone2 = DynamicDrone(fov_angle=fov_angle_rad)
    else:
        temp_drone1 = Drone(fov_angle=fov_angle_rad)
        temp_drone2 = Drone(fov_angle=fov_angle_rad)
    
    # 位置を軌道の始点に設定
    temp_drone1.T.p = p_start1.copy()
    temp_drone2.T.p = p_start2.copy()
    
    simulator.add_drone(temp_drone1)
    simulator.add_drone(temp_drone2)
    
    # 安全集合の値が最大になる姿勢を探索
    success = False
    best_safety_value = float('-inf')
    best_R1 = None
    best_R2 = None
    
    # 初期姿勢として、Y軸方向を向く回転行列を試す
    if trajectory_type == 'snake' or trajectory_type == 'snake_3d':
        # Y軸方向を向く回転行列
        R_y = np.array([
            [0, 0, 1],  # X軸 -> Z軸
            [1, 0, 0],  # Y軸 -> X軸
            [0, 1, 0]   # Z軸 -> Y軸
        ])
        temp_drone1.T.R = R_y.copy()
        temp_drone2.T.R = R_y.copy()
        
        # 安全集合の値を計算
        safety_value = simulator.calculate_safety_value(0, 1)
        
        if safety_value > best_safety_value:
            best_safety_value = safety_value
            best_R1 = R_y.copy()
            best_R2 = R_y.copy()
            
        if safety_value >= min_barrier:
            success = True
            print(f"安全な姿勢を設定しました（Y軸方向）")
            print(f"安全集合の値: {safety_value}")
    
    # より良い姿勢を探索するためのランダムサンプリング
    print(f"より良い姿勢を探索中... 最大試行回数: {max_attempts}")
    
    # 残りの試行回数でランダムサンプリング
    remaining_attempts = max_attempts
    while remaining_attempts > 0:
        remaining_attempts -= 1
        
        # 両方のドローンにランダムな回転行列を生成
        R_random1 = random_rotation_matrix()
        R_random2 = random_rotation_matrix()
        temp_drone1.T.R = R_random1
        temp_drone2.T.R = R_random2
        
        # 安全集合の値を計算
        safety_value = simulator.calculate_safety_value(0, 1)
        
        # より良い値が見つかった場合は更新
        if safety_value > best_safety_value:
            best_safety_value = safety_value
            best_R1 = R_random1.copy()
            best_R2 = R_random2.copy()
            print(f"より良い姿勢を見つけました: 安全集合の値 = {safety_value:.4f}")
        
        # 十分に良い値が見つかった場合は早期終了
        if safety_value >= min_barrier and safety_value > 0:
            success = True
            print(f"安全な姿勢を見つけました: 安全集合の値 = {safety_value:.4f}")
            break
        
        # 進捗表示（100回ごと）
        if remaining_attempts % 100 == 0:
            print(f"残り試行回数: {remaining_attempts}, 現在の最良値: {best_safety_value:.4f}")
    
    # 最良の姿勢を使用
    print(f"最終的な安全集合の値: {best_safety_value:.4f}")
    
    if not success:
        print(f"警告: {max_attempts}回の試行で安全な姿勢を見つけられませんでした")
    
    # 一時的なドローンを削除
    simulator.drones = []
    
    # 正式なドローン1と2を追加（位置は軌道の始点、姿勢は安全な値）
    if dynamics_model == 'dynamics':
        drone1 = DynamicDrone(fov_angle=fov_angle_rad, T=SE3(R=best_R1, p=p_start1))
        drone2 = DynamicDrone(fov_angle=fov_angle_rad, T=SE3(R=best_R2, p=p_start2))
    else:
        drone1 = Drone(fov_angle=fov_angle_rad, T=SE3(R=best_R1, p=p_start1))
        drone2 = Drone(fov_angle=fov_angle_rad, T=SE3(R=best_R2, p=p_start2))
    
    simulator.add_drone(drone1)
    simulator.add_drone(drone2)
    
    return drone1, drone2, success


def find_safe_pose_for_second_drone(simulator, p_start, fov_angle_rad, min_barrier=0.0, max_attempts=1000, trajectory_type='fixed', dynamics_model='kinematics'):
    """
    2つ目のドローンの安全な姿勢を探索
    
    Parameters:
    -----------
    simulator : Simulator
        シミュレータ
    p_start : array_like, shape (3,)
        2つ目のドローンの初期位置
    fov_angle_rad : float
        視野角（ラジアン）
    min_barrier : float, optional
        最小安全集合値（デフォルトは0.0）
    max_attempts : int, optional
        最大試行回数（デフォルトは1000）
    trajectory_type : str, optional
        軌道タイプ（デフォルトは'fixed'）
    dynamics_model : str, optional
        動力学モデル（'kinematics'または'dynamics'）（デフォルトは'kinematics'）
        
    Returns:
    --------
    drone2 : Drone or DynamicDrone
        2つ目のドローン
    success : bool
        安全な姿勢が見つかったかどうか
    """
    # 一時的にドローン2を追加（位置は固定、姿勢はランダム）
    if dynamics_model == 'dynamics':
        temp_drone2 = DynamicDrone(fov_angle=fov_angle_rad)
    else:
        temp_drone2 = Drone(fov_angle=fov_angle_rad)
    
    # 位置を軌道の始点に設定
    temp_drone2.T.p = p_start.copy()
    simulator.add_drone(temp_drone2)
    
    # 安全集合の値が最大になる姿勢を探索
    success = False
    best_safety_value = float('-inf')
    best_R = None
    
    # 初期姿勢として、Y軸方向を向く回転行列を試す
    if trajectory_type == 'snake' or trajectory_type == 'snake_3d':
        # Y軸方向を向く回転行列（ドローン1と同じ方向）
        R_y = np.array([
            [0, 0, 1],  # X軸 -> Z軸
            [1, 0, 0],  # Y軸 -> X軸
            [0, 1, 0]   # Z軸 -> Y軸
        ])
        temp_drone2.T.R = R_y
        
        # 安全集合の値を計算
        safety_value = simulator.calculate_safety_value(0, 1)
        
        if safety_value > best_safety_value:
            best_safety_value = safety_value
            best_R = R_y.copy()
            
        if safety_value >= min_barrier:
            success = True
            print(f"安全な姿勢を設定しました（Y軸方向）")
            print(f"安全集合の値: {safety_value}")
    
    # より良い姿勢を探索するためのランダムサンプリング
    print(f"より良い姿勢を探索中... 最大試行回数: {max_attempts}")
    
    # 探索範囲を細かく設定するためのパラメータ
    # オイラー角の範囲を細かく分割
    n_samples = 20  # 各軸の分割数
    
    # 系統的なサンプリング（グリッドサーチ）
    for roll in np.linspace(-np.pi, np.pi, n_samples):
        for pitch in np.linspace(-np.pi/2, np.pi/2, n_samples):
            for yaw in np.linspace(-np.pi, np.pi, n_samples):
                # 試行回数をカウント
                if max_attempts <= 0:
                    break
                max_attempts -= 1
                
                # オイラー角から回転行列を生成
                rot = R.from_euler('xyz', [roll, pitch, yaw])
                R_matrix = rot.as_matrix()
                temp_drone2.T.R = R_matrix
                
                # 安全集合の値を計算
                safety_value = simulator.calculate_safety_value(0, 1)
                
                # より良い値が見つかった場合は更新
                if safety_value > best_safety_value:
                    best_safety_value = safety_value
                    best_R = R_matrix.copy()
                    print(f"より良い姿勢を見つけました: 安全集合の値 = {safety_value:.4f}")
                
                # 十分に良い値が見つかった場合は早期終了
                if safety_value >= min_barrier:
                    success = True
                    if max_attempts % 100 == 0:  # 進捗表示
                        print(f"安全な姿勢を見つけました: 安全集合の値 = {safety_value:.4f}")
    
    # 残りの試行回数でランダムサンプリング
    while max_attempts > 0:
        max_attempts -= 1
        
        # ランダムな回転行列を生成
        R_random = random_rotation_matrix()
        temp_drone2.T.R = R_random
        
        # 安全集合の値を計算
        safety_value = simulator.calculate_safety_value(0, 1)
        
        # より良い値が見つかった場合は更新
        if safety_value > best_safety_value:
            best_safety_value = safety_value
            best_R = R_random.copy()
            print(f"より良い姿勢を見つけました: 安全集合の値 = {safety_value:.4f}")
        
        # 十分に良い値が見つかった場合は早期終了
        if safety_value >= min_barrier and safety_value > 0:
            success = True
            print(f"安全な姿勢を見つけました: 安全集合の値 = {safety_value:.4f}")
            break
        
        # 進捗表示（100回ごと）
        if max_attempts % 100 == 0:
            print(f"残り試行回数: {max_attempts}, 現在の最良値: {best_safety_value:.4f}")
    
    # 最良の姿勢を使用
    R_best = best_R
    print(f"最終的な安全集合の値: {best_safety_value:.4f}")
    
    if not success:
        print(f"警告: {max_attempts}回の試行で安全な姿勢を見つけられませんでした")
    
    # 一時的なドローン2を削除
    simulator.drones.pop()
    
    # 正式なドローン2を追加（位置は軌道の始点、姿勢は安全な値）
    if dynamics_model == 'dynamics':
        drone2 = DynamicDrone(fov_angle=fov_angle_rad, T=SE3(R=R_best, p=p_start))
    else:
        drone2 = Drone(fov_angle=fov_angle_rad, T=SE3(R=R_best, p=p_start))
    
    simulator.add_drone(drone2)
    
    return drone2, success
