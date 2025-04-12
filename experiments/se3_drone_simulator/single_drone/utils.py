"""
ドローンシミュレーションのユーティリティ関数
"""
import numpy as np
import sys
import os

# 親ディレクトリをパスに追加
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.drone import FeaturePoint


def generate_feature_points(num_features=10, center=np.array([0.0, 3.0, 0.0]), radius=1.0, seed=42):
    """
    特徴点を生成する
    
    Parameters:
    -----------
    num_features : int, optional
        特徴点の数（デフォルトは10）
    center : array_like, shape (3,), optional
        特徴点の中心位置（デフォルトは[0.0, 3.0, 0.0]）
    radius : float, optional
        特徴点の分布半径（デフォルトは1.0）
    seed : int, optional
        乱数シード（デフォルトは42）
    
    Returns:
    --------
    feature_points : list of FeaturePoint
        生成された特徴点のリスト
    """
    # 特徴点のリスト
    feature_points = []
    
    # 再現性のために乱数シードを固定
    np.random.seed(seed)
    
    # ランダムな特徴点を生成
    for _ in range(num_features):
        # 球面上にランダムな点を生成
        phi = np.random.uniform(0, 2*np.pi)
        theta = np.random.uniform(0, np.pi)
        r = radius * np.random.uniform(0.5, 1.0)
        
        x = center[0] + r * np.sin(theta) * np.cos(phi)
        y = center[1] + r * np.sin(theta) * np.sin(phi)
        z = center[2] + r * np.cos(theta)
        
        fp = FeaturePoint([x, y, z])
        feature_points.append(fp)
    
    return feature_points


def setup_drone_initial_pose(drone, feature_area_center):
    """
    ドローンの初期位置と姿勢を設定する
    
    Parameters:
    -----------
    drone : Drone
        ドローン
    feature_area_center : array_like, shape (3,)
        特徴点の中心位置
    
    Returns:
    --------
    initial_position : array_like, shape (3,)
        設定された初期位置
    initial_rotation : array_like, shape (3, 3)
        設定された初期回転行列
    """
    # 初期位置は原点から少し離れた位置
    initial_position = np.array([0.0, -5.0, 0.0])
    
    # 特徴点の中心方向を向くように姿勢を設定
    direction_to_features = feature_area_center - initial_position
    direction_to_features = direction_to_features / np.linalg.norm(direction_to_features)
    
    # 方向ベクトルから回転行列を計算
    # まず、z軸方向のベクトルを特徴点方向に向ける
    z_axis = direction_to_features
    
    # x軸方向のベクトルを計算（z軸に垂直）
    if np.abs(z_axis[0]) < np.abs(z_axis[1]):
        x_axis = np.array([0, -z_axis[2], z_axis[1]])
    else:
        x_axis = np.array([-z_axis[2], 0, z_axis[0]])
    x_axis = x_axis / np.linalg.norm(x_axis)
    
    # y軸方向のベクトルを計算（z軸とx軸に垂直）
    y_axis = np.cross(z_axis, x_axis)
    
    # 回転行列を構築
    initial_rotation = np.column_stack((x_axis, y_axis, z_axis))
    
    # ドローンの初期位置と姿勢を設定
    drone.T.p = initial_position
    drone.T.R = initial_rotation
    
    return initial_position, initial_rotation
