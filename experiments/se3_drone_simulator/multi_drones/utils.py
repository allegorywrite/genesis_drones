"""
ユーティリティ関数を提供するモジュール
"""
import numpy as np
import random
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
