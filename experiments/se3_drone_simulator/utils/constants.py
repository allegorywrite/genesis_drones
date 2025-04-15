"""
シミュレーションで使用する定数を定義するモジュール
"""
import numpy as np

# 重力加速度（デフォルト値）
GRAVITY = np.array([0, 0, 9.81])

# 重力加速度を設定する関数
def set_gravity(g):
    """
    重力加速度を設定する
    
    Parameters:
    -----------
    g : array_like, shape (3,)
        重力加速度ベクトル
    """
    global GRAVITY
    GRAVITY = np.array(g)

# 重力加速度を取得する関数
def get_gravity():
    """
    重力加速度を取得する
    
    Returns:
    --------
    g : ndarray, shape (3,)
        重力加速度ベクトル
    """
    return GRAVITY
