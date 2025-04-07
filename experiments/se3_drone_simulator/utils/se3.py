"""
SE(3)の表現と操作を行うモジュール
"""
import numpy as np
from scipy.spatial.transform import Rotation


def skew(v):
    """
    ベクトルvのスキュー対称行列を返す
    
    Parameters:
    -----------
    v : array_like, shape (3,)
        3次元ベクトル
        
    Returns:
    --------
    S : ndarray, shape (3, 3)
        スキュー対称行列
    """
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])


def unskew(S):
    """
    スキュー対称行列からベクトルを抽出
    
    Parameters:
    -----------
    S : array_like, shape (3, 3)
        スキュー対称行列
        
    Returns:
    --------
    v : ndarray, shape (3,)
        3次元ベクトル
    """
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


class SE3:
    """
    SE(3)（3次元特殊ユークリッド群）の表現と操作を行うクラス
    """
    
    def __init__(self, R=None, p=None):
        """
        SE(3)の要素を初期化
        
        Parameters:
        -----------
        R : array_like, shape (3, 3), optional
            回転行列（デフォルトは単位行列）
        p : array_like, shape (3,), optional
            位置ベクトル（デフォルトは零ベクトル）
        """
        if R is None:
            self.R = np.eye(3)
        else:
            self.R = np.array(R)
            
        if p is None:
            self.p = np.zeros(3)
        else:
            self.p = np.array(p)
    
    @classmethod
    def from_matrix(cls, T):
        """
        4x4の同次変換行列からSE(3)の要素を生成
        
        Parameters:
        -----------
        T : array_like, shape (4, 4)
            同次変換行列
            
        Returns:
        --------
        se3 : SE3
            SE(3)の要素
        """
        R = T[:3, :3]
        p = T[:3, 3]
        return cls(R, p)
    
    def to_matrix(self):
        """
        SE(3)の要素を4x4の同次変換行列に変換
        
        Returns:
        --------
        T : ndarray, shape (4, 4)
            同次変換行列
        """
        T = np.eye(4)
        T[:3, :3] = self.R
        T[:3, 3] = self.p
        return T
    
    def adjoint(self):
        """
        SE(3)の随伴表現を計算
        
        Returns:
        --------
        Ad : ndarray, shape (6, 6)
            随伴表現行列
        """
        Ad = np.zeros((6, 6))
        Ad[:3, :3] = self.R
        Ad[3:, 3:] = self.R
        Ad[3:, :3] = skew(self.p) @ self.R
        return Ad
    
    def __mul__(self, other):
        """
        SE(3)の要素同士の積
        
        Parameters:
        -----------
        other : SE3
            右側の要素
            
        Returns:
        --------
        result : SE3
            積の結果
        """
        if isinstance(other, SE3):
            R = self.R @ other.R
            p = self.p + self.R @ other.p
            return SE3(R, p)
        else:
            raise TypeError("Multiplication is only defined between SE3 objects")
    
    def inverse(self):
        """
        SE(3)の要素の逆元
        
        Returns:
        --------
        inv : SE3
            逆元
        """
        R_inv = self.R.T
        p_inv = -R_inv @ self.p
        return SE3(R_inv, p_inv)
    
    def __str__(self):
        """文字列表現"""
        return f"SE3(R=\n{self.R}, \np={self.p})"
    
    def __repr__(self):
        """オブジェクトの表現"""
        return self.__str__()


def body_velocity_to_se3_derivative(T, xi_b):
    """
    ボディ座標系における速度入力からSE(3)の時間微分を計算
    
    Parameters:
    -----------
    T : SE3 or ndarray, shape (4, 4)
        現在の姿勢と位置
    xi_b : array_like, shape (6,)
        ボディ座標系における速度入力 [omega, v]
        omega: 角速度ベクトル
        v: 速度ベクトル
        
    Returns:
    --------
    T_dot : ndarray, shape (4, 4)
        SE(3)の時間微分
    """
    if isinstance(T, SE3):
        T_mat = T.to_matrix()
    else:
        T_mat = np.array(T)
    
    omega = xi_b[:3]
    v = xi_b[3:]
    
    xi_b_wedge = np.zeros((4, 4))
    xi_b_wedge[:3, :3] = skew(omega)
    xi_b_wedge[:3, 3] = v
    
    return T_mat @ xi_b_wedge


def world_velocity_to_se3_derivative(T, xi_w):
    """
    世界座標系における速度入力からSE(3)の時間微分を計算
    
    Parameters:
    -----------
    T : SE3 or ndarray, shape (4, 4)
        現在の姿勢と位置
    xi_w : array_like, shape (6,)
        世界座標系における速度入力 [omega, v]
        omega: 角速度ベクトル
        v: 速度ベクトル
        
    Returns:
    --------
    T_dot : ndarray, shape (4, 4)
        SE(3)の時間微分
    """
    if isinstance(T, SE3):
        T_mat = T.to_matrix()
    else:
        T_mat = np.array(T)
    
    omega = xi_w[:3]
    v = xi_w[3:]
    
    xi_w_wedge = np.zeros((4, 4))
    xi_w_wedge[:3, :3] = skew(omega)
    xi_w_wedge[:3, 3] = v
    
    return xi_w_wedge @ T_mat


def integrate_se3(T, xi_b, dt, frame='body'):
    """
    SE(3)の要素を速度入力に基づいて積分
    
    Parameters:
    -----------
    T : SE3
        現在の姿勢と位置
    xi_b : array_like, shape (6,)
        速度入力 [omega, v]
        omega: 角速度ベクトル
        v: 速度ベクトル
    dt : float
        時間ステップ
    frame : str, optional
        'body'または'world'（デフォルトは'body'）
        
    Returns:
    --------
    T_next : SE3
        積分後の姿勢と位置
    """
    if frame == 'body':
        # ボディ座標系での積分
        omega = xi_b[:3]
        v = xi_b[3:]
        
        # 回転の積分
        angle = np.linalg.norm(omega) * dt
        if angle < 1e-10:
            R_delta = np.eye(3)
        else:
            axis = omega / np.linalg.norm(omega)
            R_delta = Rotation.from_rotvec(axis * angle).as_matrix()
        
        # 並進の積分（単純なオイラー法）
        p_delta = T.R @ (v * dt)
        
        # 更新
        R_next = T.R @ R_delta
        p_next = T.p + p_delta
        
    elif frame == 'world':
        # 世界座標系での積分
        omega = xi_b[:3]
        v = xi_b[3:]
        
        # 回転の積分
        angle = np.linalg.norm(omega) * dt
        if angle < 1e-10:
            R_delta = np.eye(3)
        else:
            axis = omega / np.linalg.norm(omega)
            R_delta = Rotation.from_rotvec(axis * angle).as_matrix()
        
        # 並進の積分（単純なオイラー法）
        p_delta = v * dt
        
        # 更新
        R_next = R_delta @ T.R
        p_next = T.p + p_delta
    
    else:
        raise ValueError("frame must be 'body' or 'world'")
    
    return SE3(R_next, p_next)
