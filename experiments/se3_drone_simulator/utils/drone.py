"""
ドローンのモデルを実装するモジュール
"""
import numpy as np
from .se3 import SE3, integrate_se3


class Drone:
    """
    SE(3)上のドローンモデル
    """
    
    def __init__(self, T=None, camera_direction=None, fov_angle=np.pi/3, R=None, p=None):
        """
        ドローンを初期化
        
        Parameters:
        -----------
        T : SE3, optional
            初期位置と姿勢（デフォルトは原点、単位姿勢）
        camera_direction : array_like, shape (3,), optional
            カメラの向きを表す単位ベクトル（デフォルトは[0, 0, 1]）
        fov_angle : float, optional
            カメラの視野角（デフォルトはπ/3ラジアン）
        """
        if T is None:
            self.T = SE3(R=np.eye(3), p=np.zeros(3))
        else:
            self.T = T
            
        if camera_direction is None:
            self.camera_direction = np.array([0, 0, 1])  # z軸方向
        else:
            self.camera_direction = np.array(camera_direction)
            self.camera_direction = self.camera_direction / np.linalg.norm(self.camera_direction)
            
        self.fov_angle = fov_angle
        self.cos_fov_angle = np.cos(fov_angle)
    
    def update(self, xi, dt, frame='body'):
        """
        ドローンの状態を更新
        
        Parameters:
        -----------
        xi : array_like, shape (6,)
            速度入力 [omega, v]
            omega: 角速度ベクトル
            v: 速度ベクトル
        dt : float
            時間ステップ
        frame : str, optional
            'body'または'world'（デフォルトは'body'）
        """
        self.T = integrate_se3(self.T, xi, dt, frame)
    
    def is_point_visible(self, point):
        """
        点がドローンのカメラから見えるかどうかを判定
        
        Parameters:
        -----------
        point : array_like, shape (3,)
            3次元空間内の点
            
        Returns:
        --------
        visible : bool
            点が見えるかどうか
        """
        # 点をドローンの座標系に変換
        p_local = self.T.inverse() * SE3(p=point)
        p_local = p_local.p
        
        # 点の方向ベクトル（正規化）
        direction = p_local / np.linalg.norm(p_local)
        
        # カメラの向きとのドット積
        cos_angle = np.dot(direction, self.camera_direction)
        
        # 視野角内にあるかどうか
        return cos_angle > self.cos_fov_angle
    
    def get_observation_probability(self, point):
        """
        点の観測確率を計算
        
        Parameters:
        -----------
        point : array_like, shape (3,)
            3次元空間内の点
            
        Returns:
        --------
        probability : float
            観測確率（0から1の値）
            視野角外の場合は0
        """
        # 点をドローンの座標系に変換
        p_local = self.T.inverse() * SE3(p=point)
        p_local = p_local.p
        
        # 点の方向ベクトル（正規化）
        direction = p_local / np.linalg.norm(p_local)
        
        # カメラの向きとのドット積
        cos_angle = np.dot(direction, self.camera_direction)
        
        # 視野角外なら確率0
        if cos_angle <= self.cos_fov_angle:
            return 0.0
        
        # 視野角内なら確率を計算
        # ccbf.mdの式に基づく確率計算
        # P_i^l = (β_l^T(p_i) R_i e_c - cos(Ψ_F)) / (1 - cos(Ψ_F))
        # ここでは簡略化して、cos_angleに基づく確率を返す
        return (cos_angle - self.cos_fov_angle) / (1 - self.cos_fov_angle)
    
    def get_beta_vector(self, point):
        """
        点に対するβベクトルを計算
        
        Parameters:
        -----------
        point : array_like, shape (3,)
            3次元空間内の点
            
        Returns:
        --------
        beta : ndarray, shape (3,)
            βベクトル（点の方向を表す単位ベクトル）
        """
        # 点とドローンの位置の差
        diff = point - self.T.p
        
        # 正規化して方向ベクトルを得る
        return diff / np.linalg.norm(diff)
    
    def calculate_cofov_probability(self, other_drone, point):
        """
        他のドローンとの共有視野（CoFOV）における点の観測確率を計算
        
        Parameters:
        -----------
        other_drone : Drone
            他のドローン
        point : array_like, shape (3,)
            3次元空間内の点
            
        Returns:
        --------
        probability : float
            共有視野における観測確率（0から1の値）
        """
        # 各ドローンの観測確率
        p1 = self.get_observation_probability(point)
        p2 = other_drone.get_observation_probability(point)
        
        # 両方のドローンから見える場合のみ確率を返す
        if p1 > 0 and p2 > 0:
            return p1 * p2
        else:
            return 0.0
    
    def __str__(self):
        """文字列表現"""
        return f"Drone(T={self.T}, camera_direction={self.camera_direction}, fov_angle={self.fov_angle})"
    
    def __repr__(self):
        """オブジェクトの表現"""
        return self.__str__()


class FeaturePoint:
    """
    環境内の特徴点
    """
    
    def __init__(self, position):
        """
        特徴点を初期化
        
        Parameters:
        -----------
        position : array_like, shape (3,)
            3次元空間内の位置
        """
        self.position = np.array(position)
    
    def __str__(self):
        """文字列表現"""
        return f"FeaturePoint(position={self.position})"
    
    def __repr__(self):
        """オブジェクトの表現"""
        return self.__str__()
