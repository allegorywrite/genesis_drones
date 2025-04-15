"""
ドローンのモデルを実装するモジュール
"""
import numpy as np
from .se3 import SE3, integrate_se3, skew
from .constants import get_gravity
from scipy.spatial.transform import Rotation


class Drone:
    """
    SE(3)上のドローンモデル
    """
    
    def __init__(self, T=None, camera_direction=None, fov_angle=np.pi/3):
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


class DynamicDrone(Drone):
    """
    SE(3)上の2次系ドローンモデル
    """
    
    def __init__(self, T=None, camera_direction=None, fov_angle=np.pi/3, dynamics_model='dynamics'):
        """
        2次系ドローンを初期化
        
        Parameters:
        -----------
        T : SE3, optional
            初期位置と姿勢（デフォルトは原点、単位姿勢）
        camera_direction : array_like, shape (3,), optional
            カメラの向きを表す単位ベクトル（デフォルトは[0, 0, 1]）
        fov_angle : float, optional
            カメラの視野角（デフォルトはπ/3ラジアン）
        dynamics_model : str, optional
            動力学モデル: 'dynamics'（非ホロノミック系）または'holonomic_dynamics'（ホロノミック系）
            （デフォルトは'dynamics'）
        """
        super().__init__(T, camera_direction, fov_angle)
        # 速度と角速度の状態を追加
        self.v = np.zeros(3)  # 速度（ボディフレーム）
        self.omega = np.zeros(3)  # 角速度
        # 質量と慣性モーメント
        self.M = np.eye(3)  # 質量行列
        self.J = np.eye(3)  # 慣性テンソル
        # 動力学モデル
        self.dynamics_model = dynamics_model
        self.g = get_gravity()
    
    def update(self, u, dt, frame='body'):
        """
        加速度入力に基づいてドローンの状態を更新
        
        Parameters:
        -----------
        u : array_like
            加速度入力
            dynamics_model='dynamics'の場合: shape (4,), [f, tau]
                f: 推力（スカラー）
                tau: トルク（3次元ベクトル）
            dynamics_model='holonomic_dynamics'の場合: shape (6,), [f, tau]
                f: 推力（3次元ベクトル）
                tau: トルク（3次元ベクトル）
        dt : float
            時間ステップ
        frame : str, optional
            'body'または'world'（デフォルトは'body'）
        """
        # 動力学モデルに応じて入力を分解
        if self.dynamics_model == 'holonomic_dynamics':
            # ホロノミック系（3次元ベクトル推力）
            f = u[0:3]  # 3次元ベクトル推力
            tau = u[3:6]  # トルク
        else:
            # 非ホロノミック系（スカラー推力）
            f_scalar = u[0]  # スカラー推力
            tau = u[1:4]  # トルク
            f = np.array([0, 0, f_scalar])  # z軸方向の推力
        
        # SE(3)_dynamics.mdの式に基づく更新
        # 回転の更新
        self.omega = self.omega + dt * np.linalg.inv(self.J) @ tau
        F = np.eye(3) + dt * skew(self.omega)  # 近似的な指数写像
        self.T.R = self.T.R @ F
        # RをR=>quat=>SO(3)に正規化
        quat = Rotation.from_matrix(self.T.R).as_quat()
        self.T.R = Rotation.from_quat(quat).as_matrix()
        
        # 並進の更新
        self.v = self.v + dt * (np.cross(self.v, self.omega) - self.T.R.T @ self.g + f / self.M[0, 0])
        self.T.p = self.T.p + dt * self.T.R @ self.v

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
