"""
シミュレーションエンジンを実装するモジュール
"""
import numpy as np
from utils.se3 import SE3
from utils.drone import Drone, FeaturePoint


class Simulator:
    """
    SE(3)ドローンシミュレータ
    """
    
    def __init__(self, dt=0.01):
        """
        シミュレータを初期化
        
        Parameters:
        -----------
        dt : float, optional
            シミュレーションの時間ステップ（デフォルトは0.01秒）
        """
        self.dt = dt
        self.drones = []
        self.feature_points = []
        self.time = 0.0
    
    def add_drone(self, drone):
        """
        ドローンをシミュレータに追加
        
        Parameters:
        -----------
        drone : Drone
            追加するドローン
        """
        self.drones.append(drone)
    
    def add_feature_point(self, feature_point):
        """
        特徴点をシミュレータに追加
        
        Parameters:
        -----------
        feature_point : FeaturePoint
            追加する特徴点
        """
        self.feature_points.append(feature_point)
    
    def step(self, drone_inputs):
        """
        シミュレーションを1ステップ進める
        
        Parameters:
        -----------
        drone_inputs : list of array_like
            各ドローンの速度入力のリスト
            各要素は[omega, v]の形式（shape (6,)）
        """
        if len(drone_inputs) != len(self.drones):
            raise ValueError("入力の数がドローンの数と一致しません")
        
        # 各ドローンを更新
        for i, (drone, xi) in enumerate(zip(self.drones, drone_inputs)):
            drone.update(xi, self.dt)
        
        # 時間を進める
        self.time += self.dt
    
    def get_visible_feature_points(self, drone_idx):
        """
        指定したドローンから見える特徴点のインデックスを取得
        
        Parameters:
        -----------
        drone_idx : int
            ドローンのインデックス
            
        Returns:
        --------
        visible_indices : list of int
            見える特徴点のインデックスのリスト
        """
        if drone_idx >= len(self.drones):
            raise ValueError("無効なドローンインデックス")
        
        drone = self.drones[drone_idx]
        visible_indices = []
        
        for i, fp in enumerate(self.feature_points):
            if drone.is_point_visible(fp.position):
                visible_indices.append(i)
        
        return visible_indices
    
    def get_cofov_feature_points(self, drone_idx1, drone_idx2):
        """
        2つのドローンの共有視野（CoFOV）にある特徴点のインデックスを取得
        
        Parameters:
        -----------
        drone_idx1 : int
            1つ目のドローンのインデックス
        drone_idx2 : int
            2つ目のドローンのインデックス
            
        Returns:
        --------
        cofov_indices : list of int
            共有視野にある特徴点のインデックスのリスト
        """
        if drone_idx1 >= len(self.drones) or drone_idx2 >= len(self.drones):
            raise ValueError("無効なドローンインデックス")
        
        drone1 = self.drones[drone_idx1]
        drone2 = self.drones[drone_idx2]
        cofov_indices = []
        
        for i, fp in enumerate(self.feature_points):
            if (drone1.is_point_visible(fp.position) and 
                drone2.is_point_visible(fp.position)):
                cofov_indices.append(i)
        
        return cofov_indices
    
    def calculate_cofov_probability(self, drone_idx1, drone_idx2, feature_idx):
        """
        2つのドローンの共有視野における特徴点の観測確率を計算
        
        Parameters:
        -----------
        drone_idx1 : int
            1つ目のドローンのインデックス
        drone_idx2 : int
            2つ目のドローンのインデックス
        feature_idx : int
            特徴点のインデックス
            
        Returns:
        --------
        probability : float
            共有視野における観測確率（0から1の値）
        """
        if (drone_idx1 >= len(self.drones) or 
            drone_idx2 >= len(self.drones) or 
            feature_idx >= len(self.feature_points)):
            raise ValueError("無効なインデックス")
        
        drone1 = self.drones[drone_idx1]
        drone2 = self.drones[drone_idx2]
        feature_point = self.feature_points[feature_idx]
        
        return drone1.calculate_cofov_probability(drone2, feature_point.position)
    
    def calculate_safety_value(self, drone_idx1, drone_idx2, q=0.5):
        """
        2つのドローン間の安全値（B_{ij}）を計算
        
        Parameters:
        -----------
        drone_idx1 : int
            1つ目のドローンのインデックス
        drone_idx2 : int
            2つ目のドローンのインデックス
        q : float, optional
            確率の閾値（デフォルトは0.5）
            
        Returns:
        --------
        safety_value : float
            安全値
        """
        if drone_idx1 >= len(self.drones) or drone_idx2 >= len(self.drones):
            raise ValueError("無効なドローンインデックス")
        
        drone1 = self.drones[drone_idx1]
        drone2 = self.drones[drone_idx2]
        
        # 各特徴点についての確率を計算
        probs = []
        for fp in self.feature_points:
            prob = drone1.calculate_cofov_probability(drone2, fp.position)
            probs.append(prob)
        
        # 安全値の計算
        # B_{ij} = 1 - q - \prod_{l \in \mathcal{L}} (1 - \phi_{ij}^l)
        if not probs:
            return -q  # 特徴点がない場合
        
        prod_term = 1.0
        for prob in probs:
            prod_term *= (1.0 - prob)
        
        return 1.0 - q - prod_term
