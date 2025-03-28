#!/usr/bin/env python3

import torch
import numpy as np
from typing import Any, List, Tuple, Optional, Dict
from transforms3d.quaternions import rotate_vector, qconjugate, mat2quat, qmult
from transforms3d.utils import normalized_vector

from genesis_drones.controllers.base_controller import BaseController

# URDFから取得した定数
KF = 3.16e-10  # 推力係数 [N/RPM^2]
KM = 7.94e-12  # トルク係数 [Nm/RPM^2]
ARM = 0.0397   # アーム長 [m]
MASS = 0.027   # 質量 [kg]

# RPM関連の定数
BASE_RPM = 14468.429183500699
MIN_RPM = 0.9 * BASE_RPM
MAX_RPM = 1.5 * BASE_RPM
GRAVITY = 9.81

# ホバリング用のRPM計算
# F = m*g = 4*kf*rpm^2 => rpm = sqrt(m*g/(4*kf))
HOVER_RPM = np.sqrt(MASS * GRAVITY / (4 * KF))

class CTBRController(BaseController):
    """
    Control Toolbox Based Regulator (CTBR) コントローラ
    
    四元数を使用した姿勢制御を行うコントローラ。
    """
    
    def __init__(self, drone: Any, dt: float, base_rpm: float = BASE_RPM):
        """
        CTBRコントローラの初期化
        
        Args:
            drone (DroneEntity): 制御対象のドローン
            dt (float): 時間ステップ
            base_rpm (float): 基準RPM
        """
        # 基準RPMを先に設定（resetメソッドで使用されるため）
        self.__base_rpm = base_rpm
        
        # ホバリング用のRPM
        self.hover_rpm = HOVER_RPM
        
        # 最後の制御入力
        self.last_rpms = np.array([self.hover_rpm] * 4)
        
        # 前回の目標位置
        self.prev_target = None
        
        # 親クラスの初期化
        super().__init__(drone, dt)
        
        # 制御パラメータ（調整済み）
        self.gravity = np.array([0.0, 0.0, -9.8])
        self.K_P = np.array([0.3, 0.3, 0.3])  # 位置制御の比例ゲイン
        self.K_D = np.array([1.5, 1.5, 2.0])  # 速度制御の微分ゲイン
        self.K_RATES = np.array([2.0, 2.0, 0.5])  # 角速度制御のゲイン
        
        # ミキサーマトリックス（X型の場合）
        self.mixer_matrix = np.array([
            [-0.5, -0.5, -1.0],
            [-0.5,  0.5,  1.0],
            [ 0.5,  0.5, -1.0],
            [ 0.5, -0.5,  1.0]
        ])
        
        # 速度制限
        self.max_vel = 0.5  # 最大速度 (m/s)
        
        # 推力からRPMへの変換係数
        self.thrust_to_rpm = 1.0 / KF
    
    def update(self, target: Tuple[float, float, float]) -> np.ndarray:
        """
        目標位置に基づいて制御入力を計算
        
        Args:
            target (tuple): 目標位置 (x, y, z)
            
        Returns:
            np.ndarray: 4つのプロペラRPM
        """
        # ドローンの状態を取得
        curr_pos = self.get_drone_pos().cpu().numpy()
        curr_vel = self.get_drone_vel().cpu().numpy()
        curr_quat = self.get_drone_quat().cpu().numpy()
        
        # 目標位置の変化率を制限（急激な変化を防ぐ）
        if self.prev_target is not None:
            target_array = np.array(target)
            prev_target_array = np.array(self.prev_target)
            
            # 目標位置の変化量を計算
            delta_target = target_array - prev_target_array
            
            # 変化量を制限
            max_delta = 0.05  # 最大変化量
            if np.linalg.norm(delta_target) > max_delta:
                delta_target = delta_target * (max_delta / np.linalg.norm(delta_target))
            
            # 制限された目標位置
            limited_target = prev_target_array + delta_target
            target = tuple(limited_target)
        
        # 前回の目標位置を更新
        self.prev_target = target
        
        # 位置と速度の誤差
        pos_error = np.array(target) - curr_pos
        vel_error = np.zeros(3) - curr_vel  # 目標速度は0
        
        # 目標加速度を計算
        target_acc = self.K_P * pos_error + self.K_D * vel_error - self.gravity
        
        # 目標推力を計算（ドローンのZ軸方向の推力）
        norm_thrust = np.dot(target_acc, rotate_vector([0.0, 0.0, 1.0], curr_quat))
        
        # 推力が負にならないようにする
        norm_thrust = max(0.0, norm_thrust)
        
        # 目標姿勢を計算
        z_body = normalized_vector(target_acc)
        x_body = normalized_vector(np.cross(np.array([0.0, 1.0, 0.0]), z_body))
        y_body = normalized_vector(np.cross(z_body, x_body))
        target_att = mat2quat(np.vstack([x_body, y_body, z_body]).T)
        
        # 姿勢の誤差を計算
        q_error = qmult(qconjugate(curr_quat), target_att)
        body_rates = 2 * self.K_RATES * q_error[1:]
        if q_error[0] < 0:
            body_rates = -body_rates
        
        # 角速度を制限
        max_rate = 1.0  # 最大角速度 (rad/s)
        body_rates_norm = np.linalg.norm(body_rates)
        if body_rates_norm > max_rate:
            body_rates = body_rates * (max_rate / body_rates_norm)
        
        # 制御入力をRPMに変換
        rpms = self.__compute_rpms(norm_thrust, body_rates)
        
        # RPMを制限
        rpms = self.clamp_rpms(rpms)
        
        # 最後の制御入力を保存
        self.last_rpms = rpms
        
        return rpms
    
    def __compute_rpms(self, thrust: float, body_rates: np.ndarray) -> np.ndarray:
        """
        推力と体の角速度からRPMを計算
        
        Args:
            thrust (float): 推力 [N]
            body_rates (np.ndarray): 体の角速度 [roll_rate, pitch_rate, yaw_rate] [rad/s]
            
        Returns:
            np.ndarray: 4つのプロペラRPM
        """
        # 制御入力を計算（推力と角速度）
        roll_rate = body_rates[0]
        pitch_rate = body_rates[1]
        yaw_rate = body_rates[2]
        
        # 必要な推力とトルクを計算
        thrust_force = MASS * thrust  # 推力 [N]
        roll_torque = MASS * ARM * roll_rate  # ロールトルク [Nm]
        pitch_torque = MASS * ARM * pitch_rate  # ピッチトルク [Nm]
        yaw_torque = MASS * 0.05 * yaw_rate  # ヨートルク [Nm]
        
        # X型のミキサー行列を使用してモーター出力を計算
        # モーターの配置（上から見て）:
        # M1 ↖   ↗ M2
        #    \   /
        #     \ /
        #     / \
        #    /   \
        # M4 ↙   ↘ M3
        
        # 各モーターの推力を計算
        f1 = (thrust_force - roll_torque - pitch_torque - yaw_torque) / 4.0
        f2 = (thrust_force - roll_torque + pitch_torque + yaw_torque) / 4.0
        f3 = (thrust_force + roll_torque + pitch_torque - yaw_torque) / 4.0
        f4 = (thrust_force + roll_torque - pitch_torque + yaw_torque) / 4.0
        
        # 推力からRPMに変換（F = kf * rpm^2 => rpm = sqrt(F/kf)）
        rpm1 = np.sqrt(max(0.0, f1) / KF)
        rpm2 = np.sqrt(max(0.0, f2) / KF)
        rpm3 = np.sqrt(max(0.0, f3) / KF)
        rpm4 = np.sqrt(max(0.0, f4) / KF)
        
        return np.array([rpm1, rpm2, rpm3, rpm4])
    
    def reset(self) -> None:
        """
        コントローラの状態をリセット
        """
        self.last_rpms = np.array([self.hover_rpm] * 4)
        self.prev_target = None
    
    def hover(self) -> List[float]:
        """
        ドローンをホバリングさせる
        
        Returns:
            list: ホバリング用のRPM設定
        """
        hover_rpms = [self.hover_rpm] * 4
        self.set_propellers_rpm(hover_rpms)
        return hover_rpms
    
    def clamp_rpms(self, rpms: np.ndarray) -> np.ndarray:
        """
        RPMを最小値と最大値の間に制限する
        
        Args:
            rpms (np.ndarray): RPM値の配列
            
        Returns:
            np.ndarray: 制限されたRPM値の配列
        """
        return np.clip(rpms, MIN_RPM, MAX_RPM)
    
    def get_parameters(self) -> Dict[str, Any]:
        """
        コントローラのパラメータを取得
        
        Returns:
            dict: パラメータの辞書
        """
        params = super().get_parameters()
        params.update({
            "base_rpm": self.__base_rpm,
            "hover_rpm": self.hover_rpm,
            "K_P": self.K_P.tolist(),
            "K_D": self.K_D.tolist(),
            "K_RATES": self.K_RATES.tolist(),
            "gravity": self.gravity.tolist(),
            "kf": KF,
            "km": KM,
            "arm": ARM,
            "mass": MASS,
            "max_vel": self.max_vel
        })
        return params
    
    def set_parameters(self, params: Dict[str, Any]) -> None:
        """
        コントローラのパラメータを設定
        
        Args:
            params (dict): パラメータの辞書
        """
        super().set_parameters(params)
        
        if "base_rpm" in params:
            self.__base_rpm = params["base_rpm"]
        
        if "K_P" in params:
            self.K_P = np.array(params["K_P"])
        
        if "K_D" in params:
            self.K_D = np.array(params["K_D"])
        
        if "K_RATES" in params:
            self.K_RATES = np.array(params["K_RATES"])
        
        if "gravity" in params:
            self.gravity = np.array(params["gravity"])
            
        if "max_vel" in params:
            self.max_vel = params["max_vel"]
