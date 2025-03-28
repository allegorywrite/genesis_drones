#!/usr/bin/env python3

import numpy as np
import math
from typing import Any, List, Tuple, Optional, Dict
from scipy.spatial.transform import Rotation
from genesis.utils.geom import quat_to_xyz, quat_to_R, xyz_to_quat, R_to_quat

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
HOVER_RPM = np.sqrt(MASS * GRAVITY / (4 * KF))

class VelocityController(BaseController):
    """
    速度制御用PIDコントローラ
    
    ドローンの線形速度と角速度を制御するコントローラ
    """
    
    def __init__(self, drone: Any, dt: float, base_rpm: float = BASE_RPM):
        """
        速度制御コントローラの初期化
        
        Args:
            drone (Any): 制御対象のドローン
            dt (float): 時間ステップ
            base_rpm (float): 基準RPM
        """
        self.__base_rpm = base_rpm
        self.hover_rpm = HOVER_RPM
        
        # 重力補償値のログ出力
        print(f"\n[VelocityController] 重力補償値計算:")
        print(f"GRAVITY: {GRAVITY}, MASS: {MASS}")
        print(f"重力補償値 (GRAVITY*MASS): {GRAVITY*MASS}")
        
        # 親クラスの初期化
        super().__init__(drone, dt)
        
        # 制御カウンタ
        self.control_counter = 0
        
        # 速度制御用PIDゲイン
        self.P_COEFF_VEL = np.array([.1, .1, 0.5])  # 線形速度 (z成分を強化)
        self.I_COEFF_VEL = np.array([.1, .1, .5])   # z成分をさらに強化
        self.D_COEFF_VEL = np.array([.01, .01, .01])   # z成分を増加
        
        # 姿勢制御用PIDゲイン
        self.P_COEFF_ATT = np.array([10000., 10000., 10000.])
        self.I_COEFF_ATT = np.array([.0, .0, 500.])
        self.D_COEFF_ATT = np.array([100., 100., 100.])
        
        # 角速度制御用PIDゲイン
        self.P_COEFF_ANG_VEL = np.array([8000., 8000., 8000.])
        self.I_COEFF_ANG_VEL = np.array([.0, .0, 500.])
        self.D_COEFF_ANG_VEL = np.array([100., 100., 100.])
        
        # PWM関連の定数
        self.PWM2RPM_SCALE = 0.2685
        self.PWM2RPM_CONST = 4070.3
        self.MIN_PWM = 20000
        self.MAX_PWM = 65535
        
        # ミキサーマトリックス
        self.MIXER_MATRIX = np.array([
            [-.5, -.5, -1],
            [-.5,  .5,  1],
            [.5, .5, -1],
            [.5, -.5,  1]
        ])
        
        # 最後の制御入力
        self.last_rpms = np.array([self.hover_rpm] * 4)
        
    def update(self, target_vel: Tuple[float, float, float], 
               target_ang_vel: Tuple[float, float, float] = (0, 0, 0)) -> np.ndarray:
        """
        目標速度に基づいて制御入力を計算
        
        Args:
            target_vel (tuple): 目標線形速度 (x, y, z) [m/s]
            target_ang_vel (tuple): 目標角速度 (roll, pitch, yaw) [rad/s]
            
        Returns:
            np.ndarray: 4つのプロペラRPM
        """
        # ドローンの状態を取得
        curr_vel = self.get_drone_vel().cpu().numpy()
        curr_quat = self.get_drone_quat().cpu().numpy()
        # クォータニオンが有効かチェック (ノルムが0でないこと)
        if np.linalg.norm(curr_quat) < 1e-6:
            curr_quat = np.array([1.0, 0.0, 0.0, 0.0])  # デフォルトのクォータニオン
        curr_ang_vel = self.get_drone_ang_vel().cpu().numpy()
        
        # 速度制御計算
        rpms = self._compute_velocity_control(
            self.dt,
            curr_vel,
            curr_quat,
            curr_ang_vel,
            np.array(target_vel),
            np.array(target_ang_vel)
        )
        
        # RPMを制限
        rpms = self.clamp_rpms(rpms)
        self.last_rpms = rpms
        
        return rpms
    
    def _compute_velocity_control(self,
                                dt: float,
                                curr_vel: np.ndarray,
                                curr_quat: np.ndarray,
                                curr_ang_vel: np.ndarray,
                                target_vel: np.ndarray,
                                target_ang_vel: np.ndarray) -> np.ndarray:
        """
        速度制御入力（RPM）を計算
        
        Args:
            dt (float): 時間ステップ
            curr_vel (ndarray): 現在速度
            curr_quat (ndarray): 現在姿勢
            curr_ang_vel (ndarray): 現在角速度
            target_vel (ndarray): 目標速度
            target_ang_vel (ndarray): 目標角速度
            
        Returns:
            ndarray: 4つのモーターRPM
        """
        self.control_counter += 1
        
        # 現在のヨー角を取得
        curr_rpy = quat_to_xyz(curr_quat)
        target_yaw = curr_rpy[2]  # 現在のヨー角を目標ヨー角として使用
        
        # 目標速度から目標姿勢を計算
        thrust, target_euler = self._compute_desired_attitude(
            dt,
            curr_vel,
            curr_quat,
            target_vel,
            target_yaw
        )
        
        # 目標姿勢から必要なRPMを計算
        rpms = self._compute_attitude_control(
            dt,
            thrust,
            curr_quat,
            target_euler,
            target_ang_vel
        )
        
        return rpms
    
    def reset(self) -> None:
        """コントローラの状態をリセット"""
        super().reset()
        
        # 速度制御変数の初期化
        self.last_vel_e = np.zeros(3)
        self.integral_vel_e = np.zeros(3)
        self.prev_vel = np.zeros(3)
        
        # 姿勢制御変数の初期化
        self.last_rpy = np.zeros(3)
        self.integral_rpy_e = np.zeros(3)
        
        # 角速度制御変数の初期化
        self.last_ang_vel_e = np.zeros(3)
        self.integral_ang_vel_e = np.zeros(3)
        
        # 最後の制御入力
        self.last_rpms = np.array([self.hover_rpm] * 4)
    
    def hover(self) -> List[float]:
        """ホバリング用RPMを返す"""
        hover_rpms = [self.hover_rpm] * 4
        self.set_propellers_rpm(hover_rpms)
        return hover_rpms
    
    def clamp_rpms(self, rpms: np.ndarray) -> np.ndarray:
        """RPMを制限"""
        return np.clip(rpms, MIN_RPM, MAX_RPM)
    
    def _compute_desired_attitude(self,
                                 dt: float,
                                 curr_vel: np.ndarray,
                                 curr_quat: np.ndarray,
                                 target_vel: np.ndarray,
                                 target_yaw: float = 0.0) -> Tuple[float, np.ndarray]:
        """
        目標速度から目標姿勢を計算
        
        Args:
            dt (float): 時間ステップ
            curr_vel (ndarray): 現在速度
            curr_quat (ndarray): 現在姿勢
            target_vel (ndarray): 目標速度
            target_yaw (float): 目標ヨー角
            
        Returns:
            float: スラスト値
            ndarray: 目標姿勢（ロール、ピッチ、ヨー）
        """
        # 現在の回転行列を計算
        curr_rotation = quat_to_R(curr_quat)
        
        # 速度誤差
        vel_error = target_vel - curr_vel
        curr_acc = (curr_vel - self.prev_vel) / dt
        
        # 積分誤差の更新
        self.integral_vel_e = self.integral_vel_e + vel_error * dt
        self.integral_vel_e = np.clip(self.integral_vel_e, -2., 2.)
        
        # PID目標推力
        # target_thrust = np.multiply(self.P_COEFF_VEL, vel_error) \
        #               + np.multiply(self.I_COEFF_VEL, self.integral_vel_e) \
        #               + np.multiply(self.D_COEFF_VEL, -curr_acc) \
        #               + np.array([0, 0, GRAVITY*MASS])
        target_thrust = np.multiply(self.P_COEFF_VEL, vel_error) \
                      + np.array([0, 0, GRAVITY*MASS])
        
        # 30ステップごとにログ出力
        if self.control_counter % 30 == 0:
            print(f"\n[VelocityController] curr_vel: {curr_vel}")
            print(f"[VelocityController] curr_acc: {curr_acc}")
            print(f"[VelocityController] target_vel: {target_vel}")
            print(f"[VelocityController] vel_error: {vel_error}")
            print(f"[VelocityController] 1st term: {np.multiply(self.P_COEFF_VEL, vel_error)}")
            print(f"[VelocityController] 2nd term: {np.multiply(self.I_COEFF_VEL, self.integral_vel_e)}")
            print(f"[VelocityController] 3rd term: {np.multiply(self.D_COEFF_VEL, -curr_acc)}")
            print(f"[VelocityController] 4th term: {np.array([0, 0, GRAVITY*MASS])}")
        
        self.last_vel_e = vel_error
        self.prev_vel = curr_vel
        
        # スカラー推力
        scalar_thrust = max(0., np.dot(target_thrust, curr_rotation[:,2]))
        thrust = (math.sqrt(scalar_thrust / (4*KF)) - self.PWM2RPM_CONST) / self.PWM2RPM_SCALE
        
        # 目標z軸
        target_z_ax = target_thrust / np.linalg.norm(target_thrust)
        
        # 目標x軸の計算
        target_x_c = np.array([math.cos(target_yaw), math.sin(target_yaw), 0])
        
        # 目標y軸の計算
        target_y_ax = np.cross(target_z_ax, target_x_c) / np.linalg.norm(np.cross(target_z_ax, target_x_c))
        
        # 目標x軸の計算
        target_x_ax = np.cross(target_y_ax, target_z_ax)
        
        # 目標回転行列
        target_rotation = (np.vstack([target_x_ax, target_y_ax, target_z_ax])).transpose()
        
        # 目標オイラー角
        target_euler = quat_to_xyz(R_to_quat(target_rotation))
        
        return thrust, target_euler
    
    def _compute_attitude_control(self,
                                 dt: float,
                                 thrust: float,
                                 curr_quat: np.ndarray,
                                 target_euler: np.ndarray,
                                 target_rpy_rates: np.ndarray = np.zeros(3)) -> np.ndarray:
        """
        目標姿勢から必要なRPMを計算する姿勢制御
        
        Args:
            dt (float): 時間ステップ
            thrust (float): スラスト値
            curr_quat (ndarray): 現在姿勢
            target_euler (ndarray): 目標姿勢（ロール、ピッチ、ヨー）
            target_rpy_rates (ndarray): 目標角速度
            
        Returns:
            ndarray: 4つのモーターRPM
        """
        # 現在の回転行列を計算
        curr_rotation = quat_to_R(curr_quat)
        
        # 現在のロール、ピッチ、ヨーを計算
        curr_rpy = quat_to_xyz(curr_quat)
        
        # 目標四元数を計算
        target_quat = xyz_to_quat(target_euler)
        
        # 目標回転行列を計算
        target_rotation = quat_to_R(target_quat)
        
        # 回転行列の誤差を計算
        rot_matrix_e = np.dot((target_rotation.transpose()), curr_rotation) - np.dot(curr_rotation.transpose(), target_rotation)
        rot_e = np.array([rot_matrix_e[2, 1], rot_matrix_e[0, 2], rot_matrix_e[1, 0]])
        
        # 角速度の誤差を計算
        rpy_rates_e = target_rpy_rates - (curr_rpy - self.last_rpy) / dt
        self.last_rpy = curr_rpy
        
        # 積分誤差の更新
        self.integral_rpy_e = self.integral_rpy_e - rot_e * dt
        self.integral_rpy_e = np.clip(self.integral_rpy_e, -1500., 1500.)
        self.integral_rpy_e[0:2] = np.clip(self.integral_rpy_e[0:2], -1., 1.)
        
        # PID目標トルク
        # target_torques = - np.multiply(self.P_COEFF_ATT, rot_e) \
        #                 + np.multiply(self.D_COEFF_ATT, rpy_rates_e) \
        #                 + np.multiply(self.I_COEFF_ATT, self.integral_rpy_e)
        target_torques = - np.multiply(self.P_COEFF_ATT, rot_e)
        
        # トルクの制限
        target_torques = np.clip(target_torques, -3200, 3200)
        
        # PWMを計算
        pwm = thrust + np.dot(self.MIXER_MATRIX, target_torques)
        
        # PWM制限
        pwm = np.clip(pwm, self.MIN_PWM, self.MAX_PWM)
        
        # PWMからRPMに変換
        rpms = self.PWM2RPM_SCALE * pwm + self.PWM2RPM_CONST
        rpms = np.clip(rpms, MIN_RPM, MAX_RPM)
        return rpms
    
    def get_parameters(self) -> Dict[str, Any]:
        """コントローラパラメータを取得"""
        params = super().get_parameters()
        params.update({
            "P_COEFF_VEL": self.P_COEFF_VEL.tolist(),
            "I_COEFF_VEL": self.I_COEFF_VEL.tolist(),
            "D_COEFF_VEL": self.D_COEFF_VEL.tolist(),
            "P_COEFF_ATT": self.P_COEFF_ATT.tolist(),
            "I_COEFF_ATT": self.I_COEFF_ATT.tolist(),
            "D_COEFF_ATT": self.D_COEFF_ATT.tolist(),
            "P_COEFF_ANG_VEL": self.P_COEFF_ANG_VEL.tolist(),
            "I_COEFF_ANG_VEL": self.I_COEFF_ANG_VEL.tolist(),
            "D_COEFF_ANG_VEL": self.D_COEFF_ANG_VEL.tolist(),
        })
        return params
    
    def set_parameters(self, params: Dict[str, Any]) -> None:
        """コントローラパラメータを設定"""
        super().set_parameters(params)
        
        if "P_COEFF_VEL" in params:
            self.P_COEFF_VEL = np.array(params["P_COEFF_VEL"])
        
        if "I_COEFF_VEL" in params:
            self.I_COEFF_VEL = np.array(params["I_COEFF_VEL"])
        
        if "D_COEFF_VEL" in params:
            self.D_COEFF_VEL = np.array(params["D_COEFF_VEL"])
        
        if "P_COEFF_ANG_VEL" in params:
            self.P_COEFF_ANG_VEL = np.array(params["P_COEFF_ANG_VEL"])
        
        if "I_COEFF_ANG_VEL" in params:
            self.I_COEFF_ANG_VEL = np.array(params["I_COEFF_ANG_VEL"])
        
        if "D_COEFF_ANG_VEL" in params:
            self.D_COEFF_ANG_VEL = np.array(params["D_COEFF_ANG_VEL"])
            
        if "P_COEFF_ATT" in params:
            self.P_COEFF_ATT = np.array(params["P_COEFF_ATT"])
        
        if "I_COEFF_ATT" in params:
            self.I_COEFF_ATT = np.array(params["I_COEFF_ATT"])
        
        if "D_COEFF_ATT" in params:
            self.D_COEFF_ATT = np.array(params["D_COEFF_ATT"])
