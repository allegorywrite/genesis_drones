#!/usr/bin/env python3

import torch
import numpy as np
from typing import Any, List, Tuple, Optional, Dict
from transforms3d.quaternions import rotate_vector, qconjugate, mat2quat, qmult
from transforms3d.utils import normalized_vector
from genesis.utils.geom import quat_to_xyz

from genesis_drones.controllers.base_controller import BaseController
from genesis_drones.controllers.pid_controller import PIDController

# URDFから取得した定数
KF = 3.16e-10  # 推力係数 [N/RPM^2]
KM = 7.94e-12  # トルク係数 [Nm/RPM^2]
ARM = 0.0397   # アーム長 [m]
MASS = 0.027   # 質量 [kg]

# RPM関連の定数
BASE_RPM = 14468.429183500699
MIN_RPM = 0.9 * BASE_RPM
MAX_RPM = 1.5 * BASE_RPM

# ホバリング用のRPM計算
# F = m*g = 4*kf*rpm^2 => rpm = sqrt(m*g/(4*kf))
HOVER_RPM = np.sqrt(MASS * 9.8 / (4 * KF))

class HybridController(BaseController):
    """
    ハイブリッドコントローラ
    
    PIDコントローラとCTBRコントローラの利点を組み合わせたコントローラ。
    位置と速度の制御にはPID、姿勢の制御には四元数ベースのCTBRを使用。
    """
    
    def __init__(self, drone: Any, dt: float, base_rpm: float = BASE_RPM, pid_params: Optional[Dict[str, List[float]]] = None):
        """
        ハイブリッドコントローラの初期化
        
        Args:
            drone (DroneEntity): 制御対象のドローン
            dt (float): 時間ステップ
            base_rpm (float): 基準RPM
            pid_params (dict): PIDパラメータの辞書
        """
        # 基準RPMを先に設定（resetメソッドで使用されるため）
        self.__base_rpm = base_rpm
        
        # ホバリング用のRPM
        self.hover_rpm = HOVER_RPM
        
        # 最後の制御入力
        self.last_rpms = np.array([self.hover_rpm] * 4)
        
        # 前回の目標位置
        self.prev_target = None
        
        # デフォルトのPIDパラメータ（調整済み）
        default_pid_params = {
            "pos": {
                "x": [1.5, 0.05, 0.5],  # kp, ki, kd
                "y": [1.5, 0.05, 0.5],
                "z": [2.0, 0.1, 0.8]
            },
            "vel": {
                "x": [10.0, 0.2, 10.0],
                "y": [10.0, 0.2, 10.0],
                "z": [15.0, 0.3, 10.0]
            }
        }
        
        # パラメータが指定されていない場合はデフォルト値を使用
        if pid_params is None:
            pid_params = default_pid_params
        
        # 位置制御用PIDコントローラ
        self.__pid_pos_x = PIDController(
            kp=pid_params["pos"]["x"][0], 
            ki=pid_params["pos"]["x"][1], 
            kd=pid_params["pos"]["x"][2]
        )
        self.__pid_pos_y = PIDController(
            kp=pid_params["pos"]["y"][0], 
            ki=pid_params["pos"]["y"][1], 
            kd=pid_params["pos"]["y"][2]
        )
        self.__pid_pos_z = PIDController(
            kp=pid_params["pos"]["z"][0], 
            ki=pid_params["pos"]["z"][1], 
            kd=pid_params["pos"]["z"][2]
        )
        
        # 速度制御用PIDコントローラ
        self.__pid_vel_x = PIDController(
            kp=pid_params["vel"]["x"][0], 
            ki=pid_params["vel"]["x"][1], 
            kd=pid_params["vel"]["x"][2]
        )
        self.__pid_vel_y = PIDController(
            kp=pid_params["vel"]["y"][0], 
            ki=pid_params["vel"]["y"][1], 
            kd=pid_params["vel"]["y"][2]
        )
        self.__pid_vel_z = PIDController(
            kp=pid_params["vel"]["z"][0], 
            ki=pid_params["vel"]["z"][1], 
            kd=pid_params["vel"]["z"][2]
        )
        
        # 制御モード（0: PID位置制御 + CTBR姿勢制御, 1: 完全PID制御）
        self.control_mode = 0
        
        # 親クラスの初期化
        super().__init__(drone, dt)
        
        # CTBR制御パラメータ（調整済み）
        self.gravity = np.array([0.0, 0.0, -9.8])
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
        
        # 位置の誤差
        err_pos_x = target[0] - curr_pos[0]
        err_pos_y = target[1] - curr_pos[1]
        err_pos_z = target[2] - curr_pos[2]
        
        # 目標速度の計算（PID）
        vel_des_x = self.__pid_pos_x.update(err_pos_x, self.dt)
        vel_des_y = self.__pid_pos_y.update(err_pos_y, self.dt)
        vel_des_z = self.__pid_pos_z.update(err_pos_z, self.dt)
        
        # 速度を制限
        vel_des = np.array([vel_des_x, vel_des_y, vel_des_z])
        vel_norm = np.linalg.norm(vel_des)
        if vel_norm > self.max_vel:
            vel_des = vel_des * (self.max_vel / vel_norm)
            vel_des_x, vel_des_y, vel_des_z = vel_des
        
        # 速度の誤差
        error_vel_x = vel_des_x - curr_vel[0]
        error_vel_y = vel_des_y - curr_vel[1]
        error_vel_z = vel_des_z - curr_vel[2]
        
        # 速度制御（PID）
        x_vel_del = self.__pid_vel_x.update(error_vel_x, self.dt)
        y_vel_del = self.__pid_vel_y.update(error_vel_y, self.dt)
        thrust_des = self.__pid_vel_z.update(error_vel_z, self.dt)
        
        if self.control_mode == 0:
            # ハイブリッドモード: PID位置制御 + CTBR姿勢制御
            
            # 目標加速度を計算
            target_acc = np.array([x_vel_del, y_vel_del, thrust_des]) - self.gravity
            
            # 目標姿勢を計算（CTBR）
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
            rpms = self.__compute_rpms(thrust_des, body_rates)
            
        else:
            # 完全PIDモード
            # 姿勢の誤差を計算（オイラー角）
            curr_att = quat_to_xyz(torch.tensor(curr_quat)).cpu().numpy()
            
            # 目標姿勢（ホバリング姿勢）
            err_roll = 0.0 - curr_att[0]
            err_pitch = 0.0 - curr_att[1]
            err_yaw = 0.0 - curr_att[2]
            
            # ミキサーでRPMを計算
            rpms = self.__mixer_pid(thrust_des, err_roll, err_pitch, err_yaw, x_vel_del, y_vel_del)
        
        # RPMを制限
        rpms = self.clamp_rpms(rpms)
        
        # 最後の制御入力を保存
        self.last_rpms = rpms
        
        return rpms
    
    def __compute_rpms(self, thrust: float, body_rates: np.ndarray) -> np.ndarray:
        """
        推力と体の角速度からRPMを計算（CTBRスタイル）
        
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
    
    def __mixer_pid(self, thrust, roll, pitch, yaw, x_vel, y_vel) -> np.ndarray:
        """
        制御入力をプロペラRPMに変換するミキサー（PIDスタイル）
        
        Args:
            thrust (float): スラスト
            roll (float): ロール
            pitch (float): ピッチ
            yaw (float): ヨー
            x_vel (float): X方向速度
            y_vel (float): Y方向速度
            
        Returns:
            np.ndarray: 4つのプロペラRPM
        """
        # 必要な推力とトルクを計算
        thrust_force = MASS * thrust  # 推力 [N]
        roll_torque = MASS * ARM * roll  # ロールトルク [Nm]
        pitch_torque = MASS * ARM * pitch  # ピッチトルク [Nm]
        yaw_torque = MASS * 0.05 * yaw  # ヨートルク [Nm]
        x_force = MASS * x_vel  # X方向力 [N]
        y_force = MASS * y_vel  # Y方向力 [N]
        
        # X型のミキサー行列を使用してモーター出力を計算
        # モーターの配置（上から見て）:
        # M1 ↖   ↗ M2
        #    \   /
        #     \ /
        #     / \
        #    /   \
        # M4 ↙   ↘ M3
        
        # 各モーターの推力を計算
        f1 = (thrust_force - roll_torque - pitch_torque - yaw_torque - x_force + y_force) / 4.0
        f2 = (thrust_force - roll_torque + pitch_torque + yaw_torque + x_force + y_force) / 4.0
        f3 = (thrust_force + roll_torque + pitch_torque - yaw_torque + x_force - y_force) / 4.0
        f4 = (thrust_force + roll_torque - pitch_torque + yaw_torque - x_force - y_force) / 4.0
        
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
        self.__pid_pos_x.reset()
        self.__pid_pos_y.reset()
        self.__pid_pos_z.reset()
        self.__pid_vel_x.reset()
        self.__pid_vel_y.reset()
        self.__pid_vel_z.reset()
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
    
    def set_control_mode(self, mode: int) -> None:
        """
        制御モードを設定
        
        Args:
            mode (int): 制御モード（0: ハイブリッド, 1: 完全PID）
        """
        if mode in [0, 1]:
            self.control_mode = mode
    
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
            "control_mode": self.control_mode,
            "pos_pid": {
                "x": {"kp": self.__pid_pos_x.kp, "ki": self.__pid_pos_x.ki, "kd": self.__pid_pos_x.kd},
                "y": {"kp": self.__pid_pos_y.kp, "ki": self.__pid_pos_y.ki, "kd": self.__pid_pos_y.kd},
                "z": {"kp": self.__pid_pos_z.kp, "ki": self.__pid_pos_z.ki, "kd": self.__pid_pos_z.kd}
            },
            "vel_pid": {
                "x": {"kp": self.__pid_vel_x.kp, "ki": self.__pid_vel_x.ki, "kd": self.__pid_vel_x.kd},
                "y": {"kp": self.__pid_vel_y.kp, "ki": self.__pid_vel_y.ki, "kd": self.__pid_vel_y.kd},
                "z": {"kp": self.__pid_vel_z.kp, "ki": self.__pid_vel_z.ki, "kd": self.__pid_vel_z.kd}
            },
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
        
        if "control_mode" in params:
            self.set_control_mode(params["control_mode"])
        
        if "K_RATES" in params:
            self.K_RATES = np.array(params["K_RATES"])
        
        if "gravity" in params:
            self.gravity = np.array(params["gravity"])
            
        if "max_vel" in params:
            self.max_vel = params["max_vel"]
        
        # PIDパラメータの設定
        if "pos_pid" in params:
            pos_pid = params["pos_pid"]
            if "x" in pos_pid:
                x_pid = pos_pid["x"]
                if "kp" in x_pid: self.__pid_pos_x.kp = x_pid["kp"]
                if "ki" in x_pid: self.__pid_pos_x.ki = x_pid["ki"]
                if "kd" in x_pid: self.__pid_pos_x.kd = x_pid["kd"]
            if "y" in pos_pid:
                y_pid = pos_pid["y"]
                if "kp" in y_pid: self.__pid_pos_y.kp = y_pid["kp"]
                if "ki" in y_pid: self.__pid_pos_y.ki = y_pid["ki"]
                if "kd" in y_pid: self.__pid_pos_y.kd = y_pid["kd"]
            if "z" in pos_pid:
                z_pid = pos_pid["z"]
                if "kp" in z_pid: self.__pid_pos_z.kp = z_pid["kp"]
                if "ki" in z_pid: self.__pid_pos_z.ki = z_pid["ki"]
                if "kd" in z_pid: self.__pid_pos_z.kd = z_pid["kd"]
        
        if "vel_pid" in params:
            vel_pid = params["vel_pid"]
            if "x" in vel_pid:
                x_pid = vel_pid["x"]
                if "kp" in x_pid: self.__pid_vel_x.kp = x_pid["kp"]
                if "ki" in x_pid: self.__pid_vel_x.ki = x_pid["ki"]
                if "kd" in x_pid: self.__pid_vel_x.kd = x_pid["kd"]
            if "y" in vel_pid:
                y_pid = vel_pid["y"]
                if "kp" in y_pid: self.__pid_vel_y.kp = y_pid["kp"]
                if "ki" in y_pid: self.__pid_vel_y.ki = y_pid["ki"]
                if "kd" in y_pid: self.__pid_vel_y.kd = y_pid["kd"]
            if "z" in vel_pid:
                z_pid = vel_pid["z"]
                if "kp" in z_pid: self.__pid_vel_z.kp = z_pid["kp"]
                if "ki" in z_pid: self.__pid_vel_z.ki = z_pid["ki"]
                if "kd" in z_pid: self.__pid_vel_z.kd = z_pid["kd"]
