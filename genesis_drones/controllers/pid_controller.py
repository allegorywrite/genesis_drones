#!/usr/bin/env python3

import torch
import numpy as np
from typing import Any, List, Tuple, Optional, Dict
from genesis.utils.geom import quat_to_xyz

from genesis_drones.controllers.base_controller import BaseController

# 定数
BASE_RPM = 14468.429183500699
MIN_RPM = 0.9 * BASE_RPM
MAX_RPM = 1.5 * BASE_RPM

class PIDController:
    """PIDコントローラクラス"""
    
    def __init__(self, kp, ki, kd):
        """
        PIDコントローラの初期化
        
        Args:
            kp (float): 比例ゲイン
            ki (float): 積分ゲイン
            kd (float): 微分ゲイン
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
    
    def update(self, error, dt):
        """
        PID制御の更新
        
        Args:
            error (float): 現在の誤差
            dt (float): 時間ステップ
            
        Returns:
            float: 制御出力
        """
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
    
    def reset(self):
        """PIDコントローラのリセット"""
        self.integral = 0.0
        self.prev_error = 0.0


class DronePIDController(BaseController):
    """ドローン用PIDコントローラクラス"""
    
    def __init__(self, drone: Any, dt: float, base_rpm: float = BASE_RPM, pid_params: Optional[List[List[float]]] = None):
        """
        ドローンPIDコントローラの初期化
        
        Args:
            drone (DroneEntity): 制御対象のドローン
            dt (float): 時間ステップ
            base_rpm (float): 基準RPM
            pid_params (list): PIDパラメータのリスト
        """
        # デフォルトのPIDパラメータ（墜落防止のために調整済み）
        default_pid_params = [
            [2.5, 0.1, 0.5],  # pos_x
            [2.5, 0.1, 0.5],  # pos_y
            [3.0, 0.2, 1.0],  # pos_z
            [20.0, 0.5, 20.0],  # vel_x
            [20.0, 0.5, 20.0],  # vel_y
            [25.0, 1.0, 20.0],  # vel_z
            [10.0, 0.2, 1.0],  # att_roll
            [10.0, 0.2, 1.0],  # att_pitch
            [2.0, 0.1, 0.2],  # att_yaw
        ]
        
        # パラメータが指定されていない場合はデフォルト値を使用
        if pid_params is None:
            pid_params = default_pid_params
        
        # PIDコントローラの初期化
        self.__pid_pos_x = PIDController(kp=pid_params[0][0], ki=pid_params[0][1], kd=pid_params[0][2])
        self.__pid_pos_y = PIDController(kp=pid_params[1][0], ki=pid_params[1][1], kd=pid_params[1][2])
        self.__pid_pos_z = PIDController(kp=pid_params[2][0], ki=pid_params[2][1], kd=pid_params[2][2])
        
        self.__pid_vel_x = PIDController(kp=pid_params[3][0], ki=pid_params[3][1], kd=pid_params[3][2])
        self.__pid_vel_y = PIDController(kp=pid_params[4][0], ki=pid_params[4][1], kd=pid_params[4][2])
        self.__pid_vel_z = PIDController(kp=pid_params[5][0], ki=pid_params[5][1], kd=pid_params[5][2])
        
        self.__pid_att_roll = PIDController(kp=pid_params[6][0], ki=pid_params[6][1], kd=pid_params[6][2])
        self.__pid_att_pitch = PIDController(kp=pid_params[7][0], ki=pid_params[7][1], kd=pid_params[7][2])
        self.__pid_att_yaw = PIDController(kp=pid_params[8][0], ki=pid_params[8][1], kd=pid_params[8][2])
        
        super().__init__(drone, dt)
        self.__base_rpm = base_rpm
    
    def __get_drone_att(self) -> torch.Tensor:
        """ドローンの姿勢を取得"""
        quat = self.get_drone_quat()
        return quat_to_xyz(quat)
    
    def __mixer(self, thrust, roll, pitch, yaw, x_vel, y_vel) -> torch.Tensor:
        """
        制御入力をプロペラRPMに変換するミキサー
        
        Args:
            thrust (float): スラスト
            roll (float): ロール
            pitch (float): ピッチ
            yaw (float): ヨー
            x_vel (float): X方向速度
            y_vel (float): Y方向速度
            
        Returns:
            torch.Tensor: 4つのプロペラRPM
        """
        M1 = self.__base_rpm + (thrust - roll - pitch - yaw - x_vel + y_vel)
        M2 = self.__base_rpm + (thrust - roll + pitch + yaw + x_vel + y_vel)
        M3 = self.__base_rpm + (thrust + roll + pitch - yaw + x_vel - y_vel)
        M4 = self.__base_rpm + (thrust + roll - pitch + yaw - x_vel - y_vel)
        
        return torch.Tensor([M1, M2, M3, M4])
    
    def update(self, target: Tuple[float, float, float]) -> np.ndarray:
        """
        目標位置に基づいてPID制御を更新
        
        Args:
            target (tuple): 目標位置 (x, y, z)
            
        Returns:
            np.ndarray: 4つのプロペラRPM
        """
        curr_pos = self.get_drone_pos()
        curr_vel = self.get_drone_vel()
        curr_att = self.__get_drone_att()
        
        # 位置の誤差
        err_pos_x = target[0] - curr_pos[0]
        err_pos_y = target[1] - curr_pos[1]
        err_pos_z = target[2] - curr_pos[2]
        
        # 目標速度の計算
        vel_des_x = self.__pid_pos_x.update(err_pos_x, self.dt)
        vel_des_y = self.__pid_pos_y.update(err_pos_y, self.dt)
        vel_des_z = self.__pid_pos_z.update(err_pos_z, self.dt)
        
        # 速度の誤差
        error_vel_x = vel_des_x - curr_vel[0]
        error_vel_y = vel_des_y - curr_vel[1]
        error_vel_z = vel_des_z - curr_vel[2]
        
        # 速度制御
        x_vel_del = self.__pid_vel_x.update(error_vel_x, self.dt)
        y_vel_del = self.__pid_vel_y.update(error_vel_y, self.dt)
        thrust_des = self.__pid_vel_z.update(error_vel_z, self.dt)
        
        # 姿勢の誤差
        err_roll = 0.0 - curr_att[0]
        err_pitch = 0.0 - curr_att[1]
        err_yaw = 0.0 - curr_att[2]
        
        # 姿勢制御
        roll_del = self.__pid_att_roll.update(err_roll, self.dt)
        pitch_del = self.__pid_att_pitch.update(err_pitch, self.dt)
        yaw_del = self.__pid_att_yaw.update(err_yaw, self.dt)
        
        # ミキサーでRPMを計算
        prop_rpms = self.__mixer(thrust_des, roll_del, pitch_del, yaw_del, x_vel_del, y_vel_del)
        prop_rpms = prop_rpms.cpu()
        
        return prop_rpms
    
    def reset(self):
        """全てのPIDコントローラをリセット"""
        self.__pid_pos_x.reset()
        self.__pid_pos_y.reset()
        self.__pid_pos_z.reset()
        self.__pid_vel_x.reset()
        self.__pid_vel_y.reset()
        self.__pid_vel_z.reset()
        self.__pid_att_roll.reset()
        self.__pid_att_pitch.reset()
        self.__pid_att_yaw.reset()
    
    def hover(self):
        """
        ドローンをホバリングさせる
        
        Returns:
            list: ホバリング用のRPM設定
        """
        hover_rpms = [self.__base_rpm, self.__base_rpm, self.__base_rpm, self.__base_rpm]
        self.set_propellers_rpm(hover_rpms)
        return hover_rpms
    
    def clamp_rpms(self, rpms):
        """
        RPMを最小値と最大値の間に制限する
        
        Args:
            rpms (list): RPM値のリスト
            
        Returns:
            list: 制限されたRPM値のリスト
        """
        return [max(MIN_RPM, min(int(rpm), MAX_RPM)) for rpm in rpms]
    
    def get_parameters(self) -> Dict[str, Any]:
        """
        コントローラのパラメータを取得
        
        Returns:
            dict: パラメータの辞書
        """
        params = super().get_parameters()
        params.update({
            "base_rpm": self.__base_rpm,
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
            "att_pid": {
                "roll": {"kp": self.__pid_att_roll.kp, "ki": self.__pid_att_roll.ki, "kd": self.__pid_att_roll.kd},
                "pitch": {"kp": self.__pid_att_pitch.kp, "ki": self.__pid_att_pitch.ki, "kd": self.__pid_att_pitch.kd},
                "yaw": {"kp": self.__pid_att_yaw.kp, "ki": self.__pid_att_yaw.ki, "kd": self.__pid_att_yaw.kd}
            }
        })
        return params
