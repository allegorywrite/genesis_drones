#!/usr/bin/env python3

import numpy as np
import math
from typing import Any, List, Tuple, Optional

from genesis_drones.controllers.base_controller import BaseController

class DroneFlightController:
    """ドローンの飛行を制御するクラス（位置制御と速度制御をサポート）"""
    
    def __init__(self, drone: Any, controller: BaseController, logger=None,
                 velocity_control=False, target_velocity=None, 
                 target_angular_velocity=None, velocity_profile='constant'):
        """
        ドローン飛行コントローラの初期化
        
        Args:
            drone (DroneEntity): 制御対象のドローン
            controller (BaseController): ドローンコントローラ
            logger: ロガー（オプション）
            velocity_control (bool): 速度制御モードかどうか
            target_velocity (np.ndarray): 目標速度 [m/s]
            target_angular_velocity (np.ndarray): 目標角速度 [rad/s]
            velocity_profile (str): 速度プロファイルタイプ
        """
        self.drone = drone
        self.controller = controller
        self.logger = logger
        
        # 位置制御用パラメータ
        self.is_flying = False
        self.current_target = None
        self.route_points = []
        self.current_route_index = 0
        self.distance_threshold = 0.1  # 目標に到達したと判断する距離のしきい値
        
        # 速度制御用パラメータ
        self.velocity_control = velocity_control
        self.target_velocity = np.array(target_velocity) if target_velocity is not None else np.zeros(3)
        self.target_angular_velocity = np.array(target_angular_velocity) if target_angular_velocity is not None else np.zeros(3)
        self.velocity_profile = velocity_profile
        self.time_elapsed = 0.0
        
        # 速度持続ステップ関連のパラメータ
        self.velocity_duration_steps = 0  # 持続ステップ数（0は無制限）
        self.velocity_steps_counter = 0   # 経過ステップカウンター
        self.default_velocity = np.zeros(3)  # デフォルト速度（停止時に使用）
        self.default_angular_velocity = np.zeros(3)  # デフォルト角速度
        
        # 速度プロファイル用パラメータ
        self.ramp_duration = 2.0  # ランプ時間 [s]
        self.sinusoidal_period = 5.0  # 正弦波周期 [s]
        self.max_velocity = 1.0  # 最大速度 [m/s]
    
    def log(self, message):
        """ログメッセージを出力"""
        if self.logger:
            self.logger.info(message)
        else:
            print(message)
    
    def set_route(self, points: List[Tuple[float, float, float]]) -> bool:
        """
        飛行ルートを設定
        
        Args:
            points (list): 飛行するポイントのリスト [(x1, y1, z1), (x2, y2, z2), ...]
            
        Returns:
            bool: 成功したかどうか
        """
        if not points:
            self.log("No points provided for route")
            return False
        
        self.route_points = points
        self.current_route_index = 0
        
        self.log(f"Set route with {len(points)} points")
        return True
    
    def fly_to_point(self, target_point: Tuple[float, float, float]) -> bool:
        """
        指定したポイントに飛行
        
        Args:
            target_point (tuple): 目標位置 (x, y, z)
            
        Returns:
            bool: 成功したかどうか
        """
        if self.velocity_control:
            self.log("Cannot fly to point in velocity control mode")
            return False
            
        self.is_flying = True
        self.current_target = target_point
        
        self.log(f"Flying to point {target_point}")
        return True
    
    def set_velocity_target(self, linear_vel: np.ndarray, angular_vel: np.ndarray, duration_steps: int = 0) -> None:
        """
        目標速度を設定（速度制御モード用）
        
        Args:
            linear_vel (np.ndarray): 目標線形速度 [m/s]
            angular_vel (np.ndarray): 目標角速度 [rad/s]
            duration_steps (int): 速度を維持するステップ数（0は無制限）
        """
        self.target_velocity = linear_vel
        self.target_angular_velocity = angular_vel
        self.velocity_duration_steps = duration_steps
        self.velocity_steps_counter = 0  # カウンターをリセット
        
        if duration_steps > 0 and self.logger:
            self.logger.info(f"Setting velocity for {duration_steps} steps")
    
    def _get_velocity_profile(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        速度プロファイルに基づく目標速度を計算
        
        Args:
            dt (float): 時間ステップ [s]
            
        Returns:
            Tuple[np.ndarray, np.ndarray]: (線形速度, 角速度)
        """
        self.time_elapsed += dt
        
        if self.velocity_profile == 'constant':
            return self.target_velocity, self.target_angular_velocity
            
        elif self.velocity_profile == 'ramp':
            # 線形に速度を増加
            factor = min(1.0, self.time_elapsed / self.ramp_duration)
            return factor * self.target_velocity, factor * self.target_angular_velocity
            
        elif self.velocity_profile == 'sinusoidal':
            # 正弦波状に速度を変化
            factor = math.sin(2 * math.pi * self.time_elapsed / self.sinusoidal_period)
            return factor * self.max_velocity * self.target_velocity, factor * self.target_angular_velocity
            
        else:
            return self.target_velocity, self.target_angular_velocity
    
    def start_route(self) -> bool:
        """
        ルートの飛行を開始
        
        Returns:
            bool: 成功したかどうか
        """
        if self.velocity_control:
            self.is_flying = True
            self.time_elapsed = 0.0
            return True
            
        if not self.route_points:
            self.log("No route points set")
            return False
        
        self.current_route_index = 0
        return self.fly_to_point(self.route_points[0])
    
    def update(self) -> Tuple[bool, bool]:
        """
        飛行状態を更新
        
        Returns:
            Tuple[bool, bool]: (飛行中かどうか, ウェイポイントが変更されたかどうか)
        """
        if not self.is_flying:
            return False, False
            
        if self.velocity_control:
            # 持続ステップのカウント
            if self.velocity_duration_steps > 0:
                self.velocity_steps_counter += 1
                
                # 指定ステップ数に達したら速度をリセット
                if self.velocity_steps_counter >= self.velocity_duration_steps:
                    self.target_velocity = self.default_velocity
                    self.target_angular_velocity = self.default_angular_velocity
                    self.velocity_duration_steps = 0
                    self.velocity_steps_counter = 0
                    if self.logger:
                        self.logger.info("Velocity duration reached, resetting to default velocity")
            
            # 速度制御モード
            linear_vel, angular_vel = self._get_velocity_profile(self.controller.dt)
            
            # 30ステップごとにログ出力
            if not hasattr(self, 'log_counter'):
                self.log_counter = 0
            self.log_counter += 1
            
            if self.log_counter % 30 == 0:
                drone_pos = self.drone.get_pos().cpu().numpy()
                print(f"\n[DroneFlightController] drone_pos: {drone_pos}")
                print(f"[DroneFlightController] linear_vel: {linear_vel}")
                print(f"[DroneFlightController] angular_vel: {angular_vel}")
                if self.velocity_duration_steps > 0:
                    print(f"[DroneFlightController] remaining steps: {self.velocity_duration_steps - self.velocity_steps_counter}")
            
            rpms = self.controller.update(linear_vel, angular_vel, vel_target=True)
            self.controller.set_propellers_rpm(rpms)
            return True, False
            
        else:
            # 位置制御モード
            if self.current_target is None:
                return False, False
            
            # ドローンの現在位置を取得
            drone_pos = self.drone.get_pos().cpu().numpy()
            
            # 目標位置までの距離を計算
            target = self.current_target
            distance = np.linalg.norm(target - drone_pos)
            
            # 目標に十分近づいたら次のポイントへ
            if distance <= self.distance_threshold:
                # ルートの次のポイントがあれば、そこへ飛行
                if self.route_points and self.current_route_index < len(self.route_points) - 1:
                    self.current_route_index += 1
                    next_point = self.route_points[self.current_route_index]
                    self.current_target = next_point
                    self.log(f"Reached waypoint, moving to next point: {next_point}")
                    return True, True  # 飛行中、ウェイポイント変更あり
                else:
                    # ルートの最後のポイントに到達した場合
                    self.is_flying = False
                    self.current_target = None
                    self.log(f"Reached final target {target}")
                    
                    # ホバリング状態に設定
                    if hasattr(self.controller, 'hover'):
                        self.controller.hover()
                    return False, False  # 飛行終了
            
            # コントローラを使用してRPMを計算
            # target_att = np.zeros(3)
            rpms = self.controller.update(self.current_target)
            
            # RPMを制限（コントローラ側で行われていない場合）
            if hasattr(self.controller, 'clamp_rpms'):
                rpms = self.controller.clamp_rpms(rpms)
            
            # ドローンのプロペラRPMを設定
            self.controller.set_propellers_rpm(rpms)
            
            return True, False  # 飛行中、ウェイポイント変更なし
    
    def stop(self):
        """飛行を停止してホバリング状態に"""
        self.is_flying = False
        self.current_target = None
        self.time_elapsed = 0.0
        
        # ホバリング状態に設定
        if hasattr(self.controller, 'hover'):
            self.controller.hover()
