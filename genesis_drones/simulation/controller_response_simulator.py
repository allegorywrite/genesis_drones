#!/usr/bin/env python3

"""
Controller Response Simulator Module

このモジュールはコントローラの応答を評価するためのシミュレータを提供します。
"""

import numpy as np
import torch
from typing import Dict, List, Tuple, Any, Optional

# 自作モジュールのインポート
from genesis_drones.controllers.dsl_pid_controller import DSLPIDController
from genesis_drones.controllers.flight_controller import DroneFlightController

class ControllerResponseSimulator:
    """コントローラの応答を評価するためのシミュレータ"""
    
    def __init__(self, dt=0.01):
        """
        ControllerResponseSimulatorの初期化
        
        Args:
            dt (float): シミュレーションの時間ステップ
        """
        self.dt = dt
        self.logger = None
        
        # シミュレーション環境のセットアップ
        self.setup_simulation()
    
    def setup_simulation(self):
        """シミュレーション環境のセットアップ"""
        try:
            # Genesisシーンの初期化
            from genesis_drones.utils.simulation_utils import initialize_genesis_scene
            self.scene = initialize_genesis_scene(dt=self.dt)
            
            # ドローンのセットアップ
            self.setup_drone()
            
            # シーンのビルド
            self.scene.build()
            
            if self.logger:
                self.logger.info("シミュレーション環境のセットアップ完了")
            else:
                print("シミュレーション環境のセットアップ完了")
                
        except Exception as e:
            error_msg = f"シミュレーション環境のセットアップに失敗: {e}"
            if self.logger:
                self.logger.error(error_msg)
            else:
                print(error_msg)
            raise
    
    def setup_drone(self):
        """ドローンのセットアップ"""
        try:
            # ドローンの追加
            from genesis_drones.utils.simulation_utils import add_drone_to_scene
            self.drone = add_drone_to_scene(self.scene, position=(0, 0, 0.5))
            
            # コントローラの初期化
            self.controller = DSLPIDController(
                drone=self.drone,
                dt=self.dt
            )
            
            # 飛行コントローラの初期化（速度制御モードを有効化）
            self.flight_controller = DroneFlightController(
                drone=self.drone,
                controller=self.controller,
                logger=self.logger,
                velocity_control=True,
                target_velocity=np.zeros(3),
                target_angular_velocity=np.zeros(3)
            )
            
            if self.logger:
                self.logger.info("ドローンのセットアップ完了")
            else:
                print("ドローンのセットアップ完了")
                
        except Exception as e:
            error_msg = f"ドローンのセットアップに失敗: {e}"
            if self.logger:
                self.logger.error(error_msg)
            else:
                print(error_msg)
            raise
    
    def set_logger(self, logger):
        """ロガーの設定"""
        self.logger = logger
    
    def update_pid_parameters(self, pid_params):
        """
        PIDパラメータの更新
        
        Args:
            pid_params (dict): PIDパラメータの辞書
        """
        if not pid_params:
            return
            
        params = {}
        
        # 速度制御用PIDゲイン
        if 'p_gain_vel' in pid_params:
            params['P_COEFF_VEL'] = np.array(pid_params['p_gain_vel'])
        if 'i_gain_vel' in pid_params:
            params['I_COEFF_VEL'] = np.array(pid_params['i_gain_vel'])
        if 'd_gain_vel' in pid_params:
            params['D_COEFF_VEL'] = np.array(pid_params['d_gain_vel'])
        
        # 姿勢制御用PIDゲイン
        if 'p_gain_att' in pid_params:
            params['P_COEFF_TOR'] = np.array(pid_params['p_gain_att'])
        if 'i_gain_att' in pid_params:
            params['I_COEFF_TOR'] = np.array(pid_params['i_gain_att'])
        if 'd_gain_att' in pid_params:
            params['D_COEFF_TOR'] = np.array(pid_params['d_gain_att'])
        
        # コントローラにパラメータを設定
        self.controller.set_parameters(params)
        
        # 現在のパラメータをログに出力
        if self.logger:
            self.logger.info(f"PIDパラメータを更新: {params}")
        else:
            print(f"PIDパラメータを更新: {params}")
    
    def generate_velocity_profile(self, steps, target_velocity, profile='constant'):
        """
        速度プロファイルを生成
        
        Args:
            steps (int): シミュレーションステップ数
            target_velocity (np.ndarray): 目標速度
            profile (str): 速度プロファイルタイプ
            
        Returns:
            np.ndarray: 各ステップでの目標速度 (steps, 3)
        """
        profile_array = np.zeros((steps, 3))
        
        if profile == 'constant':
            # 一定速度
            profile_array = np.ones((steps, 3)) * target_velocity
            
        elif profile == 'ramp':
            # ランプ状速度
            ramp_steps = int(steps / 2)
            
            for i in range(steps):
                if i < ramp_steps:
                    factor = i / ramp_steps
                else:
                    factor = 1.0
                profile_array[i] = factor * target_velocity
            
        elif profile == 'sinusoidal':
            # 正弦波速度
            frequency = 0.2  # Hz
            
            for i in range(steps):
                t = i * self.dt
                factor = np.sin(2 * np.pi * frequency * t)
                profile_array[i] = factor * target_velocity
            
        elif profile == 'step':
            # ステップ状速度
            step_points = [0, int(steps/3), int(2*steps/3), steps]
            velocities = [
                np.zeros(3),
                target_velocity,
                -target_velocity,
                np.zeros(3)
            ]
            
            current_vel_idx = 0
            for i in range(steps):
                if current_vel_idx < len(step_points) - 1 and i >= step_points[current_vel_idx + 1]:
                    current_vel_idx += 1
                profile_array[i] = velocities[current_vel_idx]
        
        else:
            # デフォルトは一定速度
            profile_array = np.ones((steps, 3)) * target_velocity
        
        return profile_array
    
    def reset_drone_position(self, position=(0, 0, 0.5)):
        """ドローンを指定位置にリセットする
        
        Args:
            position (tuple): リセット位置 (x, y, z)
        """
        try:
            self.drone.set_pos(position)
            if self.logger:
                self.logger.info(f"Drone position reset to {position}")
            else:
                print(f"Drone position reset to {position}")
        except Exception as e:
            error_msg = f"Failed to reset drone position: {e}"
            if self.logger:
                self.logger.error(error_msg)
            else:
                print(error_msg)

    def evaluate_controller(self, steps, target_velocity, pid_params=None, profile='constant'):
        """
        コントローラの応答を評価
        
        Args:
            steps (int): シミュレーションステップ数
            target_velocity (np.ndarray): 目標速度
            pid_params (dict, optional): PIDパラメータの辞書
            profile (str, optional): 速度プロファイルタイプ
            
        Returns:
            dict: 評価結果の辞書
        """
        # ドローンを初期位置にリセット
        self.reset_drone_position()
        
        # PIDパラメータの更新（指定されている場合）
        if pid_params:
            self.update_pid_parameters(pid_params)
        
        # ドローンを初期位置に移動（DroneEntityのAPIに合わせる）
        # 注意: DroneEntityクラスにはset_posやset_velメソッドがないため、
        # 代わりにget_posやget_velメソッドを使用して現在の状態を取得し、
        # コントローラを通じて制御する
        
        # コントローラのリセット
        self.flight_controller.reset()
        self.flight_controller.start_route()
        
        # 速度プロファイルの生成
        velocity_profile = self.generate_velocity_profile(steps, target_velocity, profile)
        
        # データ記録用の配列を初期化
        time_data = np.arange(steps) * self.dt
        target_vel_data = np.zeros((steps, 3))
        actual_vel_data = np.zeros((steps, 3))
        error_data = np.zeros((steps, 3))
        
        if self.logger:
            self.logger.info(f"評価開始: {steps}ステップ, プロファイル={profile}")
        else:
            print(f"評価開始: {steps}ステップ, プロファイル={profile}")
        
        # シミュレーションの実行
        for i in range(steps):
            # 目標速度の設定
            target_vel = velocity_profile[i]
            target_vel_data[i] = target_vel
            
            # コントローラに目標速度を設定
            self.flight_controller.set_velocity_target(target_vel, np.zeros(3))
            
            # シミュレーションステップの実行
            self.simulation_step()
            
            # 実際の速度を記録
            actual_vel = self.drone.get_vel().cpu().numpy()
            actual_vel_data[i] = actual_vel
            
            # 誤差を計算
            error_data[i] = target_vel - actual_vel
            
            # 進捗表示（10%ごと）
            if i % (steps // 10) == 0:
                if self.logger:
                    self.logger.info(f"進捗: {i / steps * 100:.1f}%")
                else:
                    print(f"進捗: {i / steps * 100:.1f}%")
        
        if self.logger:
            self.logger.info("評価完了")
        else:
            print("評価完了")
        
        # 結果の分析
        # 平均二乗誤差（MSE）の計算
        mse = np.mean(np.square(error_data), axis=0)
        rmse = np.sqrt(mse)
        
        # 最大誤差の計算
        max_error = np.max(np.abs(error_data), axis=0)
        
        # 平均誤差の計算
        mean_error = np.mean(np.abs(error_data), axis=0)
        
        # 総合評価スコア（RMSEの平均）
        overall_score = np.mean(rmse)
        
        # 結果の表示
        if self.logger:
            self.logger.info("===== 速度追従性能分析 =====")
            self.logger.info(f"RMSE (X,Y,Z): {rmse}")
            self.logger.info(f"最大誤差 (X,Y,Z): {max_error}")
            self.logger.info(f"平均誤差 (X,Y,Z): {mean_error}")
            self.logger.info(f"総合評価スコア: {overall_score}")
        else:
            print("===== 速度追従性能分析 =====")
            print(f"RMSE (X,Y,Z): {rmse}")
            print(f"最大誤差 (X,Y,Z): {max_error}")
            print(f"平均誤差 (X,Y,Z): {mean_error}")
            print(f"総合評価スコア: {overall_score}")
        
        # 評価結果を返す
        return {
            'time_data': time_data,
            'target_vel_data': target_vel_data,
            'actual_vel_data': actual_vel_data,
            'error_data': error_data,
            'rmse': rmse,
            'max_error': max_error,
            'mean_error': mean_error,
            'overall_score': overall_score
        }
    
    def simulation_step(self):
        """シミュレーションステップの実行"""
        # 飛行コントローラの更新
        self.flight_controller.update()
        
        # シーンのステップを進める
        self.scene.step()
    
    def shutdown(self):
        """シミュレーションの終了処理"""
        if self.logger:
            self.logger.info("シミュレーションの終了処理を実行")
        else:
            print("シミュレーションの終了処理を実行")
