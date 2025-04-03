#!/usr/bin/env python3

import rclpy
import numpy as np
import matplotlib.pyplot as plt
import argparse
import time
import itertools
import os
from rclpy.node import Node
from rclpy.parameter import Parameter

# 新しいシミュレータをインポート
from genesis_drones.simulation.controller_response_simulator import ControllerResponseSimulator
from genesis_drones.controllers.dsl_pid_controller import DSLPIDController

# PlotManagerとParameterOptimizerをインポート
from genesis_drones.utils.plot_manager import PlotManager
from genesis_drones.utils.parameter_optimizer import AStarParameterOptimizer, GridSearchOptimizer

class VelocityTrackingTest(Node):
    """ドローンの速度追従性能をテストするクラス"""
    
    def __init__(self, pid_params=None, steps=1000, profile='constant', 
                 target_velocity=None, plot_output=False, optimize=False,
                 vel_only=False, iterations=10, grid_size=3,
                 optimization_method='astar',
                 save_dir="/home/initial/colcon_ws/src/genesis_drones/data"):
        """
        VelocityTrackingTestの初期化
        
        Args:
            pid_params: PIDパラメータの辞書
            steps: シミュレーションステップ数
            profile: 速度プロファイルタイプ
            target_velocity: 目標速度
            plot_output: プロット出力ファイルパス
            optimize: パラメータ最適化を行うかどうか
            vel_only: 速度制御パラメータのみを最適化するかどうか
            iterations: 最適化の反復回数
            grid_size: グリッドサーチの粗さ（各パラメータの分割数）
            optimization_method: 最適化手法（'astar'または'grid'）
            save_dir: プロット保存ディレクトリ
        """
        super().__init__('velocity_tracking_test')
        
        # パラメータ設定
        self.pid_params = pid_params or {}
        self.steps = steps
        self.profile = profile
        self.target_velocity = target_velocity if target_velocity is not None else np.array([0.5, 0.0, 0.0])
        self.plot_output = plot_output
        self.dt = 0.01  # シミュレーションの時間ステップ
        self.optimize = optimize
        self.vel_only = vel_only  # 速度制御パラメータのみを最適化するかどうか
        self.iterations = iterations
        self.grid_size = grid_size  # グリッドサーチの粗さ
        self.optimization_method = optimization_method  # 最適化手法
        
        # プロット管理クラスの初期化
        self.plot_manager = PlotManager(save_dir)
        
        # データ記録用バッファ
        self.time_data = []
        self.target_vel_data = []
        self.actual_vel_data = []
        self.error_data = []
        
        # 分析結果
        self.analysis_results = {}
        
        # シミュレーション環境のセットアップ
        self.setup_simulation()
    
    def setup_simulation(self):
        """シミュレーション環境のセットアップ"""
        # 新しいシミュレータの作成
        self.simulator = ControllerResponseSimulator(dt=self.dt)
        
        # ロガーの設定
        self.simulator.set_logger(self.get_logger())
        
        # PIDパラメータの設定（指定されている場合）
        if self.pid_params:
            self.simulator.update_pid_parameters(self.pid_params)
        
        self.get_logger().info("シミュレーション環境のセットアップ完了")
    
    def run_test(self):
        """テストを実行し、データを記録"""
        self.get_logger().info(f"テスト開始: {self.steps}ステップ, プロファイル={self.profile}")
        
        # 最適化モードの場合
        if self.optimize:
            self.get_logger().info(f"パラメータ最適化モード: {self.iterations}回の反復")
            if self.vel_only:
                self.get_logger().info("速度制御パラメータのみを最適化します")
            best_params, best_score = self.optimize_parameters()
            
            # 最適なパラメータでテストを実行
            self.pid_params = best_params
            self.simulator.update_pid_parameters(self.pid_params)
            
            self.get_logger().info(f"最適なパラメータ: {best_params}")
            self.get_logger().info(f"最適なスコア: {best_score}")
            
            # 最適化結果のプロット
            self.plot_optimization_results()
    
    def analyze_results(self):
        """結果を分析"""
        # 結果の表示
        self.get_logger().info("===== 速度追従性能分析 =====")
        self.get_logger().info(f"RMSE (X,Y,Z): {self.analysis_results['rmse']}")
        self.get_logger().info(f"最大誤差 (X,Y,Z): {self.analysis_results['max_error']}")
        self.get_logger().info(f"平均誤差 (X,Y,Z): {self.analysis_results['mean_error']}")
        self.get_logger().info(f"総合評価スコア: {self.analysis_results['overall_score']}")
        
        # 使用したPIDパラメータを表示
        self.get_logger().info("===== 使用したPIDパラメータ =====")
        for key, value in self.pid_params.items():
            self.get_logger().info(f"{key}: {value}")
    
    def plot_results(self):
        """結果をプロット"""
        # 速度追従プロット
        self.plot_manager.plot_velocity_tracking(
            self.time_data,
            self.target_vel_data,
            self.actual_vel_data,
            self.analysis_results,
            self.profile,
            optimize=self.optimize,
            pid_params=self.pid_params,
            plot_output=self.plot_output,
            logger=self.get_logger()
        )
        
        # エラープロット
        self.plot_manager.plot_error(
            self.time_data,
            self.error_data,
            pid_params=self.pid_params,
            plot_output=self.plot_output,
            logger=self.get_logger()
        )
    
    def plot_optimization_results(self):
        """最適化結果をプロット"""
        if not hasattr(self, 'optimization_history'):
            return
            
        params = np.array(self.optimization_history['params'])
        scores = np.array(self.optimization_history['scores'])
        
        # パラメータとスコアの関係プロット
        self.plot_manager.plot_parameter_performance(
            params,
            scores,
            pid_params=self.pid_params,
            plot_output=self.plot_output,
            logger=self.get_logger()
        )
        
        # 最適化過程の収束プロット
        self.plot_manager.plot_convergence(
            scores,
            pid_params=self.pid_params,
            plot_output=self.plot_output,
            logger=self.get_logger()
        )
    
    def optimize_parameters(self):
        """
        PIDパラメータの最適化
        
        Returns:
            tuple: (最適なパラメータ, 最適なスコア)
        """
        self.get_logger().info("パラメータ最適化を開始")
        
        # 評価カウンタの初期化
        self.eval_counter = 0
        
        # DSLPIDControllerからデフォルト値を取得
        # 速度制御用PIDゲイン
        default_p_vel = 0.07
        default_i_vel = 0.05
        default_d_vel = 0.002
        
        # 姿勢制御用PIDゲイン
        default_p_att = 50000.0
        default_i_att = 500.0
        default_d_att = 100.0
        
        # パラメータの範囲を定義（デフォルト値を中心に）
        param_ranges = {
            # 速度制御用PIDゲイン
            'p_gain_vel': np.linspace(default_p_vel * 0.5, default_p_vel * 2.0, self.grid_size),  # P制御ゲイン
            'i_gain_vel': np.linspace(default_i_vel * 0.5, default_i_vel * 2.0, self.grid_size),  # I制御ゲイン
            'd_gain_vel': np.linspace(default_d_vel * 0.5, default_d_vel * 2.0, self.grid_size),   # D制御ゲイン
        }
        
        # 速度制御パラメータのみを最適化する場合は、姿勢制御パラメータを固定値として使用
        if not self.vel_only:
            # 姿勢制御用PIDゲインの範囲を追加
            # 姿勢制御パラメータは計算量が多くなるため、グリッドサイズを制限
            att_grid_size = min(self.grid_size, 3)
            param_ranges.update({
                'p_gain_att': np.linspace(default_p_att * 0.5, default_p_att * 1.5, att_grid_size),  # P制御ゲイン
                'i_gain_att': np.linspace(default_i_att * 0.5, default_i_att * 1.5, att_grid_size),  # I制御ゲイン
                'd_gain_att': np.linspace(default_d_att * 0.5, default_d_att * 1.5, att_grid_size)   # D制御ゲイン
            })
        
        # 評価関数の定義
        def evaluate_params(params):
            # パラメータを適切な形式に変換
            pid_params = {}
            
            # 速度制御パラメータ
            pid_params['p_gain_vel'] = [params['p_gain_vel']] * 3
            pid_params['i_gain_vel'] = [params['i_gain_vel']] * 3
            pid_params['d_gain_vel'] = [params['d_gain_vel']] * 3
            
            # 姿勢制御パラメータ（存在する場合）
            if 'p_gain_att' in params:
                pid_params['p_gain_att'] = [params['p_gain_att']] * 3
                pid_params['i_gain_att'] = [params['i_gain_att']] * 3
                pid_params['d_gain_att'] = [params['d_gain_att']] * 3
            else:
                # 姿勢制御パラメータが指定されていない場合はデフォルト値を使用
                pid_params['p_gain_att'] = [default_p_att] * 3
                pid_params['i_gain_att'] = [default_i_att] * 3
                pid_params['d_gain_att'] = [default_d_att] * 3
            
            # 評価用のステップ数を減らして高速化
            eval_steps = min(self.steps, 200)
            
            # シミュレーションの実行
            results = self.simulator.evaluate_controller(
                steps=eval_steps,
                target_velocity=self.target_velocity,
                pid_params=pid_params,
                profile=self.profile
            )
            
            # スコアの計算（RMSEの平均）
            score = results['overall_score']
            
            # 結果を記録
            if not hasattr(self, 'evaluation_results'):
                self.evaluation_results = []
            
            self.evaluation_results.append({
                'params': params,
                'pid_params': pid_params,
                'score': score,
                'results': results
            })
            
            # 進捗表示
            if hasattr(self, 'eval_counter'):
                self.eval_counter += 1
            else:
                self.eval_counter = 1
            
            self.get_logger().info(f"評価中 ({self.eval_counter}): "
                                  f"速度制御 P={params['p_gain_vel']:.3f}, I={params['i_gain_vel']:.3f}, D={params['d_gain_vel']:.3f}, "
                                  f"スコア: {score:.4f}")
            
            return score
        
        # ヒューリスティック関数の定義（A*用）
        def heuristic(state):
            # 単純なヒューリスティック：最適値からの距離
            # 実際には問題に応じて適切なヒューリスティックを設計する必要がある
            return 0.0
        
        # 最適化アルゴリズムの選択
        if self.optimization_method == 'astar':
            self.get_logger().info("A*アルゴリズムによるパラメータ最適化を開始")
            
            # A*オプティマイザの作成
            optimizer = AStarParameterOptimizer(
                param_ranges=param_ranges,
                evaluate_func=evaluate_params,
                heuristic_func=heuristic,
                logger=self.get_logger()
            )
            
            # 最適化の進行状況をリアルタイムで可視化するコールバック関数
            def optimization_callback(iteration, current_state, current_score, search_history, best_state, best_score):
                try:
                    # 現在のパラメータを取得
                    current_params = optimizer._state_to_params(current_state)
                    
                    # 最新の評価結果を取得
                    if hasattr(self, 'evaluation_results') and len(self.evaluation_results) > 0:
                        latest_result = self.evaluation_results[-1]
                        
                        # 現在の応答データを取得
                        current_response_data = latest_result['results']
                        
                        # 最適化の進行状況をプロット
                        self.get_logger().info(f"最適化進行状況をプロット: 反復回数={iteration}, スコア={current_score:.4f}")
                        self.plot_manager.plot_optimization_progress(
                            search_history=search_history,
                            param_ranges=param_ranges,
                            current_params=current_params,
                            current_response_data=current_response_data,
                            optimization_method=self.optimization_method.upper(),
                            iteration=iteration,
                            plot_output=self.plot_output,
                            logger=self.get_logger()
                        )
                except Exception as e:
                    self.get_logger().error(f"最適化進行状況のプロット中にエラーが発生: {e}")
            
            # A*による最適化
            best_params, best_score, search_history = optimizer.optimize(
                max_iterations=self.iterations,
                callback=optimization_callback
            )
            
            # 探索履歴を保存
            self.optimization_history = search_history
            
        else:  # グリッドサーチ
            self.get_logger().info("グリッドサーチによるパラメータ最適化を開始")
            
            # グリッドサーチオプティマイザの作成
            optimizer = GridSearchOptimizer(
                param_ranges=param_ranges,
                evaluate_func=evaluate_params,
                logger=self.get_logger()
            )
            
            # 最適化の進行状況をリアルタイムで可視化するコールバック関数
            def optimization_callback(iteration, current_state, current_score, search_history, best_state, best_score):
                try:
                    # 現在のパラメータを取得
                    current_params = optimizer._state_to_params(current_state)
                    
                    # 最新の評価結果を取得
                    if hasattr(self, 'evaluation_results') and len(self.evaluation_results) > 0:
                        latest_result = self.evaluation_results[-1]
                        
                        # 現在の応答データを取得
                        current_response_data = latest_result['results']
                        
                        # 最適化の進行状況をプロット
                        self.get_logger().info(f"最適化進行状況をプロット: 反復回数={iteration}, スコア={current_score:.4f}")
                        self.plot_manager.plot_optimization_progress(
                            search_history=search_history,
                            param_ranges=param_ranges,
                            current_params=current_params,
                            current_response_data=current_response_data,
                            optimization_method=self.optimization_method.upper(),
                            iteration=iteration,
                            plot_output=self.plot_output,
                            logger=self.get_logger()
                        )
                except Exception as e:
                    self.get_logger().error(f"最適化進行状況のプロット中にエラーが発生: {e}")
            
            # グリッドサーチによる最適化
            best_params, best_score, search_history = optimizer.optimize(
                max_iterations=self.iterations,
                callback=optimization_callback
            )
            
            # 探索履歴を保存
            self.optimization_history = search_history
        
        # 評価結果からデータを抽出
        self.optimization_history['params'] = []
        self.optimization_history['scores'] = []
        self.optimization_history['responses'] = []
        self.optimization_history['time_data'] = []
        self.optimization_history['target_vel_data'] = []
        self.optimization_history['error_data'] = []
        
        for result in self.evaluation_results:
            params = result['params']
            
            # パラメータを配列に変換
            param_array = [
                params['p_gain_vel'],
                params['i_gain_vel'],
                params['d_gain_vel']
            ]
            
            if 'p_gain_att' in params:
                param_array.extend([
                    params['p_gain_att'],
                    params['i_gain_att'],
                    params['d_gain_att']
                ])
            else:
                param_array.extend([
                    default_p_att,
                    default_i_att,
                    default_d_att
                ])
            
            self.optimization_history['params'].append(param_array)
            self.optimization_history['scores'].append(result['score'])
            self.optimization_history['responses'].append(result['results']['actual_vel_data'])
            self.optimization_history['time_data'].append(result['results']['time_data'])
            self.optimization_history['target_vel_data'].append(result['results']['target_vel_data'])
            self.optimization_history['error_data'].append(result['results']['error_data'])
        
        # 最終的な結果のみをプロット（各試行の結果は最適化中にリアルタイムでプロットされる）
        
        # A*探索の可視化
        if self.plot_output:
            self.plot_manager.plot_astar_search(
                search_history,
                param_ranges,
                best_params,
                optimization_method=self.optimization_method.upper(),
                plot_output=True,
                logger=self.get_logger()
            )
        
        # 最適なパラメータを適切な形式に変換
        best_pid_params = {}
        
        # 速度制御パラメータ
        best_pid_params['p_gain_vel'] = [best_params['p_gain_vel']] * 3
        best_pid_params['i_gain_vel'] = [best_params['i_gain_vel']] * 3
        best_pid_params['d_gain_vel'] = [best_params['d_gain_vel']] * 3
        
        # 姿勢制御パラメータ（存在する場合）
        if 'p_gain_att' in best_params:
            best_pid_params['p_gain_att'] = [best_params['p_gain_att']] * 3
            best_pid_params['i_gain_att'] = [best_params['i_gain_att']] * 3
            best_pid_params['d_gain_att'] = [best_params['d_gain_att']] * 3
        else:
            # 姿勢制御パラメータが指定されていない場合はデフォルト値を使用
            best_pid_params['p_gain_att'] = [default_p_att] * 3
            best_pid_params['i_gain_att'] = [default_i_att] * 3
            best_pid_params['d_gain_att'] = [default_d_att] * 3
        
        self.get_logger().info("パラメータ最適化完了")
        self.get_logger().info(f"最適なパラメータ: {best_params}")
        self.get_logger().info(f"最適なスコア: {best_score}")
        
        return best_pid_params, best_score
    
    def shutdown(self):
        """シャットダウン処理"""
        if hasattr(self, 'simulator'):
            self.simulator.shutdown()
        self.get_logger().info("シャットダウン完了")


def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(description='ドローン速度追従性能テスト')
    parser.add_argument('--steps', type=int, default=500, help='シミュレーションステップ数')
    parser.add_argument('--profile', type=str, default='constant', 
                        choices=['constant', 'ramp', 'sinusoidal', 'step'],
                        help='速度プロファイル')
    parser.add_argument('--target_vel', type=str, default='2.0,0.0,0.0', 
                        help='目標速度 (x,y,z)')
    parser.add_argument('--output', action='store_true',
                        help='出力ファイルを保存')
    parser.add_argument('--p_gain_vel', type=str, default='0.07,0.07,0.07',
                        help='速度P制御ゲイン')
    parser.add_argument('--i_gain_vel', type=str, default='0.05,0.05,0.05',
                        help='速度I制御ゲイン')
    parser.add_argument('--d_gain_vel', type=str, default='0.002,0.002,0.002',
                        help='速度D制御ゲイン')
    parser.add_argument('--optimize', action='store_true',
                        help='パラメータ最適化を行う')
    parser.add_argument('--vel_only', action='store_true',
                        help='速度制御パラメータのみを最適化する（姿勢制御パラメータは固定）')
    parser.add_argument('--iterations', type=int, default=2,
                        help='最適化の反復回数')
    parser.add_argument('--grid_size', type=int, default=3,
                        help='グリッドサーチの粗さ（各パラメータの分割数）')
    parser.add_argument('--optimization_method', type=str, default='astar',
                        choices=['astar', 'grid'],
                        help='最適化手法（astarまたはgrid）')
    args = parser.parse_args()
    
    # ROS 2の初期化
    rclpy.init()
    
    # 目標速度の解析
    target_velocity = np.array([float(x) for x in args.target_vel.split(',')])
    
    # PIDパラメータの解析
    pid_params = {
        'p_gain_vel': [float(x) for x in args.p_gain_vel.split(',')],
        'i_gain_vel': [float(x) for x in args.i_gain_vel.split(',')],
        'd_gain_vel': [float(x) for x in args.d_gain_vel.split(',')]
    }
    
    # テストの実行
    test = VelocityTrackingTest(
        pid_params=pid_params,
        steps=args.steps,
        profile=args.profile,
        target_velocity=target_velocity,
        plot_output=args.output,
        optimize=args.optimize,
        vel_only=args.vel_only,
        iterations=args.iterations,
        grid_size=args.grid_size,
        optimization_method=args.optimization_method
    )
    
    try:
        test.run_test()
    except KeyboardInterrupt:
        print("ユーザーによる中断")
    except Exception as e:
        print(f"エラー発生: {e}")
    finally:
        # クリーンアップ
        test.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
