#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime
import matplotlib.colors as mcolors
from matplotlib.patches import Patch
from typing import Dict, List, Tuple, Any, Optional

class PlotManager:
    """プロット機能を管理するクラス"""
    
    def __init__(self, save_dir="/home/initial/colcon_ws/src/genesis_drones/data"):
        """
        プロット管理クラスの初期化
        
        Args:
            save_dir: プロット保存ディレクトリ
        """
        date_dir = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_dir = os.path.join(save_dir, date_dir)
        os.makedirs(self.save_dir, exist_ok=True)
        
        # カラーマップの設定
        self.cmap = plt.cm.viridis
        
        # 最適化進行状況のプロット用のファイル名
        self.response_plot_filename = "optimization_response.png"
        self.grid_search_plot_filename = "optimization_grid_search.png"
    
    def get_save_path(self, filename):
        """保存パスを取得"""
        return os.path.join(self.save_dir, filename)
    
    def save_plot(self, fig, filename, logger=None):
        """プロットを保存"""
        save_path = self.get_save_path(filename)
        fig.savefig(save_path)
        if logger:
            logger.info(f"Plot saved to: {save_path}")
        return save_path
    
    def plot_velocity_tracking(self, time_data, target_vel_data, actual_vel_data, 
                              analysis_results, profile, optimize=False, pid_params=None, 
                              plot_output=False, logger=None):
        """速度追従性能のプロット"""
        fig = plt.figure(figsize=(15, 18))
        
        # 基本プロット - X軸
        ax1 = plt.subplot(3, 2, 1)
        ax1.plot(time_data, target_vel_data[:, 0], 'r-', label='Target Vx')
        ax1.plot(time_data, actual_vel_data[:, 0], 'b-', label='Actual Vx')
        ax1.set_xlabel('Time [s]')
        ax1.set_ylabel('Velocity X [m/s]')
        ax1.set_title(f'X-axis Velocity Tracking (RMSE: {analysis_results["rmse"][0]:.4f})')
        ax1.legend()
        ax1.grid(True)
        
        # 基本プロット - Y軸
        ax2 = plt.subplot(3, 2, 3)
        ax2.plot(time_data, target_vel_data[:, 1], 'r-', label='Target Vy')
        ax2.plot(time_data, actual_vel_data[:, 1], 'b-', label='Actual Vy')
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Velocity Y [m/s]')
        ax2.set_title(f'Y-axis Velocity Tracking (RMSE: {analysis_results["rmse"][1]:.4f})')
        ax2.legend()
        ax2.grid(True)
        
        # 基本プロット - Z軸
        ax3 = plt.subplot(3, 2, 5)
        ax3.plot(time_data, target_vel_data[:, 2], 'r-', label='Target Vz')
        ax3.plot(time_data, actual_vel_data[:, 2], 'b-', label='Actual Vz')
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Velocity Z [m/s]')
        ax3.set_title(f'Z-axis Velocity Tracking (RMSE: {analysis_results["rmse"][2]:.4f})')
        ax3.legend()
        ax3.grid(True)
        
        # タイトルの追加
        plt_title = f'Drone Velocity Tracking Performance (Profile: {profile})'
        if optimize:
            plt_title += ' [Optimized]'
        plt.suptitle(plt_title, fontsize=16)
        plt.tight_layout()
        plt.subplots_adjust(top=0.92)
        
        # 保存または表示
        if plot_output and pid_params:
            # 速度制御パラメータ
            p_vel = pid_params['p_gain_vel'][0]
            i_vel = pid_params['i_gain_vel'][0]
            d_vel = pid_params['d_gain_vel'][0]
            
            # 姿勢制御パラメータ（存在する場合）
            if 'p_gain_att' in pid_params:
                p_att = pid_params['p_gain_att'][0]
                i_att = pid_params['i_gain_att'][0]
                d_att = pid_params['d_gain_att'][0]
                filename = f'response_vel_p{p_vel:.3f}_i{i_vel:.3f}_d{d_vel:.3f}_att_p{p_att:.1f}_i{i_att:.1f}_d{d_att:.1f}.png'
            else:
                filename = f'response_p{p_vel:.3f}_i{i_vel:.3f}_d{d_vel:.3f}.png'
            
            self.save_plot(fig, filename, logger)
        
        return fig
    
    def plot_error(self, time_data, error_data, pid_params=None, plot_output=False, logger=None):
        """エラープロット"""
        fig = plt.figure(figsize=(10, 8))
        for i, axis in enumerate(['X', 'Y', 'Z']):
            plt.plot(time_data, error_data[:, i], label=f'{axis} Error')
        plt.xlabel('Time [s]')
        plt.ylabel('Error [m/s]')
        plt.title('Velocity Tracking Error')
        plt.legend()
        plt.grid(True)
        
        if plot_output and pid_params:
            # 速度制御パラメータ
            p_vel = pid_params['p_gain_vel'][0]
            i_vel = pid_params['i_gain_vel'][0]
            d_vel = pid_params['d_gain_vel'][0]
            
            # 姿勢制御パラメータ（存在する場合）
            if 'p_gain_att' in pid_params:
                p_att = pid_params['p_gain_att'][0]
                i_att = pid_params['i_gain_att'][0]
                d_att = pid_params['d_gain_att'][0]
                filename = f'error_vel_p{p_vel:.3f}_i{i_vel:.3f}_d{d_vel:.3f}_att_p{p_att:.1f}_i{i_att:.1f}_d{d_att:.1f}.png'
            else:
                filename = f'error_p{p_vel:.3f}_i{i_vel:.3f}_d{d_vel:.3f}.png'
            
            self.save_plot(fig, filename, logger)
        
        return fig
    
    def plot_parameter_performance(self, params, scores, pid_params=None, plot_output=False, logger=None):
        """パラメータとスコアの関係プロット"""
        # 速度制御パラメータのプロット
        fig_vel = plt.figure(figsize=(15, 5))
        fig_vel.suptitle('速度制御パラメータと性能の関係', fontsize=16)
        
        # 速度制御 Pゲイン vs スコア
        ax1 = plt.subplot(1, 3, 1)
        scatter = ax1.scatter(params[:, 0], scores, c=scores, cmap=self.cmap)
        ax1.set_xlabel('P Gain (Velocity)')
        ax1.set_ylabel('Score (RMSE)')
        ax1.set_title('P Gain vs Performance')
        ax1.grid(True)
        plt.colorbar(scatter, ax=ax1, label='Score (RMSE)')
        
        # 速度制御 Iゲイン vs スコア
        ax2 = plt.subplot(1, 3, 2)
        scatter = ax2.scatter(params[:, 1], scores, c=scores, cmap=self.cmap)
        ax2.set_xlabel('I Gain (Velocity)')
        ax2.set_ylabel('Score (RMSE)')
        ax2.set_title('I Gain vs Performance')
        ax2.grid(True)
        plt.colorbar(scatter, ax=ax2, label='Score (RMSE)')
        
        # 速度制御 Dゲイン vs スコア
        ax3 = plt.subplot(1, 3, 3)
        scatter = ax3.scatter(params[:, 2], scores, c=scores, cmap=self.cmap)
        ax3.set_xlabel('D Gain (Velocity)')
        ax3.set_ylabel('Score (RMSE)')
        ax3.set_title('D Gain vs Performance')
        ax3.grid(True)
        plt.colorbar(scatter, ax=ax3, label='Score (RMSE)')
        
        plt.tight_layout()
        plt.subplots_adjust(top=0.88)
        
        # 姿勢制御パラメータのプロット（パラメータが6次元の場合のみ）
        if params.shape[1] >= 6:
            fig_att = plt.figure(figsize=(15, 5))
            fig_att.suptitle('姿勢制御パラメータと性能の関係', fontsize=16)
            
            # 姿勢制御 Pゲイン vs スコア
            ax4 = plt.subplot(1, 3, 1)
            scatter = ax4.scatter(params[:, 3], scores, c=scores, cmap=self.cmap)
            ax4.set_xlabel('P Gain (Attitude)')
            ax4.set_ylabel('Score (RMSE)')
            ax4.set_title('P Gain vs Performance')
            ax4.grid(True)
            plt.colorbar(scatter, ax=ax4, label='Score (RMSE)')
            
            # 姿勢制御 Iゲイン vs スコア
            ax5 = plt.subplot(1, 3, 2)
            scatter = ax5.scatter(params[:, 4], scores, c=scores, cmap=self.cmap)
            ax5.set_xlabel('I Gain (Attitude)')
            ax5.set_ylabel('Score (RMSE)')
            ax5.set_title('I Gain vs Performance')
            ax5.grid(True)
            plt.colorbar(scatter, ax=ax5, label='Score (RMSE)')
            
            # 姿勢制御 Dゲイン vs スコア
            ax6 = plt.subplot(1, 3, 3)
            scatter = ax6.scatter(params[:, 5], scores, c=scores, cmap=self.cmap)
            ax6.set_xlabel('D Gain (Attitude)')
            ax6.set_ylabel('Score (RMSE)')
            ax6.set_title('D Gain vs Performance')
            ax6.grid(True)
            plt.colorbar(scatter, ax=ax6, label='Score (RMSE)')
            
            plt.tight_layout()
            plt.subplots_adjust(top=0.88)
        else:
            fig_att = None
        
        # ファイル保存
        if plot_output and pid_params:
            # 速度制御パラメータ
            best_p_vel = pid_params['p_gain_vel'][0]
            best_i_vel = pid_params['i_gain_vel'][0]
            best_d_vel = pid_params['d_gain_vel'][0]
            
            # 姿勢制御パラメータ（存在する場合）
            if 'p_gain_att' in pid_params:
                best_p_att = pid_params['p_gain_att'][0]
                best_i_att = pid_params['i_gain_att'][0]
                best_d_att = pid_params['d_gain_att'][0]
                filename = f'parameter_optimization_vel_p{best_p_vel:.3f}_i{best_i_vel:.3f}_d{best_d_vel:.3f}_att_p{best_p_att:.1f}_i{best_i_att:.1f}_d{best_d_att:.1f}.png'
            else:
                filename = f'parameter_optimization_p{best_p_vel:.3f}_i{best_i_vel:.3f}_d{best_d_vel:.3f}.png'
            
            self.save_plot(fig_vel, filename, logger)
            
            # 姿勢制御パラメータのプロットが存在する場合は保存
            if fig_att is not None:
                filename = f'parameter_optimization_attitude_vel_p{best_p_vel:.3f}_i{best_i_vel:.3f}_d{best_d_vel:.3f}_att_p{best_p_att:.1f}_i{best_i_att:.1f}_d{best_d_att:.1f}.png'
                self.save_plot(fig_att, filename, logger)
        
        return fig_vel
    
    def plot_convergence(self, scores, pid_params=None, plot_output=False, logger=None):
        """最適化過程の収束プロット"""
        fig = plt.figure(figsize=(10, 6))
        plt.plot(scores, 'o-')
        best_idx = np.argmin(scores)
        plt.plot(best_idx, scores[best_idx], 'ro', markersize=10)
        plt.xlabel('Iteration')
        plt.ylabel('Score (RMSE)')
        plt.title('Optimization Convergence')
        plt.grid(True)
        
        if plot_output and pid_params:
            # 速度制御パラメータ
            best_p_vel = pid_params['p_gain_vel'][0]
            best_i_vel = pid_params['i_gain_vel'][0]
            best_d_vel = pid_params['d_gain_vel'][0]
            
            # 姿勢制御パラメータ（存在する場合）
            if 'p_gain_att' in pid_params:
                best_p_att = pid_params['p_gain_att'][0]
                best_i_att = pid_params['i_gain_att'][0]
                best_d_att = pid_params['d_gain_att'][0]
                filename = f'convergence_vel_p{best_p_vel:.3f}_i{best_i_vel:.3f}_d{best_d_vel:.3f}_att_p{best_p_att:.1f}_i{best_i_att:.1f}_d{best_d_att:.1f}.png'
            else:
                filename = f'convergence_p{best_p_vel:.3f}_i{best_i_vel:.3f}_d{best_d_vel:.3f}.png'
            
            self.save_plot(fig, filename, logger)
        
        return fig
    
    def plot_optimization_progress(self, search_history, param_ranges, current_params, 
                                  current_response_data, optimization_method="A*", 
                                  iteration=0, plot_output=True, logger=None):
        """
        最適化の進行状況をリアルタイムでプロット
        
        Args:
            search_history: 探索履歴
            param_ranges: パラメータの範囲
            current_params: 現在のパラメータ
            current_response_data: 現在の応答データ（時間、目標速度、実際の速度、誤差）
            optimization_method: 最適化手法の名前
            iteration: 現在の反復回数
            plot_output: プロット出力フラグ
            logger: ロガー
            
        Returns:
            tuple: (応答プロット, グリッドサーチプロット)
        """
        if logger:
            logger.info(f"最適化進行状況をプロット: 反復回数={iteration}")
        # 1. 目標速度に対する応答のプロット
        time_data = current_response_data.get('time_data', [])
        target_vel_data = current_response_data.get('target_vel_data', [])
        actual_vel_data = current_response_data.get('actual_vel_data', [])
        error_data = current_response_data.get('error_data', [])
        score = current_response_data.get('overall_score', 0.0)
        
        # 速度制御パラメータ
        p_vel = current_params.get('p_gain_vel', 0.0)
        i_vel = current_params.get('i_gain_vel', 0.0)
        d_vel = current_params.get('d_gain_vel', 0.0)
        
        # 姿勢制御パラメータ（存在する場合）
        p_att = current_params.get('p_gain_att', None)
        i_att = current_params.get('i_gain_att', None)
        d_att = current_params.get('d_gain_att', None)
        
        # 応答プロットの作成
        fig_response = plt.figure(figsize=(15, 10))
        
        # 速度追従プロット - X軸
        ax1 = plt.subplot(3, 1, 1)
        ax1.plot(time_data, target_vel_data[:, 0], 'r-', label='Target Vx')
        ax1.plot(time_data, actual_vel_data[:, 0], 'b-', label='Actual Vx')
        ax1.set_xlabel('Time [s]')
        ax1.set_ylabel('Velocity X [m/s]')
        ax1.set_title(f'X-axis Velocity Tracking')
        ax1.legend()
        ax1.grid(True)
        
        # 速度追従プロット - Y軸
        ax2 = plt.subplot(3, 1, 2)
        ax2.plot(time_data, target_vel_data[:, 1], 'r-', label='Target Vy')
        ax2.plot(time_data, actual_vel_data[:, 1], 'b-', label='Actual Vy')
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Velocity Y [m/s]')
        ax2.set_title(f'Y-axis Velocity Tracking')
        ax2.legend()
        ax2.grid(True)
        
        # 速度追従プロット - Z軸
        ax3 = plt.subplot(3, 1, 3)
        ax3.plot(time_data, target_vel_data[:, 2], 'r-', label='Target Vz')
        ax3.plot(time_data, actual_vel_data[:, 2], 'b-', label='Actual Vz')
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Velocity Z [m/s]')
        ax3.set_title(f'Z-axis Velocity Tracking')
        ax3.legend()
        ax3.grid(True)
        
        # タイトルの追加
        plt_title = f'Iteration {iteration}: Vel(P={p_vel:.3f}, I={i_vel:.3f}, D={d_vel:.3f})'
        if p_att is not None and i_att is not None and d_att is not None:
            plt_title += f', Att(P={p_att:.1f}, I={i_att:.1f}, D={d_att:.1f})'
        plt_title += f', Score={score:.4f}'
        
        plt.suptitle(plt_title, fontsize=16)
        plt.tight_layout()
        plt.subplots_adjust(top=0.92)
        
        # 同じファイルに上書き保存
        if plot_output:
            try:
                response_path = self.get_save_path(self.response_plot_filename)
                fig_response.savefig(response_path)
                if logger:
                    logger.info(f"応答プロットを更新: {response_path}")
                plt.close(fig_response)
            except Exception as e:
                if logger:
                    logger.error(f"応答プロットの保存に失敗: {e}")
        
        # 2. A*グリッドサーチの進行状況のプロット
        try:
            fig_grid = self.plot_astar_search(
                search_history, 
                param_ranges, 
                current_params, 
                optimization_method=optimization_method, 
                plot_output=False,  # 内部で保存しない
                logger=logger
            )
            
            # 同じファイルに上書き保存
            if plot_output and fig_grid is not None:
                grid_path = self.get_save_path(self.grid_search_plot_filename)
                fig_grid.savefig(grid_path)
                if logger:
                    logger.info(f"グリッドサーチプロットを更新: {grid_path}")
                plt.close(fig_grid)
        except Exception as e:
            if logger:
                logger.error(f"グリッドサーチプロットの生成または保存に失敗: {e}")
        
        return fig_response, fig_grid
    
    def plot_astar_search(self, search_history, param_ranges, best_params, 
                         optimization_method="A*", plot_output=False, logger=None):
        """
        A*探索またはグリッドサーチの可視化
        
        Args:
            search_history: 探索履歴
            param_ranges: パラメータの範囲
            best_params: 最適なパラメータ
            optimization_method: 最適化手法の名前
            plot_output: プロット出力フラグ
            logger: ロガー
            
        Returns:
            tuple: (P-Iグラフ, I-Dグラフ, D-Pグラフ)
        """
        # パラメータの名前
        param_names = list(param_ranges.keys())
        
        # グリッドスコアの取得
        grid_scores = search_history.get('grid_scores', {})
        
        # 訪問した状態と評価値
        visited_states = search_history.get('visited_states', [])
        scores = search_history.get('scores', [])
        
        # 最良経路
        best_path = search_history.get('best_path', [])
        
        # P-I, I-D, D-Pの3グラフを作成
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        fig.suptitle(f'{optimization_method}によるパラメータ最適化', fontsize=16)
        
        # パラメータのペア
        param_pairs = [
            (param_names[0], param_names[1]),  # P-I
            (param_names[1], param_names[2]),  # I-D
            (param_names[2], param_names[0])   # D-P
        ]
        
        # 各パラメータペアについてグリッドプロット
        for i, (param_x, param_y) in enumerate(param_pairs):
            ax = axes[i]
            
            # グリッドキー
            grid_key = (param_x, param_y)
            
            # グリッドスコアが存在する場合
            if grid_key in grid_scores:
                # グリッドスコアの取得
                grid_data = grid_scores[grid_key]
                
                # グリッドの範囲
                x_range = param_ranges[param_x]
                y_range = param_ranges[param_y]
                
                # グリッドの作成
                grid_x, grid_y = np.meshgrid(np.arange(len(x_range)), np.arange(len(y_range)))
                grid_z = np.ones((len(y_range), len(x_range))) * np.nan
                
                # グリッドにスコアを設定
                for (x_idx, y_idx), score in grid_data.items():
                    if param_x == grid_key[0]:
                        grid_z[y_idx, x_idx] = score
                    else:
                        grid_z[x_idx, y_idx] = score
                
                # グリッドのプロット
                im = ax.pcolormesh(grid_x, grid_y, grid_z, cmap=self.cmap, alpha=0.7)
                plt.colorbar(im, ax=ax, label='Score (RMSE)')
                
                # 訪問した状態のプロット
                x_indices = []
                y_indices = []
                c_scores = []
                
                for state, score in zip(visited_states, scores):
                    # パラメータのインデックスを取得
                    param_indices = {name: idx for idx, name in enumerate(param_names)}
                    x_idx = state[param_indices[param_x]]
                    y_idx = state[param_indices[param_y]]
                    
                    x_indices.append(x_idx)
                    y_indices.append(y_idx)
                    c_scores.append(score)
                
                # 訪問した状態をプロット
                scatter = ax.scatter(x_indices, y_indices, c=c_scores, cmap=self.cmap, 
                                    edgecolor='k', s=50, zorder=3)
                
                # 最良経路のプロット（A*の場合のみ）
                if optimization_method == "A*" and best_path:
                    path_x = []
                    path_y = []
                    
                    for state in best_path:
                        # パラメータのインデックスを取得
                        param_indices = {name: idx for idx, name in enumerate(param_names)}
                        x_idx = state[param_indices[param_x]]
                        y_idx = state[param_indices[param_y]]
                        
                        path_x.append(x_idx)
                        path_y.append(y_idx)
                    
                    # 最良経路をプロット
                    ax.plot(path_x, path_y, 'r-', linewidth=2, zorder=4)
                    
                    # 開始点と終了点をプロット
                    ax.plot(path_x[0], path_y[0], 'go', markersize=10, zorder=5)
                    ax.plot(path_x[-1], path_y[-1], 'ro', markersize=10, zorder=5)
                
                # 最適なパラメータのプロット
                best_x_idx = None
                best_y_idx = None
                
                for i, name in enumerate(param_names):
                    if name == param_x:
                        best_x_idx = np.where(param_ranges[name] == best_params[name])[0][0]
                    if name == param_y:
                        best_y_idx = np.where(param_ranges[name] == best_params[name])[0][0]
                
                if best_x_idx is not None and best_y_idx is not None:
                    ax.plot(best_x_idx, best_y_idx, 'r*', markersize=15, zorder=6)
                
                # 軸ラベルの設定
                ax.set_xlabel(f'{param_x} Index')
                ax.set_ylabel(f'{param_y} Index')
                ax.set_title(f'{param_x}-{param_y} Parameter Space')
                
                # 軸の目盛りを実際のパラメータ値に設定
                ax.set_xticks(np.arange(len(x_range)))
                ax.set_yticks(np.arange(len(y_range)))
                ax.set_xticklabels([f'{val:.3f}' for val in x_range])
                ax.set_yticklabels([f'{val:.3f}' for val in y_range])
                
                # グリッド線の表示
                ax.grid(True)
            else:
                ax.text(0.5, 0.5, f'No data for {param_x}-{param_y}', 
                       horizontalalignment='center', verticalalignment='center',
                       transform=ax.transAxes)
        
        plt.tight_layout()
        plt.subplots_adjust(top=0.9)
        
        # ファイル保存
        if plot_output and best_params:
            # 速度制御パラメータ
            best_p_vel = best_params['p_gain_vel']
            best_i_vel = best_params['i_gain_vel']
            best_d_vel = best_params['d_gain_vel']
            
            # 姿勢制御パラメータ（存在する場合）
            if 'p_gain_att' in best_params:
                best_p_att = best_params['p_gain_att']
                best_i_att = best_params['i_gain_att']
                best_d_att = best_params['d_gain_att']
                filename = f'{optimization_method.lower()}_grid_search_vel_p{best_p_vel:.3f}_i{best_i_vel:.3f}_d{best_d_vel:.3f}_att_p{best_p_att:.1f}_i{best_i_att:.1f}_d{best_d_att:.1f}.png'
            else:
                filename = f'{optimization_method.lower()}_grid_search_p{best_p_vel:.3f}_i{best_i_vel:.3f}_d{best_d_vel:.3f}.png'
            
            self.save_plot(fig, filename, logger)
        
        return fig
    
    def plot_trial_result(self, time_data, target_vel_data, actual_vel_data, error_data, 
                         p, i, d, score, iteration, p_att=None, i_att=None, d_att=None,
                         plot_output=False, logger=None):
        """各試行の結果をプロット"""
        fig = plt.figure(figsize=(15, 10))
        
        # 速度追従プロット - X軸
        ax1 = plt.subplot(3, 1, 1)
        ax1.plot(time_data, target_vel_data[:, 0], 'r-', label='Target Vx')
        ax1.plot(time_data, actual_vel_data[:, 0], 'b-', label='Actual Vx')
        ax1.set_xlabel('Time [s]')
        ax1.set_ylabel('Velocity X [m/s]')
        ax1.set_title(f'X-axis Velocity Tracking')
        ax1.legend()
        ax1.grid(True)
        
        # 速度追従プロット - Y軸
        ax2 = plt.subplot(3, 1, 2)
        ax2.plot(time_data, target_vel_data[:, 1], 'r-', label='Target Vy')
        ax2.plot(time_data, actual_vel_data[:, 1], 'b-', label='Actual Vy')
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Velocity Y [m/s]')
        ax2.set_title(f'Y-axis Velocity Tracking')
        ax2.legend()
        ax2.grid(True)
        
        # 速度追従プロット - Z軸
        ax3 = plt.subplot(3, 1, 3)
        ax3.plot(time_data, target_vel_data[:, 2], 'r-', label='Target Vz')
        ax3.plot(time_data, actual_vel_data[:, 2], 'b-', label='Actual Vz')
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Velocity Z [m/s]')
        ax3.set_title(f'Z-axis Velocity Tracking')
        ax3.legend()
        ax3.grid(True)
        
        # タイトルの追加
        plt_title = f'Trial {iteration}: Vel(P={p:.3f}, I={i:.3f}, D={d:.3f})'
        if p_att is not None and i_att is not None and d_att is not None:
            plt_title += f', Att(P={p_att:.1f}, I={i_att:.1f}, D={d_att:.1f})'
        plt_title += f', Score={score:.4f}'
        
        plt.suptitle(plt_title, fontsize=16)
        plt.tight_layout()
        plt.subplots_adjust(top=0.92)
        
        # 保存
        if plot_output:
            if p_att is not None and i_att is not None and d_att is not None:
                filename = f'trial_{iteration}_vel_p{p:.3f}_i{i:.3f}_d{d:.3f}_att_p{p_att:.1f}_i{i_att:.1f}_d{d_att:.1f}.png'
            else:
                filename = f'trial_{iteration}_p{p:.3f}_i{i:.3f}_d{d:.3f}.png'
            self.save_plot(fig, filename, logger)
        
        # エラープロット
        fig_err = plt.figure(figsize=(10, 6))
        for i, axis in enumerate(['X', 'Y', 'Z']):
            plt.plot(time_data, error_data[:, i], label=f'{axis} Error')
        plt.xlabel('Time [s]')
        plt.ylabel('Error [m/s]')
        plt.title(f'Trial {iteration}: Velocity Tracking Error')
        plt.legend()
        plt.grid(True)
        
        if plot_output:
            if p_att is not None and i_att is not None and d_att is not None:
                filename = f'trial_{iteration}_error_vel_p{p:.3f}_i{i:.3f}_d{d:.3f}_att_p{p_att:.1f}_i{i_att:.1f}_d{d_att:.1f}.png'
            else:
                filename = f'trial_{iteration}_error_p{p:.3f}_i{i:.3f}_d{d:.3f}.png'
            self.save_plot(fig_err, filename, logger)
        
        return fig, fig_err
