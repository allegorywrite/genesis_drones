#!/usr/bin/env python3

import os
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Dict, Any

# genesisのインポート
import genesis as gs

# 自作モジュールのインポート
from genesis_drones.controllers.pid_controller import DronePIDController
from genesis_drones.controllers.ctbr_controller import CTBRController
from genesis_drones.controllers.hybrid_controller import HybridController
from genesis_drones.controllers.flight_controller import DroneFlightController
from genesis_drones.utils.simulation_utils import initialize_genesis_scene, add_drone_to_scene
from genesis_drones.utils.waypoint_utils import WaypointManager, create_default_routes
from genesis_drones.utils.timing_utils import TimingLogger

class ControllerTest:
    """コントローラのテストクラス"""
    
    def __init__(self, dt: float = 0.01, show_viewer: bool = True):
        """
        初期化
        
        Args:
            dt (float): シミュレーションの時間ステップ
            show_viewer (bool): ビューアを表示するかどうか
        """
        # シーンの初期化
        self.scene = initialize_genesis_scene(dt=dt, show_viewer=show_viewer)
        self.dt = dt
        
        # ドローンの初期位置
        self.drone_positions = [
            (0.0, 0.0, 0.5),  # PIDコントローラ用
            (1.0, 0.0, 0.5),  # CTBRコントローラ用
            (2.0, 0.0, 0.5),  # ハイブリッドコントローラ用
        ]
        
        # ドローンの追加
        self.drones = []
        for pos in self.drone_positions:
            drone = add_drone_to_scene(self.scene, position=pos)
            self.drones.append(drone)
        
        # コントローラの作成
        self.controllers = []
        self.flight_controllers = []
        
        # PIDコントローラ
        pid_controller = DronePIDController(
            drone=self.drones[0],
            dt=self.dt
        )
        self.controllers.append(pid_controller)
        
        # CTBRコントローラ
        ctbr_controller = CTBRController(
            drone=self.drones[1],
            dt=self.dt
        )
        self.controllers.append(ctbr_controller)
        
        # ハイブリッドコントローラ
        hybrid_controller = HybridController(
            drone=self.drones[2],
            dt=self.dt
        )
        self.controllers.append(hybrid_controller)
        
        # 飛行コントローラの作成
        for i, controller in enumerate(self.controllers):
            flight_controller = DroneFlightController(
                drone=self.drones[i],
                controller=controller
            )
            self.flight_controllers.append(flight_controller)
        
        # ウェイポイントマネージャの作成
        self.waypoint_manager = WaypointManager(self.scene)
        
        # ロガーの作成
        self.logger = TimingLogger(None, log_interval=100)
        
        # 飛行ルートの設定
        self.routes = self._create_test_routes()
        for i, (flight_controller, route) in enumerate(zip(self.flight_controllers, self.routes)):
            flight_controller.set_route(route)
            # ルートのマーカーを追加
            self.waypoint_manager.add_route_markers(route, drone_id=i, size=0.05, add_lines=True)
        
        # シーンをビルド
        self.scene.build()
        
        # 飛行データの記録用
        self.flight_data = {
            "pid": {"pos": [], "time": []},
            "ctbr": {"pos": [], "time": []},
            "hybrid": {"pos": [], "time": []}
        }
        self.controller_names = ["pid", "ctbr", "hybrid"]
    
    def _create_test_routes(self) -> List[List[Tuple[float, float, float]]]:
        """
        テスト用の飛行ルートを作成
        
        Returns:
            list: 各ドローンの飛行ルート
        """
        # 基本的な円形のルート
        routes = []
        
        # 各コントローラ用のルート
        for i, pos in enumerate(self.drone_positions):
            # 初期位置
            start_x, start_y, start_z = pos
            
            # 円形のルート
            radius = 0.5
            height = 1.0
            points = []
            
            # 初期位置
            points.append((start_x, start_y, start_z))
            
            # 上昇
            points.append((start_x, start_y, height))
            
            # 円形のポイント
            num_points = 8
            for j in range(num_points):
                angle = (2 * np.pi * j) / num_points
                x = start_x + radius * np.cos(angle)
                y = start_y + radius * np.sin(angle)
                points.append((x, y, height))
            
            # 最後に初期位置の上空に戻る
            points.append((start_x, start_y, height))
            
            # 着陸
            points.append((start_x, start_y, start_z))
            
            routes.append(points)
        
        return routes
    
    def run_simulation(self, max_steps: int = 2000) -> Dict[str, Dict[str, List]]:
        """
        シミュレーションを実行
        
        Args:
            max_steps (int): 最大ステップ数
            
        Returns:
            dict: 飛行データ
        """
        # 飛行開始
        for flight_controller in self.flight_controllers:
            flight_controller.start_route()
        
        # シミュレーション実行
        step = 0
        start_time = time.time()
        
        while step < max_steps:
            # 時間計測開始
            timer_start = self.logger.start_timer()
            
            # シーンの更新
            self.scene.step()
            
            # 各ドローンの更新
            all_finished = True
            for i, flight_controller in enumerate(self.flight_controllers):
                # 飛行コントローラの更新
                is_flying, waypoint_changed = flight_controller.update()
                
                # 飛行中かどうかを記録
                if is_flying:
                    all_finished = False
                
                # 現在位置を記録
                drone_pos = self.drones[i].get_pos().cpu().numpy()
                controller_name = self.controller_names[i]
                self.flight_data[controller_name]["pos"].append(drone_pos.copy())
                self.flight_data[controller_name]["time"].append(step * self.dt)
            
            # 時間計測終了
            self.logger.stop_timer(timer_start, "total_step")
            self.logger.increment_step(self.dt)
            
            # すべてのドローンが飛行終了したら終了
            if all_finished:
                print(f"All drones finished flying at step {step}")
                break
            
            step += 1
        
        # 実行時間を表示
        elapsed_time = time.time() - start_time
        print(f"Simulation completed in {elapsed_time:.2f} seconds ({step} steps)")
        
        # 最終的な統計情報を表示
        self.logger.log_final_stats(self.dt)
        
        return self.flight_data
    
    def plot_results(self, flight_data: Dict[str, Dict[str, List]]) -> None:
        """
        結果をプロット
        
        Args:
            flight_data (dict): 飛行データ
        """
        # 3Dプロット
        fig = plt.figure(figsize=(15, 10))
        
        # 3D軌跡
        ax1 = fig.add_subplot(221, projection='3d')
        ax1.set_title('Flight Trajectories')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')
        
        colors = {'pid': 'r', 'ctbr': 'g', 'hybrid': 'b'}
        labels = {'pid': 'PID Controller', 'ctbr': 'CTBR Controller', 'hybrid': 'Hybrid Controller'}
        
        for controller_name, data in flight_data.items():
            positions = np.array(data["pos"])
            ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                    color=colors[controller_name], label=labels[controller_name])
            
            # ルートポイントをプロット
            route_index = self.controller_names.index(controller_name)
            route = np.array(self.routes[route_index])
            ax1.scatter(route[:, 0], route[:, 1], route[:, 2], 
                       color=colors[controller_name], marker='o', s=30, alpha=0.5)
        
        ax1.legend()
        
        # X座標の時間変化
        ax2 = fig.add_subplot(222)
        ax2.set_title('X Position vs Time')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('X Position (m)')
        
        for controller_name, data in flight_data.items():
            positions = np.array(data["pos"])
            times = np.array(data["time"])
            ax2.plot(times, positions[:, 0], color=colors[controller_name], label=labels[controller_name])
        
        ax2.legend()
        
        # Y座標の時間変化
        ax3 = fig.add_subplot(223)
        ax3.set_title('Y Position vs Time')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Y Position (m)')
        
        for controller_name, data in flight_data.items():
            positions = np.array(data["pos"])
            times = np.array(data["time"])
            ax3.plot(times, positions[:, 1], color=colors[controller_name], label=labels[controller_name])
        
        ax3.legend()
        
        # Z座標の時間変化
        ax4 = fig.add_subplot(224)
        ax4.set_title('Z Position vs Time')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Z Position (m)')
        
        for controller_name, data in flight_data.items():
            positions = np.array(data["pos"])
            times = np.array(data["time"])
            ax4.plot(times, positions[:, 2], color=colors[controller_name], label=labels[controller_name])
        
        ax4.legend()
        
        plt.tight_layout()
        
        # 結果を保存
        plt.savefig('controller_comparison.png')
        print("Results saved to controller_comparison.png")
        
        # 結果を表示
        plt.show()
    
    def calculate_metrics(self, flight_data: Dict[str, Dict[str, List]]) -> Dict[str, Dict[str, float]]:
        """
        評価指標を計算
        
        Args:
            flight_data (dict): 飛行データ
            
        Returns:
            dict: 評価指標
        """
        metrics = {}
        
        for controller_name, data in flight_data.items():
            positions = np.array(data["pos"])
            times = np.array(data["time"])
            
            # ルートポイントを取得
            route_index = self.controller_names.index(controller_name)
            route = np.array(self.routes[route_index])
            
            # 各ウェイポイントへの到達時間
            arrival_times = []
            waypoint_errors = []
            
            for waypoint in route[1:]:  # 最初のポイント（初期位置）を除く
                # 各ウェイポイントに最も近づいた時のインデックスを探す
                distances = np.sqrt(np.sum((positions - waypoint)**2, axis=1))
                min_dist_idx = np.argmin(distances)
                min_dist = distances[min_dist_idx]
                
                arrival_times.append(times[min_dist_idx])
                waypoint_errors.append(min_dist)
            
            # 軌跡の滑らかさ（速度の変化率）
            velocity = np.diff(positions, axis=0) / np.diff(times)[:, np.newaxis]
            acceleration = np.diff(velocity, axis=0) / np.diff(times[:-1])[:, np.newaxis]
            smoothness = np.mean(np.sqrt(np.sum(acceleration**2, axis=1)))
            
            # 評価指標を記録
            metrics[controller_name] = {
                "mean_waypoint_error": np.mean(waypoint_errors),
                "max_waypoint_error": np.max(waypoint_errors),
                "total_flight_time": times[-1],
                "smoothness": smoothness
            }
        
        return metrics
    
    def print_metrics(self, metrics: Dict[str, Dict[str, float]]) -> None:
        """
        評価指標を表示
        
        Args:
            metrics (dict): 評価指標
        """
        print("\n=== Controller Performance Metrics ===")
        
        # 表のヘッダー
        headers = ["Controller", "Mean Error (m)", "Max Error (m)", "Flight Time (s)", "Smoothness"]
        print(f"{headers[0]:<15} {headers[1]:<15} {headers[2]:<15} {headers[3]:<15} {headers[4]:<15}")
        print("-" * 75)
        
        # 各コントローラの評価指標
        for controller_name, metric in metrics.items():
            print(f"{controller_name:<15} "
                  f"{metric['mean_waypoint_error']:<15.4f} "
                  f"{metric['max_waypoint_error']:<15.4f} "
                  f"{metric['total_flight_time']:<15.4f} "
                  f"{metric['smoothness']:<15.4f}")

def main():
    """メイン関数"""
    # Genesisの初期化（simulation_utils.initialize_genesis_scene内で初期化されるため、ここでは行わない）
    # gs.init(backend=gs.cpu)
    
    # コントローラテストの作成
    test = ControllerTest(dt=0.01, show_viewer=True)
    
    # シミュレーションの実行
    flight_data = test.run_simulation(max_steps=2000)
    
    # 結果のプロット
    test.plot_results(flight_data)
    
    # 評価指標の計算と表示
    metrics = test.calculate_metrics(flight_data)
    test.print_metrics(metrics)

if __name__ == "__main__":
    main()
