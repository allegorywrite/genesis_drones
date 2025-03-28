"""
Drone Manager Module

このモジュールはドローン関連の処理を担当します。
"""

import numpy as np
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point
from rclpy.callback_groups import ReentrantCallbackGroup

class DroneManager:
    """ドローン関連の処理を担当するクラス"""
    
    def __init__(self, node, num_drones=1, dt=0.01, velocity_control=False, 
                 target_velocity=None, target_angular_velocity=None, velocity_profile="constant"):
        """
        DroneManagerの初期化
        
        Args:
            node: ROS2ノード
            num_drones: ドローンの数
            dt: シミュレーションの時間ステップ
            velocity_control: 速度制御モードかどうか
            target_velocity: 目標速度 [m/s]
            target_angular_velocity: 目標角速度 [rad/s]
            velocity_profile: 速度プロファイル
        """
        self.node = node
        self.logger = node.get_logger()
        self.num_drones = num_drones
        self.dt = dt
        self.velocity_control = velocity_control
        
        # 速度パラメータの設定
        if target_velocity is None:
            self.target_velocity = np.array([0.0, 0.0, 0.0])
        else:
            self.target_velocity = target_velocity
            
        if target_angular_velocity is None:
            self.target_angular_velocity = np.array([0.0, 0.0, 0.0])
        else:
            self.target_angular_velocity = target_angular_velocity
            
        self.velocity_profile = velocity_profile
        
        # ドローンとコントローラの初期化
        self.drones = []
        self.flight_controllers = []
        
        # コールバックグループの作成（並行実行のため）
        self.callback_group = ReentrantCallbackGroup()
    
    def setup_drones(self, scene, waypoint_manager):
        """
        ドローンのセットアップ
        
        Args:
            scene: Genesisシーン
            waypoint_manager: ウェイポイントマネージャー
            
        Returns:
            list: ドローンオブジェクトのリスト
        """
        try:
            from genesis_drones.utils.simulation_utils import add_drone_to_scene
            from genesis_drones.controllers.dsl_pid_controller import DSLPIDController
            from genesis_drones.controllers.flight_controller import DroneFlightController
            
            # 各ドローンの初期化
            for i in range(self.num_drones):
                # ドローンの位置をずらして配置
                drone_position = (0, 0, 0.5)
                
                # ドローンの追加
                drone = add_drone_to_scene(scene, position=drone_position)
                self.drones.append(drone)
                
                # コントローラの初期化
                controller = DSLPIDController(
                    drone=drone,
                    dt=self.dt
                )
                
                # 飛行コントローラの初期化
                flight_controller = DroneFlightController(
                    drone=drone,
                    controller=controller,
                    logger=self.logger,
                    velocity_control=self.velocity_control,
                    target_velocity=self.target_velocity,
                    target_angular_velocity=self.target_angular_velocity,
                    velocity_profile=self.velocity_profile
                )
                self.flight_controllers.append(flight_controller)
                
                # 各ドローンに対するサービスの作成
                self.node.create_service(
                    Trigger,
                    f'/drone_{i}/hover',
                    lambda req, res, drone_id=i: self.hover_callback(req, res, drone_id),
                    callback_group=self.callback_group
                )
                
                # 特定のポイントに飛行するためのトピックのサブスクライバー
                self.node.create_subscription(
                    Point,
                    f'/drone_{i}/fly_to_point',
                    lambda msg, drone_id=i: self.fly_to_point_topic_callback(msg, drone_id),
                    10,
                    callback_group=self.callback_group
                )
            
            return self.drones
        except Exception as e:
            self.logger.error(f"Error setting up drones: {e}")
            return []
    
    def setup_routes(self, routes_to_add, waypoint_manager):
        """
        飛行ルートのセットアップ
        
        Args:
            routes_to_add: 追加するルートのリスト [(drone_id, points), ...]
            waypoint_manager: ウェイポイントマネージャー
        """
        try:
            # ルートの設定
            for drone_id, points in routes_to_add:
                if 0 <= drone_id < self.num_drones:
                    # 飛行コントローラにルートを設定
                    self.flight_controllers[drone_id].set_route(points)
                    
                    # 飛行を開始
                    self.flight_controllers[drone_id].start_route()
                    
                    # velocity_controlがfalseの場合のみアクティブマーカーを作成
                    if points and not self.velocity_control:
                        waypoint_manager.create_active_marker(drone_id, points[0], size=0.15)
        except Exception as e:
            self.logger.error(f"Error setting up routes: {e}")
    
    def update_drones(self, waypoint_manager, timing_logger=None):
        """
        ドローンの更新
        
        Args:
            waypoint_manager: ウェイポイントマネージャー
            timing_logger: タイミングロガー
            
        Returns:
            list: 各ドローンの状態 [(is_flying, waypoint_changed), ...]
        """
        results = []
        
        for i, drone in enumerate(self.drones):
            if timing_logger:
                drone_start_time = timing_logger.start_timer()
            
            # 飛行コントローラの更新
            if timing_logger:
                controller_start_time = timing_logger.start_timer()
                
            is_flying, waypoint_changed = self.flight_controllers[i].update()
            results.append((is_flying, waypoint_changed))
            
            if timing_logger:
                timing_logger.stop_timer(controller_start_time, f"drone_{i}_controller_update")
            
            # マーカー更新用のタイマー開始（velocity_controlに関わらず）
            if timing_logger:
                marker_start_time = timing_logger.start_timer()
            
            # velocity_controlがfalseの場合のみアクティブマーカーを更新
            if not self.velocity_control:
                if is_flying and self.flight_controllers[i].current_target is not None:
                    # ウェイポイントが変更された場合は、アクティブマーカーを更新
                    if waypoint_changed:
                        waypoint_manager.create_active_marker(i, self.flight_controllers[i].current_target, size=0.15)
                    else:
                        waypoint_manager.update_active_marker(i, self.flight_controllers[i].current_target)
            
            if timing_logger:
                timing_logger.stop_timer(marker_start_time, f"drone_{i}_marker_update")
                timing_logger.stop_timer(drone_start_time, f"drone_{i}_total")
        
        return results
    
    def hover_callback(self, request, response, drone_id):
        """ドローンをホバリングさせるサービスコールバック"""
        try:
            # 飛行を停止
            self.flight_controllers[drone_id].stop()
            
            response.success = True
            response.message = f"Drone {drone_id} is now hovering"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Failed to hover drone {drone_id}: {str(e)}"
            return response
    
    def fly_to_point_topic_callback(self, msg, drone_id):
        """特定のポイントに飛行するトピックコールバック"""
        try:
            # メッセージからポイントを取得
            target_point = (msg.x, msg.y, msg.z)
            
            # 飛行を開始
            success = self.flight_controllers[drone_id].fly_to_point(target_point)
            
            if success:
                self.logger.info(f"Drone {drone_id} is flying to point {target_point}")
            else:
                self.logger.error(f"Failed to start flying drone {drone_id} to point {target_point}")
        except Exception as e:
            self.logger.error(f"Error in fly_to_point topic callback: {str(e)}")
