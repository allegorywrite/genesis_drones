"""
Simulation Node Module

このモジュールはマルチドローンシミュレーションのメインノードを提供します。
"""

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist

# 自作モジュールのインポート
from genesis_drones.simulation.camera_manager import CameraManager
from genesis_drones.simulation.drone_manager import DroneManager
from genesis_drones.simulation.tf_publisher import TFPublisher

class MultiDroneSimulation(Node):
    """マルチドローンシミュレーションのノード"""
    
    def __init__(self, num_drones=2, show_camera=False, record_camera=False, render_camera=True, output_file="fly_route_camera.mp4"):
        """
        マルチドローンシミュレーションのノード
        
        Args:
            num_drones (int): シミュレーションするドローンの数
            show_camera (bool): カメラ画像をウィンドウに表示するかどうか
            record_camera (bool): カメラ映像を録画するかどうか
            render_camera (bool): カメラのレンダリングを行うかどうか
            output_file (str): 録画ファイルの出力先
        """
        super().__init__('multi_drone_simulation')
        
        # ROS 2パラメータの宣言
        self.declare_parameter('num_drones', num_drones)
        self.declare_parameter('show_camera', show_camera)
        self.declare_parameter('record_camera', record_camera)
        self.declare_parameter('render_camera', render_camera)
        self.declare_parameter('output_file', output_file)
        self.declare_parameter('dt', 0.01)  # シミュレーションの時間ステップ
        self.declare_parameter('route', '')  # 単一ドローンの飛行ルート
        self.declare_parameter('routes', '')  # 複数ドローンの飛行ルート
        self.declare_parameter('drone_id', 0)  # 飛行ルートを適用するドローンのID
        self.declare_parameter('velocity_control', False)  # 速度制御モード
        self.declare_parameter('target_velocity', '0.0,0.0,0.0')  # 目標速度 [m/s]
        self.declare_parameter('target_angular_velocity', '0.0,0.0,0.0')  # 目標角速度 [rad/s]
        self.declare_parameter('velocity_profile', 'constant')  # 速度プロファイル
        self.declare_parameter('velocity_command_duration', 10)  # 速度コマンドの持続ステップ数（デフォルト10ステップ）
        
        # パラメータの取得
        num_drones = self.get_parameter('num_drones').value
        self.show_camera = self.get_parameter('show_camera').value
        self.record_camera = self.get_parameter('record_camera').value
        self.render_camera = self.get_parameter('render_camera').value
        self.output_file = self.get_parameter('output_file').value
        self.dt = self.get_parameter('dt').value
        self.velocity_control = self.get_parameter('velocity_control').value
        self.target_velocity = self.get_parameter('target_velocity').value
        self.target_angular_velocity = self.get_parameter('target_angular_velocity').value
        self.velocity_profile = self.get_parameter('velocity_profile').value
        self.velocity_command_duration = self.get_parameter('velocity_command_duration').value
        
        # パラメータの型変換（文字列からbool型への変換）
        if isinstance(self.show_camera, str):
            self.show_camera = self.show_camera.lower() == 'true'
        if isinstance(self.record_camera, str):
            self.record_camera = self.record_camera.lower() == 'true'
        if isinstance(self.render_camera, str):
            self.render_camera = self.render_camera.lower() == 'true'
        if isinstance(self.velocity_control, str):
            self.velocity_control = self.velocity_control.lower() == 'true'
            
        # 速度パラメータの変換
        try:
            self.target_velocity = np.array([float(x) for x in self.target_velocity.split(',')])
            self.target_angular_velocity = np.array([float(x) for x in self.target_angular_velocity.split(',')])
        except Exception as e:
            self.get_logger().error(f"Error parsing velocity parameters: {e}")
            self.target_velocity = np.array([0.0, 0.0, 0.0])
            self.target_angular_velocity = np.array([0.0, 0.0, 0.0])
            
        # カメラレンダリングの設定をログに出力
        self.get_logger().info(f"Camera rendering: {'enabled' if self.render_camera else 'disabled'}")
        self.get_logger().info(f"Velocity control: {'enabled' if self.velocity_control else 'disabled'}")
        
        # 処理時間計測用のロガーを初期化
        try:
            from genesis_drones.utils.timing_utils import TimingLogger
            self.timing_logger = TimingLogger(self.get_logger(), log_interval=100)
        except ImportError:
            self.get_logger().warn("TimingLogger could not be imported")
            self.timing_logger = None
        
        # Genesisシーンの初期化
        try:
            from genesis_drones.utils.simulation_utils import initialize_genesis_scene
            self.scene = initialize_genesis_scene(dt=self.dt)
        except ImportError:
            self.get_logger().error("Failed to initialize Genesis scene")
            raise
        
        # ウェイポイントマネージャーの初期化
        try:
            from genesis_drones.utils.waypoint_utils import WaypointManager
            self.waypoint_manager = WaypointManager(self.scene)
        except ImportError:
            self.get_logger().error("Failed to initialize WaypointManager")
            raise
        
        # TFパブリッシャーの初期化
        try:
            from genesis_drones.utils.tf_utils import create_tf_importer
            importer = create_tf_importer()
            self.tf_publisher = TFPublisher(self.get_logger(), importer)
        except ImportError:
            self.get_logger().warn("TF publisher could not be initialized")
            self.tf_publisher = TFPublisher(self.get_logger(), None)
        
        # ドローンマネージャーの初期化
        self.drone_manager = DroneManager(
            node=self,
            num_drones=num_drones,
            dt=self.dt,
            velocity_control=self.velocity_control,
            target_velocity=self.target_velocity,
            target_angular_velocity=self.target_angular_velocity,
            velocity_profile=self.velocity_profile
        )
        
        # カメラマネージャーの初期化
        self.camera_manager = CameraManager(
            node=self,
            show_camera=self.show_camera,
            record_camera=self.record_camera,
            render_camera=self.render_camera,
            output_file=self.output_file
        )
        
        # シーンのビルド前にマーカーを追加するためのフラグ
        self.routes_to_add = []
        self.scene_built = False
        
        # ルートの設定
        self._setup_routes()
        
        # ドローンのセットアップ
        self.drones = self.drone_manager.setup_drones(self.scene, self.waypoint_manager)
        
        # カメラのセットアップ
        self.cameras = []
        for i in range(num_drones):
            camera = self.camera_manager.setup_camera(self.scene, i)
            self.cameras.append(camera)
        
        # シーンのビルド
        self.scene.build()
        self.scene_built = True
        self.get_logger().info(f"Initialized {num_drones} drones")
        
        # ルートの設定（シーンビルド後）
        self.drone_manager.setup_routes(self.routes_to_add, self.waypoint_manager)
        
        # 速度コマンドのサブスクライバー
        self.velocity_subscribers = []
        for i in range(num_drones):
            subscriber = self.create_subscription(
                Twist,
                f'/drone_{i}/cmd_vel',
                lambda msg, drone_id=i: self.velocity_command_callback(msg, drone_id),
                10,
                callback_group=ReentrantCallbackGroup()
            )
            self.velocity_subscribers.append(subscriber)
            self.get_logger().info(f"Subscribed to /drone_{i}/cmd_vel topic")
        
        # シミュレーションタイマーの設定
        self.timer = self.create_timer(self.dt, self.simulation_step)
    
    def _setup_routes(self):
        """飛行ルートのセットアップ"""
        try:
            from genesis_drones.utils.waypoint_utils import create_default_routes
            from genesis_drones.utils.simulation_utils import parse_route_string, parse_multi_drone_routes
            
            # デフォルトのルートを取得
            num_drones = self.get_parameter('num_drones').value
            default_routes = create_default_routes(num_drones)
            
            # コマンドライン引数からルートを追加
            # 複数ドローンの飛行ルートが指定されている場合
            routes_param = self.get_parameter('routes').value if self.has_parameter('routes') else None
            if routes_param:
                try:
                    # ルート文字列をパース
                    drone_routes = parse_multi_drone_routes(routes_param, num_drones)
                    
                    if drone_routes:
                        # ルートを追加
                        self.routes_to_add.extend(drone_routes)
                        
                        # ルート線を追加（オプション）
                        for drone_id, points in drone_routes:
                            # velocity_controlがfalseの場合のみアクティブマーカーを作成
                            if points and not self.velocity_control:
                                self.waypoint_manager.create_active_marker(drone_id, points[0], size=0.15)
                except Exception as e:
                    self.get_logger().error(f"Error parsing routes: {e}")
            
            # 単一ドローンの飛行ルートが指定されている場合
            route_param = self.get_parameter('route').value if self.has_parameter('route') else None
            drone_id_param = self.get_parameter('drone_id').value if self.has_parameter('drone_id') else 0
            
            if route_param:
                try:
                    # ルート文字列をパース
                    points = parse_route_string(route_param)
                    
                    if points and 0 <= drone_id_param < num_drones:
                        # ルートを追加
                        self.routes_to_add.append((drone_id_param, points))
                        
                        # velocity_controlがfalseの場合のみアクティブマーカーを作成
                        if points and not self.velocity_control:
                            self.waypoint_manager.create_active_marker(drone_id_param, points[0], size=0.15)
                except Exception as e:
                    self.get_logger().error(f"Error parsing route: {e}")
            
            # デフォルトのルートを追加（引数で指定されていない場合）
            if not self.routes_to_add:
                self.routes_to_add = default_routes
                
                # velocity_controlがfalseの場合のみ、各ドローンの最初のウェイポイントにアクティブマーカーを作成
                if not self.velocity_control:
                    for drone_id, points in default_routes:
                        if points:
                            self.waypoint_manager.create_active_marker(drone_id, points[0], size=0.15)
        except Exception as e:
            self.get_logger().error(f"Error setting up routes: {e}")
    
    def simulation_step(self):
        """シミュレーションステップの実行"""
        if self.timing_logger:
            step_start_time = self.timing_logger.start_timer()
            
        # 各ドローンの更新
        drone_states = self.drone_manager.update_drones(self.waypoint_manager, self.timing_logger)
        
        # TF情報の発行
        for i, drone in enumerate(self.drones):
            if self.timing_logger:
                tf_start_time = self.timing_logger.start_timer()
                
            self.tf_publisher.publish_drone_tf(self.scene.cur_t, drone, i)
            
            if self.timing_logger:
                self.timing_logger.stop_timer(tf_start_time, f"drone_{i}_tf_publish")
            
            # カメラの更新
            self.camera_manager.update_camera(
                i, 
                drone, 
                self.cameras[i], 
                self.drone_manager.flight_controllers[i], 
                self.timing_logger
            )
        
        # シミュレーションステップを進める
        if self.timing_logger:
            scene_step_start_time = self.timing_logger.start_timer()
            
        self.scene.step()
        
        if self.timing_logger:
            self.timing_logger.stop_timer(scene_step_start_time, "scene_step")
            
            # 全体の処理時間
            self.timing_logger.stop_timer(step_start_time, "total_step")
                
            # ステップカウントを増やし、必要に応じて統計情報をログ出力
            self.timing_logger.increment_step(self.dt)
    
    def velocity_command_callback(self, msg, drone_id):
        """
        速度コマンドのコールバック
        
        Args:
            msg (Twist): 速度コマンド
            drone_id (int): ドローンID
        """
        if 0 <= drone_id < len(self.drone_manager.flight_controllers):
            # 線形速度と角速度を取得
            linear_vel = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
            angular_vel = np.array([msg.angular.x, msg.angular.y, msg.angular.z])
            
            # ハードコードされた持続ステップ数を使用
            duration_steps = self.velocity_command_duration
            
            # 30回に1回ログ出力
            if not hasattr(self, 'log_counter'):
                self.log_counter = {}
            if drone_id not in self.log_counter:
                self.log_counter[drone_id] = 0
            self.log_counter[drone_id] += 1
            
            if self.log_counter[drone_id] % 30 == 0:
                self.get_logger().info(f"Drone {drone_id} velocity command: linear={linear_vel}, angular={angular_vel}, duration={duration_steps} steps")
            
            # 速度目標を設定（持続ステップ数付き）
            self.drone_manager.flight_controllers[drone_id].set_velocity_target(linear_vel, angular_vel, duration_steps)
    
    def shutdown(self):
        """シミュレーションの終了処理"""
        # 最終的な統計情報をログに出力
        if self.timing_logger:
            self.timing_logger.log_final_stats(self.dt)
        
        # TFパブリッシャーの終了処理
        self.tf_publisher.shutdown()
        
        # カメラマネージャーの終了処理
        self.camera_manager.shutdown()
        
        # シャットダウン処理の完了
        self.get_logger().info("Shutdown complete")
