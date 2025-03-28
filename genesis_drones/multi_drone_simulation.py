#!/usr/bin/env python3

import os
import math
import rclpy
import numpy as np
import cv2
import argparse
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty

# Genesisのインポート
import genesis as gs

# 自作モジュールのインポート
try:
    # パッケージがインストールされている場合
    from genesis_drones.utils.waypoint_utils import WaypointManager, create_default_routes
    from genesis_drones.controllers.pid_controller import DronePIDController
    from genesis_drones.controllers.ctbr_controller import CTBRController
    from genesis_drones.controllers.dsl_pid_controller import DSLPIDController
    from genesis_drones.controllers.hybrid_controller import HybridController
    from genesis_drones.controllers.flight_controller import DroneFlightController
    from genesis_drones.utils.simulation_utils import (
        process_xacro, parse_route_string, parse_multi_drone_routes, cleanup_temp_files,
        initialize_genesis_scene, add_drone_to_scene, add_camera_to_scene, update_camera_position
    )
    from genesis_drones.utils.visualization_utils import (
        process_camera_image, convert_to_bgr, display_camera_image,
        draw_drone_info, draw_route_on_image, create_multi_view_image
    )
    from genesis_drones.utils.tf_utils import create_tf_importer, get_tf_from_link, finish_tf_importer, HAS_TF
    from genesis_drones.utils.timing_utils import TimingLogger
except ImportError:
    # 相対インポートを使用
    from .utils.waypoint_utils import WaypointManager, create_default_routes
    from .controllers.pid_controller import DronePIDController
    from .controllers.ctbr_controller import CTBRController
    from .controllers.dsl_pid_controller import DSLPIDController
    from .controllers.flight_controller import DroneFlightController
    from .utils.simulation_utils import (
        process_xacro, parse_route_string, parse_multi_drone_routes, cleanup_temp_files,
        initialize_genesis_scene, add_drone_to_scene, add_camera_to_scene, update_camera_position
    )
    from .utils.visualization_utils import (
        process_camera_image, convert_to_bgr, display_camera_image,
        draw_drone_info, draw_route_on_image, create_multi_view_image
    )
    from .utils.tf_utils import create_tf_importer, get_tf_from_link, finish_tf_importer, HAS_TF
    from .utils.timing_utils import TimingLogger

# カメラ機能の有効化フラグ
HAS_CAMERA = True

class MultiDroneSimulation(Node):
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
        
        # 処理時間計測用のロガーを初期化
        self.timing_logger = TimingLogger(self.get_logger(), log_interval=100)
        
        # ROS 2の初期化
        self.image_publishers = []
        self.bridge = CvBridge()
        
        # コールバックグループの作成（並行実行のため）
        self.callback_group = ReentrantCallbackGroup()
        
        # ROS 2パラメータの宣言
        self.declare_parameter('num_drones', num_drones)
        self.declare_parameter('show_camera', show_camera)
        self.declare_parameter('record_camera', record_camera)
        self.declare_parameter('render_camera', render_camera)
        self.declare_parameter('output_file', output_file)
        self.declare_parameter('dt', 0.033)  # シミュレーションの時間ステップ
        self.declare_parameter('route', '')  # 単一ドローンの飛行ルート
        self.declare_parameter('routes', '')  # 複数ドローンの飛行ルート
        self.declare_parameter('drone_id', 0)  # 飛行ルートを適用するドローンのID
        
        # パラメータの取得
        num_drones = self.get_parameter('num_drones').value
        self.show_camera = self.get_parameter('show_camera').value
        self.record_camera = self.get_parameter('record_camera').value
        self.render_camera = self.get_parameter('render_camera').value
        self.output_file = self.get_parameter('output_file').value
        self.dt = self.get_parameter('dt').value
        
        # パラメータの型変換（文字列からbool型への変換）
        if isinstance(self.show_camera, str):
            self.show_camera = self.show_camera.lower() == 'true'
        if isinstance(self.record_camera, str):
            self.record_camera = self.record_camera.lower() == 'true'
        if isinstance(self.render_camera, str):
            self.render_camera = self.render_camera.lower() == 'true'
            
        # カメラレンダリングの設定をログに出力
        self.get_logger().info(f"Camera rendering: {'enabled' if self.render_camera else 'disabled'}")
        
        # Genesisシーンの初期化
        self.scene = initialize_genesis_scene(dt=self.dt)
        
        # xacroファイルを使用
        xacro_path = "/home/initial/colcon_ws/src/genesis_drones/genesis_drones/urdf/crazyflie_camera.urdf.xacro"
        
        # xacroファイルを処理して一時的なURDFファイルを作成
        urdf_path = process_xacro(xacro_path)
        
        # ドローンの初期化
        self.drones = []
        self.cameras = []
        self.pid_controllers = []  # PIDコントローラのリスト
        self.flight_controllers = []  # 飛行コントローラのリスト
        self.temp_files = []  # 一時ファイルのリスト
        
        # 一時ファイルが作成された場合、リストに追加
        if urdf_path != xacro_path:
            self.temp_files.append(urdf_path)
        
        # ウェイポイントマネージャーの初期化
        self.waypoint_manager = WaypointManager(self.scene)
        
        # 各ドローンの初期化
        for i in range(num_drones):
            # ドローンの位置をずらして配置
            drone_position = (0, 0, 0.5)
            
            # ドローンの追加
            drone = add_drone_to_scene(self.scene, position=drone_position)
            self.drones.append(drone)
            
            # PIDコントローラの初期化
            # pid_controller = DronePIDController(
            #     drone=drone,
            #     dt=self.dt
            # )
            # self.pid_controllers.append(pid_controller)

            ctbr_controller = DSLPIDController(
                drone=drone,
                dt=self.dt
            )
            # self.pid_controllers.append(ctbr_controller)
            
            # 飛行コントローラの初期化
            flight_controller = DroneFlightController(
                drone=drone,
                controller=ctbr_controller,
                logger=self.get_logger()
            )
            self.flight_controllers.append(flight_controller)
            
            # カメラの設定
            if HAS_CAMERA:
                try:
                    # カメラの追加
                    camera_position = (0, 0, 0.5)  # ドローンの少し前方
                    camera_lookat = (0, 0, 0.5)  # より遠くを見る
                    
                    camera = add_camera_to_scene(
                        self.scene,
                        position=camera_position,
                        lookat=camera_lookat
                    )
                    self.cameras.append(camera)
                    
                    # 録画の開始
                    if self.record_camera and i == 0:  # 最初のドローンのカメラのみ録画
                        camera.start_recording()
                except Exception as e:
                    self.get_logger().error(f"Failed to add camera: {e}")
                    camera = None
                    self.cameras.append(camera)
            else:
                # カメラ機能が利用できない場合
                camera = None
                self.cameras.append(camera)
            
            # ROS 2パブリッシャーの設定
            pub = self.create_publisher(
                Image,
                f'/drone_{i}/camera/image_raw',
                10
            )
            self.image_publishers.append(pub)
            
            # 各ドローンに対するサービスの作成
            self.create_service(
                Trigger,
                f'/drone_{i}/hover',
                lambda req, res, drone_id=i: self.hover_callback(req, res, drone_id),
                callback_group=self.callback_group
            )
            
            # 特定のポイントに飛行するためのトピックのサブスクライバー
            self.create_subscription(
                Point,
                f'/drone_{i}/fly_to_point',
                lambda msg, drone_id=i: self.fly_to_point_topic_callback(msg, drone_id),
                10,
                callback_group=self.callback_group
            )
        
        # TFインポーターの設定
        self.importer = create_tf_importer()
        
        # シミュレーションタイマーの設定
        self.timer = self.create_timer(self.dt, self.simulation_step)
        
        # シーンのビルド前にマーカーを追加するためのフラグ
        self.routes_to_add = []
        self.scene_built = False
        
        # デフォルトのルートを取得
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
                        # 最初のウェイポイントにアクティブマーカーを作成
                        if points:
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
                    
                    # 最初のウェイポイントにアクティブマーカーを作成
                    if points:
                        self.waypoint_manager.create_active_marker(drone_id_param, points[0], size=0.15)
            except Exception as e:
                self.get_logger().error(f"Error parsing route: {e}")
        
        # デフォルトのルートを追加（引数で指定されていない場合）
        if not self.routes_to_add:
            self.routes_to_add = default_routes
            
            # 各ドローンの最初のウェイポイントにアクティブマーカーを作成
            for drone_id, points in default_routes:
                if points:
                    self.waypoint_manager.create_active_marker(drone_id, points[0], size=0.15)
        
        # シーンのビルド
        self.scene.build()
        self.scene_built = True
        self.get_logger().info(f"Initialized {num_drones} drones")
        
        # ルートの設定（シーンビルド後）
        for drone_id, points in self.routes_to_add:
            # 飛行コントローラにルートを設定
            self.flight_controllers[drone_id].set_route(points)
            
            # 飛行を開始
            self.flight_controllers[drone_id].start_route()
    
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
                self.get_logger().info(f"Drone {drone_id} is flying to point {target_point}")
            else:
                self.get_logger().error(f"Failed to start flying drone {drone_id} to point {target_point}")
        except Exception as e:
            self.get_logger().error(f"Error in fly_to_point topic callback: {str(e)}")
    
    def simulation_step(self):
        """シミュレーションステップの実行"""
        step_start_time = self.timing_logger.start_timer()
            
        # 各ドローンの更新
        for i, drone in enumerate(self.drones):
            drone_start_time = self.timing_logger.start_timer()
                
            # 飛行コントローラの更新
            controller_start_time = self.timing_logger.start_timer()
            is_flying, waypoint_changed = self.flight_controllers[i].update()
            self.timing_logger.stop_timer(controller_start_time, f"drone_{i}_controller_update")
            
            # 現在のターゲットポイントにアクティブマーカーを更新
            marker_start_time = self.timing_logger.start_timer()
            if is_flying and self.flight_controllers[i].current_target is not None:
                # ウェイポイントが変更された場合は、アクティブマーカーを更新
                if waypoint_changed:
                    self.waypoint_manager.create_active_marker(i, self.flight_controllers[i].current_target, size=0.15)
                else:
                    self.waypoint_manager.update_active_marker(i, self.flight_controllers[i].current_target)
            self.timing_logger.stop_timer(marker_start_time, f"drone_{i}_marker_update")
            
            # TF情報の発行
            tf_start_time = self.timing_logger.start_timer()
            if HAS_TF and self.importer:
                try:
                    tf_msg = get_tf_from_link(
                        self.scene.cur_t, 
                        drone.get_link("base_link"), 
                        "map", 
                        f"drone_{i}/base_link"
                    )
                    if tf_msg:
                        self.importer.write(tf_msg)
                    
                    # カメラリンクがあれば、そのTFも発行
                    try:
                        camera_link = drone.get_link("camera_link")
                        if camera_link:
                            tf_msg = get_tf_from_link(
                                self.scene.cur_t, 
                                camera_link, 
                                "map", 
                                f"drone_{i}/camera_link"
                            )
                            if tf_msg:
                                self.importer.write(tf_msg)
                    except Exception as e:
                        self.get_logger().debug(f"No camera link for drone {i}: {e}")
                except Exception as e:
                    self.get_logger().error(f"Error publishing TF for drone {i}: {e}")
            self.timing_logger.stop_timer(tf_start_time, f"drone_{i}_tf_publish")
            
            # カメラ画像を取得してROS 2トピックに発行
            camera_start_time = self.timing_logger.start_timer()
            if HAS_CAMERA:
                camera = self.cameras[i]
                if camera:
                    try:
                        # カメラの位置をドローンに合わせて更新（FPS視点）
                        pos_start_time = self.timing_logger.start_timer()
                        drone_pos = drone.get_pos().cpu().numpy()
                        drone_quat = drone.get_quat().cpu().numpy()
                        
                        # カメラの位置を更新（ドローンの姿勢に基づいて）
                        # 進行方向ベクトルは不要になったので、Noneを渡す
                        update_camera_position(camera, drone_pos, drone_quat, None)
                        self.timing_logger.stop_timer(pos_start_time, f"drone_{i}_camera_position_update")
                        
                        # カメラ画像のレンダリングと処理
                        if self.render_camera:
                            # カメラ画像のレンダリング
                            render_start_time = self.timing_logger.start_timer()
                            img = camera.render()
                            self.timing_logger.stop_timer(render_start_time, f"drone_{i}_camera_render")
                            
                            if img is not None:
                                # 画像処理
                                process_start_time = self.timing_logger.start_timer()
                                processed_img = process_camera_image(img)
                                
                                # ドローン情報を描画
                                info_img = draw_drone_info(
                                    processed_img, 
                                    i, 
                                    drone_pos, 
                                    self.flight_controllers[i].current_target
                                )
                                
                                # ルート情報を描画
                                if self.flight_controllers[i].route_points:
                                    route_img = draw_route_on_image(
                                        info_img,
                                        self.flight_controllers[i].route_points,
                                        self.flight_controllers[i].current_route_index,
                                        drone_pos
                                    )
                                else:
                                    route_img = info_img
                                
                                # BGRに変換
                                bgr_img = convert_to_bgr(route_img)
                                self.timing_logger.stop_timer(process_start_time, f"drone_{i}_image_processing")
                                
                                # カメラ画像をウィンドウに表示
                                display_start_time = self.timing_logger.start_timer()
                                if self.show_camera:
                                    display_camera_image(bgr_img, f'Drone {i} Camera')
                                self.timing_logger.stop_timer(display_start_time, f"drone_{i}_display_image")
                                
                                # ROS 2トピックに発行
                                publish_start_time = self.timing_logger.start_timer()
                                try:
                                    ros_img = self.bridge.cv2_to_imgmsg(processed_img, encoding="bgr8")
                                    ros_img.header.stamp = self.get_clock().now().to_msg()
                                    ros_img.header.frame_id = f"drone_{i}/camera_link"
                                    self.image_publishers[i].publish(ros_img)
                                except Exception as e:
                                    self.get_logger().error(f"Error publishing image: {e}")
                                self.timing_logger.stop_timer(publish_start_time, f"drone_{i}_publish_image")
                        else:
                            # レンダリングをスキップする場合は、時間計測のみ行う
                            render_start_time = self.timing_logger.start_timer()
                            self.timing_logger.stop_timer(render_start_time, f"drone_{i}_camera_render")
                            
                            # 画像処理をスキップ
                            process_start_time = self.timing_logger.start_timer()
                            self.timing_logger.stop_timer(process_start_time, f"drone_{i}_image_processing")
                            
                            # 表示をスキップ
                            display_start_time = self.timing_logger.start_timer()
                            self.timing_logger.stop_timer(display_start_time, f"drone_{i}_display_image")
                            
                            # 発行をスキップ
                            publish_start_time = self.timing_logger.start_timer()
                            self.timing_logger.stop_timer(publish_start_time, f"drone_{i}_publish_image")
                    except Exception as e:
                        self.get_logger().error(f"Error rendering image from drone {i}: {e}")
            self.timing_logger.stop_timer(camera_start_time, f"drone_{i}_camera_total")
                
            # ドローン全体の処理時間
            self.timing_logger.stop_timer(drone_start_time, f"drone_{i}_total")
        
        # シミュレーションステップを進める
        scene_step_start_time = self.timing_logger.start_timer()
        self.scene.step()
        self.timing_logger.stop_timer(scene_step_start_time, "scene_step")
            
        # 全体の処理時間
        self.timing_logger.stop_timer(step_start_time, "total_step")
            
        # ステップカウントを増やし、必要に応じて統計情報をログ出力
        self.timing_logger.increment_step(self.dt)
    
    def shutdown(self):
        """シミュレーションの終了処理"""
        # 最終的な統計情報をログに出力
        self.timing_logger.log_final_stats(self.dt)
        
        # TFインポーターの終了処理
        finish_tf_importer(self.importer)
        
        # 録画の停止
        if self.record_camera and len(self.cameras) > 0 and self.cameras[0] is not None:
            try:
                self.cameras[0].stop_recording(save_to_filename=self.output_file)
                self.get_logger().info(f"Saved camera recording to {self.output_file}")
            except Exception as e:
                self.get_logger().error(f"Failed to save camera recording: {e}")
        
        # OpenCVのウィンドウを閉じる
        if self.show_camera:
            cv2.destroyAllWindows()
        
        # 一時ファイルを削除
        cleanup_temp_files(self.temp_files)


def main():
    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description='マルチドローンシミュレーション')
    parser.add_argument('--num_drones', type=int, default=4, help='シミュレーションするドローンの数')
    parser.add_argument('--show_camera', action='store_true', help='カメラ画像をウィンドウに表示する')
    parser.add_argument('--route', type=str, help='飛行ルート（例: "1,1,2;-1,2,1;0,0,0.5"）')
    parser.add_argument('--drone_id', type=int, default=0, help='飛行ルートを適用するドローンのID')
    parser.add_argument('--routes', type=str, help='複数ドローンの飛行ルート（例: "0:1,1,2;-1,2,1|1:0,0,1;1,1,1"）')
    parser.add_argument('--record', action='store_true', help='カメラ映像を録画する')
    parser.add_argument('--no-render-camera', action='store_false', dest='render_camera', help='カメラのレンダリングを行わない（処理速度向上）')
    parser.add_argument('--output', type=str, default="fly_route_camera.mp4", help='録画ファイルの出力先')
    args = parser.parse_args()
    
    # ROS 2の初期化
    rclpy.init()
    
    # コマンドライン引数の値をログに出力
    print(f"Command line args: render_camera={args.render_camera}")
    
    # シミュレーションの開始
    simulation = MultiDroneSimulation(
        num_drones=args.num_drones, 
        show_camera=args.show_camera,
        record_camera=args.record,
        render_camera=args.render_camera,
        output_file=args.output
    )
    
    # コマンドライン引数からROS 2パラメータに値を設定
    if args.route:
        simulation.set_parameter(rclpy.parameter.Parameter('route', value=args.route))
    if args.routes:
        simulation.set_parameter(rclpy.parameter.Parameter('routes', value=args.routes))
    if args.drone_id != 0:  # デフォルト値でない場合のみ設定
        simulation.set_parameter(rclpy.parameter.Parameter('drone_id', value=args.drone_id))
    
    try:
        # ROS 2のスピン
        rclpy.spin(simulation)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了処理
        simulation.shutdown()
        simulation.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
