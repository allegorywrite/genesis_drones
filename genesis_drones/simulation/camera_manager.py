"""
Camera Manager Module

このモジュールはカメラ関連の処理を担当します。
"""

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node

class CameraManager:
    """カメラ関連の処理を担当するクラス"""
    
    def __init__(self, node, show_camera=False, record_camera=False, render_camera=True, output_file="output.mp4"):
        """
        CameraManagerの初期化
        
        Args:
            node: ROS2ノード
            show_camera: カメラ画像をウィンドウに表示するかどうか
            record_camera: カメラ映像を録画するかどうか
            render_camera: カメラのレンダリングを行うかどうか
            output_file: 録画ファイルの出力先
        """
        self.node = node
        self.logger = node.get_logger()
        self.show_camera = show_camera
        self.record_camera = record_camera
        self.render_camera = render_camera
        self.output_file = output_file
        self.bridge = CvBridge()
        self.image_publishers = []
        self.cameras = []
        self.has_camera = True
        
        try:
            # カメラ関連の機能が利用可能かチェック
            from genesis_drones.utils.visualization_utils import (
                process_camera_image, convert_to_bgr, display_camera_image,
                draw_drone_info, draw_route_on_image, create_multi_view_image
            )
            from genesis_drones.utils.simulation_utils import (
                add_camera_to_scene, update_camera_position
            )
            self.has_camera = True
        except ImportError:
            self.logger.warn("Camera utils could not be imported")
            self.has_camera = False
    
    def setup_camera(self, scene, drone_id, drone_name_prefix="drone"):
        """
        カメラのセットアップ
        
        Args:
            scene: Genesisシーン
            drone_id: ドローンID
            drone_name_prefix: ドローン名のプレフィックス
            
        Returns:
            object: カメラオブジェクト（失敗した場合はNone）
        """
        if not self.has_camera:
            return None
            
        try:
            from genesis_drones.utils.simulation_utils import add_camera_to_scene
            
            # カメラの追加
            camera_position = (0, 0, 0.5)  # ドローンの少し前方
            camera_lookat = (0, 0, 0.5)  # より遠くを見る
            
            camera = add_camera_to_scene(
                scene,
                position=camera_position,
                lookat=camera_lookat
            )
            
            # ROS 2パブリッシャーの設定
            pub = self.node.create_publisher(
                Image,
                f'/{drone_name_prefix}_{drone_id}/camera/image_raw',
                10
            )
            self.image_publishers.append(pub)
            
            # 録画の開始
            if self.record_camera and drone_id == 0:  # 最初のドローンのカメラのみ録画
                camera.start_recording()
                
            self.cameras.append(camera)
            return camera
        except Exception as e:
            self.logger.error(f"Failed to add camera for {drone_name_prefix}_{drone_id}: {e}")
            self.cameras.append(None)
            self.image_publishers.append(None)
            return None
    
    def update_camera(self, drone_id, drone, camera, flight_controller, timing_logger=None):
        """
        カメラの更新
        
        Args:
            drone_id: ドローンID
            drone: ドローンオブジェクト
            camera: カメラオブジェクト
            flight_controller: 飛行コントローラ
            timing_logger: タイミングロガー
            
        Returns:
            bool: 更新に成功したかどうか
        """
        if not self.has_camera or camera is None:
            return False
            
        try:
            from genesis_drones.utils.simulation_utils import update_camera_position
            from genesis_drones.utils.visualization_utils import (
                process_camera_image, convert_to_bgr, display_camera_image,
                draw_drone_info, draw_route_on_image
            )
            
            # タイミングロガーの初期化
            if timing_logger:
                camera_start_time = timing_logger.start_timer()
            
            # カメラの位置をドローンに合わせて更新（FPS視点）
            if timing_logger:
                pos_start_time = timing_logger.start_timer()
                
            drone_pos = drone.get_pos().cpu().numpy()
            drone_quat = drone.get_quat().cpu().numpy()
            
            # カメラの位置を更新（ドローンの姿勢に基づいて）
            update_camera_position(camera, drone_pos, drone_quat, None)
            
            if timing_logger:
                timing_logger.stop_timer(pos_start_time, f"drone_{drone_id}_camera_position_update")
            
            # カメラ画像のレンダリングと処理
            if self.render_camera:
                # カメラ画像のレンダリング
                if timing_logger:
                    render_start_time = timing_logger.start_timer()
                    
                img = camera.render()
                
                if timing_logger:
                    timing_logger.stop_timer(render_start_time, f"drone_{drone_id}_camera_render")
                
                if img is not None:
                    # 画像処理
                    if timing_logger:
                        process_start_time = timing_logger.start_timer()
                        
                    processed_img = process_camera_image(img)
                    
                    # ドローン情報を描画
                    info_img = draw_drone_info(
                        processed_img, 
                        drone_id, 
                        drone_pos, 
                        flight_controller.current_target
                    )
                    
                    # ルート情報を描画
                    if flight_controller.route_points:
                        route_img = draw_route_on_image(
                            info_img,
                            flight_controller.route_points,
                            flight_controller.current_route_index,
                            drone_pos
                        )
                    else:
                        route_img = info_img
                    
                    # BGRに変換
                    bgr_img = convert_to_bgr(route_img)
                    
                    if timing_logger:
                        timing_logger.stop_timer(process_start_time, f"drone_{drone_id}_image_processing")
                    
                    # カメラ画像をウィンドウに表示
                    if timing_logger:
                        display_start_time = timing_logger.start_timer()
                        
                    if self.show_camera:
                        display_camera_image(bgr_img, f'Drone {drone_id} Camera')
                        
                    if timing_logger:
                        timing_logger.stop_timer(display_start_time, f"drone_{drone_id}_display_image")
                    
                    # ROS 2トピックに発行
                    if timing_logger:
                        publish_start_time = timing_logger.start_timer()
                        
                    try:
                        ros_img = self.bridge.cv2_to_imgmsg(processed_img, encoding="bgr8")
                        ros_img.header.stamp = self.node.get_clock().now().to_msg()
                        ros_img.header.frame_id = f"drone_{drone_id}/camera_link"
                        self.image_publishers[drone_id].publish(ros_img)
                    except Exception as e:
                        self.logger.error(f"Error publishing image: {e}")
                        
                    if timing_logger:
                        timing_logger.stop_timer(publish_start_time, f"drone_{drone_id}_publish_image")
            else:
                # レンダリングをスキップする場合は、時間計測のみ行う
                if timing_logger:
                    render_start_time = timing_logger.start_timer()
                    timing_logger.stop_timer(render_start_time, f"drone_{drone_id}_camera_render")
                    
                    # 画像処理をスキップ
                    process_start_time = timing_logger.start_timer()
                    timing_logger.stop_timer(process_start_time, f"drone_{drone_id}_image_processing")
                    
                    # 表示をスキップ
                    display_start_time = timing_logger.start_timer()
                    timing_logger.stop_timer(display_start_time, f"drone_{drone_id}_display_image")
                    
                    # 発行をスキップ
                    publish_start_time = timing_logger.start_timer()
                    timing_logger.stop_timer(publish_start_time, f"drone_{drone_id}_publish_image")
            
            if timing_logger:
                timing_logger.stop_timer(camera_start_time, f"drone_{drone_id}_camera_total")
                
            return True
        except Exception as e:
            self.logger.error(f"Error updating camera for drone {drone_id}: {e}")
            return False
    
    def shutdown(self):
        """CameraManagerの終了処理"""
        # 録画の停止
        if self.record_camera and len(self.cameras) > 0 and self.cameras[0] is not None:
            try:
                self.cameras[0].stop_recording(save_to_filename=self.output_file)
                self.logger.info(f"Saved camera recording to {self.output_file}")
            except Exception as e:
                self.logger.error(f"Failed to save camera recording: {e}")
        
        # OpenCVのウィンドウを閉じる
        if self.show_camera:
            cv2.destroyAllWindows()
