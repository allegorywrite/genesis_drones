"""
TF Publisher Module

このモジュールはTF情報の発行を担当します。
"""

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

class TFPublisher:
    """TF情報の発行を担当するクラス"""
    
    def __init__(self, logger, importer=None):
        """
        TFPublisherの初期化
        
        Args:
            logger: ROS2のロガー
            importer: TFインポーター
        """
        self.logger = logger
        self.importer = importer
        self.has_tf = False
        
        try:
            # TF関連の機能が利用可能かチェック
            from genesis_drones.utils.tf_utils import HAS_TF
            self.has_tf = HAS_TF
        except ImportError:
            self.logger.warn("TF utils could not be imported")
            self.has_tf = False
    
    def publish_drone_tf(self, scene_time, drone, drone_id, drone_name_prefix="drone"):
        """
        ドローンのTF情報を発行
        
        Args:
            scene_time: シーンの現在時刻
            drone: ドローンオブジェクト
            drone_id: ドローンID
            drone_name_prefix: ドローン名のプレフィックス
            
        Returns:
            bool: 発行に成功したかどうか
        """
        if not self.has_tf or not self.importer:
            return False
            
        try:
            from genesis_drones.utils.tf_utils import get_tf_from_link
            
            # ベースリンクのTF情報を発行
            tf_msg = get_tf_from_link(
                scene_time, 
                drone.get_link("base_link"), 
                "map", 
                f"{drone_name_prefix}_{drone_id}/base_link"
            )
            if tf_msg:
                self.importer.write(tf_msg)
            
            # カメラリンクがあれば、そのTFも発行
            try:
                camera_link = drone.get_link("camera_link")
                if camera_link:
                    tf_msg = get_tf_from_link(
                        scene_time, 
                        camera_link, 
                        "map", 
                        f"{drone_name_prefix}_{drone_id}/camera_link"
                    )
                    if tf_msg:
                        self.importer.write(tf_msg)
            except Exception as e:
                self.logger.debug(f"No camera link for {drone_name_prefix}_{drone_id}: {e}")
                
            return True
        except Exception as e:
            self.logger.error(f"Error publishing TF for {drone_name_prefix}_{drone_id}: {e}")
            return False
    
    def shutdown(self):
        """TFPublisherの終了処理"""
        if self.has_tf and self.importer:
            try:
                from genesis_drones.utils.tf_utils import finish_tf_importer
                finish_tf_importer(self.importer)
                self.logger.info("TF importer finished")
            except Exception as e:
                self.logger.error(f"Error finishing TF importer: {e}")
