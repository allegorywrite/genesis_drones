#!/usr/bin/env python3

import numpy as np
from typing import Any, List, Tuple, Optional

from genesis_drones.controllers.base_controller import BaseController

class DroneFlightController:
    """ドローンの飛行を制御するクラス"""
    
    def __init__(self, drone: Any, controller: BaseController, logger=None):
        """
        ドローン飛行コントローラの初期化
        
        Args:
            drone (DroneEntity): 制御対象のドローン
            controller (BaseController): ドローンコントローラ
            logger: ロガー（オプション）
        """
        self.drone = drone
        self.controller = controller
        self.logger = logger
        
        self.is_flying = False
        self.current_target = None
        self.route_points = []
        self.current_route_index = 0
        self.distance_threshold = 0.1  # 目標に到達したと判断する距離のしきい値
    
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
        self.is_flying = True
        self.current_target = target_point
        
        self.log(f"Flying to point {target_point}")
        return True
    
    def start_route(self) -> bool:
        """
        ルートの飛行を開始
        
        Returns:
            bool: 成功したかどうか
        """
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
        if not self.is_flying or self.current_target is None:
            return False, False
        
        # ドローンの現在位置を取得
        drone_pos = self.drone.get_pos().cpu().numpy()
        
        # 目標位置までの距離を計算
        target = self.current_target
        x_diff = target[0] - drone_pos[0]
        y_diff = target[1] - drone_pos[1]
        z_diff = target[2] - drone_pos[2]
        distance = np.sqrt(x_diff**2 + y_diff**2 + z_diff**2)
        
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
        rpms = self.controller.update(target)
        
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
        
        # ホバリング状態に設定
        if hasattr(self.controller, 'hover'):
            self.controller.hover()
