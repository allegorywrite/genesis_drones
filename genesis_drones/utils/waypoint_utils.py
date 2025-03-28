#!/usr/bin/env python3

import numpy as np
import genesis as gs
from typing import List, Tuple, Optional, Any

class WaypointManager:
    """ウェイポイントの管理と描画を行うクラス"""
    
    def __init__(self, scene: Any):
        """
        ウェイポイントマネージャーの初期化
        
        Args:
            scene (gs.Scene): Genesisのシーン
        """
        self.scene = scene
        self.waypoint_markers = []
        self.route_lines = []
        self.active_markers = {}  # ドローンIDをキーとする現在のウェイポイントマーカー
        
        # ドローンIDに応じた色の定義
        self.colors = [
            (1.0, 0.0, 0.0),  # 赤
            (0.0, 1.0, 0.0),  # 緑
            (0.0, 0.0, 1.0),  # 青
            (1.0, 1.0, 0.0),  # 黄
            (1.0, 0.0, 1.0),  # マゼンタ
            (0.0, 1.0, 1.0),  # シアン
            (0.5, 0.5, 0.5),  # グレー
            (1.0, 0.5, 0.0),  # オレンジ
        ]
    
    def add_waypoint_marker(self, position: Tuple[float, float, float], 
                           color: Tuple[float, float, float] = (1.0, 0.0, 0.0), 
                           size: float = 0.1) -> Optional[Any]:
        """
        ウェイポイントのマーカーを追加する
        
        Args:
            position (tuple): マーカーの位置 (x, y, z)
            color (tuple): マーカーの色 (r, g, b)
            size (float): マーカーのサイズ
            
        Returns:
            Entity: 追加されたマーカーのエンティティ
        """
        try:
            # 球体のマーカーを追加
            marker = self.scene.add_entity(
                morph=gs.morphs.Sphere(
                    pos=position,
                    radius=size,
                    fixed=True,  # 物理シミュレーションの影響を受けないように固定
                    collision=False
                ),
                surface=gs.surfaces.Rough(
                    diffuse_texture=gs.textures.ColorTexture(
                        color=color
                    )
                )
            )
            self.waypoint_markers.append(marker)
            return marker
        except Exception as e:
            print(f"Failed to add waypoint marker: {e}")
            return None
    
    def add_route_markers(self, points: List[Tuple[float, float, float]], 
                         drone_id: int = 0, 
                         size: float = 0.1,
                         add_lines: bool = False) -> List[Any]:
        """
        ルート上の全てのポイントにマーカーを追加する
        
        Args:
            points (list): ポイントのリスト [(x1, y1, z1), (x2, y2, z2), ...]
            drone_id (int): ドローンのID（色の選択に使用）
            size (float): マーカーのサイズ
            add_lines (bool): ポイント間に線を追加するかどうか
            
        Returns:
            list: 追加されたマーカーのリスト
        """
        if not points:
            return []
        
        # ドローンIDに応じた色を選択
        color = self.colors[drone_id % len(self.colors)]
        markers = []
        
        # 各ポイントにマーカーを追加
        for point in points:
            marker = self.add_waypoint_marker(point, color, size)
            if marker:
                markers.append(marker)
        
        # ポイント間に線を追加（オプション）
        if add_lines and len(points) > 1:
            self._add_route_lines(points, color)
        
        return markers
    
    def _add_route_lines(self, points: List[Tuple[float, float, float]], 
                        color: Tuple[float, float, float]) -> None:
        """
        ルートポイント間に線を追加する
        
        Args:
            points (list): ポイントのリスト [(x1, y1, z1), (x2, y2, z2), ...]
            color (tuple): 線の色 (r, g, b)
        """
        try:
            for i in range(len(points) - 1):
                start = points[i]
                end = points[i + 1]
                
                # 線の方向と長さを計算
                direction = np.array(end) - np.array(start)
                length = np.linalg.norm(direction)
                
                if length > 0:
                    # 線の中心点と向きを計算
                    center = (np.array(start) + np.array(end)) / 2
                    
                    # 線（細長い円柱）を追加
                    line = self.scene.add_entity(
                        morph=gs.morphs.Capsule(
                            pos=tuple(center),
                            radius=0.02,  # 線の太さ
                            length=length,
                            fixed=True,  # 物理シミュレーションの影響を受けないように固定
                            collision=False
                        ),
                        surface=gs.surfaces.Rough(
                            diffuse_texture=gs.textures.ColorTexture(
                                color=color
                            )
                        )
                    )
                    
                    # 線の向きを設定
                    if hasattr(line, 'set_rotation'):
                        # 方向ベクトルから回転を計算（簡易的な実装）
                        dir_norm = direction / length
                        # Z軸方向の単位ベクトル
                        z_axis = np.array([0, 0, 1])
                        # 回転軸（Z軸と方向ベクトルの外積）
                        rotation_axis = np.cross(z_axis, dir_norm)
                        # 回転角（Z軸と方向ベクトルのなす角）
                        angle = np.arccos(np.dot(z_axis, dir_norm))
                        
                        if np.linalg.norm(rotation_axis) > 0.001:  # 回転軸が十分な大きさを持つ場合
                            # 回転を設定
                            line.set_rotation(rotation_axis[0], rotation_axis[1], rotation_axis[2], angle)
                    
                    self.route_lines.append(line)
        except Exception as e:
            print(f"Failed to add route lines: {e}")
    
    def create_active_marker(self, drone_id: int, position: Tuple[float, float, float], 
                           size: float = 0.15) -> Optional[Any]:
        """
        現在のウェイポイントを示す単一のマーカーを作成または更新
        
        Args:
            drone_id (int): ドローンID
            position (tuple): マーカーの位置 (x, y, z)
            size (float): マーカーのサイズ
            
        Returns:
            Entity: 作成されたマーカーのエンティティ
        """
        # ドローンIDに応じた色を選択
        color = self.colors[drone_id % len(self.colors)]
        
        # 既存のマーカーがあれば位置を更新
        if drone_id in self.active_markers and self.active_markers[drone_id] is not None:
            try:
                self.active_markers[drone_id].set_pos(position)
                return self.active_markers[drone_id]
            except Exception as e:
                print(f"Failed to update active marker for drone {drone_id}: {e}")
                # 失敗した場合は新しいマーカーを作成
        
        # 新しいマーカーを作成
        try:
            # 球体のマーカーを追加
            marker = self.scene.add_entity(
                morph=gs.morphs.Sphere(
                    pos=position,
                    radius=size,
                    fixed=False,  # 物理シミュレーションの影響を受けないように固定
                    collision=False
                ),
                surface=gs.surfaces.Rough(
                    diffuse_texture=gs.textures.ColorTexture(
                        color=color
                    )
                )
            )
            self.active_markers[drone_id] = marker
            return marker
        except Exception as e:
            print(f"Failed to create active marker for drone {drone_id}: {e}")
            return None
    
    def update_active_marker(self, drone_id: int, position: Tuple[float, float, float]) -> None:
        """
        現在のウェイポイントマーカーの位置を更新
        
        Args:
            drone_id (int): ドローンID
            position (tuple): 新しい位置 (x, y, z)
        """
        if drone_id in self.active_markers and self.active_markers[drone_id] is not None:
            try:
                self.active_markers[drone_id].set_pos(position)
            except Exception as e:
                print(f"Failed to update active marker position for drone {drone_id}: {e}")
    
    def clear_markers(self) -> None:
        """全てのウェイポイントマーカーをクリアする"""
        # 通常のマーカーをクリア
        for marker in self.waypoint_markers:
            try:
                self.scene.remove_entity(marker)
            except Exception as e:
                print(f"Failed to remove waypoint marker: {e}")
        
        self.waypoint_markers = []
        
        # アクティブマーカーをクリア
        for drone_id, marker in self.active_markers.items():
            if marker is not None:
                try:
                    self.scene.remove_entity(marker)
                except Exception as e:
                    print(f"Failed to remove active marker for drone {drone_id}: {e}")
        
        self.active_markers = {}
        
        # 線もクリア
        for line in self.route_lines:
            try:
                self.scene.remove_entity(line)
            except Exception as e:
                print(f"Failed to remove route line: {e}")
        
        self.route_lines = []

def create_default_routes(num_drones: int) -> List[Tuple[int, List[Tuple[float, float, float]]]]:
    """
    デフォルトの飛行ルートを作成する
    
    Args:
        num_drones (int): ドローンの数
        
    Returns:
        list: (drone_id, points)のリスト
    """
    default_routes = []
    
    # 基本的な円形のルート
    for i in range(num_drones):
        angle_offset = (2 * np.pi * i) / num_drones
        radius = 1.5
        height = 1.0
        
        # 円形のルートを生成
        points = []
        # 初期位置（ドローンの配置位置）
        points.append((0, 0, 0.5))
        # 上昇
        points.append((i*0.1, 0, height))
        
        # 円形のポイントを追加
        num_points = 8  # 円周上のポイント数
        for j in range(num_points):
            angle = angle_offset + (2 * np.pi * j) / num_points
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            points.append((x, y, height))
        
        # 最後に初期位置の上空に戻る
        points.append((0, 0, height))
        # 着陸
        points.append((0, 0, 0.5))
        
        default_routes.append((i, points))
    
    return default_routes
