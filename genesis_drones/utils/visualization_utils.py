#!/usr/bin/env python3

import cv2
import numpy as np
from typing import Any, Tuple, Optional, List

def process_camera_image(img: Any) -> np.ndarray:
    """
    カメラ画像を処理
    
    Args:
        img: カメラからの画像データ
        
    Returns:
        np.ndarray: 処理された画像
    """
    # 画像データの処理
    if img is None:
        return np.zeros((240, 320, 3), dtype=np.uint8)
    
    if isinstance(img, tuple) and len(img) >= 1:
        # タプルの最初の要素が画像データ
        img_data = img[0]
        
        if img_data is None:
            return np.zeros((240, 320, 3), dtype=np.uint8)
        
        if isinstance(img_data, np.ndarray):
            # NumPy配列の場合
            if img_data.ndim == 3 and img_data.shape[2] > 3:
                # RGBAの場合はRGBに変換
                return img_data[:, :, :3]
            return img_data
        
        elif isinstance(img_data, list):
            # リストの場合
            try:
                data = np.array(img_data)
                height, width = 240, 320  # カメラの解像度
                
                if data.size == width * height * 3:  # RGB
                    return data.reshape(height, width, 3)
                elif data.size == width * height * 4:  # RGBA
                    rgba = data.reshape(height, width, 4)
                    return rgba[:, :, :3]  # RGBのみ使用
                else:
                    return np.zeros((height, width, 3), dtype=np.uint8)
            except Exception as e:
                print(f"Error processing image data: {e}")
                return np.zeros((240, 320, 3), dtype=np.uint8)
    
    elif isinstance(img, list) and all(isinstance(x, list) for x in img):
        # 2次元リストの場合
        height, width = 240, 320  # カメラの解像度
        img_array = np.zeros((height, width, 3), dtype=np.uint8)
        
        try:
            for y in range(min(height, len(img))):
                for x in range(min(width, len(img[y]))):
                    pixel = img[y][x]
                    if len(pixel) >= 3:  # RGB値があることを確認
                        img_array[y, x] = pixel[:3]  # RGBのみ使用
            return img_array
        except Exception as e:
            print(f"Error processing image list: {e}")
            return np.zeros((height, width, 3), dtype=np.uint8)
    
    elif isinstance(img, np.ndarray):
        # NumPy配列の場合
        if img.ndim == 3 and img.shape[2] >= 3:
            return img[:, :, :3]
        return img
    
    # その他の場合は空の画像を返す
    return np.zeros((240, 320, 3), dtype=np.uint8)

def convert_to_bgr(img: np.ndarray) -> np.ndarray:
    """
    RGB画像をBGR形式に変換（OpenCV用）
    
    Args:
        img (np.ndarray): RGB画像
        
    Returns:
        np.ndarray: BGR画像
    """
    if img.ndim == 3 and img.shape[2] == 3:
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img

def display_camera_image(img: np.ndarray, window_name: str = 'Camera') -> None:
    """
    カメラ画像をウィンドウに表示
    
    Args:
        img (np.ndarray): 表示する画像
        window_name (str): ウィンドウ名
    """
    cv2.imshow(window_name, img)
    cv2.waitKey(1)  # 1ミリ秒待機

def draw_drone_info(img: np.ndarray, drone_id: int, position: Tuple[float, float, float], 
                   target: Optional[Tuple[float, float, float]] = None) -> np.ndarray:
    """
    画像にドローン情報を描画
    
    Args:
        img (np.ndarray): 描画する画像
        drone_id (int): ドローンID
        position (tuple): ドローンの位置
        target (tuple): 目標位置（オプション）
        
    Returns:
        np.ndarray: 描画された画像
    """
    # 画像のコピーを作成
    result = img.copy()
    
    # ドローン情報のテキスト
    text_lines = [
        f"Drone ID: {drone_id}",
        f"Position: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})"
    ]
    
    # 目標位置がある場合は追加
    if target is not None:
        text_lines.append(f"Target: ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f})")
        
        # 目標までの距離を計算
        distance = np.sqrt(
            (target[0] - position[0])**2 + 
            (target[1] - position[1])**2 + 
            (target[2] - position[2])**2
        )
        text_lines.append(f"Distance: {distance:.2f}")
    
    # テキストを描画
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    font_thickness = 1
    font_color = (255, 255, 255)  # 白色
    
    y_offset = 20
    for i, line in enumerate(text_lines):
        y = y_offset + i * 20
        cv2.putText(result, line, (10, y), font, font_scale, font_color, font_thickness)
    
    return result

def draw_route_on_image(img: np.ndarray, route_points: List[Tuple[float, float, float]], 
                       current_index: int, drone_position: Tuple[float, float, float]) -> np.ndarray:
    """
    画像にルート情報を描画
    
    Args:
        img (np.ndarray): 描画する画像
        route_points (list): ルートポイントのリスト
        current_index (int): 現在のルートインデックス
        drone_position (tuple): ドローンの現在位置
        
    Returns:
        np.ndarray: 描画された画像
    """
    # 画像のコピーを作成
    result = img.copy()
    
    # 画像の中心と縮尺
    center_x, center_y = result.shape[1] // 2, result.shape[0] // 2
    scale = 50  # ピクセル/メートル
    
    # ドローンの位置を画像座標に変換
    drone_x = int(center_x + drone_position[0] * scale)
    drone_y = int(center_y - drone_position[1] * scale)  # Y軸は反転
    
    # ドローンの位置を描画
    cv2.circle(result, (drone_x, drone_y), 5, (0, 0, 255), -1)  # 赤い円
    
    # ルートポイントを描画
    for i, point in enumerate(route_points):
        # ポイントの位置を画像座標に変換
        point_x = int(center_x + point[0] * scale)
        point_y = int(center_y - point[1] * scale)  # Y軸は反転
        
        # 現在のポイントは緑、過去のポイントは青、未来のポイントは黄色
        if i == current_index:
            color = (0, 255, 0)  # 緑
        elif i < current_index:
            color = (255, 0, 0)  # 青
        else:
            color = (0, 255, 255)  # 黄色
        
        # ポイントを描画
        cv2.circle(result, (point_x, point_y), 3, color, -1)
        
        # ポイント間を線で結ぶ
        if i > 0:
            prev_point = route_points[i-1]
            prev_x = int(center_x + prev_point[0] * scale)
            prev_y = int(center_y - prev_point[1] * scale)
            cv2.line(result, (prev_x, prev_y), (point_x, point_y), (255, 255, 255), 1)
    
    # 現在のポイントとドローンを線で結ぶ
    if 0 <= current_index < len(route_points):
        current_point = route_points[current_index]
        current_x = int(center_x + current_point[0] * scale)
        current_y = int(center_y - current_point[1] * scale)
        cv2.line(result, (drone_x, drone_y), (current_x, current_y), (0, 255, 0), 1)
    
    return result

def create_multi_view_image(images: List[np.ndarray]) -> np.ndarray:
    """
    複数の画像を1つのマルチビュー画像に結合
    
    Args:
        images (list): 画像のリスト
        
    Returns:
        np.ndarray: 結合された画像
    """
    if not images:
        return np.zeros((240, 320, 3), dtype=np.uint8)
    
    # 画像の数に応じてレイアウトを決定
    num_images = len(images)
    
    if num_images == 1:
        return images[0]
    
    elif num_images == 2:
        # 横に2つ並べる
        return np.hstack(images)
    
    elif num_images <= 4:
        # 2x2グリッド
        # 足りない画像は黒で埋める
        while len(images) < 4:
            images.append(np.zeros_like(images[0]))
        
        top_row = np.hstack(images[:2])
        bottom_row = np.hstack(images[2:4])
        return np.vstack([top_row, bottom_row])
    
    else:
        # 3x3グリッド
        # 足りない画像は黒で埋める
        while len(images) < 9:
            images.append(np.zeros_like(images[0]))
        
        rows = []
        for i in range(0, 9, 3):
            row = np.hstack(images[i:i+3])
            rows.append(row)
        
        return np.vstack(rows)
