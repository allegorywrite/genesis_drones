#!/usr/bin/env python3

import numpy as np
import cv2
from genesis.vis.camera import Camera

def process_camera_image(img, width=320, height=240):
    """
    カメラ画像を処理する関数
    
    Args:
        img: カメラからのレンダリング結果
        width (int): 画像の幅
        height (int): 画像の高さ
        
    Returns:
        numpy.ndarray: 処理された画像
    """
    # 画像がNoneの場合は空の配列を返す
    if img is None:
        return np.zeros((height, width, 3), dtype=np.uint8)
    
    # タプルの場合の処理（Genesisのカメラレンダリング結果がタプルの場合）
    if isinstance(img, tuple) and len(img) == 4:
        # タプルの最初の要素が画像データ
        # 空のnumpy配列を作成
        img_array = np.zeros((height, width, 3), dtype=np.uint8)
        
        # タプルから画像データを抽出
        try:
            # タプルの最初の要素が画像データの場合
            if img[0] is not None:
                # 画像データの形式によって処理を分岐
                if isinstance(img[0], np.ndarray):
                    # 既にnumpy配列の場合
                    img_array = img[0]
                    if img_array.ndim == 3 and img_array.shape[2] > 3:
                        img_array = img_array[:, :, :3]  # RGBのみ使用
                elif isinstance(img[0], list):
                    # リストの場合、numpy配列に変換
                    data = np.array(img[0])
                    if data.size > 0:
                        # データの形状を推測して再構成
                        if data.size == width * height * 3:  # RGB
                            img_array = data.reshape(height, width, 3)
                        elif data.size == width * height * 4:  # RGBA
                            rgba = data.reshape(height, width, 4)
                            img_array = rgba[:, :, :3]  # RGBのみ使用
        except Exception as e:
            print(f"Error processing image data: {e}")
        
        return img_array
    
    # 画像データが配列のリストの場合
    elif isinstance(img, list) and all(isinstance(x, list) for x in img):
        # RGB画像として再構築（アルファチャンネルを除外）
        img_array = np.zeros((height, width, 3), dtype=np.uint8)
        
        # 1次元リストを2次元画像に変換
        for y in range(height):
            for x in range(width):
                if y*width + x < len(img):
                    pixel = img[y*width + x]
                    if len(pixel) >= 3:  # RGB値があることを確認
                        img_array[y, x] = pixel[:3]  # RGBのみ使用
        
        return img_array
    
    # 既にnumpy配列の場合は形状を確認
    elif isinstance(img, np.ndarray):
        if img.ndim == 3 and img.shape[2] >= 3:
            # 3チャンネル以上ある場合、最初の3チャンネルのみ使用
            return img[:, :, :3]
        return img
    
    # その他の場合は空の配列を返す
    return np.zeros((height, width, 3), dtype=np.uint8)

def convert_to_bgr(img):
    """
    画像をBGR形式に変換する関数
    
    Args:
        img (numpy.ndarray): 入力画像
        
    Returns:
        numpy.ndarray: BGR形式の画像
    """
    if img is None:
        return None
    
    # OpenCVでBGR形式に変換（必要に応じて）
    if img.ndim == 3 and img.shape[2] == 3:
        # BGRに変換（RGBの場合）
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    
    return img

def display_camera_image(img, window_name='Camera'):
    """
    カメラ画像をウィンドウに表示する関数
    
    Args:
        img (numpy.ndarray): 表示する画像
        window_name (str): ウィンドウ名
    """
    if img is not None:
        cv2.imshow(window_name, img)
        cv2.waitKey(1)  # 1ミリ秒待機

def update_camera_position(camera, drone_pos, drone_quat=None, direction=None):
    """
    ドローンの位置と進行方向に合わせてカメラの位置と向きを更新する関数
    
    Args:
        camera (Camera): カメラオブジェクト
        drone_pos (numpy.ndarray): ドローンの位置
        drone_quat (numpy.ndarray, optional): ドローンの姿勢クォータニオン
        direction (numpy.ndarray, optional): ドローンの進行方向ベクトル
    """
    if camera is None or drone_pos is None:
        return
    
    # 進行方向が指定されている場合
    if direction is not None and isinstance(direction, np.ndarray) and direction.size >= 2:
        # 進行方向の単位ベクトルを計算
        direction_norm = np.linalg.norm(direction[:2])
        if direction_norm > 0.01:  # 十分な速度がある場合のみ向きを変更
            # 単位ベクトルに変換
            direction_unit = direction / direction_norm
            
            # カメラをドローンの少し上方に配置
            camera_pos = (
                drone_pos[0],
                drone_pos[1],
                drone_pos[2] + 0.1  # ドローンの少し上方に配置
            )
            
            # 進行方向を見るようにlookatを設定
            lookat_pos = (
                drone_pos[0] + direction_unit[0] * 2.0,
                drone_pos[1] + direction_unit[1] * 2.0,
                drone_pos[2]
            )
            
            # カメラの位置と向きを設定
            camera.set_pose(
                pos=camera_pos,
                lookat=lookat_pos
            )
        else:
            # 速度が小さい場合はデフォルトの向きを使用
            camera.set_pose(
                pos=(drone_pos[0], drone_pos[1], drone_pos[2] + 0.1),
                lookat=(drone_pos[0] + 1.0, drone_pos[1], drone_pos[2])
            )
    else:
        # 進行方向が指定されていない場合はデフォルトの向きを使用
        camera.set_pose(
            pos=(drone_pos[0], drone_pos[1], drone_pos[2] + 0.1),  # ドローンの少し上方に配置
            lookat=(drone_pos[0] + 1.0, drone_pos[1], drone_pos[2])  # 前方を見る
        )
