#!/usr/bin/env python3
"""
prob_analysis.mdに従って、以下の3つの関数を3Dポテンシャルとして描画するスクリプト：
1. Psi_{i}^l - 視野内にあるかどうかを示す関数
2. P_i^l - 確率関数
3. sigma^2 (1/f^2) [(P_beta_i/d_i^2) + (P_beta_j/d_j^2)]^(-1) - 共分散行列に関する式
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import sys
import os
import matplotlib as mpl
import argparse
from scipy.spatial.transform import Rotation

# 日本語フォントの設定
mpl.rcParams['font.family'] = 'DejaVu Sans'

# se3_drone_simulatorのパスを追加
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from se3_drone_simulator.utils.se3 import SE3
from se3_drone_simulator.utils.drone import Drone

def create_3d_grid(x_range, y_range, z_range, resolution):
    """
    3Dグリッドを生成する関数
    
    Parameters:
    -----------
    x_range : tuple
        x軸の範囲 (min, max)
    y_range : tuple
        y軸の範囲 (min, max)
    z_range : tuple
        z軸の範囲 (min, max)
    resolution : int
        各軸の解像度
        
    Returns:
    --------
    X, Y, Z : ndarray
        3Dグリッドの座標
    """
    x = np.linspace(x_range[0], x_range[1], resolution)
    y = np.linspace(y_range[0], y_range[1], resolution)
    z = np.linspace(z_range[0], z_range[1], resolution)
    X, Y, Z = np.meshgrid(x, y, z)
    return X, Y, Z

def plot_psi_function(resolution_r=100, resolution_theta=100):
    """
    Psi_{i}^l関数を3Dポテンシャルとして描画する関数（極座標メッシュ使用）
    
    Parameters:
    -----------
    resolution_r : int, optional
        半径方向の解像度（デフォルトは100）
    resolution_theta : int, optional
        角度方向の解像度（デフォルトは100）
    """
    # ドローンの設定
    drone_pos = np.array([0, 0, 0])
    drone_rot = np.eye(3)  # 単位回転行列
    drone = Drone(SE3(drone_rot, drone_pos), fov_angle=np.pi/3)
    
    # 極座標グリッドの生成
    
    r = np.linspace(0.1, 3, resolution_r)  # 半径（0は特異点なので避ける）
    theta = np.linspace(0, 2*np.pi, resolution_theta)  # 角度
    
    # 極座標から直交座標への変換
    R, THETA = np.meshgrid(r, theta)
    X = R * np.cos(THETA)
    Y = R * np.sin(THETA)
    
    # Psi_{i}^l値の計算
    Psi = np.zeros_like(X)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            point = np.array([X[i,j], Y[i,j], 1.0])  # z=1.0の平面上の点
            if drone.is_point_visible(point):
                Psi[i,j] = 1.0
            else:
                Psi[i,j] = 0.0
    
    # 3Dポテンシャルの描画
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 極座標メッシュを使用した滑らかな表面プロット
    surf = ax.plot_surface(X, Y, Psi, cmap=cm.viridis, edgecolor='green',linewidth=1.5, antialiased=True,
                          rcount=resolution_r, ccount=resolution_theta)
    
    # グラフの設定
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    # ax.set_zlabel(r'$\Psi_{i}^l$')
    # ax.set_title(r'$\Psi_{i}^l$ - Field of View Function')
    fig.colorbar(surf, shrink=0.5, aspect=5)
    
    # 保存と表示
    plt.savefig('psi_function_potential.png', dpi=300, bbox_inches='tight')
    plt.show()

def plot_p_function(resolution_r=100, resolution_theta=100):
    """
    P_i^l関数を3Dポテンシャルとして描画する関数（極座標メッシュ使用）
    
    Parameters:
    -----------
    resolution_r : int, optional
        半径方向の解像度（デフォルトは100）
    resolution_theta : int, optional
        角度方向の解像度（デフォルトは100）
    """
    # ドローンの設定
    drone_pos = np.array([0, 0, 0])
    drone_rot = np.eye(3)  # 単位回転行列
    drone = Drone(SE3(drone_rot, drone_pos), fov_angle=np.pi/3)
    
    # 極座標グリッドの生成
    
    r = np.linspace(0.1, 3, resolution_r)  # 半径（0は特異点なので避ける）
    theta = np.linspace(0, 2*np.pi, resolution_theta)  # 角度
    
    # 極座標から直交座標への変換
    R, THETA = np.meshgrid(r, theta)
    X = R * np.cos(THETA)
    Y = R * np.sin(THETA)
    
    # P_i^l値の計算
    P = np.zeros_like(X)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            point = np.array([X[i,j], Y[i,j], 1.0])  # z=1.0の平面上の点
            P[i,j] = drone.get_observation_probability(point)
    
    # 3Dポテンシャルの描画
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 極座標メッシュを使用した滑らかな表面プロット
    surf = ax.plot_surface(X, Y, P, cmap=cm.viridis, edgecolor='green',linewidth=1.5, antialiased=True,
                          rcount=resolution_r, ccount=resolution_theta)
    
    # グラフの設定
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    # ax.set_zlabel('$P_i^l$')
    # ax.set_title('$P_i^l$ - Observation Probability Function')
    fig.colorbar(surf, shrink=0.5, aspect=5)
    
    # 保存と表示
    plt.savefig('p_function_potential.png', dpi=300, bbox_inches='tight')
    plt.show()

def plot_covariance_function(resolution_r=100, resolution_theta=100):
    """
    共分散行列に関する式を3Dポテンシャルとして描画する関数（極座標メッシュ使用）
    ドローンの位置関係は別のグラフに表示
    x, y平面上にドローン1と2のFoVの接する断面の外形を楕円として描画
    
    Parameters:
    -----------
    resolution_r : int, optional
        半径方向の解像度（デフォルトは100）
    resolution_theta : int, optional
        角度方向の解像度（デフォルトは100）
    """
    # パラメータ設定
    sigma = 1.0  # ノイズの標準偏差
    f = 1.0  # 焦点距離
    
    # ドローンの設定（z軸上で向かい合う）
    drone1_pos = np.array([0, 0, -3])  # z軸負方向に配置
    drone2_pos = np.array([0, 0, 3])   # z軸正方向に配置
    
    # 任意のオイラー角でdrone1_rotを回転
    euler = np.array([np.pi/6, 0, 0])
    drone1_rot = Rotation.from_euler('xyz', euler).as_matrix()
    
    # 任意のオイラー角でdrone2_rotを回転
    euler = np.array([5*np.pi/6, 0, 0])
    drone2_rot = Rotation.from_euler('xyz', euler).as_matrix()
    
    drone1 = Drone(SE3(drone1_rot, drone1_pos), fov_angle=np.pi/6)
    drone2 = Drone(SE3(drone2_rot, drone2_pos), fov_angle=np.pi/6)
    
    # 極座標グリッドの生成
    r = np.linspace(0.1, 3, resolution_r)  # 半径（0は特異点なので避ける）
    theta = np.linspace(0, 2*np.pi, resolution_theta)  # 角度
    
    # 極座標から直交座標への変換
    R, THETA = np.meshgrid(r, theta)
    X = R * np.cos(THETA)
    Y = R * np.sin(THETA)
    
    # 共分散行列の値を計算
    Cov = np.zeros_like(X)
    Prop_revise = np.zeros_like(X)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            point = np.array([X[i,j], Y[i,j], 0.0])  # z=0.0の平面上の点
            
            # 点がどちらのドローンの視野にも入っていない場合は0
            if not (drone1.is_point_visible(point) and drone2.is_point_visible(point)):
                Cov[i,j] = 100.0
                continue
            
            # 各ドローンからの距離
            d1 = np.linalg.norm(point - drone1.T.p)
            d2 = np.linalg.norm(point - drone2.T.p)
            
            # βベクトル
            beta1 = drone1.get_beta_vector(point)
            beta2 = drone2.get_beta_vector(point)
            
            # 投影行列
            P_beta1 = np.eye(3) - np.outer(beta1, beta1)
            P_beta2 = np.eye(3) - np.outer(beta2, beta2)
            
            # 行列の和
            sum_matrix = P_beta1 / (d1**2) + P_beta2 / (d2**2)
            
            # 行列の逆行列の最大固有値（不確かさの指標）
            try:
                # 行列が特異でない場合
                inv_matrix = np.linalg.inv(sum_matrix)
                # トレースを使用
                Cov[i,j] = sigma**2 / (f**2) * np.trace(inv_matrix)
            except np.linalg.LinAlgError:
                # 行列が特異な場合
                Cov[i,j] = 0.0

            drone_1_prob = drone1.get_observation_probability(point)
            drone_2_prob = drone2.get_observation_probability(point)
            
            Prop_revise[i,j] = drone_1_prob * drone_2_prob
    
    Cov = np.clip(Cov, 0.01, 100)
    Prob = np.exp(-Cov) * Prop_revise
    
    # 2つのサブプロットを持つ図を作成
    fig = plt.figure(figsize=(18, 8))
    
    # ドローンの色を設定
    drone_colors = ['royalblue', 'crimson']
    
    # 1. ポテンシャルを描画するグラフ
    ax1 = fig.add_subplot(121, projection='3d')
    
    # 極座標メッシュを使用した滑らかな表面プロット
    surf = ax1.plot_surface(X, Y, Prob, cmap=cm.viridis, edgecolor='green',linewidth=1.5, antialiased=True, 
                          rcount=resolution_r, ccount=resolution_theta)
    
    # z=0平面上のドローン1と2のFoVの断面を楕円として描画
    # ドローン1のFoVの断面を計算
    center1, axes1, angle1, valid1 = calculate_fov_intersection_with_plane(drone1)
    
    # ドローン2のFoVの断面を計算
    center2, axes2, angle2, valid2 = calculate_fov_intersection_with_plane(drone2)
    
    # 楕円を描画するための角度
    theta = np.linspace(0, 2*np.pi, 100)
    
    # グラフの設定
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    # ax1.set_zlabel('Covariance')
    # ax1.set_title(r'$\sigma^2(1/f^2)[(P_{\beta_i}/d_i^2) + (P_{\beta_j}/d_j^2)]^{-1}$ - Covariance Matrix')
    fig.colorbar(surf, ax=ax1, shrink=0.5, aspect=5)
    ax1.legend()
    
    # 視点を調整
    ax1.view_init(elev=30, azim=20)
    
    # 2. ドローンの位置関係を描画するグラフ
    ax2 = fig.add_subplot(122, projection='3d')
    
    # ドローンの位置を表示
    
    # ドローン1の位置と姿勢を表示
    ax2.scatter([drone1.T.p[0]], [drone1.T.p[1]], [drone1.T.p[2]], color=drone_colors[0], s=100, label='Drone 1')
    
    # ドローン1の座標軸（姿勢）を表示
    length = 0.5
    R1 = drone1.T.R
    ax2.quiver(drone1.T.p[0], drone1.T.p[1], drone1.T.p[2], 
             R1[0, 0]*length, R1[1, 0]*length, R1[2, 0]*length, 
             color='r', arrow_length_ratio=0.1)
    ax2.quiver(drone1.T.p[0], drone1.T.p[1], drone1.T.p[2], 
             R1[0, 1]*length, R1[1, 1]*length, R1[2, 1]*length, 
             color='g', arrow_length_ratio=0.1)
    ax2.quiver(drone1.T.p[0], drone1.T.p[1], drone1.T.p[2], 
             R1[0, 2]*length, R1[1, 2]*length, R1[2, 2]*length, 
             color='b', arrow_length_ratio=0.1)
    
    # ドローン2の位置と姿勢を表示
    ax2.scatter([drone2.T.p[0]], [drone2.T.p[1]], [drone2.T.p[2]], color=drone_colors[1], s=100, label='Drone 2')
    
    # ドローン2の座標軸（姿勢）を表示
    R2 = drone2.T.R
    ax2.quiver(drone2.T.p[0], drone2.T.p[1], drone2.T.p[2], 
             R2[0, 0]*length, R2[1, 0]*length, R2[2, 0]*length, 
             color='r', arrow_length_ratio=0.1)
    ax2.quiver(drone2.T.p[0], drone2.T.p[1], drone2.T.p[2], 
             R2[0, 1]*length, R2[1, 1]*length, R2[2, 1]*length, 
             color='g', arrow_length_ratio=0.1)
    ax2.quiver(drone2.T.p[0], drone2.T.p[1], drone2.T.p[2], 
             R2[0, 2]*length, R2[1, 2]*length, R2[2, 2]*length, 
             color='b', arrow_length_ratio=0.1)
    
    # ドローン1の視野角（円錐）を表示
    draw_fov_cone(ax2, drone1, drone_colors[0])
    
    # ドローン2の視野角（円錐）を表示
    draw_fov_cone(ax2, drone2, drone_colors[1])
    
    # グラフの設定
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.set_title('Drone Positions and Field of View')
    ax2.legend()
    
    # 視点を調整
    ax2.view_init(elev=30, azim=20)
    
    # 軸の範囲を設定
    ax2.set_xlim([-2, 2])
    ax2.set_ylim([-2, 2])
    ax2.set_zlim([-2, 2])
    
    # グラフ間の間隔を調整
    plt.tight_layout()
    
    # 保存と表示
    plt.savefig('covariance_function_potential.png', dpi=300, bbox_inches='tight')
    plt.show()

def calculate_fov_intersection_with_plane(drone, z=0.0):
    """
    ドローンの視野角（円錐）とz平面の交差を計算する関数
    
    Parameters:
    -----------
    drone : Drone
        ドローン
    z : float, optional
        交差するz平面の高さ（デフォルトは0.0）
        
    Returns:
    --------
    center : ndarray, shape (2,)
        楕円の中心座標 (x, y)
    axes : ndarray, shape (2,)
        楕円の長軸と短軸の長さ
    angle : float
        楕円の回転角（ラジアン）
    is_valid : bool
        交差が有効かどうか（円錐がz平面と交差しない場合はFalse）
    """
    # カメラの向きを取得
    p = drone.T.p
    R = drone.T.R
    direction = R @ drone.camera_direction
    
    # 円錐の頂点
    apex = p
    
    # 方向ベクトルのz成分が0に近い場合（平面と平行）
    if abs(direction[2]) < 1e-6:
        return None, None, None, False
    
    # 頂点から平面までの距離
    t = (z - apex[2]) / direction[2]
    
    # 負の距離は円錐の後ろ側なので無効
    if t <= 0:
        return None, None, None, False
    
    # 円錐と平面の交点（楕円の中心）
    center_3d = apex + t * direction
    center = center_3d[:2]  # x, y座標のみ
    
    # 円錐の底面の半径（交差点での）
    radius = np.tan(drone.fov_angle) * t
    
    # 楕円の長軸と短軸
    # 円錐が傾いていると楕円になる
    # 傾き角度に応じて楕円の形状を計算
    cos_angle = np.abs(direction[2])  # z方向との角度のコサイン
    
    # 楕円の長軸（円錐の断面の最大半径）
    major_axis = radius / cos_angle
    
    # 楕円の短軸（円錐の断面の最小半径）
    minor_axis = radius
    
    # 楕円の回転角
    # z軸周りの回転角を計算
    angle = np.arctan2(direction[1], direction[0])
    
    return center, np.array([major_axis, minor_axis]), angle, True

def draw_fov_cone(ax, drone, color, alpha=0.3, h=2.0):
    """
    ドローンの視野角（円錐）を描画する関数
    
    Parameters:
    -----------
    ax : matplotlib.axes.Axes
        描画対象の3Dアクセス
    drone : Drone
        ドローン
    color : str
        描画色
    alpha : float, optional
        透明度（デフォルトは0.3）
    h : float, optional
        円錐の高さ（デフォルトは2.0）
    """
    # カメラの向きを取得
    p = drone.T.p
    R = drone.T.R
    direction = R @ drone.camera_direction
    
    # 円錐の頂点
    apex = p
    
    # 円錐の底面の中心
    center = apex + direction * h
    
    # 円錐の底面の半径
    radius = np.tan(drone.fov_angle) * h
    
    # 円錐の底面の法線ベクトル
    normal = direction
    
    # 法線ベクトルに垂直な2つのベクトルを見つける
    if np.abs(normal[0]) < np.abs(normal[1]):
        v1 = np.array([0, -normal[2], normal[1]])
    else:
        v1 = np.array([-normal[2], 0, normal[0]])
    v1 = v1 / np.linalg.norm(v1)
    v2 = np.cross(normal, v1)
    
    # 円の点を生成
    n_points = 16
    theta = np.linspace(0, 2*np.pi, n_points, endpoint=False)
    
    # 底面の円の点を生成
    circle_points = np.zeros((n_points, 3))
    for i, t in enumerate(theta):
        circle_points[i] = center + radius * (np.cos(t) * v1 + np.sin(t) * v2)
    
    # 底面の円を描画
    ax.plot(circle_points[:, 0], circle_points[:, 1], circle_points[:, 2], 
           color=color, alpha=alpha)
    
    # 頂点から円周への線分を描画
    for i in range(n_points):
        ax.plot([apex[0], circle_points[i, 0]], 
               [apex[1], circle_points[i, 1]], 
               [apex[2], circle_points[i, 2]], 
               color=color, alpha=alpha)
    
    # 底面を塗りつぶす（Poly3DCollectionを使用）
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    
    # 円錐の底面を三角形に分割して塗りつぶす
    triangles = []
    for i in range(1, n_points - 1):
        triangles.append([circle_points[0], circle_points[i], circle_points[i+1]])
    
    # 三角形の集合を作成
    poly3d = Poly3DCollection(triangles, alpha=alpha, color=color)
    ax.add_collection3d(poly3d)

def main():
    """
    メイン関数
    """
    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description='3つの関数を3Dポテンシャルとして描画するスクリプト')
    parser.add_argument('--resolution-r', type=int, default=40, help='半径方向の解像度（デフォルト: 100）')
    parser.add_argument('--resolution-theta', type=int, default=40, help='角度方向の解像度（デフォルト: 100）')
    args = parser.parse_args()
    
    # 解像度の設定
    resolution_r = args.resolution_r
    resolution_theta = args.resolution_theta
    
    print(f"解像度設定: 半径方向={resolution_r}, 角度方向={resolution_theta}")
    
    # 各関数を描画
    # print("1. Psi_{i}^l function plotting...")
    # plot_psi_function(resolution_r, resolution_theta)
    
    # print("2. P_i^l function plotting...")
    # plot_p_function(resolution_r, resolution_theta)
    
    print("3. Covariance matrix function plotting...")
    plot_covariance_function(resolution_r, resolution_theta)
    
    print("All functions have been plotted successfully.")

if __name__ == "__main__":
    main()
