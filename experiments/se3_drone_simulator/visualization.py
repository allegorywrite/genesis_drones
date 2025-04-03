"""
可視化を行うモジュール
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from se3 import SE3
from drone import Drone, FeaturePoint
from simulator import Simulator
import matplotlib.gridspec as gridspec


class Visualizer:
    """
    SE(3)ドローンシミュレータの可視化
    """
    
    def __init__(self, simulator, figsize=(12, 10)):
        """
        可視化を初期化
        
        Parameters:
        -----------
        simulator : Simulator
            シミュレータ
        figsize : tuple, optional
            図のサイズ（デフォルトは(12, 10)）
        """
        self.simulator = simulator
        
        # メインの3Dシミュレーション用の図
        self.fig_sim = plt.figure(figsize=figsize)
        self.ax = self.fig_sim.add_subplot(111, projection='3d')
        self.drone_artists = []
        self.fov_artists = []
        self.feature_artists = []
        self.cofov_artists = []
        self.trajectory_artists = []
        self.invisible_feature_artists = []  # どちらの視野にも入っていない特徴点
        
        # 安全集合B_ijの時間遷移用の図
        self.fig_safety = plt.figure(figsize=(8, 6))
        self.ax_safety = self.fig_safety.add_subplot(111)
        self.ax_safety.set_xlabel('Time [s]')
        self.ax_safety.set_ylabel('Safety Value B_{ij}')
        self.ax_safety.set_title('Safety Value B_{ij} Time Transition')
        self.ax_safety.grid(True)
        self.safety_line, = self.ax_safety.plot([], [], 'r-', linewidth=2)
        self.safety_values = []
        self.time_values = []
        
        # ドローンの軌道を記録
        self.trajectories = [[] for _ in range(len(simulator.drones))]
        
        # ドローンの色
        self.drone_colors = ['r', 'b', 'g', 'c', 'm', 'y']
        
        # 描画範囲
        self.xlim = (-5, 5)
        self.ylim = (-5, 5)
        self.zlim = (-5, 5)
    
    def setup_plot(self):
        """
        プロットの初期設定
        """
        self.ax.set_xlim(self.xlim)
        self.ax.set_ylim(self.ylim)
        self.ax.set_zlim(self.zlim)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('SE(3) Drone Simulator')
        
        # 各ドローンの描画アーティストを初期化
        for i, drone in enumerate(self.simulator.drones):
            color = self.drone_colors[i % len(self.drone_colors)]
            
            # ドローン本体（点）
            drone_point = self.ax.scatter([], [], [], color=color, s=100, label=f'Drone {i}')
            
            # 座標軸（姿勢）
            x_axis = self.ax.quiver([], [], [], [], [], [], color='r', arrow_length_ratio=0.1)
            y_axis = self.ax.quiver([], [], [], [], [], [], color='g', arrow_length_ratio=0.1)
            z_axis = self.ax.quiver([], [], [], [], [], [], color='b', arrow_length_ratio=0.1)
            
            # 視野角（円錐）
            fov = None  # 視野角は別途更新
            
            self.drone_artists.append((drone_point, x_axis, y_axis, z_axis))
            self.fov_artists.append(fov)
        
        # 特徴点の描画アーティストを初期化
        for fp in self.simulator.feature_points:
            feature_point = self.ax.scatter([], [], [], color='k', s=30)
            self.feature_artists.append(feature_point)
        
        # 凡例
        self.ax.legend()
    
    def update_plot(self):
        """
        プロットを更新
        """
        # ドローンの描画を更新
        for i, drone in enumerate(self.simulator.drones):
            color = self.drone_colors[i % len(self.drone_colors)]
            drone_point, x_axis, y_axis, z_axis = self.drone_artists[i]
            
            # 位置と姿勢
            p = drone.T.p
            R = drone.T.R
            
            # 軌道に現在位置を追加
            self.trajectories[i].append(p.copy())
            
            # ドローン本体
            drone_point._offsets3d = ([p[0]], [p[1]], [p[2]])
            
            # 座標軸（姿勢）
            length = 0.5
            x_axis.set_segments([[[p[0], p[1], p[2]], 
                                 [p[0] + R[0, 0]*length, p[1] + R[1, 0]*length, p[2] + R[2, 0]*length]]])
            y_axis.set_segments([[[p[0], p[1], p[2]], 
                                 [p[0] + R[0, 1]*length, p[1] + R[1, 1]*length, p[2] + R[2, 1]*length]]])
            z_axis.set_segments([[[p[0], p[1], p[2]], 
                                 [p[0] + R[0, 2]*length, p[1] + R[1, 2]*length, p[2] + R[2, 2]*length]]])
            
            # 視野角（円錐）の更新
            self._update_fov(i, drone)
        
        # 特徴点の描画を更新（どちらの視野にも入っていない特徴点は薄く表示）
        self._update_feature_points()
        
        # 共有視野の描画を更新
        self._update_cofov()
        
        # 軌道を更新
        self._update_trajectories()
        
        # 安全集合B_ijの時間遷移を更新
        self._update_safety_plot()
        
        # 描画を更新
        self.fig_sim.canvas.draw_idle()
        self.fig_safety.canvas.draw_idle()
    
    def _update_fov(self, drone_idx, drone):
        """
        視野角（円錐）の描画を更新
        
        Parameters:
        -----------
        drone_idx : int
            ドローンのインデックス
        drone : Drone
            ドローン
        """
        # 古い視野角の描画を削除
        if self.fov_artists[drone_idx] is not None:
            for artist in self.fov_artists[drone_idx]:
                artist.remove()
        
        # 視野角の描画パラメータ
        color = self.drone_colors[drone_idx % len(self.drone_colors)]
        alpha = 0.3
        h = 3.0  # 円錐の高さ
        
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
        
        # 円錐の描画（線分の集合）
        fov_artists = []
        
        # 円の点を生成（少ない点数で）
        n_points = 16
        theta = np.linspace(0, 2*np.pi, n_points, endpoint=False)
        
        # 底面の円の点を生成
        circle_points = np.zeros((n_points, 3))
        for i, t in enumerate(theta):
            circle_points[i] = center + radius * (np.cos(t) * v1 + np.sin(t) * v2)
        
        # 底面の円を描画
        circle = self.ax.plot(circle_points[:, 0], circle_points[:, 1], circle_points[:, 2], 
                             color=color, alpha=alpha)[0]
        fov_artists.append(circle)
        
        # 頂点から円周への線分を描画
        for i in range(n_points):
            line = self.ax.plot([apex[0], circle_points[i, 0]], 
                               [apex[1], circle_points[i, 1]], 
                               [apex[2], circle_points[i, 2]], 
                               color=color, alpha=alpha)[0]
            fov_artists.append(line)
        
        # 視野角の描画アーティストを保存
        self.fov_artists[drone_idx] = fov_artists
    
    def _update_trajectories(self):
        """
        ドローンの軌道を更新
        """
        # 古い軌道の描画を削除
        for artist in self.trajectory_artists:
            artist.remove()
        self.trajectory_artists = []
        
        # 各ドローンの軌道を描画
        for i, trajectory in enumerate(self.trajectories):
            if len(trajectory) < 2:
                continue
                
            color = self.drone_colors[i % len(self.drone_colors)]
            
            # 軌道の点を取得
            trajectory_points = np.array(trajectory)
            
            # 軌道を描画
            trajectory_artist = self.ax.plot(
                trajectory_points[:, 0],
                trajectory_points[:, 1],
                trajectory_points[:, 2],
                '--',  # 破線
                color=color,
                alpha=0.5,
                linewidth=1
            )[0]
            
            self.trajectory_artists.append(trajectory_artist)
    
    def _update_feature_points(self):
        """
        特徴点の描画を更新
        """
        # 古い特徴点の描画を削除
        for artist in self.feature_artists:
            artist.remove()
        self.feature_artists = []
        
        # 古い「どちらの視野にも入っていない特徴点」の描画を削除
        for artist in self.invisible_feature_artists:
            artist.remove()
        self.invisible_feature_artists = []
        
        # ドローンが2機以上ある場合
        if len(self.simulator.drones) >= 2:
            # ドローン1から見える特徴点
            visible_indices1 = self.simulator.get_visible_feature_points(0)
            # ドローン2から見える特徴点
            visible_indices2 = self.simulator.get_visible_feature_points(1)
            # 共有視野にある特徴点
            cofov_indices = self.simulator.get_cofov_feature_points(0, 1)
            
            # 各特徴点について
            for i, fp in enumerate(self.simulator.feature_points):
                if i in cofov_indices:
                    # 共有視野内の特徴点は_update_cofovで描画するのでスキップ
                    continue
                elif i in visible_indices1 or i in visible_indices2:
                    # どちらかの視野に入っている特徴点は通常の濃さで表示
                    feature_point = self.ax.scatter([fp.position[0]], [fp.position[1]], [fp.position[2]], 
                                                  color='k', s=30)
                    self.feature_artists.append(feature_point)
                else:
                    # どちらの視野にも入っていない特徴点は薄く表示
                    invisible_point = self.ax.scatter([fp.position[0]], [fp.position[1]], [fp.position[2]], 
                                                    color='k', s=20, alpha=0.1)
                    self.invisible_feature_artists.append(invisible_point)
        else:
            # ドローンが1機以下の場合は全ての特徴点を通常の濃さで表示
            for fp in self.simulator.feature_points:
                feature_point = self.ax.scatter([fp.position[0]], [fp.position[1]], [fp.position[2]], 
                                              color='k', s=30)
                self.feature_artists.append(feature_point)
    
    def _update_cofov(self):
        """
        共有視野の描画を更新
        """
        # 古い共有視野の描画を削除
        for artist in self.cofov_artists:
            artist.remove()
        self.cofov_artists = []
        
        # ドローンが2機以上ある場合のみ共有視野を描画
        if len(self.simulator.drones) < 2:
            return
        
        # 2機のドローンの共有視野にある特徴点を強調表示
        cofov_indices = self.simulator.get_cofov_feature_points(0, 1)
        
        for idx in cofov_indices:
            fp = self.simulator.feature_points[idx]
            cofov_point = self.ax.scatter([fp.position[0]], [fp.position[1]], [fp.position[2]], 
                                         color='y', s=100, alpha=0.7)
            self.cofov_artists.append(cofov_point)
    
    def _update_safety_plot(self):
        """
        安全集合B_ijの時間遷移プロットを更新
        """
        # ドローンが2機以上ある場合のみ安全値を計算
        if len(self.simulator.drones) < 2:
            return
        
        # 安全値を計算
        safety_value = self.simulator.calculate_safety_value(0, 1)
        
        # 時間と安全値を記録
        self.time_values.append(self.simulator.time)
        self.safety_values.append(safety_value)
        
        # プロットを更新
        self.safety_line.set_data(self.time_values, self.safety_values)
        
        # 軸の範囲を自動調整
        self.ax_safety.relim()
        self.ax_safety.autoscale_view()
    
    def animate(self, num_frames, drone_inputs_func=None):
        """
        アニメーションを作成
        
        Parameters:
        -----------
        num_frames : int
            フレーム数
        drone_inputs_func : callable, optional
            各フレームでのドローンの入力を計算する関数
            引数としてシミュレータとフレーム番号を取り、
            ドローンの入力のリストを返す必要がある
            
        Returns:
        --------
        anim : FuncAnimation
            アニメーションオブジェクト
        """
        self.setup_plot()
        
        def update(frame):
            # ドローンの入力を計算
            if drone_inputs_func is not None:
                drone_inputs = drone_inputs_func(self.simulator, frame)
            else:
                # デフォルトの入力（静止）
                drone_inputs = [np.zeros(6) for _ in range(len(self.simulator.drones))]
            
            # シミュレーションを1ステップ進める
            self.simulator.step(drone_inputs)
            
            # プロットを更新
            self.update_plot()
            
            # 更新されたアーティストのリストを返す
            artists = []
            for drone_artist in self.drone_artists:
                artists.extend(drone_artist)
            for fov_artist in self.fov_artists:
                if fov_artist is not None:
                    artists.extend(fov_artist)
            artists.extend(self.feature_artists)
            artists.extend(self.invisible_feature_artists)
            artists.extend(self.cofov_artists)
            artists.extend(self.trajectory_artists)
            artists.append(self.safety_line)
            
            return artists
        
        # メインのシミュレーションアニメーション
        anim = FuncAnimation(self.fig_sim, update, frames=range(num_frames), 
                            interval=50, blit=False)
        
        return anim
    
    def show(self):
        """
        プロットを表示
        """
        plt.show()
