"""
ドローンシミュレーションの可視化モジュール
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec


class SingleDroneVisualizer:
    """
    単一ドローンのシミュレータの可視化
    """
    
    def __init__(self, drone, feature_points, target_position=None, figsize=(12, 10), 
                 keep_fov_history=False, fov_save_interval=10):
        """
        可視化を初期化
        
        Parameters:
        -----------
        drone : Drone
            ドローン
        feature_points : list of FeaturePoint
            特徴点のリスト
        target_position : array_like, shape (3,), optional
            目標位置（デフォルトはNone）
        figsize : tuple, optional
            図のサイズ（デフォルトは(12, 10)）
        keep_fov_history : bool, optional
            過去の視野錐台描画を残すかどうか（デフォルトはFalse）
        fov_save_interval : int, optional
            視野錐台を保存する間隔（フレーム数）（デフォルトは10）
        """
        self.drone = drone
        self.feature_points = feature_points
        self.target_position = target_position
        
        # メインの図を作成
        self.fig = plt.figure(figsize=figsize)
        
        # グリッドレイアウトを設定
        gs = gridspec.GridSpec(2, 2, height_ratios=[3, 1])
        
        # 3Dシミュレーション用のサブプロット
        self.ax = self.fig.add_subplot(gs[0, :], projection='3d')
        
        # 安全集合と制約余裕用のサブプロット
        self.ax_safety = self.fig.add_subplot(gs[1, 0])
        self.ax_constraint = self.fig.add_subplot(gs[1, 1])
        
        # 安全集合のプロット設定
        self.ax_safety.set_xlabel('Time [s]')
        self.ax_safety.set_ylabel('Safety Value')
        self.ax_safety.set_title('Safety Value (B) and dB/dt')
        self.ax_safety.grid(True)
        self.safety_line, = self.ax_safety.plot([], [], 'r-', linewidth=2, label='B')
        self.ax_dot_line, = self.ax_safety.plot([], [], 'g--', linewidth=2, label='dB/dt')
        self.ax_safety.legend()
        
        # 制約余裕のプロット設定
        self.ax_constraint.set_xlabel('Time [s]')
        self.ax_constraint.set_ylabel('Constraint Margin')
        self.ax_constraint.set_title('Constraint Margin (b-Ax)')
        self.ax_constraint.grid(True)
        self.constraint_line, = self.ax_constraint.plot([], [], 'b-', linewidth=2, label='b-Ax')
        self.ax_constraint.legend()
        
        # 値を記録するためのリスト
        self.time_values = []
        self.gamma_values = []
        self.constraint_margin_values = []
        self.ax_values = []
        
        # 最新のCBF値
        self.cbf_values = None
        
        # 描画アーティスト
        self.drone_point = None
        self.x_axis = None
        self.y_axis = None
        self.z_axis = None
        self.fov_artists = []
        self.feature_artists = []
        self.visible_feature_artists = []
        self.trajectory_artist = None
        
        # 過去の視野錐台を保存するためのリスト
        self.past_fov_artists = []
        
        # 過去の視野錐台を保存するかどうか
        self.keep_fov_history = keep_fov_history
        
        # 視野錐台を保存する間隔（フレーム数）
        self.fov_save_interval = fov_save_interval
        
        # 現在のフレーム番号
        self.current_frame = 0
        
        # 目標位置の描画アーティスト
        self.target_artist = None
        self.target_line_artist = None
        
        # ドローンの軌道を記録
        self.trajectory = []
        
        # ドローンの色(黒)
        # self.drone_color = 'b'
        self.drone_color = 'k'
        
        # 描画範囲
        self.xlim = (-5, 5)
        self.ylim = (-5, 5)
        self.zlim = (-5, 5)
        
        # 時間ステップ
        self.dt = 0.01
        self.time = 0.0
    
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
        self.ax.set_title('Single Drone Simulator - Visible Points Highlighted')
        
        # ドローンの描画アーティストを初期化
        # ドローン本体（点）
        self.drone_point = self.ax.scatter([], [], [], color=self.drone_color, s=100, label='Drone')
        
        # 座標軸（姿勢）
        self.x_axis = self.ax.quiver([], [], [], [], [], [], color='r', arrow_length_ratio=0.1)
        self.y_axis = self.ax.quiver([], [], [], [], [], [], color='g', arrow_length_ratio=0.1)
        self.z_axis = self.ax.quiver([], [], [], [], [], [], color='b', arrow_length_ratio=0.1)
        
        # 特徴点の描画アーティストを初期化
        for fp in self.feature_points:
            feature_point = self.ax.scatter([], [], [], color='k', s=30)
            self.feature_artists.append(feature_point)
        
        # 凡例
        self.ax.legend()
    
    def update_plot(self):
        """
        プロットを更新
        """
        # ドローンの描画を更新
        # 位置と姿勢
        p = self.drone.T.p
        R = self.drone.T.R
        
        # 軌道に現在位置を追加
        self.trajectory.append(p.copy())
        
        # ドローン本体
        self.drone_point._offsets3d = ([p[0]], [p[1]], [p[2]])
        
        # 座標軸（姿勢）
        length = 0.5
        self.x_axis.set_segments([[[p[0], p[1], p[2]], 
                               [p[0] + R[0, 0]*length, p[1] + R[1, 0]*length, p[2] + R[2, 0]*length]]])
        self.y_axis.set_segments([[[p[0], p[1], p[2]], 
                               [p[0] + R[0, 1]*length, p[1] + R[1, 1]*length, p[2] + R[2, 1]*length]]])
        self.z_axis.set_segments([[[p[0], p[1], p[2]], 
                               [p[0] + R[0, 2]*length, p[1] + R[1, 2]*length, p[2] + R[2, 2]*length]]])
        
        # 視野角（円錐）の更新
        self._update_fov()
        
        # 特徴点の描画を更新
        for i, fp in enumerate(self.feature_points):
            self.feature_artists[i]._offsets3d = ([fp.position[0]], [fp.position[1]], [fp.position[2]])
        
        # 視野内の特徴点を強調表示
        self._update_visible_features()
        
        # 目標位置を更新
        self._update_target_position()
        
        # 軌道を更新
        self._update_trajectory()
        
        # 安全集合と制約余裕のプロットを更新
        self._update_safety_plots()
        
        # 描画を更新
        self.fig.canvas.draw_idle()
    
    def _update_safety_plots(self):
        """
        安全集合と制約余裕のプロットを更新
        """
        # 時間を記録
        self.time_values.append(self.time)
        
        # CBF値が設定されている場合
        if self.cbf_values is not None:
            gamma_val, constraint_margin, ax_value = self.cbf_values
            
            # 値を記録
            self.gamma_values.append(gamma_val)
            self.constraint_margin_values.append(constraint_margin)
            self.ax_values.append(ax_value)
        else:
            # 値が設定されていない場合は0を記録
            self.gamma_values.append(0)
            self.constraint_margin_values.append(0)
            self.ax_values.append(0)
        
        # プロットを更新
        self.safety_line.set_data(self.time_values, self.gamma_values)
        self.ax_dot_line.set_data(self.time_values, self.ax_values)
        self.constraint_line.set_data(self.time_values, self.constraint_margin_values)
        
        # 軸の範囲を自動調整
        self.ax_safety.relim()
        self.ax_safety.autoscale_view()
        
        self.ax_constraint.relim()
        self.ax_constraint.autoscale_view()
    
    def set_cbf_values(self, gamma_val, constraint_margin, ax_value=0.0):
        """
        CBF値を設定
        
        Parameters:
        -----------
        gamma_val : float
            安全集合の値
        constraint_margin : float
            制約余裕の値
        ax_value : float, optional
            Ax値（デフォルトは0.0）
        """
        self.cbf_values = (gamma_val, constraint_margin, ax_value)
    
    def _update_fov(self):
        """
        視野角（円錐）の描画を更新
        """
        # 古い視野角の描画を削除
        for artist in self.fov_artists:
            artist.remove()
        self.fov_artists = []
        
        # 視野角の描画パラメータ
        color = self.drone_color
        alpha = 1.0
        h = 3.0  # 円錐の高さ
        
        # カメラの向きを取得
        p = self.drone.T.p
        R = self.drone.T.R
        direction = R @ self.drone.camera_direction
        
        # 円錐の頂点
        apex = p
        
        # 円錐の底面の中心
        center = apex + direction * h
        
        # 円錐の底面の半径
        radius = np.tan(self.drone.fov_angle) * h
        
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
        self.fov_artists.append(circle)
        
        # 頂点から円周への線分を描画
        for i in range(n_points):
            line = self.ax.plot([apex[0], circle_points[i, 0]], 
                               [apex[1], circle_points[i, 1]], 
                               [apex[2], circle_points[i, 2]], 
                               color=color, alpha=alpha)[0]
            self.fov_artists.append(line)
        
        # 過去の視野錐台を保存するかどうか
        if self.keep_fov_history and self.current_frame % self.fov_save_interval == 0:
            # 現在の視野錐台のコピーを作成して保存
            saved_fov_artists = []
            
            # 底面の円を描画（薄い色で）
            saved_circle = self.ax.plot(circle_points[:, 0], circle_points[:, 1], circle_points[:, 2], 
                                      color=color, alpha=alpha*0.8)[0]
            saved_fov_artists.append(saved_circle)
            
            # 頂点から円周への線分を描画（薄い色で）
            for i in range(n_points):
                saved_line = self.ax.plot([apex[0], circle_points[i, 0]], 
                                        [apex[1], circle_points[i, 1]], 
                                        [apex[2], circle_points[i, 2]], 
                                        color=color, alpha=alpha*0.8)[0]
                saved_fov_artists.append(saved_line)
            
            # 保存した視野錐台をリストに追加
            self.past_fov_artists.extend(saved_fov_artists)
    
    def _update_trajectory(self):
        """
        ドローンの軌道を更新
        """
        # 軌道が空の場合は何もしない
        if len(self.trajectory) < 2:
            return
        
        # 古い軌道の描画を削除
        if self.trajectory_artist is not None:
            self.trajectory_artist.remove()
        
        # 軌道の点を取得
        trajectory_points = np.array(self.trajectory)
        
        # 軌道を描画
        self.trajectory_artist = self.ax.plot(
            trajectory_points[:, 0],
            trajectory_points[:, 1],
            trajectory_points[:, 2],
            'k--',  # 黒の破線
            alpha=0.5,
            linewidth=1
        )[0]
    
    def _update_visible_features(self):
        """
        視野内の特徴点を強調表示
        """
        # 古い視野内特徴点の描画を削除
        for artist in self.visible_feature_artists:
            artist.remove()
        self.visible_feature_artists = []
        
        # 視野内の特徴点を強調表示
        for i, fp in enumerate(self.feature_points):
            if self.drone.is_point_visible(fp.position):
                # 観測確率を計算
                prob = self.drone.get_observation_probability(fp.position)
                # 確率に応じて色を変える（赤→黄→緑）
                if prob < 0.33:
                    color = 'r'  # 低確率
                elif prob < 0.66:
                    color = 'y'  # 中確率
                else:
                    color = 'g'  # 高確率
                
                # サイズも確率に応じて変える
                size = 50 + 100 * prob
                
                # 強調表示
                visible_point = self.ax.scatter([fp.position[0]], [fp.position[1]], [fp.position[2]], 
                                              color=color, s=size, alpha=0.7)
                self.visible_feature_artists.append(visible_point)
    
    def step(self, xi):
        """
        シミュレーションを1ステップ進める
        
        Parameters:
        -----------
        xi : array_like, shape (6,)
            ドローンの速度入力 [omega, v]
        """
        # ドローンを更新
        self.drone.update(xi, self.dt)
        
        # 時間を進める
        self.time += self.dt
    
    def _update_target_position(self):
        """
        目標位置の描画を更新
        """
        # 目標位置が設定されていない場合は何もしない
        if self.target_position is None:
            return
        
        # 古い目標位置の描画を削除
        if self.target_artist is not None:
            self.target_artist.remove()
        if self.target_line_artist is not None:
            self.target_line_artist.remove()
        
        # 目標位置を大きな星マーカーで表示
        self.target_artist = self.ax.scatter(
            [self.target_position[0]], 
            [self.target_position[1]], 
            [self.target_position[2]],
            marker='*', color='r', s=200, alpha=0.7, label='Target Position'
        )
        
        # ドローンと目標位置を結ぶ線
        drone_pos = self.drone.T.p
        self.target_line_artist = self.ax.plot(
            [drone_pos[0], self.target_position[0]],
            [drone_pos[1], self.target_position[1]],
            [drone_pos[2], self.target_position[2]],
            '--', color='r', alpha=0.5
        )[0]
        
    def animate(self, num_frames, input_func=None):
        """
        アニメーションを作成
        
        Parameters:
        -----------
        num_frames : int
            フレーム数
        input_func : callable, optional
            各フレームでのドローンの入力を計算する関数
            引数としてドローンとフレーム番号を取り、
            ドローンの入力と追加情報を返す必要がある
            
        Returns:
        --------
        anim : FuncAnimation
            アニメーションオブジェクト
        """
        self.setup_plot()
        
        def update(frame):
            # フレーム番号を更新
            self.current_frame = frame
            
            # ドローンの入力を計算
            if input_func is not None:
                result = input_func(self.drone, frame)
                
                # 入力関数が追加情報を返す場合
                if isinstance(result, tuple) and len(result) >= 4:
                    xi, gamma_val, constraint_margin, ax_value = result
                    # CBF値を設定
                    self.set_cbf_values(gamma_val, constraint_margin, ax_value)
                elif isinstance(result, tuple) and len(result) >= 3:
                    xi, gamma_val, constraint_margin = result
                    # CBF値を設定（Ax値なし）
                    self.set_cbf_values(gamma_val, constraint_margin)
                else:
                    # 追加情報がない場合は入力のみ
                    xi = result
            else:
                # デフォルトの入力（静止）
                xi = np.zeros(6)
            
            # シミュレーションを1ステップ進める
            self.step(xi)
            
            # プロットを更新
            self.update_plot()
            
            # 更新されたアーティストのリストを返す
            artists = [self.drone_point, self.x_axis, self.y_axis, self.z_axis]
            artists.extend(self.fov_artists)
            artists.extend(self.past_fov_artists)  # 過去の視野錐台も含める
            artists.extend(self.feature_artists)
            artists.extend(self.visible_feature_artists)
            if self.trajectory_artist is not None:
                artists.append(self.trajectory_artist)
            if self.target_artist is not None:
                artists.append(self.target_artist)
            if self.target_line_artist is not None:
                artists.append(self.target_line_artist)
            artists.append(self.safety_line)
            artists.append(self.ax_dot_line)
            artists.append(self.constraint_line)
            
            return artists
        
        anim = FuncAnimation(self.fig, update, frames=range(num_frames), 
                            interval=50, blit=False)
        
        return anim
    
    def show(self):
        """
        プロットを表示
        """
        plt.show()
