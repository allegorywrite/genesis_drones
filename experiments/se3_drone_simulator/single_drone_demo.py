"""
単一ドローンのシミュレーションデモ
視野内の点を強調表示
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from se3 import SE3
from drone import Drone, FeaturePoint
from simulator import Simulator


class SingleDroneVisualizer:
    """
    単一ドローンのシミュレータの可視化
    """
    
    def __init__(self, drone, feature_points, figsize=(10, 8)):
        """
        可視化を初期化
        
        Parameters:
        -----------
        drone : Drone
            ドローン
        feature_points : list of FeaturePoint
            特徴点のリスト
        figsize : tuple, optional
            図のサイズ（デフォルトは(10, 8)）
        """
        self.drone = drone
        self.feature_points = feature_points
        self.fig = plt.figure(figsize=figsize)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # 描画アーティスト
        self.drone_point = None
        self.x_axis = None
        self.y_axis = None
        self.z_axis = None
        self.fov_artists = []
        self.feature_artists = []
        self.visible_feature_artists = []
        self.trajectory_artist = None
        
        # ドローンの軌道を記録
        self.trajectory = []
        
        # ドローンの色
        self.drone_color = 'b'
        
        # 描画範囲
        self.xlim = (-5, 5)
        self.ylim = (-5, 5)
        self.zlim = (-5, 5)
        
        # 時間ステップ
        self.dt = 0.05
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
        
        # 軌道を更新
        self._update_trajectory()
        
        # 描画を更新
        self.fig.canvas.draw_idle()
    
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
        alpha = 0.3
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
            ドローンの入力を返す必要がある
            
        Returns:
        --------
        anim : FuncAnimation
            アニメーションオブジェクト
        """
        self.setup_plot()
        
        def update(frame):
            # ドローンの入力を計算
            if input_func is not None:
                xi = input_func(self.drone, frame)
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
            artists.extend(self.feature_artists)
            artists.extend(self.visible_feature_artists)
            if self.trajectory_artist is not None:
                artists.append(self.trajectory_artist)
            
            return artists
        
        anim = FuncAnimation(self.fig, update, frames=range(num_frames), 
                            interval=50, blit=False)
        
        return anim
    
    def show(self):
        """
        プロットを表示
        """
        plt.show()


def main():
    """
    メイン関数
    """
    # ドローンの初期化（視野角を狭く設定：π/6 = 30度）
    drone = Drone(fov_angle=np.pi/6)
    
    # 特徴点の追加
    feature_points = []
    point_num = 5
    
    # 格子状に特徴点を配置
    for x in np.linspace(-3, 3, point_num):
        for y in np.linspace(-3, 3, point_num):
            for z in np.linspace(-3, 3, point_num):
                if abs(x) > 1 or abs(y) > 1 or abs(z) > 1:  # 中心付近は除外
                    fp = FeaturePoint([x, y, z])
                    feature_points.append(fp)
    
    # 可視化の初期化
    visualizer = SingleDroneVisualizer(drone, feature_points)
    
    # ドローンの入力関数（円運動）
    def drone_input_func(drone, frame):
        # x軸周りの回転と前進
        omega = np.array([1.0, 0.0, 0.0])  # x軸周りの回転
        v = np.array([1.0, 1.0, 0.0])      # x軸方向の前進
        xi = np.concatenate([omega, v])
        return xi
    
    # アニメーションの作成と表示
    num_frames = 200
    anim = visualizer.animate(num_frames, drone_input_func)
    
    # アニメーションオブジェクトを保持するためのグローバル変数
    global global_anim
    global_anim = anim
    
    # アニメーションの表示
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
