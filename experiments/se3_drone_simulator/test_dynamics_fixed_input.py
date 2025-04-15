"""
固定値のf, τを入力するテストスクリプト
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import argparse
import sys
import os

# 親ディレクトリをパスに追加
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.se3 import SE3
from utils.drone import DynamicDrone, FeaturePoint
from utils.simulator import Simulator
from utils.visualization import Visualizer


class InputVisualizer(Visualizer):
    """
    入力（f, τ）を可視化する機能を追加したビジュアライザ
    """
    
    def __init__(self, simulator, figsize=(12, 10), keep_fov_history=False, fov_save_interval=10):
        """
        可視化を初期化
        
        Parameters:
        -----------
        simulator : Simulator
            シミュレータ
        figsize : tuple, optional
            図のサイズ（デフォルトは(12, 10)）
        keep_fov_history : bool, optional
            過去の視野錐台描画を残すかどうか（デフォルトはFalse）
        fov_save_interval : int, optional
            視野錐台を保存する間隔（フレーム数）（デフォルトは10）
        """
        super().__init__(simulator, figsize, keep_fov_history, fov_save_interval)
        
        # 入力の可視化用のアーティスト
        self.input_artists = []
        
        # 現在の入力値
        self.current_inputs = []
    
    def update_inputs(self, inputs):
        """
        入力値を更新
        
        Parameters:
        -----------
        inputs : list of array_like
            各ドローンの入力のリスト
            各要素は[f, tau]の形式（shape (4,)）
        """
        self.current_inputs = inputs
    
    def update_plot(self):
        """
        プロットを更新
        """
        # 親クラスの更新処理を呼び出し
        super().update_plot()
        
        # 入力の可視化を更新
        self._update_input_visualization()
    
    def _update_input_visualization(self):
        """
        入力（f, τ）の可視化を更新
        """
        # 古い入力の可視化を削除
        for artist in self.input_artists:
            artist.remove()
        self.input_artists = []
        
        # 入力がない場合は何もしない
        if not self.current_inputs:
            return
        
        # 各ドローンの入力を可視化
        for i, (drone, input_vec) in enumerate(zip(self.simulator.drones, self.current_inputs)):
            if not isinstance(drone, DynamicDrone):
                continue
                
            color = self.drone_colors[i % len(self.drone_colors)]
            
            # ドローンの位置と姿勢
            p = drone.T.p
            R = drone.T.R
            
            # 入力を分解
            f = input_vec[0]  # スカラー推力
            tau = input_vec[1:4]  # トルク
            
            # 推力の可視化（z軸方向の矢印）
            # 推力の大きさに応じて矢印の長さを調整
            f_scale = 0.1  # スケーリング係数
            f_length = f_scale * f
            
            # 推力ベクトル（ボディフレーム）
            f_vec_body = np.array([0, 0, f_length])
            
            # 推力ベクトル（ワールドフレーム）
            f_vec_world = R @ f_vec_body
            
            # 推力の矢印を描画
            f_arrow = self.ax.quiver(
                p[0], p[1], p[2],
                f_vec_world[0], f_vec_world[1], f_vec_world[2],
                color='m', arrow_length_ratio=0.2, linewidth=2
            )
            self.input_artists.append(f_arrow)
            
            # トルクの可視化（3つの軸周りの矢印）
            # トルクの大きさに応じて矢印の長さを調整
            tau_scale = 0.5  # スケーリング係数
            
            # 各軸周りのトルクを可視化
            for j in range(3):
                # トルクの大きさ
                tau_mag = tau_scale * tau[j]
                
                # トルクベクトル（ボディフレーム）
                tau_vec_body = np.zeros(3)
                tau_vec_body[j] = tau_mag
                
                # トルクベクトル（ワールドフレーム）
                tau_vec_world = R @ tau_vec_body
                
                # トルクの矢印を描画
                tau_arrow = self.ax.quiver(
                    p[0], p[1], p[2],
                    tau_vec_world[0], tau_vec_world[1], tau_vec_world[2],
                    color=['r', 'g', 'b'][j], arrow_length_ratio=0.2, linewidth=2
                )
                self.input_artists.append(tau_arrow)
            
            # 入力値をテキストで表示
            text = f"f = {f:.2f}, τ = [{tau[0]:.2f}, {tau[1]:.2f}, {tau[2]:.2f}]"
            text_artist = self.ax.text(
                p[0], p[1], p[2] + 0.5,
                text,
                color=color,
                fontsize=10,
                horizontalalignment='center'
            )
            self.input_artists.append(text_artist)


def main():
    """
    メイン関数
    """
    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description='固定値のf, τを入力するテスト')
    parser.add_argument('--f', type=float, default=9.81, help='推力（デフォルト: 9.81）')
    parser.add_argument('--tau-x', type=float, default=0.0, help='x軸周りのトルク（デフォルト: 0.0）')
    parser.add_argument('--tau-y', type=float, default=0.0, help='y軸周りのトルク（デフォルト: 0.0）')
    parser.add_argument('--tau-z', type=float, default=0.0, help='z軸周りのトルク（デフォルト: 0.0）')
    parser.add_argument('--dt', type=float, default=0.01, help='時間ステップ（デフォルト: 0.01）')
    parser.add_argument('--num-frames', type=int, default=200, help='フレーム数（デフォルト: 200）')
    args = parser.parse_args()
    
    # シミュレータの初期化
    simulator = Simulator(dt=args.dt)
    
    # 2次系ドローンを追加
    # ボディフレームの初期位置は回転なしでワールド座標系と同じ
    R_init = np.array([
        [1, 0, 0], 
        [0, 0, 1], 
        [0, -1, 0]]
    )
    # R_init = np.array([
    #     [1, 0, 0], 
    #     [0, 1, 0], 
    #     [0, 0, 1]]
    # )
    T_init = SE3(R=R_init, p=np.zeros(3))
    
    # ボディフレームからカメラフレームへの変換行列
    # この変換行列により、カメラのy軸が下を向く
    camera_direction = np.array([0, 1, 0])  # カメラは-y方向（下向き）を向く
    
    # ドローンを初期化（カメラの向きを指定）
    drone = DynamicDrone(T=T_init, camera_direction=camera_direction, fov_angle=np.pi/6)
    simulator.add_drone(drone)
    
    # 特徴点を追加（格子状）
    n_points = 3
    for x in np.linspace(-3, 3, n_points):
        for y in np.linspace(-3, 3, n_points):
            for z in np.linspace(-3, 3, n_points):
                if abs(x) > 1 or abs(y) > 1 or abs(z) > 1:  # 中心付近は除外
                    fp = FeaturePoint([x, y, z])
                    simulator.add_feature_point(fp)
    
    # 入力の可視化機能を追加したビジュアライザを初期化
    visualizer = InputVisualizer(simulator, keep_fov_history=True, fov_save_interval=10)
    
    # 固定入力値
    fixed_input = np.array([args.f, args.tau_x, args.tau_y, args.tau_z])
    
    # ドローンの入力関数
    def drone_inputs_func(sim, frame):
        # 固定入力値を返す
        inputs = [fixed_input]
        
        # 入力値をビジュアライザに設定
        visualizer.update_inputs(inputs)
        
        return inputs
    
    # アニメーションの作成と表示
    anim = visualizer.animate(args.num_frames, drone_inputs_func)
    
    # アニメーションオブジェクトを保持するためのグローバル変数
    global global_anim
    global_anim = anim
    
    # アニメーションの表示
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
