"""
HOCBFの可視化モジュール
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class HOCBFVisualizer:
    """
    HOCBFの可視化クラス
    """
    
    def __init__(self, figsize=(12, 6)):
        """
        可視化を初期化
        
        Parameters:
        -----------
        figsize : tuple, optional
            図のサイズ（デフォルトは(12, 6)）
        """
        # メインの図を作成
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=figsize)
        
        # グラフ1: y=h_dot, x=hのプロット設定
        self.ax1.set_xlabel(r'$B$')
        self.ax1.set_ylabel(r'$\dot B$')
        self.ax1.set_title(r'$\dot B + \gamma_0 B > 0$')
        self.ax1.grid(True)
        
        # グラフ2: y=h_ddot+gamma_0*h_dot, x=h_dot+gamma_0*hのプロット設定
        # self.ax2.set_xlabel('dB/dt + gamma_0*B')
        self.ax2.set_xlabel(r'$\dot B + \gamma_0 B$')
        self.ax2.set_ylabel(r'$\ddot B + \gamma_0 \dot B$')
        self.ax2.set_title(r'$\ddot B + \gamma_0 \dot B + \gamma_1 (\dot B + \gamma_0 B) > 0$')
        self.ax2.grid(True)
        
        # データを記録するためのリスト
        self.h_values = []
        self.h_dot_values = []
        self.h_ddot_values = []
        
        # 係数
        self.gamma0 = 0.1
        self.gamma1 = 0.1
        
        # プロットアーティスト
        self.scatter1 = None
        self.scatter2 = None
        self.line1 = None
        self.line2 = None
        
        # 描画範囲（原点中心）
        self.h_min = -1.0
        self.h_max = 1.0
        self.h_dot_min = -1.0
        self.h_dot_max = 1.0
        self.B_min = -1.0
        self.B_max = 1.0
        self.B_dot_min = -1.0
        self.B_dot_max = 1.0
        
        # 初期化
        self._setup_plot()
    
    def _setup_plot(self):
        """
        プロットの初期設定
        """
        # グラフ1の初期設定
        self.ax1.set_xlim(self.h_min, self.h_max)
        self.ax1.set_ylim(self.h_dot_min, self.h_dot_max)
        
        # グラフ2の初期設定
        self.ax2.set_xlim(self.B_min, self.B_max)
        self.ax2.set_ylim(self.B_dot_min, self.B_dot_max)
        
        # 散布図の初期化
        self.scatter1 = self.ax1.scatter([], [], c='blue', s=30, alpha=0.7, label='Data')
        self.scatter2 = self.ax2.scatter([], [], c='blue', s=30, alpha=0.7, label='Data')
        
        # 境界線の描画
        # グラフ1: h_dot + gamma_0*h = 0
        h_values = np.linspace(self.h_min, self.h_max, 100)
        h_dot_values = -self.gamma0 * h_values
        self.line1, = self.ax1.plot(h_values, h_dot_values, 'r-', linewidth=2, 
                                   label=r'$\dot B + \gamma_0 B = 0$')
        
        # グラフ2: B_ddot + gamma_0*B_dot + gamma_1*(B_dot + gamma_0*B) = 0
        B_values = np.linspace(self.B_min, self.B_max, 100)
        B_dot_values = -self.gamma1 * B_values
        self.line2, = self.ax2.plot(B_values, B_dot_values, 'b-', linewidth=2, 
                                   label=r'$\ddot B + \gamma_0 \dot B + \gamma_1 (\dot B + \gamma_0 B) = 0$')
        
        # 凡例
        self.ax1.legend()
        self.ax2.legend()
        
        # タイトル
        self.fig.suptitle('HOCBF Visualization')
        
        # レイアウト調整
        self.fig.tight_layout()
    
    # 1step前の値を取得する
    def get_previous_values(self):
        if len(self.h_values) == 0:
            return 0.0, 0.0, 0.0
        return self.h_values[-1], self.h_dot_values[-1], self.h_ddot_values[-1]
    
    def update(self, h, h_dot, h_ddot, gamma0=None, gamma1=None):
        """
        データを更新
        
        Parameters:
        -----------
        h : float
            安全集合の値
        h_dot : float
            安全集合の1階微分
        h_ddot : float
            安全集合の2階微分
        gamma0 : float, optional
            CBFのゲイン0
        gamma1 : float, optional
            CBFのゲイン1
        """
        # 係数の更新
        if gamma0 is not None:
            self.gamma0 = gamma0
        if gamma1 is not None:
            self.gamma1 = gamma1
        
        # データを記録
        self.h_values.append(h)
        self.h_dot_values.append(h_dot)
        self.h_ddot_values.append(h_ddot)
        
        # 散布図のデータを更新
        self.scatter1.set_offsets(np.column_stack((self.h_values, self.h_dot_values)))
        
        # B = h_dot + gamma0*h
        B_values = [h_dot + self.gamma0 * h for h, h_dot in zip(self.h_values, self.h_dot_values)]
        
        # B_dot = h_ddot + gamma0*h_dot
        B_dot_values = [h_ddot + self.gamma0 * h_dot for h_dot, h_ddot in zip(self.h_dot_values, self.h_ddot_values)]
        
        self.scatter2.set_offsets(np.column_stack((B_values, B_dot_values)))
        
        # 境界線の更新
        # グラフ1: h_dot + gamma_0*h = 0
        h_values = np.linspace(self.h_min, self.h_max, 100)
        h_dot_values = -self.gamma0 * h_values
        self.line1.set_data(h_values, h_dot_values)
        
        # グラフ2: h_ddot + gamma_0*h_dot + gamma_1*(h_dot + gamma_0*h) = 0
        B_values_line = np.linspace(self.B_min, self.B_max, 100)
        B_dot_values_line = -self.gamma1 * B_values_line
        self.line2.set_data(B_values_line, B_dot_values_line)
        
        # 凡例を更新
        self.ax1.legend()
        self.ax2.legend()
        
        # 軸の範囲を原点中心で調整
        if len(self.h_values) > 1:
            h_min, h_max = min(self.h_values), max(self.h_values)
            h_dot_min, h_dot_max = min(self.h_dot_values), max(self.h_dot_values)
            
            # マージンを追加
            margin = 0.1
            h_range = max(h_max - h_min, 0.1)
            h_dot_range = max(h_dot_max - h_dot_min, 0.1)
            
            # 原点を含む範囲に設定
            self.h_min = min(h_min - margin * h_range, -0.1)
            self.h_max = max(h_max + margin * h_range, 0.1)
            self.h_dot_min = min(h_dot_min - margin * h_dot_range, -0.1)
            self.h_dot_max = max(h_dot_max + margin * h_dot_range, 0.1)
            
            # 範囲を対称にする
            h_abs_max = max(abs(self.h_min), abs(self.h_max))
            h_dot_abs_max = max(abs(self.h_dot_min), abs(self.h_dot_max))
            
            self.h_min = -h_abs_max
            self.h_max = h_abs_max
            self.h_dot_min = -h_dot_abs_max
            self.h_dot_max = h_dot_abs_max
            
            self.ax1.set_xlim(self.h_min, self.h_max)
            self.ax1.set_ylim(self.h_dot_min, self.h_dot_max)
            
            # B = h_dot + gamma0*h
            B_min = min(B_values)
            B_max = max(B_values)
            
            # B_dot = h_ddot + gamma0*h_dot
            B_dot_min = min(B_dot_values)
            B_dot_max = max(B_dot_values)
            
            # マージンを追加
            B_range = max(B_max - B_min, 0.1)
            B_dot_range = max(B_dot_max - B_dot_min, 0.1)
            
            # 原点を含む範囲に設定
            self.B_min = min(B_min - margin * B_range, -0.1)
            self.B_max = max(B_max + margin * B_range, 0.1)
            self.B_dot_min = min(B_dot_min - margin * B_dot_range, -0.1)
            self.B_dot_max = max(B_dot_max + margin * B_dot_range, 0.1)
            
            # 範囲を対称にする
            B_abs_max = max(abs(self.B_min), abs(self.B_max))
            B_dot_abs_max = max(abs(self.B_dot_min), abs(self.B_dot_max))
            
            self.B_min = -B_abs_max
            self.B_max = B_abs_max
            self.B_dot_min = -B_dot_abs_max
            self.B_dot_max = B_dot_abs_max
            
            self.ax2.set_xlim(self.B_min, self.B_max)
            self.ax2.set_ylim(self.B_dot_min, self.B_dot_max)
        
        # 描画を更新
        self.fig.canvas.draw_idle()
    
    def show(self):
        """
        プロットを表示
        """
        plt.show(block=False)
    
    def close(self):
        """
        プロットを閉じる
        """
        plt.close(self.fig)
