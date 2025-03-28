#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import numpy as np
import sys
import os
import time
import getch

class KeyboardControlNode(Node):
    """キーボード入力でドローンを制御するノード"""
    
    def __init__(self):
        """初期化"""
        super().__init__('keyboard_control_node')
        
        # パラメータの宣言
        self.declare_parameter('linear_speed', 0.5)  # 線形速度 [m/s]
        self.declare_parameter('angular_speed', 0.5)  # 角速度 [rad/s]
        self.declare_parameter('drone_id', 0)  # 制御対象のドローンID
        
        # パラメータの取得
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.drone_id = self.get_parameter('drone_id').value
        
        # 速度コマンドのパブリッシャー
        self.velocity_pub = self.create_publisher(
            Twist,
            f'/drone_{self.drone_id}/cmd_vel',
            10
        )
        
        # 速度コマンド
        self.linear_vel = np.zeros(3)   # x, y, z
        self.angular_vel = np.zeros(3)  # roll, pitch, yaw
        
        # キーの状態
        self.pressed_keys = set()
        
        # 終了フラグ
        self.running = True
        
        # タイマーの設定（10Hzで速度コマンドを発行）
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        # キーボード入力スレッドの開始
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.get_logger().info('キーボード制御ノードを開始しました')
        self.get_logger().info('使用方法:')
        self.get_logger().info('  矢印キー: 前後左右に移動')
        self.get_logger().info('  W/S: 上昇/下降')
        self.get_logger().info('  A/D: 左右回転')
        self.get_logger().info('  スペース: ホバリング（停止）')
        self.get_logger().info('  Q: 終了')
    
    def keyboard_loop(self):
        """キーボード入力を処理するループ"""
        try:
            self.get_logger().info('キーボード入力の準備完了')
            
            while self.running:
                try:
                    # キー入力を非ブロッキングで読み取る
                    if os.name == 'nt':  # Windows
                        import msvcrt
                        if msvcrt.kbhit():
                            key = msvcrt.getch().decode('utf-8')
                            self.process_key(key)
                    else:  # Linux/Mac
                        key = getch.getch()
                        self.process_key(key)
                except Exception as e:
                    self.get_logger().error(f"キー入力の取得中にエラーが発生しました: {e}")
                
                time.sleep(0.01)  # CPUの使用率を下げるための短い待機
        except Exception as e:
            self.get_logger().error(f"キーボード入力ループでエラーが発生しました: {e}")
    
    def process_key(self, key):
        """キー入力を処理"""
        # デバッグ用にキー入力をログに出力（INFOレベルに変更）
        # self.get_logger().info(f'キー入力: {repr(key)}')
        
        # 特殊キー処理
        if key == 'q' or key == 'Q':  # 終了
            self.running = False
            self.get_logger().info('終了します')
            return
        elif key == ' ':  # スペース（ホバリング）
            self.pressed_keys.clear()
            self.linear_vel = np.zeros(3)
            self.angular_vel = np.zeros(3)
            # self.get_logger().info('ホバリング')
            return
            
        # 矢印キーとその他のキー処理
        # Windowsの場合、矢印キーは特殊なコードを返す
        if os.name == 'nt':
            if key == '\xe0':  # 矢印キーの前置コード
                key = getch.getch()  # 実際の矢印キーコードを取得
                self.get_logger().info(f'Windowsの矢印キーコード: {repr(key)}')
                if key == 'H':  # 上矢印
                    self.pressed_keys.add('up')
                elif key == 'P':  # 下矢印
                    self.pressed_keys.add('down')
                elif key == 'K':  # 左矢印
                    self.pressed_keys.add('left')
                elif key == 'M':  # 右矢印
                    self.pressed_keys.add('right')
        else:
            # Linux/Macの場合、矢印キーはエスケープシーケンス
            if key == '\x1b':  # ESCキー
                try:
                    # エスケープシーケンスの残りを読み取る
                    seq = getch.getch()
                    # self.get_logger().info(f'エスケープシーケンス1: {repr(seq)}')
                    
                    if seq == '[':
                        direction = getch.getch()
                        # self.get_logger().info(f'エスケープシーケンス2: {repr(direction)}')
                        
                        if direction == 'A':  # 上矢印
                            self.pressed_keys.add('up')
                            # self.get_logger().info('上矢印キーを認識しました')
                        elif direction == 'B':  # 下矢印
                            self.pressed_keys.add('down')
                            # self.get_logger().info('下矢印キーを認識しました')
                        elif direction == 'C':  # 右矢印
                            self.pressed_keys.add('right')
                            # self.get_logger().info('右矢印キーを認識しました')
                        elif direction == 'D':  # 左矢印
                            self.pressed_keys.add('left')
                            # self.get_logger().info('左矢印キーを認識しました')
                except Exception as e:
                    self.get_logger().error(f'エスケープシーケンス処理中にエラーが発生しました: {e}')
        
        # 通常のキー処理
        if key in ['w', 'W']:
            self.pressed_keys.add('w')
            # self.get_logger().info('Wキーを認識しました')
        elif key in ['s', 'S']:
            self.pressed_keys.add('s')
            # self.get_logger().info('Sキーを認識しました')
        elif key in ['a', 'A']:
            self.pressed_keys.add('a')
            # self.get_logger().info('Aキーを認識しました')
        elif key in ['d', 'D']:
            self.pressed_keys.add('d')
            # self.get_logger().info('Dキーを認識しました')
        # 代替キー（矢印キーが動作しない場合用）
        elif key in ['i', 'I']:  # 上
            self.pressed_keys.add('up')
            # self.get_logger().info('Iキー（上代替）を認識しました')
        elif key in ['k', 'K']:  # 下
            self.pressed_keys.add('down')
            # self.get_logger().info('Kキー（下代替）を認識しました')
        elif key in ['j', 'J']:  # 左
            self.pressed_keys.add('left')
            # self.get_logger().info('Jキー（左代替）を認識しました')
        elif key in ['l', 'L']:  # 右
            self.pressed_keys.add('right')
            # self.get_logger().info('Lキー（右代替）を認識しました')
        
        # デバッグ用に現在押されているキーをログに出力（INFOレベルに変更）
        # self.get_logger().info(f'押されているキー: {self.pressed_keys}')
        
        # 速度の更新
        self.update_velocity()
        
        # キーを離す処理（シンプルな実装として、1回のキー入力で即座にキーを離す）
        # これにより、キーを押し続ける必要がなくなり、1回のキー入力で1回の移動になる
        self.pressed_keys.clear()
    
    def update_velocity(self):
        """押されているキーに基づいて速度を更新"""
        # 速度をリセット
        self.linear_vel = np.zeros(3)
        self.angular_vel = np.zeros(3)
        
        # 線形速度の更新
        if 'up' in self.pressed_keys:  # 前進
            self.linear_vel[0] = self.linear_speed
        if 'down' in self.pressed_keys:  # 後退
            self.linear_vel[0] = -self.linear_speed
        if 'left' in self.pressed_keys:  # 左移動
            self.linear_vel[1] = self.linear_speed
        if 'right' in self.pressed_keys:  # 右移動
            self.linear_vel[1] = -self.linear_speed
        if 'w' in self.pressed_keys:  # 上昇
            self.linear_vel[2] = self.linear_speed
        if 's' in self.pressed_keys:  # 下降
            self.linear_vel[2] = -self.linear_speed
        
        # 角速度の更新
        if 'a' in self.pressed_keys:  # 左回転
            self.angular_vel[2] = self.angular_speed
        if 'd' in self.pressed_keys:  # 右回転
            self.angular_vel[2] = -self.angular_speed
        
        # デバッグ用に更新された速度をログに出力（INFOレベルに変更）
        # self.get_logger().info(f'更新された速度: linear={self.linear_vel}, angular={self.angular_vel}')
    
    def publish_velocity(self):
        """速度コマンドをパブリッシュ"""
        # Twistメッセージの作成
        twist = Twist()
        
        # 線形速度の設定
        twist.linear.x = float(self.linear_vel[0])
        twist.linear.y = float(self.linear_vel[1])
        twist.linear.z = float(self.linear_vel[2])
        
        # 角速度の設定
        twist.angular.x = float(self.angular_vel[0])
        twist.angular.y = float(self.angular_vel[1])
        twist.angular.z = float(self.angular_vel[2])
        
        # パブリッシュ
        self.velocity_pub.publish(twist)
        
        # 30回に1回ログ出力
        if not hasattr(self, 'log_counter'):
            self.log_counter = 0
        self.log_counter += 1
        
        # if self.log_counter % 30 == 0:
        #     self.get_logger().info(f'速度コマンド: linear={self.linear_vel}, angular={self.angular_vel}')
    
    def shutdown(self):
        """終了処理"""
        # 終了フラグを設定
        self.running = False
        
        # キーボードスレッドが終了するのを待機
        if hasattr(self, 'keyboard_thread') and self.keyboard_thread.is_alive():
            self.keyboard_thread.join(timeout=1.0)
            
        self.get_logger().info('キーボード制御ノードを終了しました')

def main(args=None):
    """メイン関数"""
    # ROS 2の初期化
    rclpy.init(args=args)
    
    # ノードの作成
    node = KeyboardControlNode()
    
    try:
        # スピン（別スレッドで実行）
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        # メインスレッドはキーボード入力のために待機
        try:
            while rclpy.ok():
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
    finally:
        # 終了処理
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
