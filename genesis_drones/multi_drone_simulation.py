#!/usr/bin/env python3

import rclpy
import argparse
import numpy as np
from rclpy.parameter import Parameter

# モジュール化されたシミュレーションノードのインポート
from genesis_drones.simulation.simulation_node import MultiDroneSimulation

def main():
    """
    マルチドローンシミュレーションのメイン関数
    
    コマンドライン引数を解析し、シミュレーションを開始します。
    """
    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description='マルチドローンシミュレーション')
    parser.add_argument('--num_drones', type=int, default=4, help='シミュレーションするドローンの数')
    parser.add_argument('--show_camera', action='store_true', help='カメラ画像をウィンドウに表示する')
    parser.add_argument('--route', type=str, help='飛行ルート（例: "1,1,2;-1,2,1;0,0,0.5"）')
    parser.add_argument('--drone_id', type=int, default=0, help='飛行ルートを適用するドローンのID')
    parser.add_argument('--routes', type=str, help='複数ドローンの飛行ルート（例: "0:1,1,2;-1,2,1|1:0,0,1;1,1,1"）')
    parser.add_argument('--record', action='store_true', help='カメラ映像を録画する')
    parser.add_argument('--no-render-camera', action='store_false', dest='render_camera', help='カメラのレンダリングを行わない（処理速度向上）')
    parser.add_argument('--output', type=str, default="fly_route_camera.mp4", help='録画ファイルの出力先')
    parser.add_argument('--velocity-control', action='store_true', help='速度制御モードを有効にする')
    parser.add_argument('--target-velocity', type=str, default="0.0,0.0,0.0", help='目標速度 (x,y,z) [m/s]')
    parser.add_argument('--target-angular-velocity', type=str, default="0.0,0.0,0.0", help='目標角速度 (roll,pitch,yaw) [rad/s]')
    parser.add_argument('--velocity-profile', type=str, default="constant", help='速度プロファイル (constant, ramp, sinusoidal)')
    args = parser.parse_args()
    
    # ROS 2の初期化
    rclpy.init()
    
    # コマンドライン引数の値をログに出力
    print(f"Command line args: render_camera={args.render_camera}, velocity_control={args.velocity_control}")
    
    # シミュレーションの開始
    simulation = MultiDroneSimulation(
        num_drones=args.num_drones, 
        show_camera=args.show_camera,
        record_camera=args.record,
        render_camera=args.render_camera,
        output_file=args.output
    )
    
    # コマンドライン引数からROS 2パラメータに値を設定
    if args.route:
        simulation.set_parameter(Parameter('route', value=args.route))
    if args.routes:
        simulation.set_parameter(Parameter('routes', value=args.routes))
    if args.drone_id != 0:  # デフォルト値でない場合のみ設定
        simulation.set_parameter(Parameter('drone_id', value=args.drone_id))
    if args.velocity_control:
        simulation.set_parameter(Parameter('velocity_control', value=True))
    if args.target_velocity != "0.0,0.0,0.0":
        simulation.set_parameter(Parameter('target_velocity', value=args.target_velocity))
    if args.target_angular_velocity != "0.0,0.0,0.0":
        simulation.set_parameter(Parameter('target_angular_velocity', value=args.target_angular_velocity))
    if args.velocity_profile != "constant":
        simulation.set_parameter(Parameter('velocity_profile', value=args.velocity_profile))
    
    try:
        # ROS 2のスピン
        rclpy.spin(simulation)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了処理
        simulation.shutdown()
        simulation.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
