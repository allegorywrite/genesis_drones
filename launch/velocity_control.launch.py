#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    """
    速度制御デモの起動ファイル
    """
    # 起動引数の宣言
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='1',
        description='シミュレーションするドローンの数'
    )
    
    # 速度制御モード
    velocity_control_arg = DeclareLaunchArgument(
        'velocity_control',
        default_value='true',
        description='速度制御モードを有効にするかどうか'
    )
    
    # 目標速度 (x, y, z) [m/s]
    target_velocity_arg = DeclareLaunchArgument(
        'target_velocity',
        default_value='0.0,0.0,0.0',  # デフォルトはx方向に0.5m/s
        description='目標速度 (x,y,z) [m/s]'
    )
    
    # 目標角速度 (roll, pitch, yaw) [rad/s]
    target_angular_velocity_arg = DeclareLaunchArgument(
        'target_angular_velocity',
        default_value='0.0,0.0,0.0',
        description='目標角速度 (roll,pitch,yaw) [rad/s]'
    )
    
    # 速度プロファイル (linear, angular)
    velocity_profile_arg = DeclareLaunchArgument(
        'velocity_profile',
        default_value='constant',  # constant, ramp, sinusoidal
        description='速度プロファイルタイプ (constant, ramp, sinusoidal)'
    )
    
    # カメラ表示設定
    show_camera_arg = DeclareLaunchArgument(
        'show_camera',
        default_value='false',
        description='カメラ画像をウィンドウに表示するかどうか'
    )
    
    # URDFファイルのパス
    urdf_path = os.path.join(os.path.expanduser('~'), 'colcon_ws/src/genesis_drones/genesis_drones/urdf/crazyflie_camera.urdf.xacro')
    
    # URDFファイルが存在するか確認
    if os.path.exists(urdf_path):
        # URDFファイルの処理（仮想環境を使用）
        urdf_process = ExecuteProcess(
            cmd=['/bin/bash', '-c', f'. ~/genesis_venv/bin/activate && python3 -m genesis_ros.genesis_ros --urdf {urdf_path}'],
            output='screen'
        )
    else:
        # URDFファイルが存在しない場合は、処理をスキップ
        urdf_process = None
        print(f"Warning: URDF file not found at {urdf_path}")
    
    # シミュレーションノード（ROS 2のrunコマンドを使用）
    simulation_node = Node(
        package='genesis_drones',
        executable='multi_drone_simulation',
        name='velocity_control_demo',
        output='screen',
        parameters=[
            {'num_drones': LaunchConfiguration('num_drones')},
            {'velocity_control': LaunchConfiguration('velocity_control')},
            {'target_velocity': LaunchConfiguration('target_velocity')},
            {'target_angular_velocity': LaunchConfiguration('target_angular_velocity')},
            {'velocity_profile': LaunchConfiguration('velocity_profile')},
            {'show_camera': LaunchConfiguration('show_camera')}
        ]
    )
    
    # LaunchDescriptionの作成
    ld = [
        num_drones_arg,
        velocity_control_arg,
        target_velocity_arg,
        target_angular_velocity_arg,
        velocity_profile_arg,
        show_camera_arg,
        simulation_node
    ]
    
    return LaunchDescription(ld)
