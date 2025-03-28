#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    """
    マニュアル制御（キーボード操作）の起動ファイル
    """
    # 起動引数の宣言
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='1',
        description='シミュレーションするドローンの数'
    )
    
    # 速度制御モード（常に有効）
    velocity_control_arg = DeclareLaunchArgument(
        'velocity_control',
        default_value='true',
        description='速度制御モードを有効にするかどうか'
    )
    
    # 線形速度の大きさ
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.5',
        description='線形速度の大きさ [m/s]'
    )
    
    # 角速度の大きさ
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.5',
        description='角速度の大きさ [rad/s]'
    )
    
    # 制御対象のドローンID
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='0',
        description='制御対象のドローンID'
    )
    
    # カメラ表示設定
    show_camera_arg = DeclareLaunchArgument(
        'show_camera',
        default_value='true',
        description='カメラ画像をウィンドウに表示するかどうか'
    )
    
    # シミュレーションノード
    simulation_node = Node(
        package='genesis_drones',
        executable='multi_drone_simulation',
        name='manual_control_simulation',
        output='screen',
        parameters=[
            {'num_drones': LaunchConfiguration('num_drones')},
            {'velocity_control': LaunchConfiguration('velocity_control')},
            {'show_camera': LaunchConfiguration('show_camera')}
        ]
    )
    
    # キーボード制御ノード
    keyboard_control_node = Node(
        package='genesis_drones',
        executable='keyboard_control_node.py',
        name='keyboard_control',
        output='screen',
        parameters=[
            {'linear_speed': LaunchConfiguration('linear_speed')},
            {'angular_speed': LaunchConfiguration('angular_speed')},
            {'drone_id': LaunchConfiguration('drone_id')}
        ]
    )
    
    # LaunchDescriptionの作成
    ld = [
        num_drones_arg,
        velocity_control_arg,
        linear_speed_arg,
        angular_speed_arg,
        drone_id_arg,
        show_camera_arg,
        simulation_node,
        keyboard_control_node
    ]
    
    return LaunchDescription(ld)
