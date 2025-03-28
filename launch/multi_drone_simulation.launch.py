#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    """
    マルチドローンシミュレーションの起動ファイル
    """
    # 起動引数の宣言
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='2',
        description='シミュレーションするドローンの数'
    )
    
    show_camera_arg = DeclareLaunchArgument(
        'show_camera',
        default_value='false',
        description='カメラ画像をウィンドウに表示するかどうか'
    )
    
    route_arg = DeclareLaunchArgument(
        'route',
        default_value='',
        description='飛行ルート（例: "1,1,2;-1,2,1;0,0,0.5"）'
    )
    
    routes_arg = DeclareLaunchArgument(
        'routes',
        default_value='',
        description='複数ドローンの飛行ルート（例: "0:1,1,2;-1,2,1|1:0,0,1;1,1,1"）'
    )
    
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='0',
        description='飛行ルートを適用するドローンのID'
    )
    
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='false',
        description='カメラ映像を録画するかどうか'
    )
    
    output_arg = DeclareLaunchArgument(
        'output',
        default_value='fly_route_camera.mp4',
        description='録画ファイルの出力先'
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
        name='multi_drone_simulation',
        output='screen',
        parameters=[
            {'num_drones': LaunchConfiguration('num_drones')},
            {'route': LaunchConfiguration('route')},
            {'routes': LaunchConfiguration('routes')},
            {'drone_id': LaunchConfiguration('drone_id')},
            {'record': LaunchConfiguration('record')},
            {'output': LaunchConfiguration('output')},
            {'show_camera': LaunchConfiguration('show_camera')}
        ]
    )
    
    # LaunchDescriptionの作成
    ld = [
        num_drones_arg,
        show_camera_arg,
        route_arg,
        routes_arg,
        drone_id_arg,
        record_arg,
        output_arg,
        simulation_node
    ]
    
    return LaunchDescription(ld)
