"""
設定関連の関数を提供するモジュール
"""
import argparse

def parse_arguments():
    """
    コマンドライン引数を解析
    
    Returns:
    --------
    args : argparse.Namespace
        解析された引数
    """
    parser = argparse.ArgumentParser(description='SE(3)ドローンシミュレータ')
    parser.add_argument('--fov', type=float, default=30.0,
                        help='視野角（度）（デフォルト: 30.0）')
    parser.add_argument('--min-barrier', type=float, default=0.0,
                        help='最小安全集合値（デフォルト: 0.0）')
    parser.add_argument('--max-attempts', type=int, default=1000,
                        help='最大試行回数（デフォルト: 1000）')
    parser.add_argument('--use-cbf', action='store_true',
                        help='Control Barrier Functionを使用するかどうか')
    parser.add_argument('--q', type=float, default=0.5,
                        help='確率の閾値（デフォルト: 0.5）')
    parser.add_argument('--gamma', type=float, default=0.1,
                        help='CBFのゲイン（デフォルト: 0.1）')
    parser.add_argument('--c1', type=float, default=0.5,
                        help='CBF制約の重み1（デフォルト: 0.5）')
    parser.add_argument('--c2', type=float, default=0.5,
                        help='CBF制約の重み2（デフォルト: 0.5）')
    parser.add_argument('--h', type=float, default=0.1,
                        help='時間ステップ（デフォルト: 0.1）')
    parser.add_argument('--optimization-method', type=str, default='centralized',
                        choices=['centralized', 'distributed'],
                        help='最適化手法（centralized: 集中型, distributed: 分散型）')
    parser.add_argument('--c', type=float, default=1.0,
                        help='分散型最適化のペナルティパラメータ（デフォルト: 1.0）')
    parser.add_argument('--max-iter', type=int, default=10,
                        help='分散型最適化の最大反復回数（デフォルト: 10）')
    parser.add_argument('--keep-fov-history', action='store_true',
                        help='過去の視野錐台描画を残すかどうか')
    parser.add_argument('--fov-save-interval', type=int, default=10,
                        help='視野錐台を保存する間隔（フレーム数）（デフォルト: 10）')
    parser.add_argument('--dynamics-model', type=str, choices=['kinematics', 'dynamics'], default='kinematics',
                        help='動力学モデル: kinematics (1次系) または dynamics (2次系)')
    parser.add_argument('--trajectory-type', type=str, 
                        choices=['circle', 'eight', 'lissajous', 'snake', 'snake_3d', 'step', 'fixed'], 
                        default='fixed',
                        help='目標軌道の種類: circle (円軌道), eight (8の字軌道), lissajous (リサージュ曲線), snake (蛇行軌道), snake_3d (3D蛇行軌道), step ([0,0,1]に固定), fixed (現在の固定目標位置)')
    return parser.parse_args()
