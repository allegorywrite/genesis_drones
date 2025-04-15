#!/bin/bash

# デフォルト値
F=9.81
TAU_X=0.0
TAU_Y=0.0
TAU_Z=0.0
DT=0.01
NUM_FRAMES=200

# コマンドライン引数の解析
while [[ $# -gt 0 ]]; do
  case $1 in
    --f)
      F="$2"
      shift 2
      ;;
    --tau-x)
      TAU_X="$2"
      shift 2
      ;;
    --tau-y)
      TAU_Y="$2"
      shift 2
      ;;
    --tau-z)
      TAU_Z="$2"
      shift 2
      ;;
    --dt)
      DT="$2"
      shift 2
      ;;
    --num-frames)
      NUM_FRAMES="$2"
      shift 2
      ;;
    *)
      echo "不明な引数: $1"
      exit 1
      ;;
  esac
done

cd se3_drone_simulator

# 仮想環境をアクティベート
source venv/bin/activate

# 引数に基づいてコマンドを構築
CMD="python test_dynamics_fixed_input.py --f $F --tau-x $TAU_X --tau-y $TAU_Y --tau-z $TAU_Z --dt $DT --num-frames $NUM_FRAMES"

# 実行するコマンドを表示
echo "実行: $CMD"

# テストスクリプトを実行
$CMD
