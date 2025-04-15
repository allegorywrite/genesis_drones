#!/bin/bash

# デフォルト値
FOV=30
MIN_BARRIER=0.0
MAX_ATTEMPTS=1000
USE_CBF=false
Q=0.05  # 確率の閾値を下げる
GAMMA=1.0
C1=0.5
C2=0.5
H=0.02
OPTIMIZATION_METHOD="centralized"
C=10.0
MAX_ITER=10
KEEP_FOV_HISTORY=false
FOV_SAVE_INTERVAL=10
DYNAMICS_MODEL="kinematics"  # 動力学モデル（kinematics または dynamics）
TRAJECTORY_TYPE="fixed"      # 軌道タイプ（circle, eight, lissajous, snake, snake_3d, step, fixed）

# コマンドライン引数の解析
while [[ $# -gt 0 ]]; do
  case $1 in
    --fov)
      FOV="$2"
      shift 2
      ;;
    --min-barrier)
      MIN_BARRIER="$2"
      shift 2
      ;;
    --max-attempts)
      MAX_ATTEMPTS="$2"
      shift 2
      ;;
    --use-cbf)
      USE_CBF=true
      shift
      ;;
    --q)
      Q="$2"
      shift 2
      ;;
    --gamma)
      GAMMA="$2"
      shift 2
      ;;
    --c1)
      C1="$2"
      shift 2
      ;;
    --c2)
      C2="$2"
      shift 2
      ;;
    --optimization-method)
      OPTIMIZATION_METHOD="$2"
      shift 2
      ;;
    --c)
      C="$2"
      shift 2
      ;;
    --max-iter)
      MAX_ITER="$2"
      shift 2
      ;;
    --keep-fov-history)
      KEEP_FOV_HISTORY=true
      shift
      ;;
    --fov-save-interval)
      FOV_SAVE_INTERVAL="$2"
      shift 2
      ;;
    --dynamics-model)  # 動力学モデルオプション
      DYNAMICS_MODEL="$2"
      shift 2
      ;;
    --trajectory-type)  # 軌道タイプオプション
      TRAJECTORY_TYPE="$2"
      shift 2
      ;;
    *)
      echo "不明な引数: $1"
      exit 1
      ;;
  esac
done

# カレントディレクトリをse3_drone_simulatorに変更
cd se3_drone_simulator

# 仮想環境をアクティベート
source venv/bin/activate

# フラグを設定
CBF_FLAG=""
if [ "$USE_CBF" = true ]; then
  CBF_FLAG="--use-cbf"
fi

# 実行するコマンドを構築
export PYTHONPATH=$PYTHONPATH:$(pwd)
CMD="python multi_drones/main.py --fov $FOV --min-barrier $MIN_BARRIER --max-attempts $MAX_ATTEMPTS --q $Q --gamma $GAMMA --c1 $C1 --c2 $C2 --h $H --optimization-method $OPTIMIZATION_METHOD --c $C --max-iter $MAX_ITER"

if [ -n "$CBF_FLAG" ]; then
  CMD="$CMD $CBF_FLAG"
fi

if [ "$KEEP_FOV_HISTORY" = true ]; then
  CMD="$CMD --keep-fov-history"
fi

CMD="$CMD --fov-save-interval $FOV_SAVE_INTERVAL --dynamics-model $DYNAMICS_MODEL --trajectory-type $TRAJECTORY_TYPE"

# 実行するコマンドを表示
echo "実行: $CMD"

# メインスクリプトを実行
$CMD
