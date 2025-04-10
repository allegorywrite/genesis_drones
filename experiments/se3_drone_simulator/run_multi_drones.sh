#!/bin/bash

# デフォルト値
FOV=30
MIN_BARRIER=0.0
MAX_ATTEMPTS=1000
USE_CBF=false
Q=0.5
GAMMA=0.1
C1=0.5
C2=0.5
H=0.02

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
CMD="python multi_drones/main.py --fov $FOV --min-barrier $MIN_BARRIER --max-attempts $MAX_ATTEMPTS --q $Q --gamma $GAMMA --c1 $C1 --c2 $C2 --h $H"

if [ -n "$CBF_FLAG" ]; then
  CMD="$CMD $CBF_FLAG"
fi

# 実行するコマンドを表示
echo "実行: $CMD"

# メインスクリプトを実行
$CMD
