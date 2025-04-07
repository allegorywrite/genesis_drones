#!/bin/bash

# デフォルト値
USE_CBF=false
CBF_TYPE="no-decomp"
CBF_METHOD="pcl"
OBSTACLES=0
KEEP_FOV_HISTORY=false
FOV_SAVE_INTERVAL=50

# コマンドライン引数の解析
while [[ $# -gt 0 ]]; do
  case $1 in
    --use-cbf)
      USE_CBF=true
      shift
      ;;
    --cbf-type)
      CBF_TYPE="$2"
      shift 2
      ;;
    --cbf-method)
      CBF_METHOD="$2"
      shift 2
      ;;
    --obstacles)
      OBSTACLES="$2"
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
CMD="python -m single_drone.single_drone_demo"

if [ "$USE_CBF" = true ]; then
  CMD="$CMD --use-cbf --cbf-type $CBF_TYPE --cbf-method $CBF_METHOD"
fi

if [ "$OBSTACLES" -gt 0 ]; then
  CMD="$CMD --obstacles $OBSTACLES"
fi

if [ "$KEEP_FOV_HISTORY" = true ]; then
  CMD="$CMD --keep-fov-history"
fi

CMD="$CMD --fov-save-interval $FOV_SAVE_INTERVAL"

# 実行するコマンドを表示
echo "実行: $CMD"

# 単一ドローンデモスクリプトを実行
$CMD
