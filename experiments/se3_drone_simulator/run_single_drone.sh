#!/bin/bash

# デフォルト値
USE_CBF=false
OBSTACLES=0

# コマンドライン引数の解析
while [[ $# -gt 0 ]]; do
  case $1 in
    --use-cbf)
      USE_CBF=true
      shift
      ;;
    --obstacles)
      OBSTACLES="$2"
      shift 2
      ;;
    *)
      echo "不明な引数: $1"
      exit 1
      ;;
  esac
done

# 仮想環境をアクティベート
source venv/bin/activate

# 引数に基づいてコマンドを構築
CMD="python single_drone_demo.py"

if [ "$USE_CBF" = true ]; then
  CMD="$CMD --use-cbf"
fi

if [ "$OBSTACLES" -gt 0 ]; then
  CMD="$CMD --obstacles $OBSTACLES"
fi

# 実行するコマンドを表示
echo "実行: $CMD"

# 単一ドローンデモスクリプトを実行
$CMD
