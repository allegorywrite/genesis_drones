#!/bin/bash

# 引数の処理
FOV=${1:-30}  # デフォルト値: 30度
MIN_BARRIER=${2:-0.0}  # デフォルト値: 0.0
MAX_ATTEMPTS=${3:-1000}  # デフォルト値: 1000回
USE_CBF=${4:-false}  # デフォルト値: false
Q=${5:-0.5}  # デフォルト値: 0.5
GAMMA=${6:-0.1}  # デフォルト値: 0.1

# 仮想環境をアクティベート
source venv/bin/activate

# CBFを使用するかどうかのフラグを設定
CBF_FLAG=""
if [ "$USE_CBF" = "true" ]; then
    CBF_FLAG="--use-cbf"
fi

# メインスクリプトを実行
python main.py --fov $FOV --min-barrier $MIN_BARRIER --max-attempts $MAX_ATTEMPTS $CBF_FLAG --q $Q --gamma $GAMMA
