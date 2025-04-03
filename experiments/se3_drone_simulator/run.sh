#!/bin/bash

# 引数の処理
FOV=${1:-30}  # デフォルト値: 30度
MIN_SHARED=${2:-3}  # デフォルト値: 3個
MAX_ATTEMPTS=${3:-1000}  # デフォルト値: 1000回

# 仮想環境をアクティベート
source venv/bin/activate

# メインスクリプトを実行
python main.py --fov $FOV --min-shared $MIN_SHARED --max-attempts $MAX_ATTEMPTS
