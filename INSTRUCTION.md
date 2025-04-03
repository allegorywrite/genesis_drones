scripts/velocity_tracking_plot.py
について
以下のプロットを各パラメータの試行ごとに出力してください
・目標速度に対する応答のプロット
・A*グリッドサーチの進行状況のプロット(P-I, I-D, D-Pについてのグリッドサーチを表示)
    ・同じ画像ファイルを上書きして更新する．

あなたは出力文字数が一定値を超えると途切れるようになっています．長い文章を出力する場合は，適宜区切り，write_in_fileではなくreplase_in_fileを用いてファイルを更新してください．

絶対に実装したコードを以下のコマンドで完全に実行できることを確認してから終了してください．
```bash
venv # 仮想環境立ち上げ
python3 src/genesis_drones/scripts/velocity_tracking_plot.py --steps 100 --optimize --output
```