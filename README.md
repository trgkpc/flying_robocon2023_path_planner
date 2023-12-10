# ひこロボ経路生成プログラム
- いくつかの経路パターンの組み合わせとして経路を生成
  - 等速直線運動
  - 等加速度運動
  - 等速円運動
  - 加速度制限運動

# 注意事項
- 自己責任で利用してください

# 使い方
- Step1. 経路を生成する
  - `python3 main.py`
    - 上手くいかない場合は `numpy matplotlib pytorch` をインストール
  - 出てきた結果を `main.py` と `cpp/test/rout*` に保存する
  - ビジュアライズされた結果は `output/*.png` に保存される
- Step2. 動画を作成して動作確認
  - `./generate_animation.sh` で動画が作成される
    - 上手くいかない場合は `ffmpeg` をインストール
  - `output/output.mp4` に保存される
- Step3. C++ での動作確認
  - マルチコプターに積むマイコン向けに，作成した経路を C++ に移植する
  - 経路情報は Step1 にて `cpp` 以下に保存されている
  - `cd tpp/test ; ./compile_and_run.sh` を実施し，経路がそれっぽく表示されればOK
  - `cpp` 以下のディレクトリをマイコンのプログラムにコピペすれば良い
