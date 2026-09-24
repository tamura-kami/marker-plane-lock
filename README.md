# marker-plane-lock

ArUcoマーカーを使い、ノートや作業面のカメラブレを軽減する動画補正ツールです。

## Demo
左が生の撮影動画、右が補正後の動画

![補正前後の比較プレビュー](docs/comparison.gif)

[▶ 高画質の比較動画を再生](docs/comparison.mp4)

## マーカーの印刷と配置

[A4印刷用マーカーPDFをダウンロード](docs/aruco_dict4x4_50_ids_0-3_a4.pdf) を印刷します。

各黒い正方形は、切り出すときに周囲の白い余白を残し、ID 0〜3 を作業面の四隅へ
貼り付けます。ID の配置順序は問いませんが、動画中で回転したり、隠れたりしないようにします。
4 枚すべて写っているときが最も安定します（動作には同時に2枚以上の検出が必要です）。

## Prerequisites

- CMake 3.16 以降
- C++17対応コンパイラ
- OpenCV（`aruco`, `calib3d`, `core`, `imgproc`, `video`, `videoio`）

Ubuntuでは、以下で開発パッケージを導入できます。

```bash
sudo apt install build-essential cmake libopencv-dev
```

## Build

```bash
cmake -S . -B build
cmake --build build
```

`DICT_4X4_50` のID 0-3をノートの四隅に貼り、ノート面そのものを固定する独立した
オフライン版は次の形式で実行します。複数マーカーの中心から安定したSimilarity変換を
推定し、1枚しか見えない場合は四隅を利用します。検出不能区間は前後から補間し、画面全体で
0.4px未満の推定変動は固定して、静止時に補正処理自身が揺れを作らないようにします。
モーションブラーなどでArUco検出が短時間だけ途切れた場合は、直前に検出した四隅をOptical
Flowで最大8フレーム追跡します。前後追跡誤差や四角形の形状が不正な追跡結果は採用しません。

## Usage

```bash
./build/video_marker_offline input.mp4 output.mp4
```

`--step N` を付けると、ArUcoの検出をNフレームごとに行います。中間フレームでは
Optical Flowで四隅を追跡します。既定値は `1` です。

```bash
./build/video_marker_offline input.mp4 output.mp4 --step 4
```

4枚のマーカーの内側だけを出力したい場合は、`--crop-inside-markers` を指定します。
基準フレームで各マーカーの中央寄りの角を結んだ範囲から、有効な最大の長方形を切り出すため、
マーカー自体は出力に含まれません。このオプションには、同一フレームで4枚すべてのマーカーが
検出されていることが必要です。

```bash
./build/video_marker_offline input.mp4 output.mp4 --crop-inside-markers
```

マーカー検出を確認するデバッグ動画も同時に出力できます。直接検出は緑、Optical Flowで
補完したマーカーは橙で表示します。

```bash
./build/video_marker_offline input.mp4 output.mp4 \
    --debug-markers marker_debug.mp4
```

元動画と補正動画を左右に並べた比較動画は、次のスクリプトで作成できます。解像度差は
拡大せず、中央寄せの黒い余白で自動的に揃えます。音声は元動画からコピーします。

```bash
./tools/make_comparison.sh input.mp4 stabilized.mp4 comparison.mp4
```

左右のラベルも指定できます。

```bash
./tools/make_comparison.sh input.mp4 stabilized.mp4 comparison.mp4 \
    "Input" "Stable marker lock"
```
