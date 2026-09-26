# marker-plane-lock

ArUcoマーカーを使い、ノートや作業面のカメラブレを軽減する動画補正ツールです。

## Demo
左が生の撮影動画、右が補正後の動画

![補正前後の比較プレビュー](docs/comparison.gif)

[▶ 高画質の比較動画を再生](docs/comparison.mp4)

## マーカーの印刷と配置

[A4印刷用ID 0マーカーPDFをダウンロード](docs/aruco_dict4x4_50_id0_a4.pdf) します。マーカー本体の
一辺が10・15・20・30・40・50 mmの6種類を掲載しています。印刷時は「実際のサイズ」または
100%を選び、用紙に合わせた拡大縮小は無効にしてください。

黒い正方形は周囲の白い余白を残して切り出し、作業面に貼り付けます。マーカー1枚で
上下左右の平行移動を補正できます。複数枚が見える場合は中心位置を組み合わせて推定します。

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

`DICT_4X4_50` のID 0〜3を使うオフライン版は次の形式で実行します。1枚の場合は
マーカー中心の移動から上下左右の平行移動を推定し、複数枚の場合は中心位置を組み合わせます。
補正で画面外になった領域は黒で埋め、出力解像度は入力と同じです。
検出不能区間は前後から補間し、画面全体で
0.4px未満の推定変動は固定して、静止時に補正処理自身が揺れを作らないようにします。
モーションブラーなどでArUco検出が短時間だけ途切れた場合は、直前のマーカー位置をOptical
Flowで最大8フレーム追跡します。追跡誤差が大きい結果やマーカー形状が崩れた結果は採用しません。

## Usage

```bash
./build/video_marker_offline input.mp4 output.mp4
```

`--step N` を付けると、ArUcoの検出をNフレームごとに行います。中間フレームでは
Optical Flowでマーカーを追跡します。既定値は `1` です。

```bash
./build/video_marker_offline input.mp4 output.mp4 --step 4
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
