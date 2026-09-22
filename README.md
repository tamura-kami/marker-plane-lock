# OBS Fixed Camera Stabilizer

固定WebカメラのX/Y方向の微振動を低減するためのOBS Studio Video Filterです。
現在は、OBSに依存しないPhase 1のオフライン検証ツールを開発対象とします。

## Prerequisites

- CMake 3.16 以降
- C++17対応コンパイラ
- OpenCV（`core`, `imgproc`, `video`, `videoio`, `calib3d`）

Ubuntuでは、以下で開発パッケージを導入できます。

```bash
sudo apt install build-essential cmake libopencv-dev
```

## Build

```bash
cmake -S . -B build
cmake --build build
```

検証ツールは次の形式で実行します。

```bash
./build/video_test input.mp4 output.mp4
```

動画全体を先に解析し、検出失敗区間の補間と必要最小限のcropを行うオフライン版は
次の形式で実行します。オフライン版はcrop後の映像を拡大せず、補正変換を1回だけ適用します。

```bash
./build/video_offline input.mp4 output.mp4
```

`DICT_4X4_50` のID 0-3をノートの四隅に貼り、ノート面そのものを固定する独立した
オフライン版は次の形式で実行します。複数マーカーの中心から安定したSimilarity変換を
推定し、1枚しか見えない場合は四隅を利用します。検出不能区間は前後から補間し、画面全体で
0.4px未満の推定変動は固定して、静止時に補正処理自身が揺れを作らないようにします。
モーションブラーなどでArUco検出が短時間だけ途切れた場合は、直前に検出した四隅をOptical
Flowで最大8フレーム追跡します。前後追跡誤差や四角形の形状が不正な追跡結果は採用しません。

```bash
./build/video_marker_offline input.mp4 output.mp4
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

最初のフレームを固定カメラの基準位置として保持し、映像外周の特徴点を各フレームから追跡します。
中央の手やペンを避けた特徴点群からRANSACでX/Y移動と微小回転を推定し、基準位置へ直接戻します。
直前フレームからの移動を積算しないためドリフトせず、振動を滑らかなカメラ移動として残しません。
ROIや専用マーカーの指定は不要です。

外周から十分な手がかりを得られないフレームでは、安全のためそのフレームの補正を停止します。

各フレームについて、X/Y補正量、回転角、推定の有効性、追跡特徴点数、RANSAC合意点数、信頼度、
各処理の所要時間を標準出力へ記録します。補正量は5%のcrop余白内に制限し、補正後は元解像度へ
拡大します。
