# OBS Fixed Camera Stabilizer

## 1. 概要

固定されたWebカメラで、ノートへの筆記やホワイトボードへの書き込みを撮影する際に発生する微振動をリアルタイムで補正するOBS Studio用Video Filterプラグインを開発する。

主な対象は、机やカメラアームに伝わる振動によって発生する数px〜数十px程度の映像の揺れである。

一般的な動画スタビライザーのような手持ち撮影やパン・チルトへの対応は目的としない。

本プラグインでは、

> カメラは本来固定されており、検出された画面全体の移動は不要な振動である

という前提を利用する。

---

# 2. 技術スタック

以下を使用する。

- Language: C++17以上
- Build System: CMake
- Image Processing: OpenCV
- Target Application: OBS Studio
- Plugin Type: OBS Video Filter
- Target OS:

  - Ubuntu
  - macOS

Pythonは使用しない。

補正アルゴリズムはOBS固有コードから分離し、通常のC++プログラムからも利用可能な構造にする。

---

# 3. 想定用途

以下のような固定Webカメラによる動画撮影を対象とする。

- ノートへの筆記
- 紙への描画
- ホワイトボードへの筆記
- デスク上の作業撮影
- 固定カメラによる手元撮影

特に、

- 机に手を置く
- 筆記する
- キーボードを操作する
- カメラスタンドに振動が伝わる

などによって生じる小さな画面揺れを除去する。

---

# 4. 基本構成

```text
Web Camera
    ↓
OBS Video Capture Source
    ↓
Fixed Camera Stabilizer Filter
    ↓
OBS Preview / Recording
```

カメラデバイスの取得、録画、エンコード、音声処理などはOBS側に任せる。

本プラグインは映像フレームの補正のみ担当する。

---

# 5. MVPの目標

最初のバージョンでは、

> 固定カメラに発生したX/Y方向の微振動をリアルタイムで低減できること

のみを目標とする。

高度な自動調整や汎用的な動画スタビライゼーションは実装しない。

---

# 6. プロジェクト構成

補正アルゴリズムとOBS固有コードを分離する。

概念的には以下の構成とする。

```text
obs-fixed-camera-stabilizer/
├── CMakeLists.txt
├── README.md
├── SPEC.md
│
├── src/
│   ├── stabilizer/
│   │   ├── stabilizer.hpp
│   │   └── stabilizer.cpp
│   │
│   └── obs/
│       └── obs_filter.cpp
│
└── tools/
    └── video_test.cpp
```

`stabilizer` はOBSに依存してはならない。

例えば以下のようなインターフェースを想定する。

```cpp
struct Motion {
    double dx;
    double dy;
    bool valid;
};

class Stabilizer {
public:
    Motion process(const cv::Mat& frame);
    void reset();
};
```

具体的なAPI設計は実装時に調整してよい。

---

# 7. 開発フェーズ

## Phase 1: オフラインアルゴリズム検証

最初からOBSプラグインを実装しない。

まず `tools/video_test.cpp` を作成し、録画済み動画に対して補正アルゴリズムを検証する。

```text
Input Video
    ↓
OpenCV VideoCapture
    ↓
Stabilizer
    ↓
Translation Correction
    ↓
Crop
    ↓
OpenCV VideoWriter
    ↓
Output Video
```

コマンド例:

```bash
./video_test input.mp4 output.mp4
```

これによりOBSとは無関係に補正品質を評価できるようにする。

---

# 8. フレーム間移動量の推定

連続するフレーム間からカメラの移動量を推定する。

基本処理:

```text
Frame(t-1)
Frame(t)
    ↓
Resize
    ↓
Grayscale
    ↓
Feature Detection
    ↓
Optical Flow
    ↓
Outlier Removal
    ↓
Translation Estimation
    ↓
dx, dy
```

OpenCVを利用する。

初期実装では以下を使用してよい。

```cpp
cv::goodFeaturesToTrack()
cv::calcOpticalFlowPyrLK()
cv::estimateAffinePartial2D()
```

外れ値除去にはRANSACを使用する。

ただし、最終的に必要なのはX/Y方向の平行移動量のみである。

---

# 9. 動体への対応

撮影対象には、

- 手
- 腕
- ペン
- 筆記中の文字

などの動体が存在する。

これらの移動をカメラ移動として誤認してはならない。

基本原則は、

> 多数の特徴点に共通する移動をカメラ移動とみなす

ことである。

例:

```text
背景由来

→ → → → →
 → → → →
→ → → → →

手・ペン由来

 ↑
←  ↙
   ↓
```

RANSACによって局所的な動体由来の特徴点を外れ値として除外する。

MVPではユーザーによるROI指定を実装しない。

---

# 10. 補正

MVPではX/Y方向の平行移動のみ補正する。

推定値:

```text
dx
dy
```

例えば、

```text
dx = +5 px
dy = -3 px
```

と推定された場合、

```text
x = -5 px
y = +3 px
```

の方向へ映像を移動する。

OpenCVによる検証プログラムでは `cv::warpAffine()` を利用してよい。

OBSプラグイン版ではパフォーマンスを考慮し、CPUでの `warpAffine()` が適切か、OBSのGPU描画機構を利用するべきかを実装時に判断する。

移動量推定ロジック自体は描画方法に依存させない。

---

# 11. MVPでは実装しない補正

以下は実装しない。

- 回転補正
- 拡大縮小補正
- Perspective補正
- 意図的なパンへの追従
- 意図的なチルトへの追従

必要性を確認した後に追加する。

---

# 12. Crop

映像移動によって端に無効領域が出ないよう、周囲をcropする。

MVPでは固定値を使用する。

初期値:

```text
5%
```

自動cropは実装しない。

---

# 13. 移動量推定用画像

Optical FlowはFull HD画像に対して直接実行する必要はない。

初期実装では縮小画像を使用する。

例:

```text
1920 x 1080
      ↓
480 x 270
      ↓
Feature Detection
      ↓
Optical Flow
      ↓
dx = 1.25
dy = -0.75
      ↓
scale × 4
      ↓
dx = 5
dy = -3
```

最終的な補正量は元解像度へ変換する。

縮小率は定数または設定値として管理し、コード中にマジックナンバーを散在させない。

---

# 14. 特徴点

初期値として、

```text
100〜300 points
```

程度を使用する。

MVPでは特徴点数の自動調整は行わない。

特徴点が不足した場合は再検出する。

---

# 15. 異常状態

十分な特徴点を取得できない場合は補正しない。

例えば、

```text
tracked points < threshold
```

または、

```text
RANSAC inlier ratio < threshold
```

の場合、

```text
Motion {
    dx = 0,
    dy = 0,
    valid = false
}
```

相当の結果を返す。

異常に大きな移動量が推定された場合も補正しない。

基本方針:

> 信頼できない場合は映像を加工しない。

---

# 16. ページめくり等

ノートのページをめくった場合など、画面内容が大幅に変化する可能性がある。

MVPでは高度なシーン認識は行わない。

```text
特徴点追跡失敗
    ↓
Motion invalid
    ↓
補正停止
    ↓
特徴点再検出
    ↓
追跡再開
```

という単純な処理で対応する。

---

# 17. パフォーマンス要件

主要ターゲット:

```text
1920 x 1080
60 fps
```

60fpsでは1フレームあたり、

```text
16.67 ms
```

である。

OBS自身にも処理時間が必要なため、本プラグインのCPU処理について、

```text
5〜10 ms / frame 以下
```

を目標とする。

16.67ms/frameを継続的に超えないこと。

---

# 18. パフォーマンス計測

Phase 1の時点から処理時間を計測できるようにする。

少なくとも以下を計測可能にする。

```text
resize
grayscale
feature detection
optical flow
motion estimation
frame transform
total
```

C++では `std::chrono` 等を利用してよい。

最適化は計測結果を確認してから行う。

---

# 19. レイテンシ

リアルタイムプレビューを重視する。

未来フレームを必要とする方式は使用しない。

現在および過去フレームのみから補正量を決定する。

追加遅延は可能な限り1フレーム以内とする。

---

# 20. OBSプラグイン

Phase 1およびPhase 2完了後、OBS Video Filterとして実装する。

OBS側から入力フレームを受け取り、

```text
OBS frame
   ↓
Stabilizer::process()
   ↓
Motion { dx, dy }
   ↓
映像位置補正
   ↓
Crop
   ↓
OBS output
```

とする。

OBS固有コードには画像解析アルゴリズムを直接実装しない。

---

# 21. UI

MVPでは設定項目を最小限にする。

想定:

```text
Fixed Camera Stabilizer

Enabled: ON/OFF
Crop: 5%
```

デバッグ用途として、

```text
dx
dy
tracked features
RANSAC inliers
processing time
```

を確認可能にしてもよい。

---

# 22. 将来実装候補

MVP完成後に必要性を確認して追加する。

### Reference Lock

基準位置を自動的に保持し、長時間のドリフトを防ぐ。

### Rotation Correction

```text
dx
dy
theta
```

を推定して微小回転も補正する。

### Automatic Crop

観測された最大振動量からcrop量を自動決定する。

### Automatic Parameter Adjustment

特徴点数や各種閾値を自動調整する。

### Large Scene Change Detection

ページめくりやホワイトボード消去などを検出する。

### GPU Transform

必要に応じてOBSのGPU描画機構で映像変換を行う。

---

# 23. 非目標

以下を主要用途としない。

- 手持ち撮影
- 歩行撮影
- スマートフォン動画
- 意図的なパン・チルト
- ジンバル代替
- 撮影済み動画向け高品質スタビライザー

対象は、

> 固定Webカメラに発生する不要な微振動のリアルタイム除去

である。

---

# 24. 実装上の原則

1. C++17以上を使用する。
2. OpenCVを画像解析に使用する。
3. CMakeでビルドする。
4. Pythonは使用しない。
5. アルゴリズム部分をOBSから独立させる。
6. 最初からOBSプラグインを作らず、録画済み動画でアルゴリズムを検証する。
7. まずX/Y平行移動だけを扱う。
8. 固定カメラという前提を積極的に利用する。
9. 信頼できない推定結果では補正しない。
10. 60fpsを維持できる処理量を優先する。
11. 動体である手・ペンをカメラ移動として扱わない。
12. MVPでは高度な自動調整を実装しない。
13. パフォーマンス最適化は必ず計測結果に基づいて行う。

---

# 25. MVP完成条件

以下を満たした時点でMVP完成とする。

- OBS Video Filterとして追加できる。
- Webカメラ映像に対してリアルタイム動作する。
- 1920×1080 / 60fpsで実用的な速度で動作する。
- 数px程度のX/Y方向の微振動を視覚的に低減できる。
- 手やペンの通常動作によって映像全体が大きく誤補正されない。
- 特徴点追跡失敗時にクラッシュしない。
- 補正不能時には無補正へ安全にフォールバックする。
- 固定cropによって補正時の映像端が表示されない。

---

# 26. Codexへの最初の指示

まずPhase 1のみ実装すること。

OBSプラグインはまだ実装しない。

C++17以上、CMake、OpenCVを使用し、

```text
input.mp4
   ↓
OpenCV VideoCapture
   ↓
Stabilizer
   ↓
dx / dy estimation
   ↓
translation correction
   ↓
fixed crop
   ↓
OpenCV VideoWriter
   ↓
output.mp4
```

を実装する。

`Stabilizer` はOBSに依存しないクラスとして設計すること。

最初の実装では、

- grayscale
- 縮小画像
- `goodFeaturesToTrack`
- `calcOpticalFlowPyrLK`
- RANSACによる外れ値除去
- X/Y translation推定
- `warpAffine`
- 固定crop

を使用する。

処理時間を計測し、各フレームについて少なくとも、

```text
dx
dy
valid
tracked feature count
inlier count
processing time
```

を確認できるようにする。

自動crop、回転補正、Perspective補正、AI/機械学習、ROI指定など、仕様にない機能を先回りして追加しないこと。

まず最小実装を完成させ、実際のテスト動画で補正結果を評価できる状態にすること。
