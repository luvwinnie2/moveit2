# moveit_cable_carrier — ザイルベア（3D ケーブルキャリア）を MoveIt 2 の衝突判定に入れる

FANUC CRX-5iA のようなアームに付く**変形するケーブルキャリア**を、MoveIt 2 が
経路計画中に衝突物として扱えるようにするパッケージ。

## 用語

| 語 | 意味 |
|---|---|
| ザイルベア / ケーブルベア | ケーブル・ホースを束ねて保護する可撓性のチェーン。英語では cable carrier / energy chain / drag chain |
| ドレスパック (dresspack) | ロボットアームに付けるケーブル配線一式。ロボット業界での呼び方 |
| 2D チェーン | 1 軸のみに曲がる平面チェーン（国盛化学 Silveyer KSL/KSH など）。**直線軸用** |
| 3D キャリア | 全方向に曲がるボール&ソケット式（igus triflex R など）。**6軸アームの軸3〜6 用** |
| R_min | 最小曲げ半径。これ以上きつく曲げられないというハード制約 |
| バックストップ | 逆側に曲がらないようにする爪。2D チェーンが U 字を保てる理由 |
| FK | 順運動学。関節角 → 各リンクの位置姿勢 |
| DLS | 減衰最小二乗法。冗長な系を安定に解く手法 |

## 何が問題だったのか

MoveIt には「**関節角 q によって形が変わる衝突物**」という概念がない。

- `AttachedCollisionObject` は 1 つのリンクに**固定**された剛体しか表せない
- ザイルベアは 2 つのリンクにまたがるので、どちらか一方に固定できない
- 形状は q の関数なので、事前に 1 つの形状を置くこともできない

そのため従来は「大きめの箱で囲う」といった過剰に保守的な近似しかできず、
本来通れる経路まで塞いでしまう。

## 解き方（2 層構成）

```
   ┌─ オフライン（遅くてよい / 高忠実度）─────────────────────────┐
   │  NVIDIA Newton  Cosserat ロッド + VBD ソルバ                 │
   │    · GPU で数千サンプルを一括計算                             │
   │    · Dahl 塑性モデルで曲げヒステリシスも再現                   │
   │    · Warp の自動微分で実測形状にパラメータをフィッティング      │
   │  MuJoCo  elasticity.cable コンポジット（CPU のみ・相互検証用） │
   └──────────────────┬──────────────────────────────────────┘
                      │  最大偏差を safety_margin に積む
                      ▼
   ┌─ オンライン（MoveIt の内側ループ / 高速）────────────────────┐
   │  本パッケージの角度空間ロッドソルバ  実測 p50 44 µs           │
   │    → カプセル鎖 → AttachedBody → FCL                        │
   └──────────────────────────────────────────────────────────┘
```

**物理エンジンを計画ループに入れてはいけない。** OMPL は 1 回の計画で 10⁴〜10⁵ 回
衝突判定を呼ぶ。物理ステップは 3〜6 桁遅い。
そこでオフラインで高忠実度モデルを回し、その誤差を安全余裕に変換して、
オンラインでは安価な代理モデルを使う。

## MoveIt へのつなぎ方 — なぜ衝突検出器プラグインなのか

最初に検討した `PlanningScene::setStateFeasibilityPredicate` は**使えない**。
Humble の実装を確認した結果:

- `isStateFeasible` は自身の述語だけを見て、**親シーンを参照しない**
- diff コンストラクタは述語を**コピーしない**
- `move_group` はリクエスト毎に diff シーンを作る

つまり監視シーンに述語を仕掛けても**エラーも警告もなく無視される**。

一方 `allocateCollisionDetector(alloc, parent)` は親の env をコピーして検出器の型を
保つので、diff を跨いで生き残る。よって本パッケージは
`CollisionEnvFCL` を継承した `CollisionEnvCarrier` を提供する。

処理は単純で、**判定対象の状態をコピーし、その姿勢で解いた形状を
`AttachedBody` として貼ってから FCL に委譲する**だけ。これで ACM・リンクパディング・
距離クエリ・接触名が全部そのまま使える。
`AttachedBody` の `touch_links` に取り付けリンクを入れてあるので、ACM の編集も不要。

```
collision_detector: CABLE_CARRIER      # move_group の設定でこれを選ぶ
```

## 形状ソルバ — 角度空間である理由

最初は位置ベースの投影法（PBD）で書いたが、**構造的に破綻した**。
辺長拘束と曲率拘束が互いを打ち消し合い、**どちらも満たさない点で停留**する。
反復を 200 → 1500 に増やしても結果はほぼ同一で、収束が遅いのではなく停留だった。

そこで未知数を**節点位置ではなく関節回転**に変えた。これにより:

| 拘束 | 位置ベース | 角度空間（現行） |
|---|---|---|
| 伸びない（弧長一定） | 反復で近似、ドリフトする | **構成上つねに厳密** |
| 最小曲げ半径 R_min | 反復で近似、違反が残る | **クランプで厳密** |
| 端点（可動ブラケット位置姿勢） | ハード固定 | 反復で詰める（残差は安全余裕に算入） |

端点だけが残差を持つが、それは `safety_margin` で吸収する設計。
解法は DLS で、`J Jᵀ` は鎖の長さによらず 6×6 なので安価。

## 実測値（CRX-5iA、J4/J5/J6 を 9³ = 729 通り走査）

```
carrier 'crx5ia_wrist_triflex3d': L=0.520 m  R_min=0.040 m  N=24  margin=0.024 m
mount: J3_link -> J6_link          spanned joints (3): J4 J5 J6

bracket chord   : min 0.319 m  p50 0.399  max 0.478   (carrier length 0.520 m)
carrier feasible: 721 (98.9%)
in collision    :   8 ( 1.1%)

shape solve  mean  115.2 us   p50  57.2   p95  469.6   max  743.0
full check   mean  525.8 us   p50 411.0   p95 1116.8   max 1900.3

achieved bend radius: min 0.0400 m  (hardware limit 0.0400 m)
```

達成曲げ半径がハード上限とちょうど一致 = 拘束が有効に効いていることの裏付け。

**取り付けを 2D チェーンとしてモデル化していたときは実行可能率 0.3% だった。**
KSL/KSH のような平面チェーンは片側にしか曲がらずねじれも許さないため、
J4/J5/J6 が回る手首では大半の姿勢が成立しない。
調査のとおり 6軸アームの軸3〜6 に使うのは **3D キャリア（triflex R 系）** であり、
設定を `bend_mode: spatial` に直したことで 0.3% → 98.6% になった。
**これはモデル選択が結果を決めた例で、数値のチューニングでは直らなかった。**

## 対応している配線の種類

`config/carrier_types.yaml` に型式ライブラリを置いている。各項目には出典の確度
（`measured` / `rule` / `template`）を明記してあり、`template` のまま本番に使わないこと。

| kind | 何を表すか | 効く制約 |
|---|---|---|
| `articulated_carrier` | 3D ドレスパック（igus triflex R 系）。6軸アームの軸3〜6 の標準 | 規定曲げ半径 + リンクあたり ±10° のねじれストッパ |
| `planar_chain` | 平面ケーブルチェーン（Silveyer KSL/KSH 等）。**直線軸用** | 1 軸曲げ + バックストップ（片側のみ） |
| `bare_cable` | 裸ケーブル・ホース | 外径の 10 倍則（高屈曲品で 7.5 倍） |
| `corrugated_hose` | コルゲートチューブ | 曲げ半径が規定されず、ねじれ自由 |

### 効く制約はキャリアではなくケーブル側のことが多い

`inner_cables` に中を通す配線を書くと、**キャリアの曲げ半径とケーブルの曲げ半径の
両方**を評価する。実測例（CRX-5iA、剪定 1 サイクル）:

```
キャリア (R_min 40mm) の利用率 : 0.32〜0.74   ← 余裕がある
φ10 ロボットケーブル (10x OD)  : 5.4x〜9.3x   ← 24 点中 15 点で違反
```

キャリアは 62 mm の半径で楽をしているが、φ10 ケーブルは 100 mm を要求する。
**設計変更が必要なのはケーブル側**、という判断がこれで出せる。

## 参照ソルバとの突き合わせ（実施済み）

| 参照 | 状態 |
|---|---|
| **MuJoCo** `elasticity.cable` | ✅ 動作。可動端は equality `connect` 拘束で保持 |
| **NVIDIA Newton** rod + VBD | ⚠️ 実装済みだが**両端固定ができない**（下記） |

MuJoCo との比較（62 姿勢、重力なし）:

```
平均偏差   0.44 mm
最大偏差  21.7 mm   (p99 も同値 = 外れ値 1 件)
```

重力ありだと平均 63.6 mm / 最大 193 mm まで開くが、これはモデル誤差ではなく
**重力のモデル化方法の差**。本ソルバの `gravity_sag` はヒューリスティックなので、
弾性形状そのものの精度を見るには重力を切って比べるのが正しい。
`safety_margin: 0.024` はこの実測（21.7 mm + 公差 2 mm）に基づく。

### Newton が両端を固定できない件

ドレスパックは 2 つのブラケットで両端を固定されるが、Newton の剛体経路は
アーティキュレーション（＝木構造）ベースで、2 つ目の固定はループ閉合になる。
Newton 1.0.0 + SolverVBD で 3 通り検証し、すべて同じ壁に当たった:

- `add_rod` が作ったボディへの `add_joint_fixed(-1, body)` は**黙って無視される**
  （`parent_xform` の有無にかかわらず、固定なしと**完全に同一の結果**になる）
- `wrap_in_articulation=False` は `add_articulation(joints)` を要求し、結局木に戻る
- 末端を `add_body(is_kinematic=True)` にすると明示的に拒否される
  （*"Only root bodies (whose joint parent is the world) can be kinematic."*）

そのため生成スクリプトは、緩和後に**ロッドが元のブラケット位置から動いていないかを
検査**し、動いていたらそのケースを棄却する。重力で落ちただけの形状を参照値として
出さないためで、実際にこのゲートが 64/64 件を捕捉した。
Newton にループ閉合が入るまでは MuJoCo を参照に使うこと。

なお Newton のソルバは **Python 3.11 以上が必要**。遅延インポートなので
`import newton` も `import newton.solvers` も 3.10 で成功してしまい、
ソルバクラスに触れた瞬間に初めて失敗する。Isaac Sim の 3.12 で動く。

## 既知の限界（重要）

1. **ソルバは障害物を見ない。** 出るのは最小エネルギー形状で、アームにめり込むことがある。
   実機ではリンクに載るだけなので、`touch_links` に J3〜J6 を入れて誤検出を防いでいる。
   逆に言うと、リンクに載った状態での正確な形状は再現していない。
2. **ヒステリシスは未実装。** 実物は前の形状を引きずる（Newton の Dahl モデルが該当）。
   現状は履歴によらず一意の形状を返すので、その差は `safety_margin` に含める必要がある。
3. **端点残差**が存在する（上表のとおり）。`safety_margin` で吸収する前提。
4. **重力モデルが粗い。** `gravity_sag` は反復ごとに節点を少し引っ張るだけの
   ヒューリスティックで、静力学的な釣り合いを解いていない。重力を入れた比較で
   平均 63.6 mm も開くのはこれが原因。垂れが効く長い配線では実測合わせが必要。
5. **応力は「ケーブルの曲げひずみ」として報告する。** 外殻の梁曲げ応力（σ=E·y·κ）は
   関節で曲がるボール&ソケット構造には無意味で、実際ナイロンの降伏の 10 倍という
   数字が出ていた。現在は業界標準の「曲げ半径 ÷ ケーブル外径」比で評価している。

## 使い方

```bash
# ビルド（最適化必須。付けないと 660 倍遅い＝ 75 ms/解 になる）
colcon build --packages-select moveit_cable_carrier --cmake-args -DCMAKE_BUILD_TYPE=Release

# 変形を目視する（スライダで J4/J5/J6 を動かす）
ros2 launch moveit_cable_carrier carrier_demo.launch.py
#   緑 = その姿勢で形状が成立   赤 = 成立しない（届かない／曲げ半径を割る）

# 性能と実行可能率を測る
ros2 run moveit_cable_carrier carrier_benchmark <urdf> <srdf> <carrier.yaml> 9

# ブラケット座標系入りの URDF を生成（YAML が唯一の情報源）
ros2 run moveit_cable_carrier make_carrier_urdf.py \
    crx5ia.urdf crx5ia_carrier.yaml crx5ia_with_carrier.urdf

# Isaac 用 USD の別バージョン（原本はサブレイヤ参照のみ、書き換えない）
/isaac-sim/python.sh patch_crx5ia_add_carrier.py --visual
```

### よくある失敗

| 症状 | 原因 | 対処 |
|---|---|---|
| ほぼ全姿勢が赤（実行不可） | 2D チェーンとしてモデル化している | `bend_mode: spatial`, `unilateral: false` |
| 同上 | `length` がブラケット間距離より短い | ベンチマークの `bracket chord max` より長くする |
| 1 解 75 ms と異常に遅い | `CMAKE_BUILD_TYPE` 未指定 | `-DCMAKE_BUILD_TYPE=Release` |
| 述語が呼ばれない | `setStateFeasibilityPredicate` を使った | 衝突検出器プラグインを使う（上記） |
| RViz でロボットが動かない | `JointState` に `header.stamp` が無い | 必ず現在時刻を入れる |
| Isaac と GUI が関節値を奪い合う | `/joint_states` の発行元が 2 つ | どちらか一方だけ動かす |

## 次にやること

1. Newton を導入し `generate_reference_shapes.py --backend newton --dahl` で参照形状を生成、
   `compare_reference.py` で偏差を測って `safety_margin` を**実測値に置き換える**
2. Newton の Warp 自動微分で曲げ剛性を実測形状にフィッティング
3. Newton のカップリングフレームワーク（剛体 MuJoCo-Warp ↔ VBD）で
   アーム＋キャリアを一体で回し、ブドウ棚との接触まで含めて検証
4. リンク接触を考慮した形状（現状の限界 1 番）への拡張

## 参考

- [igus triflex R ロボットドレスパック（軸3〜6 用）](https://www.igus.com/triflex/robot-energy-supply)
- [igus triflex R FANUC CRX 用](https://www.igus.com/robot-dress-pack/triflexr-dresspacks-for-fanuc-crx)
- [国盛化学 Silveyer（リンクレスケーブルチェーン）](https://www.stertec.co.jp/~kunimori/silveyer/silveyer.html)
- [Newton: Open-Source GPU Physics for Robot Simulation（SIGGRAPH 2026）](https://www.youtube.com/watch?v=ElIyRboR1A8)
- [newton.solvers.SolverVBD API](https://newton-physics.github.io/newton/latest/api/_generated/newton.solvers.SolverVBD.html)
