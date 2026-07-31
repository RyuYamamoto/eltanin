# 統合デモの設計 (`examples/navigate_on_real_map` / `test/integration`)

ROS を一切使わずに `map_io` から `sim` までを 1 プロセスの閉ループとして回す統合デモと、その回帰テスト
の設計記録。**ライブラリ (`include/` / `src/`) には 1 バイトも足していない。** 既存 API のまま全系が
回ることを示すのが目的だからである。

- 実装: `examples/navigation_loop.hpp` (閉ループ本体)、`examples/navigate_on_real_map.cpp` (CLI)、
  `test/integration/test_navigate_on_real_map.cpp` (回帰テスト)
- 前提: `docs/costmap-design.md` (レイヤ機構)、`docs/sensor-design.md` (`project_scan` の契約)、
  `docs/planner-design.md`、`docs/control-design.md`、`docs/collision-design.md`

---

## 1. 何が新しいのか

`examples/limit_on_real_map` も「読み込み → 膨張 → A\* → スムーザ → PurePursuit → VelocityLimiter →
plant」を回している。統合デモが加えるのは次の 5 点である。

| # | 追加点 | `limit_on_real_map` の状態 |
|---|---|---|
| 1 | `LayeredCostmap` (static + obstacle + inflation) をグローバル / ローカルの 2 段で使う | `InflationLayer` を直呼び。レイヤ機構を通っていない |
| 2 | 合成 LiDAR → `project_scan` → `ObstacleLayer` で**未知障害物を観測して信念に反映する** | 障害物を真の地図に焼き、ロボットは最初から知っている |
| 3 | 観測が経路を塞いだら**再計画**し、迂回してゴールまで到達する | 停止したら終了 |
| 4 | `ctest` から回る回帰テスト | example のみ |
| 5 | 到達判定つきの標準出力と通過セルの PGM | CSV + 1 枚の PGM のみ |

2 が `eltanin_sensor` と `ObstacleLayer` の初めての利用者である。

## 2. 閉ループをヘッダオンリーの共有ヘッダに置いた理由

`navigate()` は `examples/navigation_loop.hpp` の `inline` 関数で、CLI (`examples/`) と gtest
(`test/integration/`) が同じ実体を呼ぶ。テストから example のバイナリを `add_test` で起動する案は
採らなかった。

- 数値の検証 (最終誤差・停止余裕・再計画回数) が終了コード 1 個に潰れる
- どの段で失敗したかが gtest の出力に出ない
- `ELTANIN_BUILD_EXAMPLES=OFF` でテストが回らなくなる

`test/` が `examples/` のヘッダを include する向きになるが、これは既存の `*_fixture.hpp` と同じ
「テストが使えるヘッダオンリー資産」の延長である。**逆向き (examples が test を include する) にはしない。**

## 3. 1 周期の順序

```
MAX_STEPS = max_sim_time / control_dt

for step in 0 .. MAX_STEPS-1:
  1. センサ周期なら: 合成 LiDAR -> project_scan -> 占有セル上への絞り込み -> 蓄積 ->
                     前方経路が塞がれたかの判定 -> ローカル窓の原点スナップ -> local.update()
  2. tracker.compute(plant.pose(), path, dt)
  3. Status で分岐 (NoPath / GoalReached / Tracking)
  4. limiter.limit(local.costmap(), model, plant.pose(), tracking.command)
  5. Sample と leg 統計、停止カウンタを更新
  6. 経路が塞がれた、または停止が連続したら再計画 (失敗なら終了)
  7. plant.update(limited.command, dt)
  8. check_footprint(ground_truth, ..., plant.pose()) で通過姿勢を検証
```

- **原点更新と `update()` は必ず同じ周期で対にする。** `set_origin()` はセルを動かさないので、
  `update()` を伴わない原点移動は「古いセル値が違う座標に置かれた地図」を作る。
  センサ周期でない周期はローカル窓を動かさない。5 Hz / dt 0.05 では窓中心からのずれは最大 0.1 m で、
  半幅 3.0 m に対して無視できる。
- **`step == 0` はセンサ周期**なので、最初の `limit()` は必ず有効なローカル地図を見る。
- **検証は plant を進めた後に置く。** 検証したいのは「制限された指令が実際に到達させた姿勢」である。

## 4. ローカル窓を 6.0 m 角にした根拠

`VelocityLimiter` が `check_footprint` を呼ぶ最遠点までの距離:

```
予測到達距離 = prediction_time x |v| = 2.0 s x 0.5 m/s = 1.000 m
+ フットプリント外接半径 (0.44 x 0.30 m の矩形)        = 0.266 m
--------------------------------------------------------------
判定が触るロボット中心からの最大距離                   = 1.266 m
```

一方、窓の外周 `inflation_radius = 0.55 m` は「窓外の障害物が窓内へ膨張してこない」ため信用できない
(`InflationLayer` はマスタ内の `LETHAL` からしか膨張しない)。

| 窓の一辺 | 半幅 | 信頼できる半幅 | 1.266 m に対する余裕 |
|---|---|---|---|
| 4.0 m | 2.00 m | 1.45 m | 0.18 m |
| 6.0 m | 3.00 m | 2.45 m | **1.18 m** |

6.0 m (120 x 120 セル) を採る。実行時間が問題になってもここは縮めない (§10)。

ローカルの `default_cost` は `FREE_SPACE` ではなく `NO_INFORMATION` にする。`docs/costmap-design.md`
§14.2 が「ローカルには `FREE_SPACE`」と書いているのは `StaticLayer` を持たない前提の記述であり、
本デモのローカル窓は `StaticLayer` を持つ。窓は静的地図の内側にクランプするので全セルが書かれるが、
万一書かれないセルが出たときに `NO_INFORMATION` = 障害物として保守側に倒れるほうが正しい。

## 5. ローカル窓の原点を格子にスナップする理由

`LayeredCostmap::center_on()` は使わない。`origin = robot - extent/2` は静的地図の格子に乗らないため、
`StaticLayer` の再標本化位置が毎周期揺れ、障害物と膨張の境界が 1 セル揺らぐ。

```
res    = static.resolution()
o0     = static.origin()
half   = window_cells * res / 2
k      = clamp(floor((robot - half - o0) / res), 0, static_size - window_cells)   -- 成分ごと
origin = o0 + k * res
```

これでローカルのセル中心は `o0 + (k + mx + 0.5) * res` となり、静的地図側の
`floor((w - o0)/res) = k + mx` が厳密に成り立つ (0.5 は整数境界から最も遠いので丸め差でずれない)。
`StaticLayer` の再標本化が 1:1 になることを `NavigationLoopWindow.SnapsTheLocalWindowOntoTheStaticGrid` が
固定している (実地図を必要としないので常に走る)。クランプが効いた周期数は `window_clamped_cycles`
として出力する。

**`std::floor((w - origin) / resolution)` をデモ側に書いている点について。**
`docs/costmap-design.md` §13-5 は「`MapGeometry` の外でこの式を書かない」としているが、これは
world → cell 変換の重複を禁じる規約である。ここで求めているのは変換ではなく「窓原点を格子に丸める」
計算である。将来 `MapGeometry::snapped_origin_for(...)` を足すかは別タスクとする。

## 6. 合成 LiDAR

| 項目 | 値 | 根拠 |
|---|---|---|
| センサ原点 | ロボット base 原点 | オフセットを入れるなら `Transform2D::from_pose(pose) * base_to_sensor` の 1 合成で済む |
| ビーム数 | 180 (全周) | `range_max` 3.0 m でのビーム間隔 0.105 m。停止判断に効く距離 (0.7 m 付近) では 0.024 m = 1 セル未満 |
| `angle_min` | `-pi` | 全周は `ScanFilter::angle_range` では表現できない (`docs/sensor-design.md` §5) ので `nullopt` にする |
| `range_min` / `range_max` | 0.1 m / 3.0 m | 後者はローカル窓の半幅と一致。予測到達 1.266 m に対して十分 |
| ノイズ | 無し | 決定性 (A-12) のため |
| 当たらなかったビーム | `+inf` | `project_scan` が `isfinite` で落とす |

レイキャストは固定ステップのレイマーチ (`0.5 x resolution = 0.025 m`) で、DDA は採らない。実装が短く
境界条件が明白なほうを採る。固定ステップの欠点は「レイが角だけをかすめるセルを取り落とす」ことだが、
帰結は「薄壁の背後にある真の障害物セルを拾う」= コストマップとしては保守側であり、§7 の後段フィルタに
より**偽の障害物を書く方向には決して転ばない**。

レイキャストは `eltanin_sensor` に足さず `examples/` に閉じた。利用者がデモ 1 つしかなく、
`docs/sensor-design.md` §11 も「レイトレースは現契約の外」と明記しているためである。

## 7. 投影点を占有セル上に絞る (本設計で一番効いている判断)

`project_scan` が返した world 系の点のうち、**ground truth のセルが `LETHAL_OBSTACLE` のものだけ**を
`ObstacleLayer` に渡し、蓄積する。

理由。`ObstacleLayer::set_points()` は与えられた点のセルを無条件に `LETHAL_OBSTACLE` にする。
レイマーチが求めた距離は `float` に落ちて `ScanData::ranges` に入り、`project_scan` は
「センサ系で `(r cos a, r sin a)` を作ってから `Transform2D` で回す」というデモ側とは**別の演算順**で
点を復元する。差は 1e-8 m 程度でも、点がセル境界の近くにあるとセルが 1 つずれうる。ずれた先が
自由セルだと、

- ローカル地図では壁が 1 セル内側に太る (膨張半径 0.55 m に対し 0.05 m なので実害は小さい)
- **グローバルの蓄積では、自由空間に置かれた偽の `LETHAL` が恒久的に残り、狭い通路を塞いで再計画を
  失敗させうる** (`docs/planner-design.md` §6.1 の横方向マージンは実マップで 1.62 cm しかない)

後者が危険なので構造で消す。フィルタは「実際に書き込む点そのもの」を検査するので浮動小数点の一致に
依存しない。センサモデルとしても「レンジ計測点は占有セルの上に乗る」という妥当な制約である。

ローカル `ObstacleLayer` にはフィルタ後の**全点**を渡す。静的壁の点は `StaticLayer` が既に `LETHAL` に
しているセルなので上書きは無操作であり、除く必要がない。

## 8. 観測点の蓄積とグローバル更新

- 蓄積するのは「ground truth で `LETHAL` かつ**静的地図で `LETHAL` でない**」点だけ。これが
  「未知障害物を発見した」の定義になる。全観測点を積む案は 400 スキャン x 数百点まで単調増加し、
  膨張が `max` である以上結果は同じなので採らない。
- セル線形インデックスを `std::unordered_set` で重複排除し、初出のときだけ発見時刻つきで
  `std::vector` に積む。**`unordered_set` は所属判定にしか使わない**ので、出力順は発見順の `vector` が
  決め、決定性が保たれる。点はセル中心に正規化するので `obstacles.csv` はコストマップと 1:1 に対応する。
- 蓄積点は再計画時にグローバル `ObstacleLayer` へ**全量** `set_points()` する (置き換え API なので)。
- グローバル `update()` は全域 1600 万セルを触るので**再計画時のみ**。ループ終了後、最後の `update()`
  以降に蓄積点が増えていたら 1 回だけ追加で `update()` してから `costmap.pgm` を書く。
- `ObstacleLayer` に clearing が無いので観測点は消えない。**動く障害物は扱えない。**

## 9. 再計画・停止検出・ストール

再計画のトリガは 2 つあり、主が観測、従が停止である。

### 9.0 観測トリガ (主)

**新規観測セルが「現在の経路の前方 4 m 以内」を塞いだら、その周期で再計画する。** 判定は

```
半径 = circumscribed_radius (0.266 m)
経路上のロボット最近傍点から前方 4 m の各姿勢について、蓄積観測点との距離が半径以下なら塞がれている
```

`circumscribed_radius` を使うのは、**A\* が「セルが `Free` か」で問うているのと同じ条件**にするためである
(膨張モデルでは `cost < circumscribed_cost` ⟺ 最近致死セルまでの距離 > `circumscribed_radius`)。
新しい経路は蓄積点を避けて計画されるので、直後に同じ点で再びトリガすることはない。

これが実際のパターンである。LiDAR の `range_max` は 3.0 m なので**遠方の未知障害物は観測できず**、
初期経路はその上を通る (実測: leg 0 の 19 姿勢が最終コストマップで非 `Free`)。2.9 m まで近づいて
初めて観測でき、その時点で信念に入って再計画が走る。

実測: t = 34.0 s に 2.925 m 先を初観測 → 同じ周期で再計画 → **一度も止まらずに迂回**。

- **判定にコストマップ更新は不要。** 蓄積点と前方経路点の距離計算だけで、新規点 25 × 経路点 80 程度。
  全域 `update()` (0.285 s / `-O2`) を毎周期回すことはできないので、この安さが要点である。
- **`PurePursuit::reset()` は呼ばない。** 走行中の再計画では既に新経路と向きが揃っており、`reset()` は
  速度ランプを 0 に落として不要な減速を作る。`reset()` は停止トリガのときだけ呼ぶ。

### 9.1 停止トリガ (従)

**制限後の指令が並進・回転ともに厳密に 0 の周期が 5 回連続 (0.25 s)。** 観測トリガが間に合わない場合
(障害物が予測地平の内側に突然現れた、観測しても迂回路が経路の 4 m 先より遠い、など) の受け皿であり、
ストール検出の入口でもある。`replan_on_blocked_path = false` にすると**これだけ**になり、
「観測 → 減速 → 停止 → 再計画 → 迂回」の挙動になる (`--replan-on-stop-only`)。

| | 観測トリガ (既定) | 停止トリガのみ |
|---|---|---|
| leg 0 の周期数 | 681 (t = 34.05 s で分岐) | 785 (t = 39.25 s で分岐) |
| leg 0 の制限が効いた周期 / 衝突予測 | **0 / 0** | 15 / 35 |
| leg 1 | 427 姿勢 / 22.08 m | 381 姿勢 / 19.74 m |
| 総周期数 / シミュレーション時間 | 1557 / 77.85 s | 1626 / 81.30 s |
| 停止 | **しない** | 障害物の 0.491 m 手前 |
| 最終位置誤差 | 0.0114 m | 0.0168 m |
| 障害物までの最小フットプリント余裕 | 0.104 m | 0.111 m |

**最小余裕はどちらもほぼ同じ**であることに注意。観測トリガは停止を無くすが横方向の余裕は改善しない —
それは §15.2 に書いた別の話 (A\* の `Free` 判定が余裕の下限を保証しない) である。

`VelocityLimiter` は既定シナリオでは一度も制限しなくなるので、回帰テストは 2 本に分けてある
(`AvoidsAnObservedObstacleWithoutStopping` と `StopsForAnUnknownObstacleThenReplansToTheGoal`)。

### 9.2 停止トリガが振動しない理由

`detail::limit_command` を読むと、前進中に `collision_distance <= collision_margin` になれば
`v_out = 0` かつ `ratio = 0` から `w_out = 0` になり、指令は厳密に `(0, 0)` になる。指令 0 では plant が
動かず、リミッタは無状態なので次周期も同じ判定になる。**トリガは振動しない。** その場旋回中
(`|v| <= 1e-9`) は衝突予測が立たなければ `w_out` が保たれるので、再計画直後の向き合わせをトリガと
誤認しない。1 周期でも原理的には足りるが、一過性のゼロで再計画しないよう 5 周期にしている。

### 9.3 再計画の手順とストール検出

**再計画**: 蓄積点をグローバル `ObstacleLayer` に全量反映 → `update()` → 現在姿勢を start として
`plan` + `smooth` → 新しい leg として追従を継続。上限 3 回。停止トリガのときだけ
`PurePursuit::reset()` を呼ぶ (§9.0)。

**ストール検出**: **1 回以上再計画したうえで、停止トリガが立ち**、前回再計画以降の走行弧長が 0.20 m
未満なら `NavigateOutcome::Stalled` で終了する。ストールは「再計画しても解決しなかった」ことなので、
最初の停止には必ず 1 回の再計画を与える。**観測トリガはストール判定に使わない** — 走行中に起きる
再計画であり「進めない」ことの証拠にならないためである。その場旋回は弧長 0 だが停止トリガも立たない
ので誤検出しない。「停止 → 迂回して 20 cm 以上進んで → また停止」は正常な再計画として扱い、上限で
打ち切る。これが `MAX_STEPS` まで空回りしないことの保証である。

実測 (`--obstacle-half-width 20 --obstacle-fraction 0.02`、障害物がロボットを飲み込む配置):
5 周期で停止 → 再計画 → **再計画は成功する** (信念には観測した 16 セルしか無く、現在位置は自由セルの
ままだから) → 5 周期でまた停止 → `Stalled`。**合計 10 周期**で、上限 20,000 周期には遠く届かない。
「信念では通れるのに真の世界では通れない」という状況をストール検出が正しく打ち切っている。

**到達判定**: `PurePursuit::Status::GoalReached` **かつ**最終位置誤差 ≤ 0.10 m。`nearest_index()` は
経路全体の最近傍なので、迂回路が自分の終端付近を通ると誤って `GoalReached` になりうる。位置誤差を
必須にすることで「黙って成功する」ことは防げる。**goal yaw は判定しない** (`PurePursuit` は yaw を
収束させない)。

## 10. ground truth を 1 枚で済ませた理由

`ground_truth = 静的地図のコピー` に未知障害物を `LETHAL` で焼き、**そのうえで `InflationLayer` を
1 回かける**。この 1 枚がレイキャストの当たり判定と通過姿勢の検証を兼ねる。

- 当たり判定は `== LETHAL_OBSTACLE`。膨張が書く最大値は `INSCRIBED_INFLATED_OBSTACLE` (253) で 254 を
  新たに作らないので、この述語は膨張後も真の障害物セルだけを指す。
- 検証は `check_footprint(ground_truth, ...)`。**生の地図で検証すると値が 0 / 254 / 255 の 3 値しかなく、
  自由セルは `Free` で短絡して多角形判定が一度も走らない** — 「中心セルが致死か」しか見ない弱い検証に
  なる。`check_footprint()` を使う限り膨張は必須である。
  `check_footprint_exact()` (`docs/collision-design.md` §2.5) なら生の 3 値地図でも多角形判定が走るが、
  検証側を切り替えると検証の意味が変わる (より厳しくなる) ので、独立した判断として扱う。
- 未知障害物は**膨張前に**焼く。ロボットの信念には焼かないので、プランナは最初この障害物を知らない。

## 11. 停止余裕の測り方

停止した周期の姿勢について、世界系フットプリント多角形から ground truth の `LETHAL` セル中心までの
**厳密距離**で定義する (`eltanin::contains` で内包なら 0、そうでなければ各辺への
`eltanin::distance_to_segment` の最小値)。外接半径で代用すると余裕を 0.05 m 近く過小評価する。

2 つ出す。

| 値 | 範囲 | 意味 |
|---|---|---|
| `stop_clearance` | 停止姿勢の周囲 `circumscribed_radius + 1.0 m` の全 `LETHAL` セル | 何にも当たっていないことの保守的な確認 |
| `stop_obstacle_clearance` | 注入した障害物のセル矩形のみ | 「そのために止まった障害物」までの余裕 |

実測 (dt 0.05): `stop_clearance = 0.25 m`、`stop_obstacle_clearance = 0.2958 m`。どちらも
`collision_margin = 0.2 m` 以上である。前者が小さいのは、停止地点の側方に静的壁があるためで、
異常ではない。

**これは「停止した周期」の値であり、走行全体の最小値ではない。** 走行全体で障害物に最も近づくのは
迂回中で、フットプリント最短距離は **0.111 m** (leg 1、t = 43.1 s、姿勢 (36.554, -27.500, yaw 1.075))
である。§15.2 に理由を書いた。既定の観測トリガでは停止しないのでこの 2 値は `+inf` になり、`meta.txt`
にも出ない (そのときの最小余裕は 0.104 m で、停止する場合とほぼ同じ)。

## 12. 実測値 (参照地図 = navyu の `map/`、4000 x 4000 / 0.05 m)

start (36.125, -45.325) → goal (34.675, -9.025) を自動選択。

| | クリーン | 未知障害物あり (既定) | 未知障害物あり (停止トリガのみ) |
|---|---|---|---|
| leg 0 の経路 | 727 姿勢 / 37.321 m | 727 姿勢 / 37.321 m | 727 姿勢 / 37.321 m |
| leg 1 (迂回路) | — | 427 姿勢 / 22.084 m | 381 姿勢 / 19.736 m |
| 総周期数 | 1568 | 1557 | 1626 |
| 制限が効いた周期 | **0** | **0** | 15 |
| 衝突を予測した周期 | 18 | 28 | 35 |
| 最小 `collision_distance` | 0.7 m | 0.5 m | 0.2 m |
| 蓄積した観測点 | **0** | 25 | 25 |
| 再計画 | 0 | 1 (観測) | 1 (停止) |
| 全域 `update()` | 1 | 3 | 3 |
| 最終位置誤差 | 0.0237 m | 0.0114 m | 0.0168 m |
| 衝突した通過姿勢 | 0 | 0 | 0 |
| 経路からの最大距離 | 0.0524 m | 0.0482 m | 0.0418 m |

クリーン地図で制限が 0 回なのは**正しい**。計画経路は膨張帯を避けるため衝突予測は常に 0.7 m 以上先で、
制動上限 `sqrt(2 x 0.5 x 0.5) = 0.707 m/s` が要求速度 0.5 m/s を上回り拘束しない。だから未知障害物の
シナリオがないと `VelocityLimiter` は回帰テストの対象にならない。クリーン地図で蓄積が 0 点なのも
正しい: 観測できる障害物はすべて静的地図が説明する。

### 12.1 実行時間とセンサ周期の決め方

統合テスト 9 件の合計 (直列 `ctest -L integration`):

| 構成 | 合計 | 最長の 1 件 |
|---|---|---|
| 既定 (`-O0` + `assert`) | 28.2 s | 7.0 s |
| ASan + UBSan | 88.4 s | 22.2 s |
| `Release` (`-O2`) | 2.3 s | 0.6 s |

ピーク RSS は ASan 有効時 485 MB (`plan()` の一時配列 160 MB が支配項)。

**センサ周期 5 Hz / ビーム 180 は実行時間から決めた。** 10 Hz / 360 ビームでは障害物ケースが
ASan で 56.9 s、7 件合計が 200 s を超えて受け入れ条件 (120 s) を割る。5 Hz / 180 ビームに落としても
**軌跡・観測点数・停止余裕・最終誤差はすべて 10 Hz / 360 ビームと完全に一致した**ので、忠実度の損失は
このシナリオでは観測されていない。さらに削る必要が出たときは `lidar_range_max` を 2.0 m にする →
レイマーチを DDA にする、の順に検討する。**ローカル窓は縮めない** (§4 の余裕が削られる)。

決定性テストだけは 20 s ぶんのシミュレーションに切り詰めている (同じ走行を 2 回するため)。
全長の走行が決定的であることは、他のテストが同じ数値を再現していることで間接的に担保される。

## 13. 既知の欠落と限界

| # | 内容 |
|---|---|
| 0 | **`Layer` に境界付き更新が無い** ため `LayeredCostmap::update()` は全域再生成しかできず (1600 万セルで 0.285 s / `-O2`)、グローバルコストマップを制御周期で更新できない。だから観測トリガは「経路が塞がれたか」の安い判定で代用し、全域 `update()` は再計画のときだけ呼んでいる。nav2 相当の `updateBounds()` / `updateCosts(master, min_i, min_j, max_i, max_j)` を `eltanin_map` に入れるのが本筋で、それは §16 の申し送り |
| 1 | **ローカルプランナが無い** (`planner` は未着手)。局所回避は `PurePursuit` + `VelocityLimiter` だけで、迂回はグローバル再計画でしか実現できない |
| 2 | **ゴール最終接近の減速・停止制御が無い。** `GoalReached` の時点でも要求指令は 0.5 m/s のままで、デモはそこで打ち切る。goal yaw も収束しない |
| 3 | 観測面だけが `ObstacleLayer` に入るため、A\* が障害物の未観測な内部を通る経路を出しうる。近づけば再観測して再び停止 → 再計画になり、上限で打ち切られる。通過姿勢が無衝突であることは検証で保証する |
| 4 | 自己位置は plant の真値。推定誤差・オドメトリ誤差・センサノイズを入れていない |
| 5 | `ObstacleLayer` に clearing が無いので観測点は消えない。動く障害物は扱えない |
| 6 | リカバリ行動は「停止 → 再計画」1 種のみ。後退・その場旋回・待機のポリシーは無い |
| 7 | 参照地図が無い環境では実地図を使う 5 件がスキップされる。窓のスナップを固定する 2 件と合成地図で失敗系を固定する 1 件は常に走る |
| 8 | C++ 側に可視化は無い。標準出力・CSV・PGM までで、図の生成は `examples/plot_navigation_results.py` (ビルドに含まれない開発ツール) に委ねる |

## 14. 出力

| ファイル | 内容 |
|---|---|
| `costmap.pgm` | 走行範囲を囲む切り出しのグローバルコストマップ (生のコスト値) |
| `traversed.pgm` | 同じ `geometry` で通過セルのみ `LETHAL_OBSTACLE` のマスク |
| `path.csv` | `leg,index,x,y,yaw` — 再計画分も `leg` で区別して同じファイルに入る |
| `trajectory.csv` | `t,leg,x,y,yaw,v_in,w_in,v_out,w_out,collision_distance,has_collision,predicted_poses` |
| `obstacles.csv` | `t,x,y` — 未知障害物として蓄積したセル中心、発見順 |
| `meta.txt` | 切り出し `geometry` / 各半径 / 全パラメータ / leg 別統計 / `outcome` / 停止余裕 |

2 枚の PGM を同じ `geometry` にしているので、外部で重ね描きできる。`write_pgm` は生のコスト値を書く
(0 / 252 / 253 / 254 / 255 のどれにも意味がある) ため、**通過セルをコストマップに重ねてはいけない。**

失敗時は `NavigateOutcome` に対応する 1 行を標準エラーに出して `EXIT_FAILURE` を返す。文言には必ず
「どの段か」が入る (例:
`replan_failed: replan 1 failed: plan() found no path from (36.1, -27.9) to (34.7, -9.0)`)。
出力は失敗時にも書くので、失敗した走行もそのまま図にできる。

## 15. 可視化による確認

`examples/plot_navigation_results.py` が出力ディレクトリから 4 枚の図を作る。`plot_collision_results.py`
と同じ扱いで、**CMake からは参照されない開発ツール**である (Python がビルド依存にならない)。

```bash
./build/examples/eltanin_navigate_on_real_map /tmp/nav
python3 examples/plot_navigation_results.py --run /tmp/nav --out /tmp/nav-plots
```

| 図 | 内容 |
|---|---|
| `overview.png` | コストマップ全体に leg ごとの計画経路・走行軌跡・観測セル・start / goal を重ねる |
| `stop.png` | 障害物周辺の拡大。停止した周期・観測セル・真の障害物矩形・迂回路 |
| `commands.png` | 要求指令と制限後指令の時系列 + `collision_distance`。再計画時刻を破線で示す |
| `traversed.png` | `traversed.pgm` を `costmap.pgm` に重ねる (両者が同一 `geometry` でなければ重ならない) |

図は `--replan-on-stop-only` の走行に対するものである (既定の観測トリガでは停止しないので `stop.png` の
タイトルがその旨に変わる)。

### 15.1 図で確認できたこと

- **軌跡が幾何的に妥当。** 廊下の中央を進み、障害物の手前で止まり、右側へ迂回してゴールに達する。
- **`traversed.pgm` がコストマップとセル単位で一致し、軌跡が途切れない。** dt 0.05 の 1 周期の移動は
  0.025 m で resolution 0.05 m より小さいので隣接セルが連続して埋まる、という設計どおりである。
- **リミッタは障害物の前以外では恒等写像。** `commands.png` で要求指令 (太い灰) が制限後 (細い黒) に
  完全に隠れ、t = 38〜39 s の停止だけで分離する。クリーン地図で「制限が効いた周期 0」の裏付けになる。
- **制動が階段状。** `collision_distance` が 0.9 → 0.8 → ... → 0.2 m と 0.1 m 刻みで落ちる
  (量子化 = `|v| x prediction_dt` = 0.5 x 0.2)。`v_out` は 0.5 → 0.447 → 0.316 → 0 で、
  `docs/collision-design.md` §3.1 の予測と一致する。
- **ゴール手前の最後の約 9 s で角速度が ±0.4 rad/s で振動する。** §13-2 (最終接近の制御が無い) が
  実際にどう現れるかがこの図でしか見えない。追従誤差は 0.042 m に収まっており経路は外れていない。

### 15.2 図から誤読しかけた点 (数値で確認して訂正)

`stop.png` では迂回路が障害物の膨張帯に入っているように見える。**実際は入っていない。**
最終コストマップ上で leg 1 の 381 姿勢はすべて `Free` (最大コスト 61 < `circumscribed_cost` 78) で、
これは膨張の勾配が急 (`cost_scaling_factor` 10.0) なため、目には濃く見えても閾値未満だからである。

そのうえで、走行全体の最小フットプリント余裕は停止時の 0.2958 m ではなく**迂回中の 0.111 m** である。
これは欠陥ではなく設計の帰結で、理由は 2 つある。

1. A\* の `Free` 判定が保証するのは「フットプリントが `LETHAL` セル中心を含まない」= 衝突しないこと
   だけで、余裕の下限ではない。外接半径 0.266 m ぶん離れた経路上でも、姿勢によっては余裕が 0 に近づく。
2. `VelocityLimiter` は要求指令の予測円弧上しか見ない。障害物が**側方**にあるときは予測が当たらない
   ので介入しない (leg 1 の `limited_cycles` は 0)。

通過姿勢の検証は全 1626 周期で無衝突であり、うち 7 周期はコスト 78 以上 (`Circumscribed` 帯) の
セル上を通っている — つまり**二段構えの厳密な多角形判定が実際に働いて `Free` を返した**周期である。
横方向の余裕を確保したい場合は距離場を使ったコストか局所回避が必要で、どちらも本タスクの外である。

---

## 16. 後続タスクへの申し送り: 境界付きコストマップ更新

本デモで一番強い制約は **`Layer` が更新範囲を受け取れないこと**である。

```cpp
class Layer { virtual void update_costs(Costmap & master) = 0; };
```

範囲が渡らないので `LayeredCostmap::update()` はマスタ全域を `default_cost` で埋め直して全レイヤを
再適用するしかなく、1600 万セルで 1 回 0.285 s (`-O2`) / 約 2.7 s (ASan) かかる。結果として

- グローバルコストマップは**再計画のときしか更新できない** (§9.0 の観測トリガは、コストマップを更新
  せずに「経路が塞がれたか」だけを幾何で判定して代用している)
- センサ観測がグローバルの信念に入るのは、常に再計画の瞬間だけになる

nav2 は同じ問題を 2 段階で解いている。

1. 各層が `updateBounds(robot_x, robot_y, robot_yaw, &min_i, &min_j, &max_i, &max_j)` で
   **自分が汚した矩形を申告**する。
2. `updateCosts(master, min_i, min_j, max_i, max_j)` が**その矩形だけ**を書く。

外向きの `map_msgs/OccupancyGridUpdate` (`<costmap>/costmap_updates`) は、この dirty rect をそのまま
直列化したものである。**差分はトピックの都合ではなく内部の境界追跡の帰結**であり、順序としては
ROS ブリッジより前に `eltanin_map` 側に入るべきものだと考える (ブリッジ側で全グリッドを比較して差分を
作るのは、ライブラリに無い機能の代替にすぎない)。

eltanin に入れる場合の最小の形:

```cpp
class Layer
{
public:
  /// Cells this layer will touch, in master cell coordinates; empty when there is nothing to do.
  virtual std::optional<map::CellRect> update_bounds(const Pose2D & robot) = 0;
  virtual void update_costs(Costmap & master, const map::CellRect & bounds) = 0;
};
```

- `StaticLayer` は初回のみ全域、以降は空を返す。
- `ObstacleLayer` は新規点の外接矩形を返す。
- `InflationLayer` はその矩形を `inflation_radius` ぶん広げた矩形を返す (**膨張は必ず最後**なので、
  前段が広げた矩形を受けて自分の矩形を作れる)。
- `LayeredCostmap::update()` は各層の矩形の和を取り、その範囲だけを `default_cost` で埋めて回す。
- 既存の `MapGeometry::world_rect_to_cells()` がそのまま「ローカル窓 ∩ グローバル」の計算に使える。

これが入ると、本デモの観測トリガは「幾何で経路を検査する」代用をやめて、**観測のたびにグローバル
コストマップを更新して再計画する**素直な形に書き換えられる。T7 は「コアを 1 バイトも変えない」制約
(A-11) なので、ここでは実装していない。
