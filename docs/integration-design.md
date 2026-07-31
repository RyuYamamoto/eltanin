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
| 3 | 停止を検出して**再計画**し、迂回してゴールまで到達する | 停止したら終了 |
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
                     ローカル窓の原点スナップ -> local.update()
  2. tracker.compute(plant.pose(), path, dt)
  3. Status で分岐 (NoPath / GoalReached / Tracking)
  4. limiter.limit(local.costmap(), model, plant.pose(), tracking.command)
  5. Sample と leg 統計、停止カウンタを更新
  6. 停止が連続したら再計画 (成功なら次の周期へ、失敗なら終了)
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

## 9. 停止検出・再計画・ストール

**停止トリガ**: 制限後の指令が並進・回転ともに厳密に 0 の周期が 5 回連続 (0.25 s)。

`detail::limit_command` を読むと、前進中に `collision_distance <= collision_margin` になれば
`v_out = 0` かつ `ratio = 0` から `w_out = 0` になり、指令は厳密に `(0, 0)` になる。指令 0 では plant が
動かず、リミッタは無状態なので次周期も同じ判定になる。**トリガは振動しない。** その場旋回中
(`|v| <= 1e-9`) は衝突予測が立たなければ `w_out` が保たれるので、再計画直後の向き合わせをトリガと
誤認しない。1 周期でも原理的には足りるが、一過性のゼロで再計画しないよう 5 周期にしている。

**再計画**: 蓄積点をグローバルに反映 → `update()` → 現在姿勢を start として `plan` + `smooth` →
`PurePursuit::reset()` → 新しい leg として追従を継続。上限 3 回。

**ストール検出**: **1 回以上再計画したうえで**、前回再計画以降の走行弧長が 0.20 m 未満なのに再び停止
トリガが立ったら `Outcome::Stalled` で終了する。ストールは「再計画しても解決しなかった」ことなので、
最初の停止には必ず 1 回の再計画を与える。その場旋回は弧長 0 だがトリガも立たないので誤検出しない。
「停止 → 迂回して 20 cm 以上進んで → また停止」は正常な再計画として扱い、上限で打ち切る。
これが `MAX_STEPS` まで空回りしないことの保証である。

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
  なる。膨張は必須である。
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

## 12. 実測値 (参照地図 = navyu の `map/`、4000 x 4000 / 0.05 m)

start (36.125, -45.325) → goal (34.675, -9.025) を自動選択。

| | クリーン | 未知障害物あり |
|---|---|---|
| leg 0 の経路 | 727 姿勢 / 37.321 m | 727 姿勢 / 37.321 m |
| leg 1 (迂回路) | — | 381 姿勢 / 19.736 m |
| 総周期数 | 1568 | 1626 |
| 制限が効いた周期 | **0** | 15 |
| 衝突を予測した周期 | 18 | 35 |
| 最小 `collision_distance` | 0.7 m | 0.2 m |
| 蓄積した観測点 | **0** | 25 |
| 再計画 | 0 | 1 |
| 全域 `update()` | 1 | 3 |
| 最終位置誤差 | 0.0237 m | 0.0168 m |
| 衝突した通過姿勢 | 0 | 0 |
| 経路からの最大距離 | 0.0524 m | 0.0418 m |

クリーン地図で制限が 0 回なのは**正しい**。計画経路は膨張帯を避けるため衝突予測は常に 0.7 m 以上先で、
制動上限 `sqrt(2 x 0.5 x 0.5) = 0.707 m/s` が要求速度 0.5 m/s を上回り拘束しない。だから未知障害物の
シナリオがないと `VelocityLimiter` は回帰テストの対象にならない。クリーン地図で蓄積が 0 点なのも
正しい: 観測できる障害物はすべて静的地図が説明する。

### 12.1 実行時間とセンサ周期の決め方

統合テスト 8 件の合計 (直列 `ctest`):

| 構成 | 合計 | 最長の 1 件 |
|---|---|---|
| 既定 (`-O0` + `assert`) | 30.0 s | 8.4 s |
| ASan + UBSan | 95.0 s | 26.4 s |
| `Release` (`-O2`) | 2.0 s | 0.6 s |

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
| 1 | **ローカルプランナが無い** (`planner` は未着手)。局所回避は `PurePursuit` + `VelocityLimiter` だけで、迂回はグローバル再計画でしか実現できない |
| 2 | **ゴール最終接近の減速・停止制御が無い。** `GoalReached` の時点でも要求指令は 0.5 m/s のままで、デモはそこで打ち切る。goal yaw も収束しない |
| 3 | 観測面だけが `ObstacleLayer` に入るため、A\* が障害物の未観測な内部を通る経路を出しうる。近づけば再観測して再び停止 → 再計画になり、上限で打ち切られる。通過姿勢が無衝突であることは検証で保証する |
| 4 | 自己位置は plant の真値。推定誤差・オドメトリ誤差・センサノイズを入れていない |
| 5 | `ObstacleLayer` に clearing が無いので観測点は消えない。動く障害物は扱えない |
| 6 | リカバリ行動は「停止 → 再計画」1 種のみ。後退・その場旋回・待機のポリシーは無い |
| 7 | 参照地図が無い環境では実地図を使う 5 件がスキップされる。窓のスナップを固定する 2 件と合成地図で失敗系を固定する 1 件は常に走る |
| 8 | 可視化は無い。標準出力・CSV・PGM までで、図の生成は外部ツールに委ねる |

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

失敗時は `Outcome` に対応する 1 行を標準エラーに出して `EXIT_FAILURE` を返す。文言には必ず「どの段か」
が入る (例: `replan_failed: replan 1 failed: plan() found no path from (36.1, -27.9) to (34.7, -9.0)`)。
出力は失敗時にも書くので、失敗した走行もそのまま図にできる。
