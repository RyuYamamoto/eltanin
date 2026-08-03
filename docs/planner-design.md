# eltanin グローバルプランナ設計方針 (8 近傍 A*、Hybrid A*、経路スムーザ)

対象: `include/eltanin/planner/`、`src/planner/` (`eltanin_planner` / `eltanin::planner`)。
移植元は `navyu_path_planner` (`astar_planner.hpp` / `node.hpp` / `base_global_planner.hpp` / `smoother.hpp`)。
`docs/costmap-design.md` §6 が用意した縫い目 (セル型 × 判定モデル) の最初の利用者である。

前提文書: `docs/costmap-design.md` (§3 単位 / §4 3 値 / §6 縫い目 / §6.4 座標変換 / §7.2 2 パス化 / §9 命名 / §10 エラー通知 / §12 モジュール構成)。

---

## 1. 対象と範囲

やること:

- 8 近傍 A* グローバルプランナ。セル型と判定モデルの 2 つでテンプレート化する。
- Hybrid A* グローバルプランナ。向き付き状態と定曲率 motion primitive を使う。
- `Planner` 基底を介して A* / Hybrid A* を実行時に切り替えられるようにする。
- フラット配列による探索状態。`new` / `delete` / スマートポインタを 1 つも持たない。
- 最近傍の通行可セル探索 (`find_nearest_traversable`) を独立した関数として切り出す。
- 衝突チェック付き反復平滑化スムーザ。端点を保持する。

やらないこと (それぞれ理由と移送先):

| 項目 | 理由 |
|---|---|
| 2 パスフォールバック探索による狭路緩和 | §7.2 の将来拡充。`Traversability::Free` のみ通行可の 1 パス探索である |
| 距離場 (EDT) の生成 | §7.1。本モジュールは `DistanceMap` を手で作って実体化を検証するだけ |
| ローカルプランナ / 軌道生成 | 利用者 (制御) が居ない |
| コスト値を経路コストに加算する重み付け探索 | 入れると期待長がパラメータ依存になり、最適性テストの基準が失われる。1 パス探索の正しさを固定するのが先 |
| 経路のリサンプリング / 間引き / カスプ分割 | 利用者が居ない。`Path` の解説どおり自由関数の担当 |
| A* / smoother の経路点間の線分衝突判定 | 点単位の判定に限る (§8)。Hybrid A* は primitive を区間サンプリングする (§12) |
| A* の探索打ち切り | `closed` がセル数で有界なので停止性は保証される。Hybrid A* は `max_expansions` を持つ |
| 失敗理由の返却 | §10 の分類 B は `std::optional`。理由の列挙は利用者が現れてから |

---

## 2. 縫い目の切り方 — 探索本体は非テンプレートである

タスク要求は「将来 `DistanceMap` + `CollisionRadii` に差し替えても本体を変更しないこと」であった。これを**型の上で構造的に保証する**ため、判定モデルの適用を探索の前に全セル 1 回の分類として前置し、A* 本体を非テンプレート関数として `.cpp` に置いた。

```
Planner::plan<Map, Model>()               ヘッダテンプレート
  ├─ 前提条件 assert / world_to_map
  ├─ goal セルの分類                       ← ここまでで nullopt になる経路は
  ├─ find_nearest_traversable()               3 値グリッドを確保しない
  ├─ build_traversability_grid()          ← セル型と判定モデルが見える最後の場所
  └─ plan_on_grid()                       非テンプレート仮想関数
       ├─ AStarPlanner                    8 近傍セル探索
       └─ HybridAStarPlanner              向き付き motion primitive 探索
```

`Planner::plan_on_grid()` と各派生実装は `MapGeometry` と `std::vector<std::uint8_t>` しか受け取らない。**セル型も判定モデルも見えない。** 探索本体・経路復元は非テンプレートであり、実体化の追加でコードが複製されることもない。

この方式の帰結として、「同一の通行可否分類になるコストマップと距離場が同一の `Path` を返す」ことは**構造的に自明**になる。両者は同一の 3 値グリッドを生成し、以降の計算は同じ関数 1 本を通る。テスト `PlannerSeam.BothModelsProduceTheSamePath` はその構造が壊れていないことの回帰である。

**コストと、それを受け入れる理由**

| 項目 | コスト |
|---|---|
| 追加メモリ | セル数 × 1 byte (4000×4000 で 16 MB) |
| 追加時間 | 全セル 1 回の `classify()` + 1 byte 書き込み |

この構造は既に呼び出しごとに `g_score` 64 MB + `parent` 64 MB + `closed` 16 MB を確保し `fill` する (§9)。3 値グリッドはそこへの +11 % のメモリと +1 回の線形走査であり、計算量の性質を変えない。実際に触るのが一部のセルだけでも全セルを分類する無駄は確かにあるが、それは既存のフラット配列 3 本と同じ無駄であり、見直すなら §9 の「探索作業領域を保持するプランナ型への移行」と同時に見直すべき事柄である。

**採らなかった対案**: 探索本体もヘッダテンプレートにして遅延分類する。全セル分類の無駄を避けられる代わりに、(a) 中核をテンプレート抜きでテストできない、(b) 実体化ごとに探索本体が複製される、(c) 「本体を変更しない」がコンパイラの挙動任せになる、を失う。上記の同一 `Path` が構造的に保証されなくなるのが決め手である。

**3 値を 2 値に潰さない。** `Free` / `Circumscribed` / `Inscribed` をそのまま 1 byte に格納する。これは §7.2 の 2 パス化が「近傍展開の通行可否述語 1 本の差し替え」で済む状態を保つためであり、§4 が 3 値にした理由をそのまま引き継いでいる。列挙子の値 0 / 1 / 2 は `planner.hpp` の `static_assert` で固定した (`traversability.hpp` は変更していない)。

**`CellMap` concept を当初 `planner/` に置いた理由**: `map/` への追加は T1〜T3 の公開 API を変更しないという制約に触れる。利用者は `plan` / `smooth` / `find_nearest_traversable` の 3 つで、§9.2 (利用者の無い public を作らない) に反しない。**T6 で `eltanin_safety` が 2 人目の利用者になったため `include/eltanin/map/cell_map.hpp` (`eltanin::map::CellMap`) へ移した。** 単一リポジトリで利用者を全部直せるため後方互換の別名は置いていない。

---

## 3. 探索構造 — navyu のポインタ木を置き換えた

| 項目 | navyu | eltanin |
|---|---|---|
| ノード | `new Node2D` を近傍展開ごとに実行。リポジトリ全体に `delete` が 1 つも無く全リーク | フラット配列 `g_score` (`float`) / `parent` (`std::int32_t`) / `closed` (`std::uint8_t`) |
| close list | `std::unordered_map<int, Node2D *>` (ノードあたり 1 ハッシュ挿入) | セル数長の `std::vector<std::uint8_t>` に添字アクセス |
| 近傍テーブル | `get_motion()` が呼び出しごとに `std::vector<Node2D>` をヒープ確保 | `constexpr std::array<int, 8>` |
| `g` の改善判定 | 無い。同一セルへの経路をすべて open list に積む | `g_score` 配列で改善時のみ再投入 |
| 経路復元 | `find_path` が start をもう一度 `emplace_back` して二重に入れる | `parent` を辿る。`parent[start] == -1` なので start は 1 回だけ入る |

`new` / `delete` / `malloc` / スマートポインタの出現数は `src/planner/` と `include/eltanin/planner/` で 0 件である。ASan (LeakSanitizer 既定有効) で全テストがリーク 0 件で通る。

**`parent` の型は `std::int32_t`。** `-1` が「親なし」を表す。セル数 2^31 = 21 億までを表現でき 4000×4000 = 1.6e7 に十分だが、`plan_on_grid()` の先頭で `cells <= INT32_MAX` を `assert` している。より大きな地図が必要になったら型を広げる。

**decrease-key を実装しない。** `std::priority_queue` に decrease-key が無く、実装するには索引付きヒープを自作することになる。`g` が改善したときに `open` へ再投入し、pop 時に `closed` を検査してスキップする。navyu と同じ構造だが `g_score` の改善判定を持つ点が違い、無制限の再投入にはならない。

---

## 4. 範囲外読み出しと行の回り込みを構造的に再発不能にした

navyu の `astar_planner.hpp:68,71` は `get_grid_index(x, y)` で添字化した**後に** `costmap_[...]` を引いており、範囲検査が無かった。

`size_x = 4`, `size_y = 2` の格子で `(mx, my) = (0, 1)` から `-x` 方向へ展開すると `get_grid_index(-1, 1) = -1 + 4 * 1 = 3` となり、これは**セル `(3, 0)` の添字**である。`(0, 0)` からの `-x` 展開は添字 `-1` で `std::vector` の範囲外読み出しになる。すなわち navyu は「グリッド左端が壁として機能せず、前の行の右端に回り込む」状態だった。

**規約: 近傍展開では `MapGeometry::in_bounds()` を検査してから `index()` を呼ぶ。** `index()` は `assert(in_bounds(...))` を前提条件とする API であり、添字を作ってから範囲を判定する順序を書かない。角抜け判定に使う `free_cell()` も `in_bounds(...) && grid[index(...)] == FREE` の短絡評価で同じ順序を守る。

回帰テストは `test/planner/test_astar_bounds.cpp` の `DoesNotWrapAroundTheLeftEdge` / `DoesNotWrapAroundTheRightEdge` である。フィクスチャは上記の格子そのもの:

```
my=1:  S # # .
my=0:  . # # G
```

左右の列は連結していないので期待値は `nullopt`。**実装を navyu と同じ「int で添字化してから範囲を判定する」順序に差し替えるとこの 2 件が実際に落ちることを確認した。** 回り込みが起きない実装ではどちらも `nullopt` を返す。

`world ↔ cell` 変換は `MapGeometry::world_to_map()` / `map_to_world()` のみを使う。`eltanin_planner` の中に `/ resolution` を含む式は 1 つも存在しない (§6.4 / §13-5)。navyu の `convert_map_to_grid` (`static_cast<int>` で負値をゼロ方向に切り捨て、`origin` より小さい world 座標を範囲内と誤判定) と `convert_grid_to_map` (セル角を返し往復整合しない) は移植していない。

---

## 5. 近傍・コスト・ヒューリスティック

| 項目 | 規約 |
|---|---|
| 近傍 | 8 近傍。移動コストは直交 `resolution`、斜め `√2 * resolution` [m] |
| 角抜け | **禁止する。** 斜め移動 `(dx, dy)` は `(dx, 0)` と `(0, dy)` の両方が in_bounds かつ `Free` のときのみ許す |
| 通行可否 | `Traversability::Free` のみ通行可。`Circumscribed` / `Inscribed` はともに通行不可 |
| ヒューリスティック | オクタイル距離 `(dx + dy + (√2 - 2) * min(dx, dy)) * resolution`。`double` で計算し `float` にキャストする |
| 重み | 1.0 固定 (最適性のため) |
| 単位 | `g` / `h` / `f` はすべて [m]。セル単位の 1 / √2 を API に出さない |
| タイブレーク | `std::pair<float, std::size_t>` + `std::greater<>` で f 昇順 → 添字昇順。決定的 |
| 停止性 | `closed[i] = 1` は各セルにつき高々 1 回。`open` は有限回の push しか受けない |

**角抜けを禁止した理由**: navyu は直交隣接 2 セルが両方障害物でも斜めに通り抜けられる。膨張があっても市松模様の障害物配置では実際に貫通する。**navyu と経路が一致しなくなることは意図した差である** (§8.1.1 のとおり比較すべきは通行可否分類であり、経路の数値一致ではない)。

角抜け禁止の下でもオクタイル距離は許容的かつ整合的である。移動の禁止は辺の削除に等しく、コストを増やす方向にしか働かないため下界性が保たれる。

**「最適」の意味**: 8 近傍グリッド移動に限定した中での最短である。真のユークリッド最短経路 (任意方向) とは一致しない。加えて最適経路は一般に複数存在するため、テストは `path_length()` を比較し、経路点列そのものを固定するのは経路が一意なフィクスチャに限る。

**ヒューリスティックにユークリッド距離を採らない理由**: 8 近傍の移動コストに対してオクタイル距離のほうが下界が厳しく、展開ノードが減る。どちらでも最適経路長は同じである。

---

## 6. 経路の座標と yaw

**生の A* 経路はセル中心のみで構成する。** 要求 start / goal の実座標に差し替えたり前後に挿入したりしない。位置は `map_to_world()` (セル中心) のみで計算する。navyu の `origin + grid * resolution` (セル角) は §6.4 のセル中心規約に反するので採らない。

**要求 start / goal と最大 `0.707 * resolution` ずれることは仕様である。** この吸収は追従側 (`eltanin_control`) の責務である。期待経路長が解析的に決まり最適性テストが厳密になる利益のほうが大きい。

### 6.1 経路の横方向マージンはほぼゼロである (T5 への申し送り)

`Free` のみを通行可とするため経路は必ず `Circumscribed` 帯の外を通るが、**外に出るだけで余裕は持たない。** `examples/eltanin_plan_on_real_map` で実マップ (膨張半径 0.55 m、内接 0.15 m、外接 0.266 m、`cost_scaling_factor = 10`) の 727 点 / 37.4 m の経路を実測した。

| 指標 | 実測値 |
|---|---|
| `Circumscribed` 以上のセルに乗った点 | **0 / 727** (仕様どおり) |
| 経路上のコスト | 最小 0 / 最大 **66** (`circumscribed_cost = 78`) |
| コスト 66 に対応する障害物距離 | 0.2825 m |
| **外接円半径からの余裕** | **0.0162 m = 1.62 cm** |
| 非 `Free` セルまでのチェビシェフ距離 | 最小 **1 セル** / 中央値 4 セル |
| 余裕が 1 セルしかない点 | **146 / 727 (20 %)** |

つまり**横方向に 2 cm ずれると `Circumscribed` 帯に入る**。この帯は「向き次第で衝突する」領域なので、追従誤差の許容量を経路だけから読むことはできない。A* が最短経路を選ぶ以上、障害物の角では通れる限界まで内側を通るのが正しい挙動であり、プランナ側の欠陥ではない。

**追従側 (T5) が横方向誤差の予算をどこから取るかを決める必要がある。** 選択肢は次のいずれかで、いずれもプランナの外側にある。

- 膨張半径を追従誤差ぶん大きく取る (呼び出し側が `CollisionRadii` を膨らませる)
- コストを経路コストに加算する重み付け探索を入れて障害物から離れる経路を選ばせる (§1 で第一段階の対象外とした項目。最適性テストの基準が失われる点に注意)
- 追従誤差を `eltanin_safety` の速度制限で吸収する

なお `Circumscribed` に入った点を検出したいだけなら判定モデルを呼べば足りる (プランナに診断出力を足す必要はない)。

**yaw の規約** (`detail::assign_tangent_yaw()` に集約):

- `i = 0 .. n-2` の yaw を `std::atan2(p[i+1].y - p[i].y, p[i+1].x - p[i].x)` に設定する。
- `p[n-1].yaw` には触らない。呼び出し側が `goal.yaw` を入れておく (`plan()`) か、入力から保存されている (`smooth()`)。
- 差ベクトルが厳密に 0 の点は直前の確定 yaw を複製する。先頭側に連続する未確定区間は、最初に確定した yaw を後から複製する。1 つも確定しなければ (全点が同一位置) `p[n-1].yaw` を全点に複製する。
- `n <= 1` は何もしない。
- `normalize_angle()` は掛けない。`std::atan2` の値域が既に `(-pi, pi]` で `core/angle.hpp` の規約と一致する。

この関数 1 本を `plan()` と `smooth()` の両方から呼ぶことで、yaw 規約が探索側と平滑化側で食い違わないことが構造的に保証される。定義は `path_smoother.cpp` に置いた。

**goal 姿勢を経路の末尾に載せる理由**: navyu の追従器 (`navyu_path_tracker.cpp:113-121`) は経路点の**位置のみ**を読み `atan2(dy, dx)` で自前に方位を作る。経路点の `orientation` は一度も参照されず、**要求した goal の向きは経路に載らないまま捨てられていた**。末尾に載せておけば、追従側がそれを使うか無視するかを選べる。`Pose2D` が yaw を持つ型である以上、全て 0 のまま返すのは「前方が +x」という誤った意味を持たせることになる。

---

## 7. start の救済は goal に対して行わない (意図的な非対称)

- start セルが通行不可なら `find_nearest_traversable()` で最近傍の `Free` セルへ寄せる。見つからなければ `nullopt`。
- **goal セルが通行不可なら救済せず `nullopt`。**

ロボットが膨張帯に入っているのは正常な状況 (壁際からの発進、動的障害物の接近) であり、そこで計画不能になるのは実用に反する。一方 goal は利用者が選ぶ値であり、通行不可なら「そこへは行けない」と返すのが正しい情報である。切り離した `find_nearest_traversable()` を goal に適用するかは後続タスクの判断に委ねる。

寄せ替えの是非と半径を呼び出し側が制御できるよう、`PlannerParams` が `start_search_radius_cells` (既定 8 セル) を持ち、`AStarParams` / `HybridAStarParams` はこれを `common` メンバとして合成する (§13.1)。`0` は「救済しない」を意味する。既定 8 セルは `resolution 0.05` で 0.4 m であり、膨張半径 0.55 m (§14.7) と同程度をカバーする。

**`find_nearest_traversable()` の契約**

```cpp
template <CellMap Map, class Model>
  requires TraversabilityModel<Model, typename Map::value_type>
std::optional<map::MapIndex> find_nearest_traversable(
  const Map & map, const Model & model, const map::MapIndex & from, int max_radius_cells);
```

- 探索領域は `from` を中心とするチェビシェフ半径 `max_radius_cells` の正方領域。その中で**ユークリッド距離最小の `Free` セル**を返す。
- `from` 自身が範囲内かつ `Free` ならそれを返す (`r = 0` のリングで自然にそうなる)。これにより「start が通行可か」の判定と救済を 1 本の呼び出しに畳める (§14.4 の「判定を 2 本の式に分けない」と同じ方針)。
- `from` はマップ外でもよい。範囲外のリングセルは単に飛ばす。
- 同一距離の候補は `(dy, dx)` 昇順で最初に見つかったものを採る。

**単純な BFS を採らない理由**: BFS はチェビシェフ / マンハッタン距離最小になり「最近傍」の意味が変わる。リング `r` を走査しつつ、候補が見つかった後も `r * r <= best_distance_squared` の間はリング走査を続ける。

**この打ち切り条件が実際に必要な例**: リング 3 の対角 `(3, 3)` (`d² = 18`) だけが `Free` で、リング 4 の `(0, 4)` (`d² = 16`) も `Free` の場合、リング 4 まで見ないと最近傍を落とす。リング `r` の最大 `d²` は `2r²`、次リングの最小 `d²` は `(r+1)²` なので `r >= 3` で逆転が起こりうる。テスト `FindsEuclideanNearestNotChebyshev` がこの配置そのものである。

距離比較は `std::int64_t` の二乗距離で行う。浮動小数の比較を挟まないので `<` の厳密性がタイブレーク規約と正確に一致する。

**整数オーバーフロー対策**: `from` は `world_to_map()` 由来で `to_int_saturating()` により `INT_MIN` / `INT_MAX` になりうる。リングの加算を `std::int64_t` で行い、`in_bounds()` に渡す前に `int` 範囲へ飽和させる。`plan()` 経由では `from` は範囲内だが、公開 API が範囲外 `from` を受ける契約を採る以上、防御が要る。UBSan 有効テスト (`HandlesSaturatedFrom`) で担保している。

**連結性は保証しない。** 見つかったセルが元の `from` から到達可能である保証はない (壁の内側にロボットが居る場合など)。寄せ替えた先が別の閉領域になりうる。その場合 A* は素直に `nullopt` を返す。

---

## 8. スムーザ — 反復平滑化に置換した

### 8.1 navyu の Bézier スムーザを移植しなかった理由

| navyu の箇所 | 内容 |
|---|---|
| `smoother.hpp:33-45` | 二項係数を `factorial(n) / (factorial(i) * factorial(n-i))` で計算する |
| `smoother.hpp:49-51` | `t_step = 1.0 / path_size` (`path_size - 1` ではない) の浮動小数点加算ループ |
| `smoother.hpp:88-90` | セグメント確定後に `poses.clear()`。次セグメントと点を共有しない |
| `smoother.hpp:94` | 空 `Path` で `poses[poses.size() - 1]` = 添字 `-1` (未定義動作) |
| `smoother.hpp:106-108` | `split_path` が `i = 0` で `poses[i-1]` を参照 (範囲外)。加えて signed/unsigned 比較 |
| 全体 | 衝突判定を一切行わない |

**数値的破綻の閾値**: `factorial` のオーバーフローにより制御点 172 個で `inf`、173 個以上で `NaN` になる (`g++ -O0` で実測)。

タスク記述の「桁落ちで形状が壊れる」については 1 点補正しておく。**Bernstein 基底は全項が非負で総和 1 になるため、`n = 99` 程度では桁落ち (減算による相対誤差の増幅) は起きない。** 実際の数値的破綻は上記の `factorial` オーバーフローであり、閾値は離散的である。navyu の既定設定 (`displacement_threshold = 5.0` / `resolution = 0.05` → 1 セグメント 71〜100 点) では `NaN` に到達しないが、閾値を 10 m に上げるか解像度を 0.025 にすれば到達する。受け入れ条件が「200 点以上」を指定しているのは正しく、`n = 199` は `NaN` 領域である。

**形状が壊れる主因は数値ではなく定式化である。** 制御点数 - 1 次の大域 Bézier は端点以外を補間せず制御多角形から大きく離れるため、経路が自由空間から外れる。加えて `t_step` のループが `t = 1.0` に到達せず**各セグメントの終点 (= 経路全体の goal) が出力に現れない**、セグメント境界で点を共有せず C0 不連続、が重なる。

反復平滑化を採ったのは (a) 端点固定が自明、(b) 各点の移動を採用前に衝突チェックできる、(c) 等間隔な直線が不動点になる、(d) 点数に対して数値的に破綻しない、の 4 点で受け入れ条件に直接対応するからである。

### 8.2 アルゴリズムと契約

元経路を `orig`、更新対象を `p` (初期値 `orig`) として、反復ごとに内点 `i = 1 .. n-2` について

```
candidate = p[i] + weight_data * (orig[i] - p[i]) + weight_smooth * (p[i-1] + p[i+1] - 2 * p[i])
```

| 項目 | 規約 |
|---|---|
| 更新順 | in-place (Gauss-Seidel)。`p[i-1]` は同一反復で更新済みの値を使う。Jacobi 法より収束が速く、どちらも決定的である |
| 端点 | `i = 1 .. n-2` しか触らないので `p[0]` / `p[n-1]` の位置はビット同一で保存される |
| 通行可否 | `world_to_map()` が `nullopt` (マップ外) なら通行不可扱い。`Circumscribed` も通行不可 (探索と同じ規則。判定を 2 本の式に分けない) |
| 凍結 | 通行不可な候補は**その反復で採用しない**。永続化せず次の反復で再度試す |
| 停止 | 1 反復の総移動量が `tolerance` を下回るか `max_iterations` に達したら終了 |
| 点数 | 不変。追加も削除もしない |
| yaw | 常に `assign_tangent_yaw()` を通す。末尾の yaw は保存される |
| 決定性 | 反復順・更新順・比較がすべて固定。同一入力に対しビット同一 |

**凍結を永続化しない理由**: 永続凍結にすると、近傍点が後の反復で動いた結果その点が動けるようになる状況を取りこぼす。毎反復で再判定するほうが単純で、決定性も保たれる。

**入力経路が通行不可な点を含む場合の正確な契約**: 判定するのは**候補位置**であり、現在位置ではない。障害物の内部に十分深く入った点は候補も障害物内に留まるため位置が保たれるが、障害物の縁にある点は候補が自由空間に落ちれば動く (そのほうが望ましい挙動である)。スムーザは「経路を悪化させない」ことだけを保証し、通行不可な入力を積極的に修復はしない。

**衝突チェックは経路点 1 点ごとの判定である。** 連続する 2 点の間の線分が自由空間を通る保証はしない。生の A* 経路の点間隔は最大 `√2 * resolution` であり、膨張済みコストマップ上では実用上問題にならないという判断である。**平滑化で点間隔が広がることはない (点数不変・端点固定) ため、この前提は平滑化後も保たれる。** 線分レベルの判定は `eltanin_safety` の担当。

### 8.3 パラメータの整合条件 — 収束条件と凸結合領域は別物である

既定値は `weight_data = 0.5` / `weight_smooth = 0.3` / `tolerance = 1e-4` / `max_iterations = 100`。

更新式を Jacobi 形で見ると、波数 `θ` のモードの増幅率は

```
A(θ) = 1 - weight_data - 2 * weight_smooth * (1 - cos θ),   1 - cos θ ∈ (0, 2)
```

なので条件は 2 段になる。

```
weight_data + 4 * weight_smooth < 2      収束条件 (|A| < 1)
weight_data + 2 * weight_smooth <= 1     各更新が凸結合になる、より強い条件 (非振動)
```

**`assert` で守るのは収束条件のほうである** (`assert_smoother_params()`)。既定値は収束条件を満たし (`0.5 + 1.2 = 1.7 < 2`)、凸結合条件を満たさない (`0.5 + 0.6 = 1.1 > 1.0`)。すなわち既定値では高周波モードが `A = -0.7` で符号反転しながら減衰する。`0.5 / 0.3` は反復平滑化の既定値として広く使われている値であり、これを維持して条件式を収束条件で書くほうが妥当と判断した。

凸結合領域 (`wd + 2ws <= 1`) の意味は「候補が周囲 3 点の凸結合になるので経路の凸包から出ない」ことである。**この領域では出力が入力の凸包に含まれるため、矩形マップからはみ出すことが原理的に起こり得ない。** マップ外を通行不可とする判定は、凸結合領域の外 (既定値を含む) で overshoot が起きた場合の防御として機能する。

### 8.4 不動点は「等間隔かつ共線」である

二階差分 `p[i-1] + p[i+1] - 2 p[i]` が 0 になるのは **共線かつ等間隔**のときだけである。共線でも間隔が不揃いなら点は直線に沿って滑る (幾何形状としては直線のままだが、位置は動く)。

生の A* 経路の直線区間はセル中心の等間隔列なので、実用上の主張「A* が出した直線は平滑化で曲がらない」はそのまま成り立つ。テストは等間隔共線で全点不動 (許容誤差 1e-12)、不等間隔共線で共線性のみを検証している。

### 8.5 `smoothness_cost` は反復回数に対して単調ではない

平滑度の指標を `smoothness_cost(path) = Σ_{i=1}^{n-2} || p[i-1] - 2 p[i] + p[i+1] ||` と定義した (公開 API にはしない。`test/planner/planner_fixture.hpp` のヘルパ)。

この量は**入力より必ず減少する**が、**反復回数に対して単調ではない**。実測 (21 点のジグザグ、`wd = ws = 0.3`):

```
input  7.600000
k= 1   0.711716
k= 2   1.681183   ← 増加
k= 3   1.548013
k= 4   1.566334   ← 増加
...    減衰振動しながら 1.5642229 に収束
```

理由は構造的である。この反復は

```
E(p) = (wd/2) Σ |p_i - orig_i|² + (ws/2) Σ |p_{i+1} - p_i|²
```

に対する Gauss-Seidel 勾配降下 (ステップ 1) であり、**最小化しているのは隣接点間距離の二乗和であって、二階差分の和ではない。** `smoothness_cost` は相関する診断量にすぎない。

**したがってテストは `smoothness_cost` の単調性を要求しない。** 代わりに次の 3 つを固定した。

| テスト | 主張 |
|---|---|
| `SmoothnessCostStaysBelowTheInputAtEveryIterationCount` | `k = 1..40` のすべてで入力より小さい |
| `SmoothnessCostConvergesAsIterationsGrow` | `k >= 10` で連続差が `1e-6` 未満 (収束している) |
| `EnergyIsMonotonicallyNonIncreasing` | **`E` は単調非増加**。3 通りのパラメータで `k = 1..40` を検証 |

`E` の単調性は成り立つ。座標ごとの厳密最小化のステップは `1/(wd + 2ws)` であり、ステップ 1 が単調減少を保証する条件は `1 < 2/(wd + 2ws)` すなわち `wd + 2ws < 2` である。これは収束条件 `wd + 4ws < 2` より弱いので、`assert` を通るパラメータでは常に満たされる。衝突による候補の棄却は更新をスキップするだけなので `E` を増やさない。実測でも違反は収束後の 1 ULP (約 1e-17) のみであり、テストは `1e-15` の許容差を付けている。

---

## 9. メモリと数値の限界 (申し送り)

**1 回の `plan()` 呼び出しがセル数に比例したメモリを確保する。** 4000×4000 では `g_score` 64 MB + `parent` 64 MB + `closed` 16 MB + 3 値グリッド 16 MB = **約 160 MB** であり、`fill` のコストも同程度にかかる。これはフラット配列構造から必然的に生じる帰結であり、第一段階では受け入れる。

`plan()` / `smooth()` / `find_nearest_traversable()` は**自由関数**である。実測が必要になった時点で「探索作業領域を保持するプランナ型」への移行を検討する。**この移行は破壊的変更である** (自由関数がメンバ関数になる) 点を申し送る。グローバル再計画の頻度 (1 Hz 程度) では実測してから判断すべき事柄である。

**`open` の最悪サイズ。** push 回数は最悪でセル数 × 8。到達不能な goal を 4000×4000 で与えると全セルを展開し、`open` が数百 MB に達しうる。実マップテストでは**到達可能な goal のみを使う** (4 連結の flood fill で到達可能性を保証してから goal を選ぶ。4 連結で到達可能なら角抜け禁止の 8 連結でも到達可能である)。到達不能ケースは小さいフィクスチャに限る。探索の打ち切りは将来の拡充として残す。

**`float` の `g_score` による準最適経路の可能性。** 8 近傍経路長は `a * res + b * √2 * res` の形を取り、異なる経路長の最小差は大きな地図で `1e-6 m` 程度まで詰まりうる。一方 `float` の累積誤差は 500 m の経路で `5e-5 m` 程度である。すなわち**巨大地図では「厳密最適より 1e-6 m 長い」経路を返しうる**。テストフィクスチャ (n <= 50) では経路長差 (`>= 0.01 * res`) が誤差 (`~1e-5 * res`) を大きく上回るので最適性テストには影響しない。返す `Path` の位置は `double` の `map_to_world()` で計算されるため、`path_length()` は `float` の誤差を持たない。

**`Circumscribed` を通行不可とするため、狭路が閉塞して `nullopt` になる場合がある。** これは第一段階の仕様であり §7.2 の 2 パス化で緩和される。

**スレッド安全性は呼び出し側の責務。** `map` / `model` は呼び出し中に変更されない前提である。

---

## 10. navyu 欠陥との対応

| navyu の問題 | 箇所 | 対応 | 検証 |
|---|---|---|---|
| `new Node2D` を近傍展開ごとに行い `delete` が 1 つも無く全リーク | `astar_planner.hpp:65` 他 | フラット配列。`new` / `delete` ゼロ | grep 0 件 + ASan |
| 添字化した後に costmap を引くため `x = -1, y = 0` で添字 `-1` の範囲外読み出し | `astar_planner.hpp:68,71` | `in_bounds()` → `index()` の順序 | ASan / UBSan |
| `y > 0` では `x = -1` が前の行の右端に回り込み、グリッド端が壁として機能しない | 同上 | 同上 | **`DoesNotWrapAround{Left,Right}Edge`** |
| `convert_map_to_grid` が `static_cast<int>` で負値をゼロ方向に切り捨て | `base_global_planner.hpp:59` | `MapGeometry::world_to_map()` のみを使う | `StartOutsideTheMapIsRejected` / `NonFiniteCoordinatesAreRejected` |
| `convert_grid_to_map` がセル角を返し往復整合しない | `base_global_planner.hpp:65` | `map_to_world()` (セル中心) のみを使う | `EndpointsAreCellCentersNotCorners` |
| world ↔ grid 変換をプランナが自前実装で持つ | `base_global_planner.hpp:57-67` | `eltanin_planner` に座標変換の実装を置かない | grep 0 件 |
| `find_path` が start を 2 回入れる | `astar_planner.hpp:89-104` | `parent` を辿って 1 回だけ入れる | `AdjacentCellsGiveTwoPoses` |
| start / goal の範囲内・通行可否を一切検査しない | `navyu_global_planner.cpp:52-63` | 探索前に検査。start は寄せ替え、goal は `nullopt` | `AStarPlanner.*Rejected` / `BlockedStartIsRescued*` |
| 斜め移動で角抜けが起き障害物の角を点で貫通する | `node.hpp:37-51` | 角抜けを禁止 | `DetoursTheDiagonalGap*` / `DetoursASingleWall*` (期待長で固定) |
| `get_motion()` が展開ごとに `std::vector<Node2D>` を確保する | `node.hpp:37` | `constexpr` な近傍テーブル | grep 0 件 |
| `g` の改善判定がなく同一セルへの経路をすべて積む | `astar_planner.hpp:81-83` | `g_score` 配列で改善判定 | 挙動に現れないので直接テストしない |
| スムーザが大域 Bézier で制御多角形から離れ衝突判定もしない | `smoother.hpp:28-65` | 反復平滑化 + 採用前の衝突チェック | `NeverEntersAnObstacleOrItsCircumscribedBand` |
| 二項係数が階乗ベースで制御点 172 個以上で inf / 173 個以上で NaN | `smoother.hpp:33-45` | 階乗を使わない | **`StaysFiniteOnALongPath` (500 点)** |
| `t_step` のループが `t = 1.0` に到達せず各セグメントの終点が落ちる | `smoother.hpp:49-51` | 端点を動かさないことで保持する。サンプリングしない | `KeepsBothEndPointsBitIdentical` / `KeepsTheTerminalYaw` |
| セグメント境界で点を共有せず C0 不連続 | `smoother.hpp:88-90` | 経路を分割しない。全体を 1 本として平滑化する | `KeepsThePoseCount` |
| 空 `Path` で `poses[poses.size() - 1]` = 添字 `-1` | `smoother.hpp:94` | 空 / 1 点を早期に返す | `ShortPathsAreReturnedUnchanged` |
| `split_path` が `i = 0` で `poses[i-1]` を参照 | `smoother.hpp:106-108` | 移植しない (死んだコードでありカスプ分割の必要もない) | — |
| 経路の姿勢が一切設定・参照されず要求 goal の向きが捨てられる | `navyu_path_tracker.cpp:113-121` | 接線方向 yaw + 末尾に `goal.yaw` | `YawIsTangentExceptAtTheGoal` / `RecomputesYawFromTheSmoothedPositions` |

---

## 11. 依存

`eltanin_planner` が依存するのは **C++ 標準ライブラリ / Eigen / `eltanin_core` / `eltanin_map`** のみである。ROS 2 / Rerun / matplotlib-cpp / Python / yaml-cpp / `eltanin_map_io` / `eltanin_sensor` を持ち込まない。`eltanin::map_io` へのリンクはテスト実行ファイル (実マップテスト) のみに閉じている。

例外は投げない。前提条件違反は `assert`、正常系の「該当なし」は `std::optional` (§10 の分類 A / B)。`cmake/eltaninConfig.cmake.in` は新規外部依存が無いため変更していない。

---

## 12. 共通 Planner I/F と Hybrid A*

### 12.1 仮想境界

2 つの実装が揃った時点で `Planner` 基底を導入した。公開された `Planner::plan<Map, Model>()` は非仮想のテンプレートメソッドで、次を一元化する。

- world 座標の範囲検査
- blocked goal の拒否と blocked start の最近傍 `Free` セルへの救済
- `Map + Model` から `TraversabilityGrid` への型消去
- 有効な start/goal と分類済みグリッドの探索コアへの受け渡し

派生クラスが override するのは非テンプレートの `plan_on_grid()` だけである。これによりセルアクセスのホットループへ仮想呼び出しを入れず、次のように実行時選択できる。

```cpp
std::unique_ptr<planner::Planner> selected = std::make_unique<planner::AStarPlanner>();
auto path = selected->plan(map, model, start, goal);

selected = std::make_unique<planner::HybridAStarPlanner>();
path = selected->plan(map, model, start, goal);
```

アルゴリズム固有パラメータは各派生クラスの constructor に閉じ、基底 I/F を膨らませない。Theta* のような同じワンショット入力で動く Planner はこの境界に追加できる。一方、呼び出しを跨いだ `g/rhs/km` と map 更新 API を必要とする D* Lite は別の増分 Planner I/F が必要である。

### 12.2 Hybrid A* の状態と展開

離散状態は `(cell, heading_bin, previous_motion_mode)` である。`previous_motion_mode` を含めるのは steering change penalty を Markov な遷移コストにするためで、同じ `(cell, heading)` でも直前の操舵が異なる候補を潰さない。

各ノードは連続値の `Pose2D` を保持し、次の 3 個の前進 motion primitive を厳密な円弧積分で展開する。

- curvature `-1 / minimum_turning_radius`, `0`, `+1 / minimum_turning_radius`

各 primitive は `collision_check_step` 間隔でサンプリングし、全サンプルが `Traversability::Free` の場合だけ採用する。判定対象は車体基準点であり、**向き付き footprint polygon の直接判定ではない**。車体外形の clearance が必要な利用者は、従来の A* と同様に footprint 半径で膨張済みの map/model を渡す必要がある。非円形 footprint を姿勢ごとに厳密判定する拡張は、この基底へ生 map を追加せず、衝突判定 strategy を別途渡す設計で行う。

`g` は距離 [m] を基準に steering / steering change penalty を加算する。`h` はユークリッド距離であり、各遷移コストが primitive の chord 長以上なので許容的かつ整合的である。open list の同一候補順序と motion 順序を固定し、同じ入力から同じ Path を返す。

goal から `dubins_expansion_distance` 以内のノードでは、現在姿勢から要求 goal 姿勢までの最短 Dubins path を計算する。その全区間を `collision_check_step` 以下の間隔で検査し、衝突がなければ探索経路へ接続する。接続が衝突する場合は通常の探索を継続する。これにより末尾 pose は位置・yaw とも要求値へ一致し、運動学を満たさない sub-cell snap は行わない。

`dubins_path.cpp` は LSL / RSR / LSR / RSL / RLR / LRL の 6 種を評価し、最短の前進経路を返す。今回の Hybrid A* は前進のみを対象とするため Reeds-Shepp は扱わない。analytic expansion が成功した時点で返す実装なので、探索全体に対する最適性は保証しない。

### 12.3 出力上の注意

Hybrid A* の各 pose の yaw は車体姿勢であり、A* のように経路接線へ上書きしない。経路はすべて前進で、隣接 pose 間の曲率は `1 / minimum_turning_radius` 以下になる。

`eltanin_hybrid_astar_demo` は YAML/PGM 地図を読み込み、実機 Footprint から求めた半径で膨張する。入力地図全体は 4000 x 4000 セルになり得るため、まず A* 経路の周囲を切り出し、その実地図領域上で Hybrid A* を実行する。A* は探索範囲の決定だけに使い、出力経路には混ぜない。

デモは `costmap.pgm` / `path.csv` / `footprint.csv` / `meta.txt` を出力する。`examples/plot_hybrid_astar.py` は既定で全経路サンプルの Footprint を重ね、車体が掃引する領域と曲率制約を PNG に描く。長い経路では `--footprint-step` で描画だけを間引ける。`--animate` 指定時は Footprint が経路に沿って移動する GIF も生成する。Python / matplotlib / numpy / Pillow は可視化スクリプトにだけ必要であり、planner 本体の依存には含めない。

---

## 13. T8: 共通 Planner I/F の統一と Hybrid A* の欠陥修正

`7f4b990` の時点で A* と Hybrid A* は同じ `Planner` 基底の下にあったが、契約が揃っておらず、Hybrid A* 側に正しさ・堅牢性・スケーラビリティの欠陥が残っていた。本章はその是正で決めたことと、根拠になった実測値を記録する。

### 13.1 拡張点は 1 組の型に固定した

派生プランナが実装するのは `plan_on_grid(const PlanQuery &) -> PlanResult` の 1 本だけである。

```
Planner::plan<Map, Model>()          planner.hpp        非仮想テンプレート
  ├─ 地図の妥当性 / start,goal の範囲 / yaw の有限性
  ├─ goal 閉塞判定 / start 救済 (find_nearest_traversable)
  ├─ build_traversability_grid()     ← 型消去の境界
  └─ plan_on_grid(PlanQuery)         仮想 / 非テンプレート
```

`PlanQuery` は「分類済みグリッドのビュー + 救済後 start セル + goal セル + 救済後 start 姿勢 + 要求 goal 姿勢」を束ねる。位置引数 6 個をやめたので、**派生が読まないメンバがあること自体が正常**になり、`static_cast<void>(effective_start)` のような「引数に来るのに捨てる」記述が構造的に不要になった。副作用として、Release (`NDEBUG`) で `goal_index` が `assert` からしか参照されず `-Werror=unused-parameter` でビルドが落ちていた不具合も消えた。

`PlanQuery` / `PlanResult` / `TraversabilityError` は `detail` に置かない。これらは**ライブラリの拡張点そのもの**であり、隠すと派生プランナを書けない。`build_traversability_grid()` と `smooth_on_grid()` は `detail` に残す。

### 13.2 通行可否判定は `TraversabilityView` 1 本に集約した

`in_bounds()` → `index()` → `== Free` の順序規約 (§4) を守る場所をクラス内 1 箇所に閉じた。従来は A* に 1 個、Hybrid A* に 2 個、スムーザに 1 個の合計 4 個のラムダが同じ順序を各々再実装していた。

ビューは `MapGeometry *` と `std::span<const std::uint8_t>` の 4 語で、所有しない。グリッドの寿命は `Planner::plan()` のローカルが持つ。全メンバ inline / `noexcept` で、仮想関数にはしない。

Theta\* の視線判定は `free(world)` を線分上でサンプリングする自由関数としてこのビューの上に載せられる。利用者が居ないので今回は追加しない (`docs/costmap-design.md` §9.2)。

### 13.3 失敗理由を返すようにした (§1 の方針を撤回した)

§1 は「失敗理由の返却はやらない」と書いていた。これを撤回する。理由は 2 つある。

1. 利用者が現れた。examples 4 本 + 統合デモ + `eltanin_ros` が失敗を利用者へ説明する必要がある。
2. Hybrid A* の追加で失敗系が 10 分類に増え、`std::nullopt` 1 本では区別できなくなった。

`PlannerError` の判定順序は次に固定する。複合的に不正な入力はこの順で最初に該当した理由を返す。

```
InvalidMap → StartOutsideMap → GoalOutsideMap → NonFiniteYaw → GoalBlocked → StartRescueFailed
→ (探索) ParamsIncompatibleWithMap / StateSpaceTooLarge / ExpansionLimitReached / Unreachable
```

**位置の非有限は `NonFiniteYaw` にならない。** `world_to_map()` の `to_int_saturating()` が先に弾くので `StartOutsideMap` / `GoalOutsideMap` に落ちる。

`PlanResult` は `std::optional` 互換の専用型にした。`has_value()` / `operator bool` / `operator*` / `operator->` / `path()` を備えるので、理由を見ない呼び出し側は `if (!result)` のまま書ける。`std::expected` は C++20 に無く、`core` への汎用 `Result<T, E>` 追加は利用者が planner 1 モジュールのみなので採らなかった。出力引数併存 API も、「理由を渡し忘れる」形が既定になるため採らなかった。

理由を無視できる形を残したのは意図的な設計である。examples では必ず `to_string(result.error())` を出力して規範を示す。

### 13.4 非有限 yaw の検査を基底へ移した

従来 A* は `goal.yaw = NaN` を受理し、末尾姿勢の yaw を `NaN` のまま返していた (`astar_planner.cpp` が `goal_pose.yaw` を代入し、`assign_tangent_yaw()` は末尾に触らない仕様のため)。既存テスト `NonFiniteCoordinatesAreRejected` は**位置しか見ていなかった**。

検査を `Planner::plan()` に置いたので全プランナに効く。Hybrid A* 側の重複検査は削除した。`assign_tangent_yaw()` には手を入れていない (末尾 yaw を触らない契約は §6 のまま)。

### 13.5 出力契約

| 項目 | A\* (`plan_astar` / `AStarPlanner`) | Hybrid A\* |
|---|---|---|
| 先頭姿勢 | 救済後 start セルの中心。yaw は接線 | 救済後 start 姿勢そのまま (位置は救済時のみセル中心へ移る) |
| 末尾姿勢 | 位置は goal セル中心 (要求 goal から最大 `0.707 * resolution` ずれる)、yaw は要求値 | 位置・yaw とも要求 goal に一致 |
| yaw の意味 | 経路接線 | 車体姿勢 |
| yaw の有限性 | 全姿勢で有限 | 全姿勢で有限 |
| 点間隔 | `resolution` 〜 `√2 * resolution` (平滑化は点数不変・端点固定なので上限が保たれる) | `(0, 1.5 * motion_step]`、同一経路内の最大/最小比は 1.5 以下 |
| 追従可否 | そのまま追従可 (既定で平滑化済み) | そのまま追従可 |
| 決定性 | 同一入力でビット同一 | 同一入力でビット同一 |

**A\* は `start.yaw` を読まない。** 生 A* 経路はセル中心の列であり先頭 yaw は接線で決まるからである。これは欠落ではなく契約である。

**Hybrid A\* の経路は旋回箇所で左右に振れる (bang-bang)。** これは設計に内在する特性であり、旧実装から存在する。primitive が直進・左 `1/R`・右 `1/R` の 3 本しかなく中間曲率を持たないため、斜め方向や緩い旋回は直進と旋回の交互で近似せざるを得ない。さらに直進 primitive は方位を厳密に保存するので、一度 `heading_bins` (既定 72 = 5° 刻み) に量子化されるとその方向へ進み続け、goal へ向けるために単発の旋回が挿入される。曲率符号の列で見ると直進の連なりに `L` や `RL` が点在する形になる。

90° 旋回 1 本での実測 (200 x 200、`resolution 0.05`、`L`=左 `R`=右 `-`=直進):

```
旧 LLLLLL-----------L----------------------------RL---------------RL---------RR---...---LLLLLLLLLLLLLLL
新 LLLL-------------------------------------------------------L----------------...---RRR-----LLLLLLL
```

| | 旧 | 新 |
|---|---|---|
| 左右反転回数 | 10 | **2** |
| 総旋回量 (正味 1.571 rad に対し) | 3.264 rad | **2.358 rad** |
| 無駄な旋回 | 1.694 rad | **0.787 rad** |

統合デモの実経路 (2 レグ / 約 80 m) でも左右反転が 18 → 12 回 (0.23 → 0.15 回/m)、平均 `|曲率|` が 0.429 → 0.267 rad/m に減っている。構造的に消すには中間曲率の primitive を追加するか、曲率制約を保つ平滑化 (conjugate gradient 系) を掛ける必要がある。どちらも今回の範囲外であり §13.14 に申し送る。

参考として、**A\* の平滑化済み経路のほうが振れは大きい**。8 近傍のセル中心列に由来する 45° の折れが残るため、同じデモ経路で左右反転 70 回 (0.87 回/m)、最大 `|曲率|` 35.5 rad/m である (Hybrid A* は 2.51 rad/m = `1 / minimum_turning_radius` を厳密に守る)。追従側の実効的な滑らかさは `PurePursuit` の先行距離が吸収する。

### 13.6 `plan_astar()` が完成経路を返すようにした

呼び出し側がアルゴリズムを知らずにプランナを差し替えられることが I/F 統一の目的なので、`plan_astar()` は既定で平滑化まで済ませた経路を返す。生経路は `AStarParams::smoother` を `std::nullopt` にすれば得られる。

```cpp
eltanin::planner::AStarParams raw;
raw.smoother.reset();
const auto path = eltanin::planner::plan_astar(map, model, start, goal, raw);
```

Hybrid A* には平滑化を掛けない (曲率制約を壊す)。`HybridAStarParams` は smoother メンバを持たない。

平滑化の実装は二重化していない。反復スイープの本体を `detail::smooth_on_grid(path, TraversabilityView, params)` に切り出し、公開 `smooth()` テンプレートと `AStarPlanner::plan_on_grid()` の両方がこれを通る。`plan_on_grid()` 側はグリッドが既に手元にあるので追加確保はゼロである。

**受け入れたコスト (重要)**: 単独の公開 `smooth()` は、呼び出しごとに全セル分類グリッド (セル数 × 1 byte) を構築するようになった。従来は候補点ごとの遅延分類だったので経路長にしか比例しなかった。実地図 4000 x 4000 (1600 万セル) / 727 姿勢の経路で実測した内訳:

| | 旧 `7f4b990` | 新 |
|---|---|---|
| A\* 探索 | 136–142 ms | 121–128 ms |
| `plan_astar()` 既定 (探索 + 平滑化) | — | 109–127 ms |
| 単独 `smooth()` | 0.16–0.17 ms | 18.4–20.1 ms |
| ↳ うち `build_traversability_grid()` | — | 18.5–19.3 ms |
| ↳ うち平滑化スイープ本体 | 0.16 ms | 0.11–0.14 ms |

探索そのものは速くなっている (判定が `model.classify()` の間接呼び出しからグリッド 1 バイトの比較になったため)。増えたのは単独 `smooth()` の 19 ms であり、その全量がグリッド構築である。

**したがって `plan_astar()` の後に `smooth()` を呼んではならない。** `plan_astar()` は既定で平滑化済みの経路を返すので、2 度目の `smooth()` は探索 1 回分に匹敵する無駄なグリッド構築を足すだけである。`eltanin_ros` の `global_path_planner.cpp` のように「探索 → 別途 `smooth()`」の形になっている呼び出し側は、`smooth()` 呼びを削除すること。削除すれば合計は旧実装より速くなる (136–142 ms → 109–127 ms)。

`smooth()` を単独で使う正当な用途は「planner 以外から来た経路を平滑化する」場合だけである。グリッド構築を経路の bounding box に限る最適化は、平滑化の変位に上限が無いためビット同一性を壊しうる。採らなかった。

### 13.7 Hybrid A* のメモリを 1/8.4 にした

状態配列の内訳を変えた。

| 項目 | 旧 | 新 |
|---|---|---|
| 状態キー | `(cell, heading_bin, mode)`、`MODE_COUNT = 4` | `(cell, heading_bin)` |
| `g_score` | `double` 8 B | `float` 4 B |
| `best_node` | `std::size_t` 8 B | `std::uint32_t` 4 B |
| `closed` | `std::uint8_t` 1 B | ビット列 (`std::vector<std::uint64_t>`) 0.125 B |
| 1 state 合計 | 17 B | 8.125 B |
| `nodes.reserve()` | `min(state_count, cells * 8)` (400 x 400 で約 80 MB) | 定数 `INITIAL_NODE_RESERVE = 4096` |

ピーク RSS の実測 (`resolution 0.05`、`heading_bins 72`、Release `-O2`):

| ケース | 旧 | 新 | 比 |
|---|---|---|---|
| open200 (10 m 角、直進) | 190.2 MB | **25.5 MB** | 0.134 |
| corridor (通路) | 70.5 MB | 11.4 MB | 0.162 |
| wall200 (10 m 角、壁の迂回) | 346.7 MB | 73.9 MB | 0.213 |
| open400 (20 m 角) | 750.0 MB | 92.5 MB | 0.123 |

**`mode` を状態キーから落とした判断 (§12.2 の改訂)**: §12.2 は steering change penalty を Markov な遷移コストにするため `previous_motion_mode` を状態に入れた、と記録していた。落とすと同一 `(cell, heading_bin)` に異なる直前操舵で到達した候補が 1 つに潰れ、遷移コストが履歴依存になる。すなわち A* の整合性 (consistency) を厳密には失う。受け入れた理由は、Hybrid A* が解析接続の成功時点で返すため**探索全体の最適性はもともと保証していない** (§12.2 末尾) ので、整合性の厳密な保持が守っている実質的な性質が無いことである。`Node` は cost 計算のために直前 mode を `std::uint8_t` で持ち続け、start ノードは番兵値 `START_MODE` を使う。`START_MODE` 専用の 4 番目の次元 (旧実装で状態配列の 25 % を占めていた) も同時に消えた。

経路品質は劣化していない。むしろ改善した。

| ケース | 経路長 旧 → 新 | 操舵切り替え回数 旧 → 新 |
|---|---|---|
| open200 | 9.000 → 9.000 m | 0 → 0 |
| turn (90° 旋回) | 10.012 → 10.019 m (+0.07 %) | 23 → **6** |
| wall200 | 13.705 → **13.618** m | 41 → **8** |
| corridor | 11.000 → 11.000 m | 0 → 0 |

**改善分は `mode` 除去ではなく `motion_step` の既定値変更 (§13.9) に由来する。** 交絡を切り分けるため、旧実装を新しい `motion_step = √2 * resolution` で走らせて比較した (200 x 200、`resolution 0.05`、左右反転回数で測る)。

| | turn | wall200 |
|---|---|---|
| 旧 @ `motion_step = 1.0 * res` (旧既定) | 10 | 12 |
| 旧 @ `motion_step = √2 * res` | **2** | **2** |
| 新 @ `motion_step = √2 * res` (新既定) | **2** | **2** |

`motion_step` を揃えると旧新の反転回数は一致する。**`mode` を状態キーから落としたことは経路品質に対して中立である** (総旋回量は turn で 2.112 → 2.358 rad、wall200 で 3.889 → 3.654 rad と互いに一長一短)。姿勢数の差 (turn 169 → 143) は §13.11 の Dubins 出力間隔の変更による。

`steering_change_penalty` は、この長い `motion_step` の下では**ほとんど効かない**。同条件での掃引:

| `steering_change_penalty` | 0.0 | 0.1 (既定) | 0.5 | 1.0 | 3.0 |
|---|---|---|---|---|---|
| 旧 @ `1.0 * res` / turn の反転 | 14 | 10 | 6 | 6 | 6 |
| 旧 @ `√2 * res` / turn の反転 | 2 | 2 | 2 | 2 | — |
| 新 (既定) / turn の反転 | 2 | 2 | 2 | 2 | 1 (経路長 10.019 → 12.668 m と悪化) |

これも `mode` 除去の副作用ではない — 旧実装でも `motion_step` を長くすると同じく効かなくなる。primitive が長いと判断点が減り、幾何がほぼ一意に決まるのでペナルティが作用する余地が無いためである。**既定 0.1 の時点で反転 2 回であり、旧実装をどう調整しても届かない水準 (最良 6 回) なので、達成される品質としては失っていない。** 3.0 まで上げると交互を避けるために遠回りするので、上げる意味は無い。

**メモリは `heading_bins` に比例する。** 既定 72 は変えていない。`mode` 除去でメモリ目標を満たせたので角度分解能を削る必要が無かった。

**実地図全体には依然として適用できない。** 4000 x 4000 セルは状態 1.15e9 個 = 9.4 GB であり、既定の `max_state_memory_bytes` (256 MiB) が拒否する。呼び出し側の corridor 切り出し (§12.3) は残る。密配列をフラットなオープンアドレス法のハッシュへ移せばメモリが展開数比例になり、この制約を外せる可能性がある — 次段の選択肢として申し送る。

### 13.8 `plan()` は本当に例外を投げなくなった

§11 は「例外は投げない」と宣言していたが、実際には `ulimit -v 2000000` の下で 1200 x 1200 セルの地図に対し `std::bad_alloc` が `plan()` の外へ伝播していた。旧実装のガードは `std::size_t` の乗算オーバーフローだけを見ており、確保可能かは見ていなかった。

三重にした。

1. 添字計算のオーバーフロー検査 (`state_count` を確定する前の上限比較)。状態 id と node id が `std::uint32_t` に収まることも検査する。
2. `state_count * 8 B + closed の語数 * 8 B` を `max_state_memory_bytes` (既定 256 MiB) と比較し、超えるなら**確保前に** `StateSpaceTooLarge` を返す。
3. それでも `std::bad_alloc` が出る環境に備え、探索全体を `try` / `catch (const std::bad_alloc &)` で包み `StateSpaceTooLarge` に変換する。

3 の必要性は実測で確認した。`max_state_memory_bytes` を 4 GiB に上げて 1200 x 1200 の地図を `ulimit -v 600000` (600 MB) の下で計画すると、事前判定は通るが確保が失敗する。この構成で `StateSpaceTooLarge` が返り、例外は漏れない。

既定 256 MiB は、`resolution 0.05` / `heading_bins 72` でおよそ 30 m 角の地図に相当する。

### 13.9 `motion_step` が離散状態を変えられない組み合わせを拒否するようにした

旧実装は開けた地図でも `motion_step = 0.2 * resolution` で `nullopt` を返した。`closed[current.state]` を立ててから展開するので、3 本の primitive がすべて現在と同じ離散状態に落ちると即座に行き止まる。ctor は `motion_step >= 0.0` しか見ておらず、`resolution` は plan 時にしか判らないため検査されていなかった。

十分条件を primitive ごとに整理する。

| primitive | 状態が変わる条件 | 十分条件 |
|---|---|---|
| 直進 (curvature 0) | セルが変わる。方位 θ に対し変位は `max(\|dx\|,\|dy\|) >= motion_step / √2` | `motion_step >= √2 * resolution` |
| 旋回 (curvature ±1/R) | 向きビンが変わる (`motion_step / R >= 2π / heading_bins`)、または弦長でセルが変わる (`2R sin(motion_step / 2R) >= √2 * resolution`) | 上記いずれか |

**直進条件と旋回条件は別に満たす必要がある。** 既定値 (`R = 0.4`、`heading_bins = 72`、`resolution = 0.05`) では直進条件が `motion_step >= 0.0707 m`、旋回条件が `motion_step >= 0.0349 m` なので直進条件が支配的である。

決めたこと。

1. `motion_step = 0` の自動値を `resolution` から **`√2 * resolution`** に変えた。既定は必ず十分条件を満たす。
2. 明示指定が十分条件を破る場合は `ParamsIncompatibleWithMap` を返す。検査は `resolution` が判る `plan_on_grid()` の先頭に置く。

**受け入れた副作用**: 十分条件は十分であって必要ではない。旧実装で成功していた `motion_step = 0.5 * resolution` は拒否されるようになる。これは「たまたま動く設定を黙って受ける」旧実装の欠陥の裏返しである。

既定値変更の実測影響 (40 x 40 セル、`resolution 0.1` の自由空間、`open200` は `resolution 0.05`):

| `motion_step` | 旧 | 新 |
|---|---|---|
| `0.2 * res` | `nullopt` (欠陥) | `ParamsIncompatibleWithMap` |
| `0.5 * res` | 成功 365 点 / 9.099 m | `ParamsIncompatibleWithMap` |
| `1.0 * res` | 成功 201 点 / 9.000 m (旧既定) | `ParamsIncompatibleWithMap` |
| `√2 * res` | 成功 153 点 | 成功 **129 点** / 9.000 m (新既定) |
| `2.0 * res` | 成功 119 点 | 成功 92 点 |
| `3.0 * res` | 成功 91 点 | 成功 61 点 |

出力点間隔は 0.05 → 0.0707 m、点数は減る。`HybridAStarPlanner.ChangesHeadingWithBoundedCurvature` の 1 ステップ旋回上限は `resolution / R` から `√2 * resolution / R` へ更新した。

**採らなかった対案 (申し送り)**: 「離散状態が変わるまで primitive を延長する適応ステップ」。`motion_step` を積分/衝突検査の刻みとして自由に取れるようになり、出力点間隔も自動的に揃う点で本質的に優れる。ただし遷移が可変長になり、コストとヒューリスティックの許容性の再確認と探索構造の変更を伴う。今回の「作り直さない」方針の範囲を超えるため採らなかった。

### 13.10 `max_expansions` の既定を有限にした

旧既定 0 (無制限) では、到達不能な goal に対しヒューリスティックが障害物を無視するため到達可能側の状態空間を全て展開していた。統合デモは再計画時に同じ呼び出しをするので、閉塞時に制御ループが止まる。

既定を `HYBRID_ASTAR_DEFAULT_MAX_EXPANSIONS = 4000000` にした。`0 = 無制限` の意味は逃げ道として維持する。打ち切りは `ExpansionLimitReached`、探索が尽きたのは `Unreachable` で区別する。

根拠になった実測 (`resolution 0.05`、`max_expansions` を二分探索して「成功に必要な展開数」を求めた):

| ケース | 必要展開数 | 所要 |
|---|---|---|
| corridor (通路) | 142 | 7.7 ms |
| open200 / open400 | 114 / 255 | 20 / 69 ms |
| turn (90° 旋回) | 3,454 | 23 ms |
| wall200 (10 m 角、7.5 m の壁を迂回) | 710,979 | 723 ms |
| wall400 (20 m 角、15 m の壁を迂回) | 3,377,660 | 4.3 s |

到達不能ケース (200 x 200 を壁で完全に二分) の所要時間:

| 上限 | 旧 | 新 |
|---|---|---|
| 無制限 | 3205 ms | **1318 ms** |
| 2,000,000 | 2403 ms | 1393 ms |
| 4,000,000 | — | 1489 ms |

展開 1 回あたり約 1.13 µs である。200 x 200 では状態空間が 2.88e6 で有界なので、**どの上限でも 1.5 s 以内に失敗が返る**。既定 4e6 は測定した全ての到達可能ケース (最大 3.38e6) を通し、1 回の呼び出しを約 4.5 s に抑える値として選んだ。

**残る限界**: 上限は「遅い失敗」を「速い失敗」に変えるだけで、狭い通路の遠回りが必要な地図では**到達可能でも打ち切りで失敗しうる**。20 m 角の地図で 15 m の壁を迂回するケースが既に 3.38e6 を要しており、既定に対する余裕は 1.18 倍しかない。より大きい地図で迂回が必要な場合は `max_expansions` を明示的に上げるか、corridor を切り出すこと。構造的な解決は障害物考慮ヒューリスティック (A* コアの内部再利用) であり、今回は範囲外とした。

A* に打ち切りは追加しない。`closed` がセル数で有界だからである (§1 の方針を維持)。旧実装の `cells <= INT32_MAX` ガードは `StateSpaceTooLarge` を返すようにしただけで、新しい失敗経路は追加していない。

### 13.11 Dubins 区間の出力間隔を衝突検査間隔から分離した

旧実装は `collision_check_step` を Dubins 末尾の**出力サンプリング間隔にも流用**していた。`motion_step = 0.2` / `collision_check_step = 0.02` で計画すると、探索区間の間隔が 0.2 m、Dubins 区間が 0.0195 m という 10 倍差の不均一な経路が出ていた。

分けた。

- **衝突検査**: `collision_check_step` 間隔で Dubins 経路を走査する。細かさは維持する。
- **出力**: `count = max(1, ceil(length / motion_step))` の等分割で生成する。間隔は `length / count`。

`count >= 2` なら `length / count > motion_step / 2` が保証される。`count == 1` (`length <= motion_step`) のときだけ末尾区間が極小になりうるので、**その区間が公称間隔の半分未満なら 1 つ前の姿勢を落とす**。落とす姿勢は衝突検査済みの点なので経路の妥当性は変わらず、統合後の最終区間は `1.5 * motion_step` 未満に収まる。これにより経路全体の最大/最小比は 1.5 以下になり、§13.5 の契約として書ける。

実測 (`open200` / 既定パラメータ):

| | 旧 | 新 |
|---|---|---|
| 隣接点間隔 | [0.0244, 0.0500] m | [0.0671, 0.0707] m |
| 最大/最小比 | 2.05 | **1.05** |

`dubins->length() == 0.0` のとき末尾を要求 goal 姿勢へ差し替える旧来の振る舞いは維持している。

### 13.12 再現しなかった懸念の記録

Hybrid A* は離散状態での goal 到達判定を持たず、**解析接続の成功のみを終了条件にしている**。したがって解析接続が常に衝突する配置では到達可能でも失敗しうる。goal の yaw を 1 セル先の壁へ向けた配置、`dubins_expansion_distance = 0.1` の配置、`minimum_turning_radius` ぎりぎりの幅の通路の奥に goal を置く配置をいずれも試したが、失敗するケースは作れていない。**既知の不完全性として記録するが、振る舞いは `Unreachable` のままとする。** 離散到達判定を追加すると末尾姿勢が要求 goal に一致しなくなり §13.5 の契約と衝突するため、別タスクへ送る。

### 13.13 D* Lite / Theta* への見通し

D\* Lite には別 I/F が必要であるという §12.1 の結論は変わらない。呼び出しを跨ぐ状態と地図更新通知を要するので、`plan()` が `const` で作業領域を持たない今の基底には載らない。ただし `PlannerError` と `TraversabilityView` は増分プランナでもそのまま再利用できる (地図更新通知や状態保持を基底に入れていない)。

Theta\* は今の基底に載る。視線判定は `TraversabilityView::free(world)` の上に線分サンプリングの自由関数として書ける。出力は任意角の折れ線になるが、§13.5 の契約のうち点間隔以外はすべて満たせる。点間隔はプランナごとに上限を明記する契約なので、Theta\* は自身の上限を宣言すればよい。

### 13.14 申し送り

- 探索状態を密配列からフラットなオープンアドレス法のハッシュへ移す。メモリが展開数比例になり `StateSpaceTooLarge` が実質的に不要になる。実地図全体への適用が視野に入る
- 離散状態が変わるまで primitive を延長する適応ステップ (§13.9 の対案)
- 旋回箇所の bang-bang を減らす手段 (§13.5)。中間曲率の primitive を追加するか、曲率制約を保つ平滑化を掛ける。`steering_change_penalty` は既定の `motion_step` の下では効かないので、この目的には使えない
- 障害物考慮ヒューリスティック (§13.10 の残る限界の構造的解決)
- 増分プランナ (D\* Lite) 用の別 I/F
- 探索作業領域を保持するプランナ型への移行 (`plan()` の `const` を外す破壊的変更)
