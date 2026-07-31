# 衝突判定と速度制限 (`eltanin_collision`) の設計

対象は `include/eltanin/collision/collision_checker.hpp` / `velocity_limiter.hpp` と
`src/collision/` の 2 ソース、および T6 で `eltanin_core` / `eltanin_map` に足した部品と
`eltanin_sim` (`SimpleSimulator`)。

移植元は navyu の `navyu_safety_limiter`、`navyu_utils/costmap_helper`、`navyu_simulator`。

命名・公開範囲・エラー通知・依存の規約は `docs/costmap-design.md` §9〜§12 に従う。

---

## 1. モジュール名と分割

navyu の `navyu_safety_limiter` は ROS ノードの名前であり、「セーフティ」は用途である。
このライブラリは機構名で分ける方針なので、**用途名を持ち込まなかった**。

| 単位 | 責務 | 依存 |
|---|---|---|
| `eltanin::collision` (`collision_checker.hpp`) | 「衝突しているか」を答える。セル / 点群 / 多角形の各粒度 | `core` + `map` |
| `eltanin::collision` (`velocity_limiter.hpp`) | 指令から姿勢列を予測し、制動距離則で指令の大きさに上限を掛ける | 同上 |
| `eltanin::sim` (`simple_simulator.hpp`) | 差動二輪の plant (可変状態を 1 つ持つ) | `core` のみ |

`SafetyLimiter` という名前は T7 の ROS ノード側に残す (ノードは「セーフティ」という用途を表す層)。
`sim` を別ターゲットにしたのは、`core` を純関数だけの層に保つためである
(`integrate_differential_drive` は純関数、`SimpleSimulator` は可変状態を持つ plant)。

### 1.1 衝突判定は粒度ごとに分けた

判定は 1 本の巨大関数ではなく、下から積み上げた層にしてそれぞれ単独でテストする。

| 層 | 関数 | 置き場所 |
|---|---|---|
| 点 vs 多角形 | `contains(polygon, point, edge_tolerance)` | `core/polygon.hpp` (T1 から存在) |
| 点群 vs 多角形 | `contains_any(polygon, points)` | `collision_checker` (非テンプレート) |
| フットプリント vs 点群 | `footprint_hits_points(footprint, pose, points)` | 同上。スキャン点に対する判定で、マップを要らない |
| 多角形が覆うセル矩形 | `cells_covering(geometry, polygon)` | 同上 (非テンプレート) |
| セル vs 占有基準 | `is_cell_occupied(map, model, mx, my)` | 同上。マップ外は非占有 |
| 多角形 vs 占有セル群 | `contains_occupied_cell(map, model, world_polygon)` | 同上。厳密判定の本体 |
| 姿勢付きフットプリント vs マップ | `check_footprint(map, model, footprint, pose)` | 同上。二段構えの入口 |

`footprint_hits_points` はライブラリ内に呼び出し元がまだない。スキャン点は `eltanin_sensor` が
既に生成しており (`project_scan`)、コストマップを持たない構成での判定に必要になるため、
`docs/costmap-design.md` §9.2 の「利用者のない public を作らない」に対する例外として
**粒度の対称性を優先して公開した**。T7 が最初の利用者になる想定である。

---

## 2. 二段構えの衝突判定

`docs/costmap-design.md` §3 の三段階分類をそのまま判定の段数に使う。

```
第 1 段: 中心セルのコストを 3 値分類する (向きを見ない)
  Free          -> 衝突なし (短絡)
  Inscribed     -> 衝突     (短絡)
  Circumscribed -> 第 2 段へ
第 2 段: フットプリントを world 系に移し、AABB のセル矩形内で
         「占有セルの中心が多角形に内包されるか」を見る (向きを見る)
```

`detail::classify_first_stage()` がこの表を非テンプレート関数として持つ。テンプレートの中に
`switch` を書かずに済むので、政策そのものをマップなしでテストできる
(`test_collision_checker.cpp` の `FirstStage.EncodesTheTwoStagePolicy`)。

### 2.1 navyu との違い

**navyu も向きは考慮している。** `CostmapHelper::obstacle_collision_check()` は姿勢で変換した
フットプリントの AABB を走査し、セル中心の内包を winding-angle 法で判定する。構造は本実装の
第 2 段とほぼ同じである。違いは次の 2 点。

| # | navyu | 本実装 |
|---|---|---|
| 1 | 第 1 段がない。全 step で必ず AABB 走査と内包判定を払う | 中心セル 1 個の 3 値分類で大半の step を byte 比較 1 回で捨てる |
| 2 | 占有基準が `cost > 99`。膨張帯のかなり外側まで障害物扱いになり、実際には通れる姿勢でも停止する (偽陽性) | `is_obstacle` (`cost >= LETHAL_OBSTACLE`) で「膨張値」と「実障害物セル」を分離する |

二段構えは (1) を足し、(2) を置き換えたものである。「どれくらい近いか」を第 1 段 (半径ベース) に、
「厳密にどこか」を第 2 段 (占有ベース) に分けたのが設計上の要点であり、navyu はこの 2 つを
`cost > 99` という 1 本のしきい値に畳んでいた。

### 2.1.1 navyu の内包判定は内部点を取りこぼす

`CostmapHelper::is_inside_polygon()` は角度和が `2 * pi` に一致するかを

```cpp
return (2 * M_PI - std::fabs(sum_angle)) < std::numeric_limits<double>::epsilon();
```

で見ている。**許容値 2.22e-16 は 4 回の `atan2` が積む誤差より小さい。** navyu の既定フットプリント
(0.44 x 0.30 m) を 1 度刻みに回転させて内部の 12 点を判定すると、4320 件中 **41 件 (約 1 %) が
`false`** になる (残差の最大は 8.88e-16)。**衝突の見逃し側**の誤りである。

`core/polygon.cpp` の `contains()` は crossing number で、境界は `distance_to_segment` と
`edge_tolerance = 1e-9` で先に確定させる。角度和を使わないのでこの脆さがない。

### 2.2 占有基準は 3 値分類を流用しない

第 2 段で「このセルは障害物か」を決めるのに `classify()` を使うと、膨張値 (最大 253) が
`Inscribed` に落ちるため**膨張帯のセル中心が内包されただけで衝突になり、向き依存性が消える**。
そこで `ObstacleModel` concept を `core/traversability.hpp` に足し、
`CostTraversabilityModel::is_obstacle()` を `cost >= LETHAL_OBSTACLE` として実装した。

`InflationLayer` が非障害物セルに書く上限は `INSCRIBED_INFLATED_OBSTACLE = 253` であり、
`LETHAL_OBSTACLE = 254` は元の障害物セルにしか立たない (`cell = std::max(cell, cost)`)。
したがってこの基準は「膨張値ではなく実障害物セル」を正しく選ぶ。
未知セルの扱いは既存の `unknown_is_free` フラグ 1 つに従わせ、第 1 段と第 2 段で解釈が食い違わないようにした。

`CollisionRadii` (距離場側) には `is_obstacle` を**足していない**。距離場の producer がまだなく、
距離から占有を決めるには実質 `distance < resolution / 2` という別のしきい値を発明することになる。

### 2.3 座標変換は `MapGeometry` の中だけ

第 2 段が要る「world 矩形 → クランプ済みセル矩形」を `MapGeometry::world_rect_to_cells()` として
足した。`collision` 側に `std::floor((wx - origin) / resolution)` は 1 箇所もない。
既存の private な `floor_to_index()` / `to_int_saturating()` を再利用するので、
**負座標の切り捨て方向と int 範囲外・NaN の防御が既存実装と共有される**
(navyu の `static_cast<int>` による誤変換が再発しない構造)。

窓がマップと交差しなければ `std::nullopt` を返す。クランプ忘れが起こりえない形である。

### 2.4 残余する量子化誤差 (消していない)

第 1 段が読むコストは**ロボット中心セルの中心**から最近傍障害物セル中心までの距離を符号化した値で、
実姿勢はセル中心から最大 `0.707 * resolution` (既定 0.0354 m) ずれる。したがって `Free` 短絡は
最悪 `circumscribed_radius - 0.0354` の距離にある障害物を見逃しうる。

これは navyu にも nav2 にもある既知の量子化残余であり、消すには第 1 段をセル中心ではなく
実姿勢からの距離で判定する必要がある (= 距離場が要る)。厳密さを要求する呼び出し側は
`CollisionRadii` を `0.707 * resolution` だけ膨らませて構築すること (`inflation_radius` も同じだけ広くなる)。
T6 のテストはこの膨らませを行わない既定構成で書いている。

また、第 2 段は「コストマップが同じフットプリント由来の半径で膨張されている」ことを前提にする。
別の半径で膨張されたコストマップを渡すと第 1 段の短絡が安全側でなくなる。
`limit()` からは検証できないため、テストでは必ず `CollisionRadii::from_footprint()` で半径を導出している。

---

## 3. 後退時の速度制限 (navyu のバグ)

navyu の `navyu_safety_limiter.cpp` は次の形だった。

```cpp
auto sgn = [](double val) { return (val > 0.0) ? 1.0 : ((val < 0.0) ? -1.0 : 0.0); };
const double d_col = std::max(0.0, collision_distance - margin_);
const double v_lim = sgn(d_col) * sgn(linear_velocity) * std::sqrt(2.0 * alpha_ * std::fabs(d_col));
double w_lim = cmd_vel_in_.angular.z;
if (std::numeric_limits<double>::epsilon() < std::abs(linear_velocity)) {
  w_lim = cmd_vel_in_.angular.z * (v_lim / linear_velocity);
}
cmd_vel_in_.linear.x = std::min(v_lim, cmd_vel_in_.linear.x);
cmd_vel_in_.angular.z = std::min(w_lim, cmd_vel_in_.angular.z);
```

`v_lim` は `sgn()` により**符号付き**で、大きさ自体は正しく計算されている。欠陥は
**`std::min` が「符号付き量の大きさに上限を掛ける」演算になっていない**ことである。
後退 (`v_in < 0`) では `v_lim` も負になり、`std::min` は「より負の値」= 入力そのものを返す。
`collision_distance = 0.25`、`v_in = -0.5` のとき `v_lim = -0.2236` で
`std::min(-0.2236, -0.5) = -0.5`。**角速度にも同じ `std::min` を掛けている**ため、
`w_in < 0` でも同じことが起きる。

`std::clamp(v_in, -v_max, v_max)` は区間 `[-v_max, v_max]` への射影なので、符号によらず
大きさだけを切る。これが `min` との本質的な差である。

`detail::limit_command()` は大きさだけを抑える形にした。

```cpp
const double d_col = std::max(0.0, collision_distance - params.collision_margin);
const double v_max = std::sqrt(2.0 * params.max_deceleration * d_col);  // 常に非負 = 大きさの上限
const double v_in  = cmd_in.linear.x();
const double v_out = std::clamp(v_in, -v_max, v_max);                   // 符号を保つ
const double ratio = (std::abs(v_in) > MIN_LINEAR_VEL) ? v_out / v_in : 1.0;
double w_out = cmd_in.angular * ratio;                                  // 曲率を維持
```

| 性質 | 根拠 |
|---|---|
| 符号が保たれる | `clamp` は区間 `[-v_max, v_max]` への射影であり、大きさだけを切る |
| 曲率が保たれる | `w_out / v_out == w_in / v_in`。速度だけ落として角速度を残すと旋回半径が縮み、予測と実挙動がずれる |
| 無衝突は分岐なしで素通し | 無衝突を `collision_distance = +inf` で表す。`d_col = +inf` → `v_max = +inf` → `clamp` が恒等、`ratio` が厳密に 1.0。**「無衝突なら制限しない」を実装が忘れられない** |
| 純旋回の衝突だけ特別扱い | `v_in = 0` では比率が作れない。`has_collision && abs(v_in) <= 1e-9` のとき `w_out = 0` にする |
| `linear.y()` は読まず出力は常に 0 | 差動二輪に限定する。navyu は予測だけオムニ対応で plant と不整合だった |

`limit_command()` は**非テンプレートで `.cpp` に置き、マップを一切見ない**。後退バグの回帰テストが
コストマップを組まずに書けることを構造で保証している。

### 3.1 制限量は階段状に落ちる

`collision_distance` は予測 1 step の弧長 `abs(v) * dt` (既定 0.1 m) の倍数に量子化されるため、
閉ループの `v_out` は単調にも等間隔にも減らない (実測: `-0.4472, -0.4472, -0.3162, -0.3162, 0`)。
滑らかにするには `prediction_steps` を上げるしかない。テストは単調性を仮定せず、

- 各周期で「制限後の指令で進んだ姿勢が衝突していない」(安全性)
- 有限周期内に `v_out == 0.0` に到達する (停止性)
- 停止余裕が `[collision_margin, collision_margin + 2 * abs(v) * dt]` に入る (過剰に手前で止まらない)

の 3 点で固定している。

### 3.2 弧長は弦長ではない

`collision_distance` は `abs(v) * dt` の累積である。navyu の弦長累積 (`(p[i] - p[i-1]).norm()`) は
旋回時に走行距離を過小評価し、制動距離を短く見積もる。純旋回では弧長 0 になり、
`v_max = 0` から比率が作れない (§3 の純旋回分岐がここに対応する)。

### 3.3 マップ外は「打ち切って制限しない」

予測姿勢がマップから出たら、その姿勢は `predicted_poses` に入れずにループを打ち切る。
衝突扱いにすると、ロボット追従の小さなローカルコストマップで常時停止する。
呼び出し側は `predicted_poses.size() < prediction_steps + 1 && !has_collision` で打ち切りを検知できる。
`Result` に専用フラグは足していない。

### 3.4 現在姿勢は判定しない

最初に判定するのは 1 step 先である。既に重なっている姿勢は指令を 0 にしても直らないので、
リミッタの責務ではない。ただし指令が完全に 0 のときは 1 step 先 = 現在姿勢になるため、
結果として現在姿勢が判定される (`has_collision = true`、`collision_distance = 0`)。

---

## 4. 運動モデルの一元化

`integrate_differential_drive()` を `eltanin_core` の純関数 1 本にし、
**リミッタの予測と `SimpleSimulator` (plant) が同じ関数を呼ぶ**。予測と plant で式が違うと、
閉ループテストが「モデル誤差」と「制限の誤り」を区別できなくなる。

```cpp
if (std::abs(w) < ANGULAR_VEL_EPSILON) {   // 直進近似
  position += v * dt * Vector2d{cos(yaw), sin(yaw)};
} else {                                   // 円弧積分
  const double radius = v / w;
  position.x() += radius * (sin(yaw + w * dt) - sin(yaw));
  position.y() += radius * (cos(yaw) - cos(yaw + w * dt));
}
return Pose2D{position, normalize_angle(yaw + w * dt)};
```

navyu からの変更点は `normalize_angle()` を通すことだけである (何周積んでも `yaw` が発散しない)。

### 4.1 微小 `omega` 分岐の根拠は「桁落ち」だけではない

`omega -> 0` で `v / omega` が発散するので分岐そのものは必須である。ただし
「桁落ちするから直進近似の方が精度が高い」は `dt` に依存する。

- 円弧積分の桁落ち誤差 ~= `(v / omega) * ulp(1) ~= (v / omega) * 1.1e-16`
- 直進近似の打ち切り誤差 ~= `v * omega * dt^2 / 2`
- 両者が等しくなる `omega` = `sqrt(2 * 1.1e-16) / dt = 1.48e-8 / dt`

`dt = 0.01` (シミュレータ想定) では 1.5e-6 で、navyu のしきい値 1e-6 はほぼ最適。
`dt = 0.2` (既定の予測周期) では 7.4e-8 で、しきい値 1e-6 は大きすぎる。ただしその代償は
1 step あたり最大 1e-8 m であり実用上無視できる。**しきい値は navyu 踏襲の 1e-6 [rad/s] にした。**

このため「`omega = 1e-6` と `omega = 0` の差が 1e-12 以下」は原理的に成立しない。両者の差は
丸め誤差ではなく物理量 `v * omega * dt^2 / 2` である (`dt = 0.2` で 1e-8)。テストは
`v * omega * dt^2` を許容上限として、分岐のどちら側でも同じ解析上限で抑えられることと、
`v / omega` の発散・NaN が出ていないことを固定している。

### 4.2 `test/control/tracking_fixture.hpp` との差

T5 の追従テストが持つローカル積分は直進 Euler で、円弧積分とは式が違う。T6 では差し替えていない
(T5 の実測基準を動かさないため)。**したがって T5 の追従誤差の数値は円弧積分では厳密に再現しない。**
移行は別タスクとする。

---

## 5. 多角形ユーティリティ

`core/polygon.hpp` に足したのは 4 つ。

| 関数 | 契約 |
|---|---|
| `winding(polygon, area_tolerance = 1e-12)` | `signed_area` の符号。`n < 3` と `abs(area) <= tolerance` は `Degenerate` |
| `to_counter_clockwise(polygon)` | CW のとき頂点列を反転する。冪等、`Degenerate` では恒等。**先頭頂点は保存しない** |
| `is_convex(polygon)` | 連続する辺の外積の符号が一貫しているか。共線頂点は許す。巡回順に依存しない |
| `bounding_box(polygon)` | AABB を `(min, max)` で返す。前提: 頂点 1 個以上 |

- `area_tolerance` の既定値 1e-12 は `src/core/footprint.cpp` の `kDegenerateAreaTolerance` と同値。
  `inscribed_radius()` が退化とみなす多角形と `winding()` が `Degenerate` と言う多角形が一致する。
- `to_counter_clockwise` が先頭頂点を保存しないのは、`contains` / `transform` / `signed_area` が
  いずれも巡回開始位置に依存しないため問題にならない。
- `is_convex` の外積比較は**相対しきい値** (`abs(cross) <= 1e-12 * |e1| * |e2|`) にした。
  絶対比較にすると座標スケールで結果が変わる (`segment_intersection` の平行判定と同じ理由)。
- `is_convex` は全頂点が共線の多角形に `true` を返す (符号がすべて 0)。**退化検査を兼ねさせない。**
  `create()` は `inscribed_radius().has_value()` と `is_convex()` の両方を課す。
- 自己交差多角形は外積の符号が一貫しうるため `is_convex` が `true` を返しうる。
  `Polygon2D` の契約 (自己交差は非対応) の範囲外である。

### 5.1 保留した 3 ユーティリティ

T1 から移送された要件のうち**重心 / 点と多角形の符号付き距離 / 多角形同士の交差判定は実装していない。**
`docs/costmap-design.md` §9.2 の「利用者が現に存在しないものを public にしない」に従う。

- 重心: 衝突判定は原点からの内接半径を使うので要らない。
- 符号付き距離: 距離場を作るときに要るが、その producer がまだない。
- 多角形同士の交差: swept volume 判定 (連続する 2 姿勢の間を掃く形) を入れるときの道具であり、
  その利用者が現れてから足す。フットプリント同士の交差は現状どこにも要求がない。

---

## 6. `create()` の検証

`VelocityLimiter::create()` が `std::nullopt` を返す条件。

| 検査 | 根拠 |
|---|---|
| 全頂点が `allFinite()` | 非有限頂点は `transform` を通っても検出できない |
| `inscribed_radius(footprint).has_value()` | 「n >= 3」「`abs(area) > 1e-12`」「原点が内部」を一度に課す。退化検査を自作しない |
| `is_convex(footprint)` | 凸に限定する (`contains` は凹でも正しいが、内接半径の意味づけが凸を前提にする) |
| `prediction_steps >= 1` | `dt` の除算 |
| `prediction_time` が有限かつ正 | |
| `collision_margin` が有限かつ非負 | |
| `max_deceleration` が有限かつ正 | `v_max = sqrt(2 a d)` の a。0 だと常停止になる |

通過したら footprint を CCW に正規化して保持する。`params()` が返す footprint は
入力と頂点順序が違いうる。

`limit()` は不変オブジェクトの `const` メンバで、内部状態を持たない。navyu の
`navyu_safety_limiter` はメンバの `linear_vel_` を周期をまたいで持ち、一度下がると
上がりにくいラチェットになっていた。値型 + 純関数にすることでこの欠陥を構造的に排除している。

---

## 7. 移植しなかった navyu のデッドコード

| navyu | 状態 |
|---|---|
| `use_radius_foot_print_` / `foot_print_radius_` | 代入のみで参照されていない |
| `CostmapHelper::is_obstacle_in_radius` | 宣言のみで定義がない |
| `Polygon` 構造体の `update_polygon` | 未使用 |

---

## 8. navyu の欠陥との対応表

| # | navyu | 本実装 |
|---|---|---|
| 1 | 後退時に `std::min` が制限にならない (線速度・角速度の両方) | `std::clamp(v_in, -v_max, v_max)` (`src/collision/velocity_limiter.cpp`) |
| 2 | 角速度に別途 `std::min` を掛けるため、負の角速度で曲率が壊れる | 比率 `v_out / v_in` を角速度に掛けるだけにする |
| 3 | `linear_vel_` をメンバに持つラチェット | 値型 + `const limit()`。テスト `IsDeterministicAcrossCalls` |
| 4 | 第 1 段がなく全 step で AABB 走査、占有基準が `cost > 99` で偽陽性 | 二段構え (`check_footprint`)、占有は `cost >= LETHAL_OBSTACLE` |
| 4b | `is_inside_polygon` の許容値が `epsilon` で内部点を約 1 % 取りこぼす (見逃し側) | crossing number + `edge_tolerance = 1e-9` (`contains`) |
| 5 | 予測はオムニ、plant は差動二輪 | `integrate_differential_drive` を共有し `linear.y()` を無視 |
| 6 | `static_cast<int>` による world → cell 変換 | `MapGeometry::world_rect_to_cells()` (floor + 飽和変換) |
| 7 | 弦長で走行距離を測る | `abs(v) * dt` の累積 |
| 8 | マップ外を無視して次 step も判定 | 打ち切って制限しない |
| 9 | 経路なし等で指令を publish しない | 常に `Result` を返す (ゼロ指令も返す) |

---

## 9. ROS ノード化への申し送り

ROS ノード化は後続タスクである。T7 は ROS を使わない統合デモであり、そちらは
`docs/integration-design.md` を参照 (`VelocityLimiter` の最初の閉ループ利用者はそのデモである)。

- ノード名は `safety_limiter` でよい。用途名はノード側に置く。
- 可視化には `Result::predicted_poses` (world 系) と `VelocityLimiter::footprint()` (base 系) を使う。
  後者は `transform(footprint, pose)` で world 系に移すこと。**コアは描画しない。**
- 曲率に応じた減速を足す場合、`v_max` をもう 1 本作って `min` を取る形で合成できる。
  `Result` は 1 つで足り、ゾーンやモードを増やす必要はない。`collision` 側の API は変わらない。
- スキャン点に対する判定 (`footprint_hits_points`) はコストマップを作らない構成で使える。
- 実速度ではなく**要求指令**で予測している。実速度が低くても予測地平は縮まない (navyu 踏襲)。

---

## 10. テスト構成

| ファイル | 内容 |
|---|---|
| `test/core/test_differential_drive.cpp` | 直進 / 純旋回 / 円弧の 1 周閉じ / 円上にいること / 分岐跨ぎ / 正規化 / `linear.y()` 無視 / 後退の鏡像 |
| `test/core/test_polygon.cpp` (追記) | `winding` / `to_counter_clockwise` / `is_convex` / `bounding_box` / 境界・頂点上の `contains` / CW-CCW 一致 |
| `test/map/test_map_geometry.cpp` (追記) | `world_rect_to_cells` の内側 / クランプ / 非交差 / 負座標 / セル境界 / 退化窓 / 空マップ |
| `test/map/test_cost_model.cpp` (追記) | `is_obstacle` が `LETHAL` のみ / 未知セル政策 / 膨張値は `Inscribed` だが非占有 |
| `test/sim/test_simple_simulator.cpp` | 既定構築 / `set_pose` / `update` の戻り値 / **共有積分との厳密一致** / 累積 |
| `test/collision/test_collision_checker.cpp` | `FirstStage` / 8 方位の短絡 / `Circumscribed` の向き依存 / 辺上・頂点上 / クランプ / 各粒度の層 |
| `test/collision/test_velocity_limiter.cpp` | `create` の検証 9 通り / 制限式 (後退の回帰) / 曲率保持 / 素通し / 打ち切り / 決定性 / 頂点順序不変 |
| `test/collision/test_velocity_limiter_closed_loop.cpp` | 前進・後退の停止、各周期の無衝突、停止余裕、純旋回が阻害されないこと |

---

## 11. 可視化 (`examples/`)

可視化はコアに入れない (`AGENTS.md` の依存規則)。C++ の example が CSV / PGM を吐き、
作図は `examples/plot_collision_results.py` が行う。
単体テストが「値が合っているか」を固定するのに対し、可視化は**テストが見ていない範囲**を拾う。

`AGENTS.md` の禁止条項は「core planning/control/map logic に持ち込むな」であって、
matplotlib-cpp は optional 依存として明示的に許可されている。CSV を読むだけの Python スクリプトは
どのターゲットにもリンクされず CMake からも参照されないため、そもそも依存に当たらない。
ただし **CMake のカスタムターゲットにはしない** — 繋いだ時点で Python が実ビルド依存になる。

### 11.1 `eltanin_limiter_profile <output_dir>` (マップ不要)

合成マップ (6 m 角、`resolution = 0.05`) に対して既定パラメータのリミッタを走らせる。

| 出力 | 内容 |
|---|---|
| `velocity_profile.csv` | 壁までの gap に対する `v_out`。**前進と後退の両方**、および navyu の `std::min` が返す値を並べる |
| `closed_loop_forward.csv` | `SimpleSimulator` を plant にした停止走行 (周期 0.1 s) |
| `heading_sweep_diagonal.csv` | `Circumscribed` 帯 (対角 0.354 m) で yaw を 1 度刻みに 1 周させた衝突判定 |
| `heading_sweep_head_on.csv` | `Inscribed` 帯 (正面 0.25 m) の同じ掃引 |
| `bands.pgm` | セルごとの `Free` (0) / `Circumscribed` (128) / `Inscribed` (254) |
| `meta.txt` | パラメータと導出半径 |

**後退バグの可視化 (`velocity_profile.csv`)**: 同じ位置・同じ壁に対し、`yaw = 0` で `v = +0.5`、
`yaw = pi` で `v = -0.5` を与える。どちらも壁に向かう同一の運動なので、符号以外は同じでなければならない。

| gap | `v_out_forward` | `v_out_reverse` | `navyu_forward` | `navyu_reverse` |
|---|---|---|---|---|
| 0.85 | 0.5 | -0.5 | 0.5 | -0.5 |
| 0.80 | 0.447214 | -0.447214 | 0.447214 | **-0.5** |
| 0.70 | 0.316228 | -0.316228 | 0.316228 | **-0.5** |
| 0.60 | 0 | -0 | 0 | **-0.5** |
| 0.40 | 0 | -0 | 0 | **-0.5** |

navyu の列は後退で全域 -0.5 のまま = **制限が一度もかからない**。本実装は前進と厳密に鏡像になる。

**向き依存性の可視化 (`heading_sweep_*.csv`)**: 実測は解析解と一致する。障害物が base 系で
角度 `θ`、距離 `r = 0.3536` にあるとき、0.6 m 角のフットプリントに内包される条件は
`r * max(|cos θ|, |sin θ|) <= 0.3` すなわち `31.9° <= |θ| <= 58.1°`。障害物が 45° 方向にあるので
`yaw ∈ [-13.1°, 13.1°]` (mod 90°) が衝突になる。

| 配置 | 中心セルのコスト | 分類 | 衝突する向き |
|---|---|---|---|
| 対角 (5, 5)、0.354 m | 214 | `Circumscribed` | 360 サンプル中 **108**。`±13.5°` を中心に 90° 周期の 4 弧 (解析値 `±13.1°`) |
| 正面 (5, 0)、0.25 m | 253 | `Inscribed` | **360 / 360** (短絡、向きに依存しない) |

navyu の単一しきい値ではこの図は原理的に全周 1 にしかならない。

### 11.2 `eltanin_limit_on_real_map <map.yaml> <output_dir> [start goal] [obstacle_fraction]`

A* で計画 → `PurePursuit` で追従 → **その指令を `VelocityLimiter` に通し** → `SimpleSimulator` を駆動する。
`obstacle_fraction` (既定 0.5) は**計画後に**経路上へ 0.45 m 角の障害物を置く。プランナが知らない
障害物にリミッタが出会う状況であり、セーフティリミッタ本来の用途にあたる。

出力は `crop.pgm` / `path.csv` / `trajectory.csv` / `predicted.csv` (予測姿勢列) /
`footprint.csv` (world 系フットプリント) / `meta.txt`。

navyu の実マップ (`resolution 0.05`、経路 37.3 m) での実測:

| 構成 | 周期数 | 減速した周期 | 衝突検知した周期 | 最大減速量 |
|---|---|---|---|---|
| 障害物なし (`obstacle_fraction 0`) | 7538 | **0** | 99 | 0 m/s |
| 経路上に障害物 (既定) | 3600 | 55 | 155 | 0.5 m/s (完全停止) |

**障害物なしで減速が 0 回なのは正しい。** 計画経路は `inflation_radius = 0.55` の膨張を避けて通るので、
予測地平 1.0 m 内に衝突が現れても常に 0.7 m 以上先である。`d_col = 0.7 - 0.2 = 0.5` に対する
制動上限は `sqrt(2 * 0.5 * 0.5) = 0.707 m/s` で、要求速度 0.5 m/s を上回るため上限が拘束しない。
**リミッタが通常の追従を阻害しないこと**がこの数字の意味である。

障害物ありでは `v_out` が `0.447 -> 0.316 -> 0` と落ちて停止する。§3.1 が述べた
「`abs(v) * dt = 0.1 m` の量子化による階段状の減速」がそのまま観測できる。停止位置は障害物中心から
0.719 m、フットプリント前端と障害物端の間隔は 0.30 m で、閉ループテストが固定した
`[collision_margin, collision_margin + 2 * abs(v) * dt] = [0.2, 0.4]` の中に入る。

### 11.3 `examples/plot_collision_results.py`

上記 2 本の出力を PNG にする開発ツール。依存は matplotlib と numpy のみ。

```bash
./build/examples/eltanin_limiter_profile /tmp/viz
./build/examples/eltanin_limit_on_real_map map.yaml /tmp/viz-map
python3 examples/plot_collision_results.py --synthetic /tmp/viz --real /tmp/viz-map --out plots
```

`--synthetic` / `--real` は片方だけでもよい。入力が足りなければ「どの example を先に走らせるか」を
示して終了する。生成した PNG はリポジトリにコミットしない (`docs/costmap-design.md` §13-3 の
「バイナリのゴールデンファイルはコミットしない」と同じ理由)。

### 11.4 `examples/real_map_fixture.hpp`

実マップ example 3 本 (`plan` / `track` / `limit`) が共有する部品 (読み込み + 膨張、自動 start/goal、
クロップ、CSV / meta 出力) をここに集約した。抽出は挙動を変えていないことを、
`plan_on_real_map` / `track_on_real_map` の出力が抽出前とバイト一致することで確認している。
