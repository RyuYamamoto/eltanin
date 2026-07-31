# eltanin コストマップ設計方針

対象: eltanin のコストマップ、衝突判定、およびそれらに依存するプランナ / セーフティ。

この文書は後続タスク (T2 以降) が参照するための**方針の正**である。タスクごとの要件定義は `.plait/` 配下に置かれるが、`.plait/` は `.git/info/exclude` により git の無視対象であり untracked ファイルは新しい worktree に伝播しない。したがって**コストマップに関する設計判断はこの文書に記録する**。

---

## 1. 段階方針

| 段階 | 内容 | 状態 |
|---|---|---|
| 第一段階 | `GridMap<uint8_t>` のみを実データとし、コスト値の比較で三段階の衝突判定を行う | 本文書の 2〜6 章 |
| 将来の拡充 | 距離場 (EDT) の導入、2 パスフォールバック探索による狭路緩和、SDF / rolling window / 勾配利用 | 7 章 |

第一段階で距離場を持たない理由:

- 内接円・外接円を考慮した三段階の判定は、nav2 スケールの `uint8_t` だけで表現できる。距離場が追加するのは量子化回避の精度と FMM / 勾配法系での再利用性であり、どちらも navyu 相当の動作確認 (統合デモ) には不要である。
- タスクは逐次実行される。統合デモ到達までの距離が直列に伸びるため、前倒しした機能は設計判断が現実で検証されるタイミングを後ろにずらす。
- navyu と同じアーキテクチャ (膨張レイヤが存在する構成) を保つことで、出力が食い違ったときに「実装バグか設計差か」を切り分ける基準が残る。

先送りが後日高くつかないための担保は 6 章に記す。

---

## 2. コスト値の値域

nav2 準拠の `uint8_t` スケールを用いる。

| 値 | 名前 | 意味 |
|---|---|---|
| `0` | `FREE_SPACE` | 自由空間 |
| `1` 〜 `252` | (減衰域) | 障害物からの距離に応じて減衰するコスト。`MAX_NON_OBSTACLE = 252` |
| `253` | `INSCRIBED` | 障害物までの距離が内接円半径未満。向きに関係なく衝突 |
| `254` | `LETHAL` | 障害物セル本体 |
| `255` | `UNKNOWN` | 未知 |

- 定数は名前空間内の `constexpr` として定義する。マクロやグローバル `static` 変数にしない。
- **`CIRCUMSCRIBED` を固定定数として定義しない。** 外接円半径に対応するコスト値は減衰関数とスケーリング係数に依存して変わるため、定数化すると `1..252` の減衰域と意味が衝突する。しきい値は**計算する関数**として提供する (5 章)。

---

## 3. 三段階の衝突判定

ロボットのフットプリント多角形から 2 つの半径を導出する。`inscribed_radius < circumscribed_radius` である。

| 半径 | 定義 |
|---|---|
| `inscribed_radius` | ロボット原点から各辺までの距離の最小値 |
| `circumscribed_radius` | ロボット原点から各頂点までの距離の最大値 |

基準点は**ロボット基準座標系の原点** (`base_link` 相当) であり、多角形の重心ではない。

障害物までの距離 `d` に対する意味:

| 距離 | 意味 |
|---|---|
| `d < inscribed_radius` | ロボットの向きに関係なく**必ず衝突** |
| `inscribed_radius <= d < circumscribed_radius` | 向き次第で衝突しうる |
| `d >= circumscribed_radius` | どの向きでも衝突しない |

第一段階ではこの分類を**コスト値の比較**で行う。

| コスト条件 | 分類 | 第一段階での扱い |
|---|---|---|
| `cost >= LETHAL (254)` | 必ず衝突 | 通行不可 |
| `cost >= INSCRIBED (253)` | 必ず衝突 | 通行不可 |
| `cost >= circumscribed_cost` | 向き次第で衝突しうる | **通行不可として扱う** (プランナは 1 パス探索) |
| それ未満 | 通行可 | 通行可 |

`UNKNOWN (255)` は数値上 `LETHAL` より大きいため、単純な `>=` 比較では「必ず衝突」に分類される。未知セルを障害物として扱うか自由として扱うかは設定可能にし、判定モデルで明示的に分岐する。

### 用語についての注意

内接円帯にあるセルは必ず外接円帯にも含まれる (`inscribed_radius < circumscribed_radius` のため)。したがって「外接円コストではなく内接円コストしかないセル」は存在しない。緩和の対象となるのは**外接円帯のうち内接円帯に入っていないセル**である。

### 距離帯は入れ子だが、コスト符号化は互いに素

上の入れ子関係は**距離帯についての記述**である。`uint8_t` に符号化した後は入れ子ではない。

| 領域 | 距離帯 (入れ子) | `uint8_t` 符号化 (互いに素) |
|---|---|---|
| 必ず衝突 | `d < inscribed_radius` | `cost >= 253` |
| 向き次第 | `d < circumscribed_radius` (上を含む) | `circumscribed_cost <= cost < 253` |

距離では「向き次第」の帯が「必ず衝突」の帯を包含するが、コスト値では 2 つの区間は互いに素な分割になる。膨張処理やプランナの実装時に「コスト値でも入れ子になっている」と想定しないこと。

### 量子化による境界の 1 段ずれ (原理的なもの、実装バグではない)

`circumscribed_cost() = cost_at_distance(circumscribed_radius)` と定義すると、距離ベース分類 (`d >= circumscribed_radius` → 通行可) とコストベース分類 (`cost >= circumscribed_cost` → 向き次第) は境界セルで食い違う。`uint8_t` への量子化は非可逆であり、複数の距離が同じコストに丸まるため、この食い違いは**原理的に消せない**。

**決定: `cost >= circumscribed_cost` を「向き次第」とする。すなわち保守側 (通行不可を過大評価する側) に丸める。** 通行可を過大評価するより安全である。距離ベース分類とコストベース分類の結果が境界セルで一致しないことは仕様であり、バグとして追わないこと。

### 単位の規約

**公開 API の距離・半径はすべて [m] とする。セル単位の演算は膨張処理の実装内部に閉じ込める。**

理由は navyu の実例である。`inflation_layer.hpp` の `expand_cost` は `distance = std::hypot(x - mx, y - my)` (セル単位) を計算した後、同じ関数内で `distance * resolution < robot_radius_` (メートル比較) と `cell_inflation_radius < distance` (セル比較) を混在させている。**単位が API 境界を跨いで混ざる構造そのものがバグの温床だった。**

内接円・外接円・膨張半径はフットプリント多角形 (メートル) から導出されるため [m] 統一が自然であり、`resolution` の乗算 1 回は無視できるコストである。

---

## 4. 通行可否は 3 値で表す

通行可否は**3 値の列挙型**で表す。

列挙子名は nav2 の距離帯の語をそのまま使う。造語を作らない。

| 列挙子 | 意味 |
|---|---|
| `Traversability::Free` | どの向きでも衝突しない |
| `Traversability::Circumscribed` | 外接円帯のうち内接円帯外。向き次第で衝突しうる |
| `Traversability::Inscribed` | 内接円帯または障害物セル。必ず衝突 |

第一段階では「向き次第」も通行不可として扱うが、**2 値 (通行可 / 不可) にはしない**。2 値にすると将来「向き次第」を足すのが破壊的変更になる。

---

## 5. 責務の分離

混ぜないこと。

| 責務 | 入力 | 出力 | 第一段階での必要性 |
|---|---|---|---|
| 通行可否の分類 | セル値 (コスト or 距離) + 半径パラメータ | 3 値 | 必要 |
| コスト値の算出 | 距離 + 減衰パラメータ | `uint8_t` コスト | 必要 (膨張処理が使う) |
| 外接円コストしきい値の算出 | 半径 + 減衰パラメータ | `uint8_t` コスト | 必要 (コスト入力の判定モデルが使う) |

「距離からコストを求める」ことと「通行可否を分類する」ことは別の処理である。前者は膨張処理が使い、後者はプランナ / セーフティが使う。

### 5.1 減衰関数 (確定)

**nav2 と同じ指数減衰。アンカーは `inscribed_radius`。** `cost_scaling_factor` の単位・意味は nav2 と同じ (`exp(-scale * Δd)`、`Δd` は [m])。navyu の `3.0` ハードコードと `robot_radius` アンカーは踏襲しない。

| 距離条件 | コスト |
|---|---|
| `d < inscribed_radius` | `INSCRIBED_INFLATED_OBSTACLE (253)` |
| `inscribed_radius <= d <= inflation_radius` | `uint8(MAX_NON_OBSTACLE * exp(-cost_scaling_factor * (d - inscribed_radius)))` |
| `d > inflation_radius` | `FREE_SPACE (0)` |

- `d = inscribed_radius` で `exp(0) = 1` → `252`。`253 → 252` なので単調非増加が保たれる。
- `d = inflation_radius` でコストは 0 にならず、そこを超えた瞬間に 0 へ落ちる。nav2 と同じ挙動 (膨張半径外は触らない) であり、単調非増加は破らない。

#### 境界セルで 252 ではなく 251 になることがある (原理的なもの)

膨張処理はセル距離に `resolution` を掛けて [m] に直してから `cost_at_distance()` に渡す。このとき `hypot(dx, dy) * resolution` は、たとえ数学的に `inscribed_radius` と等しくても、半径リテラルの double 表現と**ビット一致しない**ことがある。

例: `resolution = 0.05`、`(dx, dy) = (3, 0)`、`inscribed_radius = 0.15` のとき `3 * 0.05 = 0.15000000000000000832...` に対し `0.15 = 0.14999999999999999444...` であり、`d < inscribed_radius` が偽になる。減衰域の式が使われ `252 * exp(-10 * 8.3e-18) = 251.99999999999997` が切り捨てられて **251** になる。

これは実装バグではなく `uint8_t` 切り捨ての原理的な帰結である。251 も 252 も同じ `Circumscribed` 帯なので下流の分類には影響しない。**手書きの期待値テストを書くときは、`hypot(dx, dy) * resolution` がどのセルでも半径リテラルと一致しないフィクスチャを選ぶこと。**

### 5.2 `circumscribed_cost()` の下限クランプ (確定)

**`circumscribed_cost()` は `std::max<std::uint8_t>(1, cost_at_distance(circumscribed_radius))` を返す。**

`cost_scaling_factor` が大きいと `252 * exp(-scale * (circumscribed - inscribed))` が 0 に切り捨てられる (例: `scale = 50`、半径差 `0.2 m` → `0.011` → `0`)。しきい値が `0` になると `FREE_SPACE (0)` すら `cost >= 0` を満たし、**マップ全体が「向き次第」= 通行不可**になる。クランプは `FREE_SPACE` が常に「通行可」に落ちることを保証する。

なお `circumscribed_cost()` は `inflation_radius` には依存しない (`circumscribed <= inflation` が不変条件のため、常に減衰域の式が使われる)。`cost_scaling_factor` と `inscribed_radius` にのみ依存する。

---

## 6. 判定モデルという縫い目

第一段階で距離場を持たないことが将来の障害にならないための担保。

### 6.1 契約

判定モデルは次を満たす型とする。

```
classify(セル値) -> 3 値
```

**確定: C++20 concept で表現する。** `include/eltanin/core/traversability.hpp` に置く。

```cpp
enum class Traversability { Free, Circumscribed, Inscribed };

template <class Model, class Cell>
concept TraversabilityModel = requires(const Model & model, Cell cell) {
  { model.classify(cell) } -> std::same_as<Traversability>;
};
```

プランナ / セーフティは「**セル型**」と「**判定モデル**」の 2 つでテンプレート化できる形にする。

**`Traversability` / `TraversabilityModel` / `CollisionRadii` / 半径導出は `core/` に置く。** 要件定義の章立てでは map 側にあるが、`CollisionRadii::classify()` が `Traversability` を返すため map 側に置くと **core → map の逆依存**が生じる。依存は `map → core` の一方向に保つ。

守るべき不変条件は次の一つだけである。

> **判定ロジックは、呼び出し側が型パラメータとして差し替えられること。実装がメンバ関数か自由関数かは問わない。**

「具体型に振る舞いを埋め込まない」のような手段の禁止として書かないこと。テンプレート引数が判定モデルであり、衝突半径パラメータ型は契約を満たす型の一つに過ぎないため、メンバ関数か自由関数かは差し替え可能性に影響しない。

### 6.2 2 つの実装

| 判定モデル | 入力 | 第一段階での用途 |
|---|---|---|
| コスト入力 | `uint8_t` コスト値 | **実際に使う。** `LETHAL` / `INSCRIBED` / 計算された外接円コストしきい値と比較して 3 値を返す |
| 距離入力 | 障害物までの距離 | **使わない。** 衝突半径パラメータ型のメソッドとして持たせる。距離が `inscribed_radius` / `circumscribed_radius` と比較され 3 値が返る |

距離入力モデルを第一段階から実装する理由は、「2 つ目の実装が現れた時点で抽象化する」という原則を第一段階の時点で満たし、将来の距離場導入が**判定モデルの差し替えのみ**で済むことを実証するため。距離場が無くても、手で与えた距離値で単体テストできる。

**T4 (`eltanin_planner`) がこの縫い目の最初の利用者になり、主張が実証された。** `plan()` / `smooth()` / `find_nearest_traversable()` を `Costmap` + `CostTraversabilityModel` と `DistanceMap` + `CollisionRadii` の 2 通りで実体化し、同一の通行可否分類になるコストマップと手で作った距離場が**ビット同一の `Path` を返す**ことをテストで固定した (`test/planner/test_planner_seam.cpp`)。

- `CollisionRadii::classify(double)` に `float` セルを渡す形が concept を満たすため、**`DistanceMap` 側で追加のアダプタは不要**である (`TraversabilityModel<CollisionRadii, float>` が `float → double` の暗黙変換で成立する)。
- プランナは判定モデルの適用を探索の前に全セル 1 回の分類として前置し、探索本体を非テンプレート関数にした。**探索本体はセル型も判定モデルも見ない**ため、上記の一致は構造的に保証される。詳細は `docs/planner-design.md` §2。
- `Map` 側の要件は `CellMap` concept (`value_type` / `geometry()` / `operator()(int, int) const`) として置いた。当初は `include/eltanin/planner/cell_map.hpp` にあったが、**T6 で `eltanin_collision` が 2 人目の利用者になったため `include/eltanin/map/cell_map.hpp` (`eltanin::map::CellMap`) へ移した。**

### 6.3 座標変換と幾何情報の分離

- 幾何情報 (`size_x` / `size_y` / `resolution` / `origin`) を独立した型に切り出す。
- **world ↔ map 変換と範囲検査はこの幾何情報型に属し、セル型に依存しない。** これがセル型を追加したときに座標変換の一元化が壊れない条件である。
- `GridMap<T>` はテンプレートとして維持する。第一段階で実体化するのは `uint8_t` のみだが、`float` / `int32_t` でも実体化できることをテストで確認する。

### 6.4 座標変換の規約

| 項目 | 規約 |
|---|---|
| `map_to_world` | **セル中心**を返す (`origin + (m + 0.5) * resolution`)。セル角基準にしない |
| `world_to_map` | `std::floor` 相当の丸めを用いる。`static_cast<int>` は負値でゼロ方向に切り捨てるため、`origin` より小さい world 座標を範囲内と誤判定する |
| 行順 | `my = 0` の行が `y = origin_y` 側 (下端) に対応する。ROS OccupancyGrid と同じ |
| インデックス | 行優先 `index = mx + size_x * my` |
| 実装箇所 | **幾何情報型が唯一の実装。** 他のモジュールは自前で `(wx - origin_x) / resolution` を書かない |

---

## 7. 将来の拡充

第一段階で意図的に含めなかったもの。なぜやらなかったかと、どう追加するかを記す。

### 7.1 距離場 (EDT) の導入

**なぜ第一段階でやらないか**: 三段階の判定は `uint8_t` だけで表現できる。距離場が追加するのは量子化回避の精度と FMM / 勾配法系での再利用性であり、navyu 相当の動作確認には不要。

**どう追加するか**:

1. `GridMap<float>` を実体化する (テンプレートは第一段階から維持されている)。
2. 障害物セル集合から距離場を生成する。アルゴリズムは **Felzenszwalb-Huttenlocher の分離可能 2 パス法**を第一候補とする。厳密ユークリッド距離が得られ、膨張半径に依存せず `O(N)`。打ち切り (`max_distance` でのクリップ) はオプション。
3. 判定モデルを距離入力版に差し替える (第一段階から実装済み)。
4. `uint8_t` コストマップは距離場からの派生物として生成する (距離 → コスト変換関数は第一段階から存在する)。

**プランナ・セーフティ本体は変更不要**である。テンプレート化された縫い目 (6 章) により、セル型と判定モデルの差し替えだけで済む。

**確認事項**: 4000×4000 の `float` 距離場は 64MB になる。メモリ方針を見直す必要がある。

### 7.2 2 パスフォールバック探索による狭路緩和

**なぜ第一段階でやらないか**: navyu 相当の動作確認に必須ではない純増分。また「緩和経路が向き依存の衝突区間を含むことをどう下流に伝えるか」「向き付き検証をどこで行うか」という論点を引き連れるが、第一段階では判断材料が揃わない。

**どう追加するか**:

| パス | 通行条件 |
|---|---|
| 1 回目 | 「向き次第」を通行不可として扱う (第一段階と同じ) |
| 2 回目 (1 回目が失敗したときのみ) | 「向き次第」を**通行可＋ペナルティ**として扱う |

**型もインタフェースも変わらない。** 3 値分類 (4 章) を第一段階から持っているため、追加されるのは 2 パス目の分岐のみである。これが 3 値にした理由である。

追加時に決める必要があるもの:

- 2 パス目で得た経路が外接円帯を通ることを下流 (制御 / セーフティ) にどう伝えるか。
- 向き付きフットプリント衝突判定をどこで行うか (プランナ内 / セーフティ / 制御)。

**T4 は 1 パス探索として実装した (2 パス化は入れていない)。** `Traversability::Free` のみ通行可、`Circumscribed` / `Inscribed` はともに通行不可である。狭路が閉塞して `nullopt` になる場合があるのは第一段階の仕様である。

**追加時に触る箇所は「近傍展開の通行可否述語 1 本」で足りることを確認した。** プランナは判定モデルを適用した結果を `std::vector<std::uint8_t>` に **3 値のまま**保持しており (2 値に潰していない)、探索本体で通行可否を決めているのは以下の 2 箇所だけである。

| 箇所 | 内容 |
|---|---|
| `src/planner/astar_planner.cpp` の近傍展開 | `grid[neighbor] != FREE` |
| 同ファイルの `free_cell()` | 角抜け判定に使う `grid[index(...)] == FREE` (`in_bounds()` の後段) |

2 パス目はこの述語を「`Free` または `Circumscribed`」に緩め、`Circumscribed` の通過にペナルティを加えた `g` で再探索する分岐を 1 本足すことになる。**型もインタフェースも変わらない。** スムーザ側の通行可否判定 (`path_smoother.hpp` の `is_traversable`) も同じ規則を使っているので、緩和を入れるならここも合わせる。

### 7.3 その他

| 項目 | なぜ第一段階でやらないか | どう追加するか |
|---|---|---|
| 符号付き距離場 (SDF) | 障害物内部の距離を必要とする手法 (最適化系ローカルプランナ) がまだない | 距離場の `float` は負値を表現できる。符号付きかどうかを型または規約で識別する方法を決める |
| rolling window (マップ原点の移動) | ローカルコストマップの実装時に必要性が確定する | 幾何情報型の原点を書き換え、セルデータをシフトする操作を追加する |
| bilinear 補間によるセル値取得 | 距離場の勾配を使う手法がまだない | 距離場導入後に幾何情報型の補間アクセサとして追加する |
| 膨張処理の `O(N)` 化 | 第一段階は navyu と同じ近傍展開方式を採り、比較基準を残す | 7.1 の距離場導入により自動的に `O(N)` になる |

---

## 8. navyu との対応

統合デモで navyu と出力を比較する際の基準。navyu は `int8_t` 0..100 スケール、eltanin は `uint8_t` 0..255 スケールであり、そのままでは値を比較できない。

### 8.1 スケール対応

| navyu (`int8_t`) | eltanin (`uint8_t`) | 意味 |
|---|---|---|
| `-1` | `255` | unknown |
| `0` | `0` | free |
| `1` 〜 `98` | `1` 〜 `252` | 減衰域。navyu の上限は `INSCRIBED_COST - 1 = 98` |
| `99` | `253` | inscribed |
| `100` | `254` | lethal |

navyu の衝突判定は `99 < cost` の単一しきい値 (`costmap_helper.cpp:102`) であり、eltanin の `cost >= INSCRIBED (253)` に相当する。navyu には外接円の概念がないため、「向き次第」に相当する分類は存在しない。

### 8.1.1 コスト値の一致を目指さないこと

上の対応表は値の意味を読み替えるためのものであり、**コスト値を一致させるための換算表ではない。**

- navyu の減衰は `exp(-3.0 * (d - robot_radius)) * 98`。`cost_scaling_factor` が `3.0` にハードコードされている。
- eltanin は `MAX_NON_OBSTACLE = 252` を基準にする。
- さらに**減衰のアンカーが `robot_radius` から `inscribed_radius` に変わる。**

スケール係数を合わせても減衰形状が完全一致する保証はなく、一致させる価値もない。

**比較すべきは通行可否の分類である。** 同一の地図・同一のフットプリントに対して、どのセルが「必ず衝突 / 向き次第 / 通行可」に落ちるかを比較する。これが下流のプランナの挙動を決める実体であり、コスト値はその中間表現に過ぎない。

### 8.2 navyu の符号化を core に持ち込まない

`navyu` の `int8_t` 0..100 スケール (`-1` = unknown) を eltanin の core の表現として持つことはしない。これは修正対象として特定した defect であり、第一級のポリシーとして支えると誤りを型システムに固定してしまう。

- navyu 相当の単一しきい値挙動が必要な場合は、**判定モデルの差し替え**で得る。
- `nav_msgs::OccupancyGrid` との相互変換は ROS ブリッジ層のアダプタとして扱う。core には入れない。

### 8.3 navyu から引き継がない実装上の欠陥

膨張処理 (T2) の実装時に再発させないもの。

| 問題 | 箇所 | 対応 |
|---|---|---|
| 膨張ループが `for (y = min_y; y < max_y; ...)` で `max_y` を含まない (off-by-one, 非対称) | `inflation_layer.hpp` | 近傍展開の範囲を対称にし、境界をテストで固定する |
| unknown (`-1`) の扱いが内接円分岐で抜けており unknown が 99 に上書きされる | `inflation_layer.hpp` | unknown の扱いを膨張処理の設定として一元化し、全経路で一貫させる |
| `cost_scaling_factor = 3.0` がハードコード | `inflation_layer.hpp` | 減衰関数のパラメータとして外部化する |
| world ↔ map 変換が 5 箇所に重複実装 | `base_global_planner.hpp`, `costmap_helper.cpp` (2 箇所), `dynamic_layer.hpp`, `inflation_layer.hpp` | 幾何情報型を唯一の実装とする (6.4) |
| 内包判定が内角和と `epsilon` の比較で数値的に不安定 | `costmap_helper.cpp:53-71` | crossing number など頑健な方式を用いる |
| 外接円の概念がなく `robot_radius` 単一パラメータのみ | `inflation_layer.hpp` | フットプリント多角形から 2 つの半径を導出する (3 章) |

---

## 9. 命名規約

型名・関数名は既存の 2D ナビゲーションスタック (nav2 / base_local_planner / teb_local_planner) で通用する語彙を用い、**造語を避ける**。該当する既存語がない場合のみ説明的な名前を使う。

| 対象 | 規約 |
|---|---|
| 半径・膨張関連 | `inscribed` / `circumscribed` / `inflation_radius` / `cost_scaling_factor` は nav2 のコストマップの語をそのまま使う |
| コスト定数 | nav2 の `FREE_SPACE` / `MAX_NON_OBSTACLE` / `INSCRIBED_INFLATED_OBSTACLE` / `LETHAL_OBSTACLE` / `NO_INFORMATION` を**そのまま採用**する。本文中の `INSCRIBED` / `LETHAL` / `UNKNOWN` は説明上の短縮表記 |
| フットプリント近似 | 種類を将来増やす場合は teb_local_planner の `CircularRobotFootprint` / `TwoCirclesRobotFootprint` / `PolygonRobotFootprint` に寄せる |
| レイヤ | nav2 の `StaticLayer` / `ObstacleLayer` / `InflationLayer` / `LayeredCostmap` に寄せる |
| 3 値の列挙子 | nav2 の距離帯の語 `Free` / `Circumscribed` / `Inscribed` (4 章) |
| 角度 | `angles` パッケージの `normalize_angle` / `normalize_angle_positive` / `shortest_angular_distance`。角度範囲判定 `angle_in_range(angle, from, to)` は `angles` に該当語がないため、本節冒頭の但し書き (該当する既存語がない場合のみ説明的な名前を使う) を適用した例である。範囲型 `AngleRange` も同じ理由で `core/angle.hpp` に置く (T3) |
| マップ YAML の内容 | nav2_map_server の `LoadParameters` / `load_map_yaml()`。`MapMetadata` は使わない (ROS の `nav_msgs/MapMetaData` は幾何情報を指す語であり、うちの `MapGeometry` がそれに相当する) |
| クランプする座標変換を将来足す場合 | nav2 の `Costmap2D::worldToMapEnforceBounds` に合わせる (13 章 5 項) |

### 9.1 やらないこと

| 禁止 | 理由 |
|---|---|
| 自前の数学定数を公開する (`kPi` など) | C++20 に `std::numbers::pi` がある。標準にあるものを再発明しない |
| 範囲検査を省いた座標変換を公開する | 呼び出し側がクランプを忘れると範囲外アクセスに直行する。`world_to_map()` の `optional` か、クランプ済みを返す形にする |
| 定数の `k` 前置 (`kInflationRadius`) | navyu を含むこのプロジェクトの既存コードに前例がない。定数は `UPPER_SNAKE` |
| 他言語由来の API 名 (`get_or` = Rust、`make_like` = numpy) | C++ / ROS の語彙で書く。`get().value_or()` や既存コンストラクタで足りる |
| 依存ライブラリの型に別名を付ける | `Eigen::Vector2d` はそのまま使う。`Vec2` のような短縮別名を挟むと出自が読めなくなる |

### 9.2 公開範囲の規約

**利用者が現に存在しないものを public にしない。** 「T2 が使うかもしれない」で API を先に生やさない。必要になったタスクが足す。

| 状況 | 置き場所 |
|---|---|
| 外部から呼ぶもの | `include/eltanin/<module>/` の公開名前空間 |
| テストのためだけに見せる必要があるもの | 同じヘッダの `detail` 名前空間 (例 `map_io::detail::occupancy_cost`) |
| 実装専用 | `.cpp` 内の無名名前空間 |

`operator()` / `operator[]` のように危険だが利益が明確なものは公開する (10 章の方式 A)。利益の説明ができないものは公開しない。

---

## 10. エラー通知方式 (確定)

方式を 1 つに統一するのではなく、**状況の 3 分類に対して方式を 1 つずつ割り当てる規則**を統一する。後続タスクはこの規則に従うこと。

| 分類 | 方式 | 適用先 |
|---|---|---|
| A. 前提条件違反 (呼び出し側のプログラミングエラー) | `assert` + 未定義動作。戻り値で表現しない | `GridMap::operator()` / `operator[]`、`MapGeometry::index()` / `map_to_world()` |
| B. 正常系の一部としての「該当なし」 | `std::optional` | `MapGeometry::world_to_map()`、`inscribed_radius()` / `circumscribed_radius()`、`CollisionRadii::from_*()`、`InflationCostModel::create()`、`GridMap::get()` |
| C. 外部入力・設定の不正 | 例外 (`map_io::MapIoError`) | `map_io::read_pgm()` / `write_pgm()` / `load_map_yaml()` / `load_map()` |

理由:

- **A**: セルアクセスは 4000×4000 = 1.6e7 セルのホットループで呼ばれる。`optional` や例外を毎セル通すのは無駄で、範囲検査は呼び出し側がループ境界で一度行うのが自然 (nav2 の `Costmap2D::getCost` も同じ立場)。既定ビルドでは `CMAKE_BUILD_TYPE` を空にしてあるため `NDEBUG` が定義されず、開発中は `assert` が実際に発火する。安全版として `get()` / `set()` を併設する。
- **B**: 「world 座標がマップ外」「ロボット原点が多角形外なので内接円半径が定義できない」は異常ではなく問い合わせの答えである。`optional` なら呼び出し側が無視できない。
- **C**: マップ読み込みは起動時 1 回でホットパスでない。失敗理由を伝える必要があり `optional` では落ちる。`std::expected` は C++23 のため使えない。

**例外を投げるのは `map_io` のみ。`eltanin_core` / `eltanin_map` は例外を投げない。**

`MapIoError` は理由を機械可読にするため列挙型を持つ。`include/eltanin/map_io/error.hpp` に置く。

```cpp
enum class MapIoErrorKind {
  FileNotFound, YamlParseError, MissingKey, InvalidValue, UnsupportedMode,
  UnsupportedOriginYaw, PgmBadMagic, PgmBadMaxval, PgmSizeMismatch, PgmTruncated, WriteFailed
};
class MapIoError : public std::runtime_error {
public:
  MapIoErrorKind kind() const noexcept;
};
```

### 10.1 命名の規約: `at()` を使わない

標準ライブラリの `at()` は例外を投げる契約なので、前提条件版 (方式 A) に使うと期待を裏切る。前提条件版は `operator()` / `operator[]`、安全版 (方式 B) は `get()` / `set()` と名前で区別する。

---

## 11. 依存規則

`AGENTS.md` の規則を絶対制約とする。

- コア (型・幾何・グリッドマップ・判定モデル) が依存してよいのは **C++ 標準ライブラリと Eigen のみ**。
- ROS 2 / Rerun / matplotlib-cpp / Python をコアに持ち込まない。
- マップの読み書き (yaml-cpp 依存)、可視化、テスト・ベンチマークは**コアとは別のターゲット**に隔離する。

---

## 12. 想定モジュール構成

タスクは逐次実行される。**空のモジュールを先置きしない。** 各タスクは自分のモジュールディレクトリと `CMakeLists.txt` を作り、ルートの `CMakeLists.txt` に `add_subdirectory` を 1 行追加する。逐次実行のためこの 1 行追加は誰とも競合しない。

ルート `CMakeLists.txt` はプロジェクト設定・オプション・依存探索・`add_subdirectory` 列挙・install/export のみを持つ。**個別モジュールのソース列挙をルートに書かない。**

| モジュール | ターゲット | 担当 | 内容 |
|---|---|---|---|
| `core/` | `eltanin_core` / `eltanin::core` | T1 | 型・角度・多角形・フットプリント半径・経路 |
| `map/` | `eltanin_map` / `eltanin::map` | T1 / T2 | グリッドマップ、幾何情報型、コスト定数、判定モデル 2 実装、距離 → コスト変換、コストマップレイヤ (static / obstacle / inflation) と `LayeredCostmap` |
| `map_io/` | `eltanin_map_io` / `eltanin::map_io` | T1 | PGM + YAML 読み込み、PGM 書き出し (yaml-cpp 依存) |
| `sensor/` | `eltanin_sensor` / `eltanin::sensor` | T3 (完了) | Scan 投影 (`ScanData` / `ScanFilter` / `project_scan`)。**依存は `eltanin_core` のみ** (tf / laser_geometry / PCL / ROS を持たない)。設計は `docs/sensor-design.md` |
| `planner/` | `eltanin_planner` / `eltanin::planner` | T4 (グローバルのみ完了 / ローカルは未着手) | 8 近傍 A* グローバルプランナ (1 パス探索)、最近傍通行可セル探索、反復平滑化スムーザ。**依存は `eltanin_core` / `eltanin_map` のみ**。設計は `docs/planner-design.md` |
| `control/` | `eltanin_control` / `eltanin::control` | T5 (完了) | Pure Pursuit 経路追従 (`PurePursuitParams` / `PurePursuit`)。**依存は `eltanin_core` のみ** (コストマップを見ない)。設計は `docs/control-design.md`。`Pose2D` / 角度の補間・累積弧長・線分交差は汎用ユーティリティなので `eltanin_core` の既存ヘッダに置いた |
| `collision/` | `eltanin_collision` / `eltanin::collision` | T6 (完了) | 二段構えのフットプリント衝突判定 (セル / 点群 / 多角形の各粒度) と制動距離則による速度制限 (`VelocityLimiter`)。**依存は `eltanin_core` / `eltanin_map` のみ**。設計は `docs/collision-design.md`。ディレクトリ名は用途名 (`safety/`) ではなく機構名にした — 「セーフティ」は T7 の ROS ノード側の語彙である。多角形の頂点順序 / 凸性 / AABB は汎用なので `eltanin_core` に置いた。重心 / 符号付き距離 / 多角形交差は利用者がないため未実装 (§9.2、理由は `docs/collision-design.md` §5.1) |
| `sim/` | `eltanin_sim` / `eltanin::sim` | T6 (完了) | 差動二輪の簡易 plant (`SimpleSimulator`)。**依存は `eltanin_core` のみ**。積分の純関数 `integrate_differential_drive` は `core` に置き、予測と plant が共有する |

統合デモ (navyu 相当の動作確認) は T7 で完了した。`examples/navigate_demo` が全モジュールを 1 プロセスで
閉ループとして回し、`test/integration` が回帰テストにしている。設計は `docs/integration-design.md`。
**ライブラリには何も足していない** — 既存 API のまま全系が回ることを示すのが目的だったからである。

**T2 のレイヤは独立ターゲット `eltanin_map_layers` にせず `eltanin_map` に統合した。** レイヤが使うのは `Costmap` / `MapGeometry` / `InflationCostModel` ですべて `eltanin_map` の中身であり、新ターゲットを切っても依存境界は 1 つも増えず、install / export の対象が増えるだけである。ヘッダは `include/eltanin/map/layers/`、ソースは `src/map/layers/` に置き、`src/map/CMakeLists.txt` の `add_library` に相対パスで列挙する。

### 12.1 ヘッダとソースの物理配置

ヘッダとソースはルートに集約する。

```
include/eltanin/<module>/*.hpp
src/<module>/CMakeLists.txt + *.cpp
test/<module>/CMakeLists.txt + test_*.cpp
```

include パスがヘッダの物理パスと一致し (`#include <eltanin/core/types.hpp>`)、ヘッダの install はルートの `install(DIRECTORY include/ ...)` 1 回で済む。

### 12.2 後続タスクの導線

1. `include/eltanin/<module>/` にヘッダを置く。**install 記述は触らなくてよい** (ディレクトリ一括)。
2. `src/<module>/CMakeLists.txt` を作る。既存モジュールのファイルをそのまま雛形にできる。ターゲットは `eltanin_<module>` + `eltanin::<module>` ALIAS、`EXPORT_NAME <module>`、`install(TARGETS ... EXPORT eltaninTargets)`。
3. `target_compile_options(<tgt> PRIVATE ${ELTANIN_WARNING_FLAGS})` を付ける (ルートで定義済み、`ELTANIN_ENABLE_WERROR` に追従する)。
4. ルート `CMakeLists.txt` に `add_subdirectory(src/<module>)` を 1 行足す。
5. `test/<module>/CMakeLists.txt` を作り、`test/CMakeLists.txt` に `add_subdirectory(<module>)` を 1 行足す。テストはモジュールごとに 1 実行ファイルにまとめ、`gtest_discover_tests(... PROPERTIES ENVIRONMENT "${ELTANIN_TEST_ENVIRONMENT}")` で登録する。

ライブラリはすべて STATIC。ヘッダオンリー (INTERFACE) にしない — 宣言と定義の分離が実際にビルドされることを常に検証したいため。ホットループのアクセサはヘッダ内 `inline` とし、**そこには仮想関数を持たせない**。

**モジュール境界の仮想ディスパッチは明示的に許可する。** `Layer::update_costs()` は更新周期 (1〜5 Hz) × レイヤ数 (3) しか呼ばれず、内側の約 1.2e7 回のセル書き込みは非仮想の実装内部で行われる。「仮想関数を持たせない」規約はセルアクセサ (`GridMap::operator()` / `operator[]`、`MapGeometry` の変換) に限定される。

### 12.3 ビルドオプション

| オプション | 既定 | 用途 |
|---|---|---|
| `ELTANIN_BUILD_TESTS` | トップレベル時 ON | GoogleTest による単体テスト |
| `ELTANIN_BUILD_EXAMPLES` | OFF | `examples/` |
| `ELTANIN_ENABLE_WERROR` | OFF | `-Werror` |
| `ELTANIN_ENABLE_ASAN` | OFF | AddressSanitizer |
| `ELTANIN_ENABLE_UBSAN` | OFF | UndefinedBehaviorSanitizer |
| `ELTANIN_TEST_MAP_DIR` | navyu の `map/` | 実マップテストの入力。不存在なら `GTEST_SKIP` |

サニタイザはルートで**ディレクトリスコープ**の `add_compile_options` / `add_link_options` により付与する。INTERFACE ライブラリ経由にすると `install(EXPORT)` がそのライブラリのエクスポートを要求するため。UBSan は既定で abort しないので、`gtest_discover_tests` の `ENVIRONMENT` に `UBSAN_OPTIONS=halt_on_error=1` を設定してある。

**`CMAKE_BUILD_TYPE` の既定値は設定しない。** 空のままなら `NDEBUG` が定義されず方式 A の `assert` が有効になる。Release では前提条件チェックが無効になる。

### 依存規則の再掲

`map_io` のみが yaml-cpp に依存する。`core` / `map` は C++ 標準ライブラリと Eigen のみ。

`yaml-cpp` は `eltanin_map_io` の **PRIVATE** 依存だが、STATIC ライブラリのため `eltaninTargets.cmake` の `INTERFACE_LINK_LIBRARIES` に `$<LINK_ONLY:yaml-cpp::yaml-cpp>` が書き出される。したがって `cmake/eltaninConfig.cmake.in` は `find_dependency(yaml-cpp)` を呼ぶ必要がある。これを外すと外部プロジェクトのリンクが未定義シンボルで落ちる。

---

## 13. T2 への申し送り (T2 で決着済み)

膨張処理 (近傍展開により `uint8_t` へ書き込み) を実装する際の前提。**6 項目すべての決着は 14 章に記す。**

1. **「向き次第」の帯の検証**: 実マップの膨張結果に対し、`circumscribed_cost <= cost < 253` の帯が**空でなく、かつ有界である**ことを受け入れ条件にする。「3 値すべてが出現すること」では弱い。`circumscribed_cost()` の計算が壊れていると、この帯は空になるか逆に地図全体を覆う。
2. **距離帯とコスト符号化の区別**: 3 章の表を参照。距離では「向き次第」の帯が「必ず衝突」の帯を包含するが、コスト値では 2 つの区間は互いに素である。区別せずに読むと「入れ子になっていること」を検証しようとして混乱する。
3. **ゴールデン比較には `write_pgm()` を使える**: `map_io::write_pgm()` はセル値をそのまま書き出す debug dump であり、`load_map` の逆変換ではない。`253` / `254` / `255` がそのまま出る。膨張結果の回帰テストとデバッグ可視化に使える。読み戻しは `read_pgm()` (しきい値変換を通さない)。
4. **単位**: `InflationCostModel::cost_at_distance()` の引数は [m]。近傍展開でセル単位のユークリッド距離を得たら `resolution` を掛けてから渡す。単位を API 境界で混ぜないこと (navyu の `inflation_layer.hpp` の失敗)。
5. **ROI をセル座標で必要としたら `MapGeometry` に足す**: 公開されている world → cell 変換は `world_to_map()` (範囲外は `nullopt`) **だけ**である。範囲検査なしの生変換は公開していない。負のインデックスをそのまま返す API は、呼び出し側がクランプを忘れたときに範囲外アクセスへ直行するため、利用者が現れる前に置かない方針にした (9.2)。

   T2 が「world 座標の窓をクランプ済みのセル矩形に変換する」操作を必要としたら、**`MapGeometry` にメンバとして追加する**こと。nav2 の `Costmap2D::worldToMapEnforceBounds` に相当する形 (常にマップ内へクランプして返す) か、矩形ごと返す形 (窓がマップと交差しなければ `nullopt`) のいずれかが素直である。後者の方が「クランプ忘れ」が起こりえないぶん安全。

   **T6 で決着した。** フットプリントの AABB をセル矩形に直す利用者が現れたため、後者の形で `MapGeometry::world_rect_to_cells(min, max) -> std::optional<CellRect>` を追加した。既存の private な `floor_to_index()` / `to_int_saturating()` を再利用するので、負座標の切り捨て方向と int 範囲外・NaN の防御が `world_to_map()` と共有される。`min <= max` は `assert` (呼び出し側の計算ミスであり設定不正ではない)。

   **`MapGeometry` の外で `std::floor((wx - origin_x) / resolution)` を書かないこと。** これを許すと navyu の 5 箇所重複が再発する。変換の実装が `MapGeometry` の中にしか無い状態を維持する。
6. **`NO_INFORMATION` の扱い**: `CostTraversabilityModel` は `unknown_is_free` フラグで分岐する。膨張処理側でも未知セルの扱いを設定として一元化し、全経路で一貫させること (navyu は内接円分岐で未知の扱いが抜けていた)。

---

## 14. T2 で確定した事項 (コストマップレイヤと膨張処理)

`include/eltanin/map/layers/` の `Layer` / `StaticLayer` / `ObstacleLayer` / `InflationLayer` と `include/eltanin/map/layered_costmap.hpp` の `LayeredCostmap` に関する決定。

### 14.1 レイヤ境界の規約

- **レイヤは master のセル値だけを書き換える。ジオメトリを変更してはならない。** `GridMap::set_origin()` が公開されているため型システムでは防げない。規約として守り、3 レイヤすべてと `LayeredCostmap::update()` の前後で `MapGeometry::operator==` を確認するテストで固定する。
  - レイヤに制限ビュー型 (セル書き込みのみ許し `const MapGeometry &` を返すラッパ) を渡す案は却下した。利用者が 1 つしかない型を増やすことになり 9.2 に反する。nav2 も `Layer::updateCosts(Costmap2D &, ...)` で同じ緩さを許している。
- **origin の更新は容器 (`LayeredCostmap`) の責務であり、レイヤの責務ではない。** navyu は `dynamic_layer` の中で `master_costmap.info.origin` を書き換えていた。`LayeredCostmap` は master への非 const アクセサを公開せず、これを型で支える。
- レイヤの適用順は登録順。**`InflationLayer` を最後に置くのは利用者の責務**であり、順序を型で強制しない。強制すると `LayeredCostmap` が膨張の存在を知る必要が生じる。
- 3 レイヤとも公開コンストラクタ + `assert` で構築する (10 章の分類 A)。空の `Costmap` は `load_map()` の正常経路では生じず、呼び出し側のプログラミングエラーだからである。工場関数 (`create()` → `optional`) にすると `unique_ptr` 所有と噛み合わず `Costmap` のムーブが 2 回起きる。
- レイヤは `LayeredCostmap::add_layer<LayerType>(args...)` で容器内に構築し、返された参照で毎サイクルのデータ供給 (`ObstacleLayer::set_points()`) を行う。返す参照は `unique_ptr` の指す先なので `LayeredCostmap` をムーブしても無効化せず、生存期間の規則は「`LayeredCostmap` の寿命の間有効」の 1 行で済む。

### 14.2 `LayeredCostmap` のリセット値とロボット追従

- **リセット値は構築時パラメータ `default_cost`。** グローバル (`StaticLayer` が全セルを書く) には `NO_INFORMATION`、ローカル (`ObstacleLayer` のみ) には `FREE_SPACE` を渡す。固定値にするとどちらかが必ず不適切になる。navyu は `data.clear()` + `resize()` のゼロ埋めに依存しており初期値が暗黙だった。
- `update()` は ① master 全体を `default_cost` で `fill` ② 登録順に `update_costs(master)`。
- **origin 更新でセルデータのシフトを行わない。** `update()` が master を全面再生成するため不要。
  - **成立の前提: master に更新サイクルを跨いだ状態を持つレイヤが存在しないこと。** 将来この前提を破るレイヤ (減衰する障害物メモリ等) を足す場合はシフトが必要になる。7.3 の rolling window は保持を伴う場合の話である。
- `center_on(robot)` は `origin = robot - (size * resolution) / 2` を計算して `set_origin()` に委譲する。navyu が `dynamic_layer` の中に持っていた計算をここ 1 箇所に置く。
  - **origin を `resolution` の格子にスナップしない。** ロボット位置に連続追従するので、更新のたびにセル境界が微小にずれる。毎サイクル master を全面再生成する現設計では無害だが、**連続する 2 サイクルのコストマップをセル単位で比較する利用者が現れたら破綻する**。その時点で nav2 の rolling window と同様に origin を格子へ丸めること。

### 14.3 `StaticLayer`

- **master の `MapGeometry` を絶対に上書きしない。** navyu `static_layer.hpp` の `master_costmap = map_;` は global costmap 側で設定した width / height / resolution / origin をすべて地図側の値に置換していた。
- ジオメトリが `operator==` で一致する場合はセル列を一括コピーする高速路を採る。
- 一致しない場合は master の各セルについて `master.geometry().map_to_world()` → `source.geometry().world_to_map()` で最近傍セルを引く。`nullopt` (ソース地図の外) のセルは**触らない**ので `LayeredCostmap` のリセット値が残る。この経路により、ローカル / rolling なコストマップでも `StaticLayer` がそのまま使える。
- 値は無条件コピーで、既存値との `max` 合成はしない。static は土台であり最初に適用される。

**危険: 最近傍サンプリングはソース地図が master より細かいと障害物を落とす。** master の 1 セルが覆うソースセルのうち中心 1 個しか読まないためである。実測では 4×4 / `resolution 0.05` の `LETHAL` 3 セルを 2×2 / `resolution 0.1` へ写すと 2 セルしか残らない。想定用途 (グローバル = ジオメトリ一致の高速路、ローカル = 同解像度で原点だけ違う) はいずれも 1:1 対応なので発生しないが、**master をソースより粗くしてはならない**。粗い master が必要になったら、覆うソースセル群の `max` で集約する形に変えること (保守側に倒れる)。この挙動はテストで固定してある。

### 14.4 `InflationLayer`

- 膨張元は master の値が `LETHAL_OBSTACLE` のセルのみ。**`NO_INFORMATION` は膨張元にしない。** 実マップは 96 % が未知であり、膨張元に含めると自由空間の大半が潰れる。
- 膨張窓は `r = ceil(inflation_radius / resolution)` セルとして `[mx - r, mx + r] × [my - r, my + r]` の**両端を含む対称窓**。マップ境界では `max` / `min` の整数クランプを行う。navyu は `for (y = min_y; y < max_y; y++)` で上端を除外し `-r 〜 +r-1` の非対称窓になっていた。
- **窓のクランプはセル空間の整数演算であり world → cell 変換を伴わない。** したがって 13 章 5 項の「ROI をセル座標で必要としたら `MapGeometry` に足す」には該当せず、`MapGeometry` への追加は不要である。
- **帯の分岐をレイヤ側で再実装しない。** 内接円帯 (253) も減衰域も `cost_at_distance()` の 1 回の呼び出しで得られる。navyu の unknown 欠陥は「内接円分岐と else 側で unknown の扱いが食い違った」ことなので、**分岐を 1 本に保つことが根本対策**である。
- **「膨張半径外」の判定を `cost_at_distance()` と同一の述語に一本化する。** 距離 LUT の構築時に `distance = hypot(dx, dy) * resolution` を 1 回だけ求め、`distance > inflation_radius` を窓外判定に、同じ `distance` を `cost_at_distance()` の引数に使う。窓外は `std::int16_t` の `-1` で標識する。
  - セル空間で `dx² + dy² <= (inflation_radius / resolution)²` を比較する形は却下した。`cost_at_distance()` 内部の `distance > inflation_radius` と**別の浮動小数点式**になり、境界セルで判定が食い違いうる。
  - 「`cost_at_distance()` が `FREE_SPACE` を返したら窓外と見なす」形も却下した。「窓の外」と「膨張半径内だが減衰値が切り捨てで 0」を混同する。`max` 合成経路では差が出ないが、`inflate_unknown == true` の置換経路では後者が `255 → 0` の書き込みになるため差が出る。
- 合成規則 (全経路で一貫させる):
  - 旧値が `NO_INFORMATION` かつ `inflate_unknown == false` → **書き込まない** (`255` を保持)。
  - 旧値が `NO_INFORMATION` かつ `inflate_unknown == true` → 算出コストで**置換**する。**算出コストが 0 でも置換する** (`cost_scaling_factor` が大きいと減衰値が切り捨てで 0 になり、未知セルが `FREE_SPACE` になる)。これは意図した挙動でありテストで固定している。
  - それ以外 → `max(旧値, 算出コスト)`。
  - **`255` が数値上の最大値であることに依存した暗黙の unknown 保護をしない。** `max` に任せると `inflate_unknown == true` が実装不能になり、意図がコードから読めない。明示分岐にする。
- **単一 in-place パスで正しい。** 膨張が書く最大値は `INSCRIBED_INFLATED_OBSTACLE (253)` であり、膨張元の条件である `LETHAL_OBSTACLE (254)` より小さい。unknown 置換経路も `255 → cost (<= 253)` なので 254 を作らない。よって走査中に膨張元が破壊も新規生成もされず、16 MB のコピーを持たずに済む。
- **結果は走査順に依存しない。** `max` は可換・結合的であり、unknown 経路も「保持」なら常に 255、「置換」なら 2 回目以降は `max` と一致するため、最終値は全膨張元にわたる `max` になる。
- 距離 LUT は `(r+1) × (r+1)` の `std::int16_t` を持ち、**解像度が変わったときだけ再構築する**。`InflationCostModel` は構築後不変なので再構築の契機は解像度だけである。実マップの `r = 11` では 288 バイト。
- **`resolution == 0` の入口を閉じる。** 既定構築の `MapGeometry` は `resolution_ = 0` であり、`ceil(inflation_radius / 0.0)` = `inf` を `int` にキャストすると未定義動作になる (UBSan が検出する)。`LayeredCostmap` のコンストラクタと `InflationLayer::update_costs()` の先頭で `assert` する。
- **注意**: `inflation_radius / resolution` が極端に大きい設定 (例 `inflation_radius = 100`, `resolution = 0.05` → `r = 2000`) では LUT が 8 MB になり、内側ループが 1 膨張元あたり 1.6e7 セルになる。これは設定の妥当性の問題であり `assert` では防がない。

### 14.5 `ObstacleLayer`

- world 座標の点列を `std::span<const Eigen::Vector2d>` で受け、**内部の `std::vector` にコピーして保持する**。`span` はビューであり、保持するとダングリングを招く。点列未設定は正常状態 (観測前) であり、`update_costs()` は何もしない。
- 変換は `MapGeometry::world_to_map()` のみを使う。navyu `dynamic_layer.hpp` の `static_cast<int>((x - origin_x) / resolution)` は負値をゼロ方向に切り捨てるため origin より小さい world 座標を範囲内と誤判定していたが、`floor` + 飽和キャストなので構造的に起こらない。
- `NO_INFORMATION` のセルも無条件に `LETHAL_OBSTACLE` で上書きする。観測は未知に対する情報の増加である。

### 14.6 13 章の申し送り 6 項目の決着

| # | 申し送り | 決着 |
|---|---|---|
| 1 | 「向き次第」の帯が空でなく有界であることを受け入れ条件にする | 実マップに対し `0 < circumscribed_count < 631664` (膨張前の `FREE_SPACE` セル数) をテストで固定した。実測は 56,217 |
| 2 | 距離帯とコスト符号化の区別 | 3 章の記述を維持。分類は `CostTraversabilityModel` に任せ、テスト側で入れ子を前提とした集計をしない |
| 3 | ゴールデン比較に `write_pgm()` を使える | 小グリッドの膨張結果を `write_pgm()` → 独立リーダで読み戻し、期待バイト列と厳密比較する。バイナリのゴールデンファイルはコミットしない |
| 4 | 単位: `cost_at_distance()` の引数は [m] | LUT 構築時に `* resolution` して [m] に統一 (14.4) |
| 5 | ROI をセル座標で必要としたら `MapGeometry` に足す | **不要だった。** 膨張窓のクランプは world → cell 変換を伴わない整数演算である (14.4) |
| 6 | `NO_INFORMATION` の扱いを膨張処理側でも設定として一元化 | `inflate_unknown` (既定 `false`) に一元化し、全経路で一貫させた (14.4) |

### 14.7 実マップに対する実測値 (回帰の基準)

`navyu_navigation/map/map.pgm` (4000×4000 / `resolution 0.05` / 膨張前 free 631,664・lethal 22,952・unknown 15,345,384) に対し、矩形フットプリント `(±0.22, ±0.15)` / `inflation_radius = 0.55` / `cost_scaling_factor = 10.0` / `inflate_unknown = false` / `unknown_is_free = false` で膨張した結果。

| 項目 | 値 |
|---|---|
| 膨張窓の半径 `r` | 11 セル |
| `circumscribed_cost()` | 78 |
| `Traversability::Inscribed` | 15,411,582 セル |
| `Traversability::Circumscribed` | 56,217 セル |
| `Traversability::Free` | 532,201 セル |

`Inscribed` の大半は `NO_INFORMATION` の 15,345,384 セルである。この地図は 96 % が未探索であり、`unknown_is_free = false` では未知が通行不可に分類されるため妥当な数値である。

---

## 15. ROS ノード構成への申し送り (T3 で記録)

想定している最終形は次のとおり。

- `local_map_node`: ray trace により障害物セルと自由セルを計算したローカルマップを出す (必要なら距離場も同時に出す)。
- `global_costmap_node`: 全域マップを読み込んで膨張コストマップを出す。`local_map_node` の出力を受けたら global マップに対する overlap 領域を計算し、`map_updates` として出力する。
- 距離場は global / local を融合して出力する。

**T3 の実装スコープには含めていない。** ray trace の利用者がまだ存在せず、追加は 9.2 節 (利用者が現に存在しない public API を生やさない) に反するためである。以下は次にコストマップまたは ROS 層を触るタスクへの前提であり、いずれも 14 章の決定の**成立条件**に関わる。

### 15.1 ray trace を入れるとローカルの既定コストは `FREE_SPACE` ではなくなる

14.2 節の「ローカル (`ObstacleLayer` のみ) には `FREE_SPACE` を渡す」は**「ray trace がない」前提付きの決定**である。ray trace で自由セルを計算するなら、ローカルマップは未観測 / 自由 / 障害物の 3 値を区別しなければならない。

- 既定コストは `NO_INFORMATION` に変わり、自由空間は ray trace の結果として書かれる。
- `FREE_SPACE` のままにすると、**global 側の overlap 合成が「センサ視野外の静的障害物」を消す**。壁の陰・センサの死角・角度フィルタで落としたセクタがすべて「観測された自由空間」として global へ伝わるためである。navyu が持っていなかった欠陥を新たに作ることになる。
- 帰結として、ローカル側の合成規則 (`ObstacleLayer` が `NO_INFORMATION` を無条件に上書きする。14.5 節) と ray trace のクリア規則の優先順位を決める必要がある。同一セルに「あるビームの終端 (障害物)」と「別のビームの通過 (自由)」が同時に立つため、**障害物を優先する**のが保守側である。

### 15.2 global を毎周期全面再生成できない → `Layer` に ROI が必要になる

`Layer::update_costs(Costmap & master)` に境界引数がない。`LayeredCostmap::update()` は master 全面 `fill` + 全レイヤ適用である。4000×4000 = 1.6e7 セルをスキャンレートで回すのは成立しない。

- nav2 が `updateBounds(...)` → `updateCosts(master, min_i, min_j, max_i, max_j)` の 2 段に分けているのはこの理由である。この構成を採るなら **`Layer` インタフェースへの ROI 引数の追加**が必要になる。T2 が意図的に持たなかった API なので、変更する場合は 14.1 節のレイヤ境界の規約と併せて再検討する。
- ROI は**前サイクルのローカル窓と今サイクルのローカル窓の和**を覆う必要がある。前回の動的障害物を消すためである。
- 14.2 節の「origin 更新でセルデータのシフトを行わない」の成立前提は「master に更新サイクルを跨いだ状態を持つレイヤが存在しないこと」である。global が動的障害物を蓄積する設計にすると、この前提を破る。蓄積せず「静的 + 最新のローカル 1 枚 + 膨張」を毎回作り直す形なら前提は保たれる。

### 15.3 `map_updates` の領域は overlap を膨張半径ぶん広げる

`r = ceil(inflation_radius / resolution)` セルとして、

| ROI | 範囲 | 理由 |
|---|---|---|
| 書き込み (publish する領域) | 変化領域 ⊕ `r` | 窓端に現れた障害物の膨張が切れないため |
| 読み出し (膨張元として走査する領域) | 変化領域 ⊕ `2r` | ⊕`r` の各セルのコストを正しく再計算するには、そのセルから `r` 以内の全 `LETHAL_OBSTACLE` セルが必要 |

overlap 領域そのものを送ると、購読側で窓端の膨張が欠ける。14.4 節で潰した navyu の非対称窓と同種の off-by-`r` である。

### 15.4 距離場の min 融合は厳密。ただし障害物集合の定義を揃えること

`d(x, A ∪ B) = min(d(x, A), d(x, B))` なので、global 距離場とローカル距離場の `min` による融合は**近似ではなく厳密**である。成立条件は 2 つ。

1. ローカル距離場を毎周期ゼロから作り直す。`min` は距離を減らす方向にしか働かないため、古い動的障害物が残ると消えない。
2. **障害物集合の定義を global / local で一致させる。** 14.7 節のとおり参照マップは 96 % が `NO_INFORMATION` であり、unknown を障害物に含めると global 距離場がほぼ全域 0 になって融合結果が壊れる。`CostTraversabilityModel` の `unknown_is_free` と同じ設定を距離場の生成側でも一元化すること (13 章 6 項と同じ論点)。

**距離場専用ノードは作らない方向を第一候補にする。** 融合距離場が必要なのはローカル窓の範囲だけで、global 側は静的なので一度計算すれば済む (グローバルプランナは global 距離場を、ローカル側は融合場を使う)。専用ノードを置くと、7.1 節が指摘する 64 MB の `float` 全域距離場を ROS 越しに流す必要が出るか、あるいは結局ローカル窓サイズの出力になり `local_map_node` に置くのと同じで hop が 1 増えるだけになる。

### 15.5 clearing 用の投影

`project_scan` はマーキング専用であり ray trace のクリアリングには使えない。センサ側の帰結なので `docs/sensor-design.md` に記録した。
