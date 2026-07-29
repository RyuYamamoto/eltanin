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
| 角度 | `angles` パッケージの `normalize_angle` / `normalize_angle_positive` / `shortest_angular_distance` |
| マップ YAML の内容 | nav2_map_server の `LoadParameters` / `load_map_yaml()`。`MapMetadata` は使わない (ROS の `nav_msgs/MapMetaData` は幾何情報を指す語であり、うちの `MapGeometry` がそれに相当する) |
| 範囲検査なしの座標変換 | nav2 の `Costmap2D::worldToMapNoBounds` に合わせ `world_to_map_no_bounds` |

### 9.1 やらないこと

| 禁止 | 理由 |
|---|---|
| 自前の数学定数を公開する (`kPi` など) | C++20 に `std::numbers::pi` がある。標準にあるものを再発明しない |
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
| `map/` | `eltanin_map` / `eltanin::map` | T1 | グリッドマップ、幾何情報型、コスト定数、判定モデル 2 実装、距離 → コスト変換 |
| `map_io/` | `eltanin_map_io` / `eltanin::map_io` | T1 | PGM + YAML 読み込み、PGM 書き出し (yaml-cpp 依存) |
| `map/layers/` | `eltanin_map_layers` | T2 | 膨張処理 (近傍展開により `uint8_t` へ書き込み)、static / obstacle レイヤ、LayeredCostmap、ROI 走査 |
| `sensor/` | `eltanin_sensor` | T3 | Scan 投影 |
| `planner/` | `eltanin_planner` | T4 | グローバル / ローカルプランナ (1 パス探索) |
| `control/` | `eltanin_control` | T5 | 経路追従、`Pose2D` / 角度の補間、累積弧長、線分交差 |
| `safety/` | `eltanin_safety` | T6 | セーフティリミッタ、厳密フットプリント衝突 (多角形の重心 / 凸性 / 符号付き距離 / 交差) |
| `sim/` | `eltanin_sim` | T6 | 簡易シミュレータ |

統合デモ (navyu 相当の動作確認) は T7。

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

ライブラリはすべて STATIC。ヘッダオンリー (INTERFACE) にしない — 宣言と定義の分離が実際にビルドされることを常に検証したいため。ホットループのアクセサはヘッダ内 `inline`、仮想関数は持たせない。

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

## 13. T2 への申し送り

膨張処理 (近傍展開により `uint8_t` へ書き込み) を実装する際の前提。

1. **「向き次第」の帯の検証**: 実マップの膨張結果に対し、`circumscribed_cost <= cost < 253` の帯が**空でなく、かつ有界である**ことを受け入れ条件にする。「3 値すべてが出現すること」では弱い。`circumscribed_cost()` の計算が壊れていると、この帯は空になるか逆に地図全体を覆う。
2. **距離帯とコスト符号化の区別**: 3 章の表を参照。距離では「向き次第」の帯が「必ず衝突」の帯を包含するが、コスト値では 2 つの区間は互いに素である。区別せずに読むと「入れ子になっていること」を検証しようとして混乱する。
3. **ゴールデン比較には `write_pgm()` を使える**: `map_io::write_pgm()` はセル値をそのまま書き出す debug dump であり、`load_map` の逆変換ではない。`253` / `254` / `255` がそのまま出る。膨張結果の回帰テストとデバッグ可視化に使える。読み戻しは `read_pgm()` (しきい値変換を通さない)。
4. **単位**: `InflationCostModel::cost_at_distance()` の引数は [m]。近傍展開でセル単位のユークリッド距離を得たら `resolution` を掛けてから渡す。単位を API 境界で混ぜないこと (navyu の `inflation_layer.hpp` の失敗)。
5. **`world_to_map_no_bounds()` を使う**: ROI 境界をセル座標で計算するとき、マップ外にはみ出した負のインデックスを一度得る必要がある。`MapGeometry::world_to_map_no_bounds()` がそれを提供する (nav2 `Costmap2D::worldToMapNoBounds` と同じ役割)。**自前で `floor` を書かないこと** (座標変換の一元化が破れる)。
6. **`NO_INFORMATION` の扱い**: `CostTraversabilityModel` は `unknown_is_free` フラグで分岐する。膨張処理側でも未知セルの扱いを設定として一元化し、全経路で一貫させること (navyu は内接円分岐で未知の扱いが抜けていた)。
