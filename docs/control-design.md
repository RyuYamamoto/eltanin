# 経路追従 (`eltanin_control`) の設計

対象は `include/eltanin/control/pure_pursuit.hpp` / `src/control/pure_pursuit.cpp` の
`eltanin::control::PurePursuit` と、T5 で `eltanin_core` に追加した 4 つの汎用ユーティリティ。

移植元は navyu の `navyu_path_tracker` (`navyu_path_tracker.cpp` / `navyu_path_tracker.hpp`) と
`navyu_utils` (`normalized_radian`)。

命名・公開範囲・エラー通知・依存の規約は `docs/costmap-design.md` §9〜§12 に従う。

---

## 1. 純関数化の契約

`PurePursuit` は ROS を知らない値型である。1 周期ぶんの指令を作る操作を次の 1 本に閉じた。

```cpp
Result compute(const Pose2D & robot, const Path & path, double dt);
```

| 契約 | 理由 |
|---|---|
| 時刻を内部で取らず `dt` を引数で受ける | navyu は速度積分の刻み幅にパラメータ `update_frequency` を使っていた (`.cpp:130-133`)。タイマ遅延・ジッタがあると積分が実時間とずれる。`dt` を受ければ呼び出し側が実経過時間を渡せる |
| `path` を `const Path &` で受け、変更しない | navyu はゴール到達を `path_.poses.clear()` = **入力データの破壊**で表現していた (`.cpp:98`)。経路の所有権と寿命は呼び出し側にあり、追従器が壊す対象ではない |
| 到達は `Status::GoalReached` で返す | 経路のクリアと再計画の判断は呼び出し側 (ROS ノード) の責務。追従器は状態を報告するだけにする |
| `NoPath` / `GoalReached` でも必ずゼロ指令を返す | navyu は経路なし・自己位置取得失敗のとき**指令を publish せずに return** していた (`.cpp:78` / `.cpp:82-85`)。駆動器には直前の `cmd_vel` が残り続ける。ゼロ指令を返せばブリッジ層は素直に publish するだけで安全側になる |
| 内部状態は `nearest_index_` / `linear_vel_` / `yaw_aligned_` | 進捗 index を単調にして自己交差や終端周回で過去区間へ戻らない。経路を差し替える呼び出し側は `reset()` する |
| 全非静的メンバに既定メンバ初期化子を付ける | navyu は `double v_;` / `double w_;` が**未初期化のまま** `gain_ * v_` として読まれていた (`.hpp:96-97`)。未定義動作である |
| 設定不正は `create()` → `std::optional` | `docs/costmap-design.md` §10 の分類 B。`InflationCostModel::create()` と同じ形。例外は投げない |
| `dt` / `robot` の不正は `std::invalid_argument` | Releaseビルドでも検証を失わず、`Status` に入力不正の状態を増やさない |
| `compute()` はヒープ確保をしない | navyu は毎周期 `std::vector<double>` に全経路点との距離を積んでいた (`.cpp:88-93`)。100 Hz × 727 点で毎周期の確保になる |

既定構築は許していない (private コンストラクタ)。したがって `params()` が返す値は必ず検証済みである。
コピー・ムーブは暗黙生成で、`create()` が `std::optional` に入れて返せる。

### 1.1 `angular_vel_` をメンバにしなかった

navyu の `w_` に対応するメンバは置いていない。**この設計では角速度が周期をまたいで読まれない**ため、
メンバに置くと「実際には状態でないものが状態に見える」。レート制限のような機能があるかのような誤読を招く。

真に状態なのは `nearest_index_` (経路進捗)、`linear_vel_` (1 次遅れの積分器)、
`yaw_aligned_` (向き合わせのラッチ) の 3 つである。
角速度を観測したいだけなら `Result::command.angular` が返っている。

---

## 2. パラメータ名は nav2 の語彙に寄せた

`docs/costmap-design.md` §9 の「既存スタックで通用する語彙を用い造語を避ける」に従う。既定値は navyu と同一。

| navyu 名 | eltanin 名 | 既定値 | 意味 |
|---|---|---|---|
| `limit_v_speed` | `desired_linear_vel` | 0.5 [m/s] | 直進速度の目標値。1 次遅れの収束先かつクランプ上限 |
| `limit_w_speed` | `max_angular_vel` | 1.0 [rad/s] | 角速度の対称上限 |
| `yaw_tolerance` | `yaw_tolerance` | 0.07 [rad] | 初期の向き合わせを終える方位誤差 |
| `gain` | `lookahead_time` | 0.1 [s] | 先行距離の速度比例項。navyu の `gain` は名前から意味が読めない |
| `look_ahead_const` | `min_lookahead_dist` | 0.3 [m] | 先行距離の定数項。曲率の分母の下限も兼ねる |

先行距離の式は navyu を踏襲する。

```
lookahead_distance = lookahead_time * linear_vel_ + min_lookahead_dist
```

navyu の直書き定数 2 つは `.cpp` の無名名前空間の `UPPER_SNAKE` 定数にした。**パラメータに昇格させない**
(§9.2「利用者が現に存在しないものを public にしない」)。値は navyu の挙動を保つため変えていない。

| 名前 | 値 | navyu の該当箇所 |
|---|---|---|
| `LINEAR_VEL_GAIN` | `1.0` [1/s] | `.cpp:130` の `a = 1.0 * (limit_v_speed_ - v_)` |
| `ALIGNMENT_ANGULAR_VEL_RATIO` | `0.5` | `.cpp:124` の `sign * limit_w_speed_ * 0.5` |
| `MIN_TARGET_DISTANCE` | `1e-9` [m] | 該当なし (navyu は退化検査を持たない) |

---

## 3. 角速度式を navyu から変えた

navyu は次の式を使っていた (`.cpp:134`)。

```
w = v * sin(alpha) / look_ahead_distance     // navyu 形
```

これには 2 つの誤りがある。**曲率の係数 2 が欠落**しており、分母が**実際の目標点距離ではなく公称先行距離**である。
eltanin は教科書形にした。

```
w = 2 * v * sin(alpha) / max(d, min_lookahead_dist)     // eltanin
```

`d` は選ばれた目標点までの実距離である。

### 3.1 誤りが効く大きさ (解析)

半径 `R` の円弧を先行距離 `L` (`L << R`) で追従するときの定常横方向誤差 `e` を求める。ロボットが円の外側 `e` に
居て接線方向を向いているとし、目標点までの弦長を `L` とすると `sin α ≈ L/(2R) + e/L` である。

- **教科書形** `κ = 2 sin α / L`: `1/R + 2e/L² = 1/(R+e) ≈ 1/R - e/R²` より `e = 0`。1 次近似では定常誤差ゼロ。
- **navyu 形** `κ = sin α / L`: `1/(2R) + e/L² = 1/R - e/R²` より `e ≈ L²/(2R)`。**外側にオフセットする。**

ただし**実測では教科書形も定常誤差ゼロではない。** 残差は `e ~ L²/(16 R)` (navyu 形の 1/8) でスケールする。
1 次近似の結論をそのまま「誤差ゼロ」と読んではいけない。

### 3.2 定常誤差のスケール則 (実測)

`test/control/tracking_fixture.hpp` の差動二輪運動学 (明示オイラー、`dt = 0.01 s`、点間隔 0.05 m、
加速度制限・遅延・ノイズなし) で半径 2.0 m・90° の円弧を追従したときの、走行 1.0 m 以降の最大横方向誤差 [m]。

| `min_lookahead_dist` | 実効 `L` | 教科書形 (実測) | `L²/(16R)` | navyu 形 (実測) | `L²/(2R)` |
|---|---|---|---|---|---|
| 0.15 | 0.20 | 0.00097 | 0.00125 | 0.01043 | 0.01000 |
| 0.20 | 0.25 | 0.00163 | 0.00195 | 0.01723 | 0.01563 |
| **0.30 (既定)** | **0.35** | **0.00351** | 0.00383 | **0.03500** | 0.03063 |
| 0.45 | 0.50 | 0.00767 | 0.00781 | 0.07279 | 0.06250 |
| 0.60 | 0.65 | 0.01379 | 0.01320 | 0.12380 | 0.10563 |

半径依存 (教科書形、既定パラメータ) も `1/R` で確認した。`e * R` が一定になる。

| `R` [m] | 1.0 | 2.0 | 3.0 | 5.0 | 10.0 |
|---|---|---|---|---|---|
| 誤差 [m] | 0.00718 | 0.00351 | 0.00233 | 0.00139 | 0.00070 |
| `e * R` | 0.00718 | 0.00702 | 0.00698 | 0.00694 | 0.00697 |

**既定値では navyu 形が 3.5 cm、教科書形が 0.35 cm で 10 倍の差がある。**
`docs/planner-design.md` §6.1 の実測によれば実マップの A* 経路は横方向に 1.62 cm ずれると
`Circumscribed` 帯 (向き次第で衝突する領域) に入る。navyu 形の定常オフセットはこの予算を単独で超える。
式を変えた根拠はここにある。

---

## 4. 実測した追従誤差 (回帰の基準)

条件は §3.2 と同じ。すべて `GoalReached` に到達している。

| ケース | 初期姿勢 | 全区間の最大 [m] | 1.0 m 走行以降の最大 [m] | 終端 [m] |
|---|---|---|---|---|
| 直線 5 m (a) | 経路上 `yaw = 0` | 1.4e-17 | 0.0 | 0.0 |
| 直線 5 m (b) | 横 0.2 m オフセット `yaw = 0` | 0.2 (初期値) | **0.01350** | 1.1e-08 |
| 直線 5 m (c) | 経路上 `yaw = 0.5 rad` | 0.00790 | **0.00052** | 1.5e-08 |
| 円弧 R=2 m 90° 左 | 始点・接線方向 | 0.00351 | **0.00351** | 0.00351 |
| 円弧 R=2 m 90° 右 | 始点・接線方向 | 0.00351 | **0.00351** | 0.00351 |
| 直線 5 m (b) navyu 形 | 横 0.2 m オフセット | 0.2 (初期値) | 0.05774 | 1.4e-04 |
| 円弧 R=2 m navyu 形 | 始点・接線方向 | 0.03500 | 0.03500 | 0.02664 |

- (a) の誤差は丸め誤差のみ。`alpha` が厳密に 0 になるため角速度も厳密に 0 で、横方向誤差が発生しない。
- (b) は初期の向き合わせ (その場旋回) の後に前進し、**1 回だけ 1.35 cm 行き過ぎて収束する**。
- 円弧は最大値と終端値が一致する = 過渡なしで定常誤差に単調漸近する。ゲートは 1.0 m で足りる。
- 全ケースで `command.linear.x() <= desired_linear_vel`、`|command.angular| <= max_angular_vel` を満たす。

### 4.1 テストに固定した閾値

`test/control/test_pure_pursuit_tracking.cpp` の名前付き定数。実測値に 1.5〜2 倍の余裕を持たせてある。

| 検査量 | 閾値 | 実測 |
|---|---|---|
| 直線 (a) 全区間の最大 | `1e-12` m | 1.4e-17 |
| 直線 (b) 1.0 m 以降の最大 | 0.020 m | 0.01350 |
| 直線 (b) 終端 | `1e-3` m | 1.1e-08 |
| 直線 (c) 1.0 m 以降の最大 | 0.002 m | 0.00052 |
| 直線 (c) 全区間の最大 | 0.012 m | 0.00790 |
| 円弧 左右 1.0 m 以降の最大 | 0.006 m | 0.00351 |
| navyu 形の同一指標 (欠陥が再現していること) | `> 0.020` m | 0.03500 |
| 教科書形 / navyu 形の比 | `< 1/3` | 0.100 |
| `GoalReached` 時のゴール残距離 | `<= spacing/2 + 1e-9` | 0.024992 (`spacing/2 = 0.025`) |

**navyu 形との数値一致は目指していない。** 角速度式を意図的に変えたため原理的に一致しない。
比較するのは追従誤差の大きさである。`NavyuFormReference` (テストのフィクスチャ内) は同じシミュレータで
navyu 形を走らせるためのもので、閾値をマジックナンバーにせず「欠陥の再現に対する相対比較」にしている。
**製品コードに navyu 形は残していない。**

---

## 5. `docs/planner-design.md` §6.1 への回答

§6.1 は「経路の横方向マージンは実マップで 1.62 cm しかなく、追従誤差の予算をどこから取るかは T5 が決める」
という申し送りを残していた。回答は次のとおり。

**T5 は横方向誤差を吸収しない。** 追従器は誤差の実測値を出す立場であり、マージンの配分は統合側の判断である。

合成経路 (直線・円弧) に対する誤差の内訳は 2 成分に分かれる。

| 成分 | 既定パラメータでの実測 | 依存性 |
|---|---|---|
| 滑らかな経路に対する定常誤差 | R=2 m の円弧で **0.35 cm** | `min_lookahead_dist` の 2 乗、曲率半径の逆数に比例 (§3.2) |
| 外乱に対する過渡 | 20 cm の初期横ずれで **1.35 cm** の行き過ぎ | 初期偏差の大きさに依存。定常項とは別枠 |

**しかし実マップの経路ではこのどちらも支配項ではない。** 決めているのは経路の**曲率半径**である (§5.1)。

### 5.1 実マップでの実測 (支配項は経路の曲率半径である)

`examples/eltanin_track_on_real_map` で A* → スムーザ → Pure Pursuit を通した実測。
参照マップ `navyu_navigation/map/map.pgm`、矩形フットプリント `(±0.22, ±0.15)`、`inflation_radius = 0.55`、
自動選択の start / goal (727 点 / 37.2 m、§6.1 が測ったものと同じ経路)、既定パラメータ、`dt = 0.01 s`。

| 指標 | 実測 (旧スムーザ既定) | 実測 (現行) |
|---|---|---|
| 経路自身の非 `Free` 点 | **0 / 727** | **0 / 727** (プランナは仕様どおり) |
| 追従軌跡の最大横方向誤差 | 5.51 cm | **4.93 cm** |
| 軌跡が `Circumscribed` 帯に入ったサンプル | 60 / 7538 (0.80 %) | **82 / 7531 (1.09 %)** |
| 軌跡が `Inscribed` 帯 (必ず衝突) に入ったサンプル | **0** | **0** |
| ゴール残距離 | 2.45 cm | 2.34 cm |
| 経路の曲率半径 中央値 / 最小 | 1.473 m / 0.134 m | **3.866 m / 0.288 m** |
| 曲率半径が 1.0 m を下回る折れ点 | 24 / 63 | 28 / 106 |

**合成円弧の 0.35 cm に対して実マップは 4.93 cm で、14 倍ある。** 原因は経路の曲率である。
先行距離 `L = 0.35 m` は経路の最小曲率半径より大きいので、Pure Pursuit は角を内側に切り、
その後外側へ行き過ぎる。**片側へのオフセットではなく角での振動**である。

`docs/planner-design.md` §13.14 でスムーザの既定重みを `0.5 / 0.3` から `0.1 / 0.4` に変えた結果、
曲率半径の中央値が 2.6 倍になり最大横方向誤差が 5.51 → 4.93 cm に下がった。一方で角を内側に切る量が
増えるため `Circumscribed` 帯に入るサンプルは 60 → 82 に増えている (`Inscribed` は 0 のまま)。
折れ点の総数が 63 → 106 に増えるのは、平滑化で多くの点がわずかに非共線になるためで、1 つ 1 つは緩い。

`max_angular_vel` は原因ではない。既定では 1.000 rad/s に張り付くが、上限を 2.0 / 4.0 に上げても
要求値は 1.113 rad/s までしか伸びず、**誤差は 4 桁一致で変わらない**。

### 5.2 パラメータを振った実測 (T7 への選択肢)

同じ経路・同じ指標。`circ` は `Circumscribed` 帯に入ったサンプル数 (総サンプル数が速度で変わるため
割合も併記する)。

| パラメータ | 最大誤差 [m] | `circ` | 割合 |
|---|---|---|---|
| **既定** (`v` 0.5 / `w` 1.0 / `mld` 0.30) | 0.0551 | 60 | 0.80 % |
| `mld` 0.20 | 0.0607 | 8 | 0.11 % |
| `mld` 0.15 | 0.0687 | **0** | 0 % |
| `mld` 0.10 | 0.0831 | **0** | 0 % |
| `v` 0.3 | 0.0500 | 93 | 0.74 % |
| `v` 0.2 | 0.0476 | 134 | 0.72 % |
| `v` 0.1 | 0.0450 | 256 | 0.69 % |
| `w` 2.0 / `w` 4.0 | 0.0551 | 60 | 0.80 % |
| `v` 0.2 / `w` 2.0 / `mld` 0.15 | **0.0202** | **0** | 0 % |
| `v` 0.1 / `w` 2.0 / `mld` 0.10 | **0.0117** | **0** | 0 % |

読み方に注意が要る点が 2 つある。

- **`min_lookahead_dist` を下げると最大誤差は増えるのに帯への侵入は消える。** 先行距離を短くすると角の
  内側への切り込みが減り (障害物に近い側の誤差が減り)、代わりに外側への行き過ぎが増える。
  外側には余裕があるので侵入は消える。**「最大誤差」と「衝突リスク」は同じ量ではない。**
- **速度を下げても侵入割合はほとんど改善しない** (0.80 % → 0.69 %)。先行距離が `0.1 v + 0.3` で
  速度にわずかしか依存しないためである。単独で効く操作ではない。

### 5.3 結論 (T7 の統合に渡すもの)

**実マップの余裕 1.62 cm は、既定パラメータの追従誤差 5.51 cm に対して足りない。** 選択肢は次のとおり。

1. **`min_lookahead_dist` を 0.15 に下げる** (最小の変更で `Circumscribed` 侵入が 0 になる)。
   ただし最大誤差は 6.9 cm に増えるので、経路の外側にも余裕が要る。
2. **`min_lookahead_dist` 0.15 + `desired_linear_vel` 0.2 + `max_angular_vel` 2.0 の組合せ** で
   最大誤差 2.0 cm・侵入 0 になる。速度を 2.5 分の 1 にする代償を払う。
3. **`CollisionRadii` を膨らませる**。ただし必要量は 4 cm 前後 (5.51 cm − 1.62 cm) で、狭所を通れなくなる。
4. **曲率に応じた減速を入れる** (nav2 RPP の regulated 部分)。T5 のスコープ外とした項目 (要件 §3.2-4) だが、
   **実マップの結果はこれが本命であることを示している。** 角で自動的に減速すれば、直線区間の速度を
   落とさずに 2 と同じ効果が得られる。`eltanin_safety` (T6) か `control` の拡張のどちらに置くかは T6 の判断。
5. **プランナ側で最小曲率半径を保証する**。8 近傍 A* + 反復平滑化は 0.133 m の折れ点を残す。
   ここを 0.5 m 程度まで丸めれば追従誤差は合成円弧の水準に近づく。プランナ側の別タスク。

**`Inscribed` 帯 (必ず衝突する領域) への侵入は 0 である。** つまり既定設定でも「必ず衝突する」状態には
至っていない。入っているのは `Circumscribed` = 向き次第で衝突しうる帯である。したがって緊急度は
「必ず直す」ではなく「T7 の統合で 1〜5 のどれを採るか決める」である。

---

## 6. 制約と前提

| # | 制約 |
|---|---|
| C-1 | `dt` の上限は 2 つある。**向き合わせの収束条件** `dt < yaw_tolerance / (ALIGNMENT_ANGULAR_VEL_RATIO * max_angular_vel)` (既定値で 0.14 s) を破ると、1 周期の旋回量が許容帯を飛び越え続けて**永久に前進しない**。もう 1 つは 1 次遅れの安定条件 `LINEAR_VEL_GAIN * dt <= 1` で、破ってもクランプが効くので発散はしない。`create()` は `dt` を知らないため検証できない。`dt = 0.1 s` で整合が完了することをテストで固定してある |
| C-2 | **ゴール到達判定は経路点間隔に依存する。** 「最近傍点が末尾の点」で判定するため、最終線分の中点を越えた時点で成立する。点間隔 `s` の経路では最大 `s/2` 手前で止まる (実測 0.024992 m / `s/2 = 0.025`)。距離許容値パラメータは追加していない (navyu 踏襲) |
| C-3 | **最終姿勢合わせを持たない。** T4 が経路末尾に載せた `goal.yaw` は T5 では使わない。要求 goal の向きに合わせる旋回が必要なら後続タスクが足す。**T5 では持たない。E-10 の `GoalApproach` が引き取った (§12)** |
| C-4 | **経路点の `yaw` を使わない。** 目標点の方位は `atan2` で作る (navyu 踏襲)。経路の `yaw` を信頼する設計は利用者が現れてから |
| C-5 | **最近傍 index は単調増加する。** 保存済み index 以降だけを探索するため、終端付近や自己交差で過去区間へ吸着しない。逆走や大きな自己位置ジャンプで過去区間へ戻す用途は対象外 |
| C-6 | **新しい経路を渡すときは呼び出し側が `reset()` を呼ぶ。** `yaw_aligned_` は経路の入れ替えを検知できないため、新しい経路でも向き合わせが再開しない。`NoPath` / `GoalReached` では自動でリセットされる |
| C-7 | 後退・カスプ・全方向移動には対応しない。速度は `[0, desired_linear_vel]`、`Twist2D::linear.y()` は常に 0 (差動二輪) |
| C-8 | 実測速度を受け取らない。`compute()` が持つのは指令値であり、状態推定・遅延補償・速度フィードバックは範囲外 (navyu も同じ) |
| C-9 | スレッド安全性は呼び出し側の責務。既存モジュールと同じ立場 |
| C-10 | 経路点の座標は有限であることを前提とする。`path` の各点の有限性は検査しない |
| C-11 | コストマップを見ない。障害物での減速・衝突予測は `eltanin_safety` (T6) の担当 |

---

## 7. 退化ケースの扱い

| # | 状況 | 挙動 |
|---|---|---|
| 1 | `path` が空 | `Status::NoPath`、ゼロ指令、状態リセット |
| 2 | `path` が 1 点 | `Status::GoalReached`、ゼロ指令、状態リセット |
| 3 | 目標点がロボット位置と一致 (重複点を含む経路) | `MIN_TARGET_DISTANCE = 1e-9 m` 未満なら `alpha = 0` とする。`std::atan2(0, 0)` は 0 を返すため、検査なしでは `alpha = -robot.yaw` という誤った値になる |
| 4 | 目標点距離が小さい | 曲率の分母を `min_lookahead_dist` で下限クランプする。ゴール直前に `d` が縮んで曲率が発散するのを塞ぐ。曲率を**過小**に見積もるので安全側 |
| 5 | `alpha == 0` かつ未整合 | 整合済みとして直進フェーズへ進む。`sign(0)` を評価する経路が存在しない |
| 6 | 終端点が `min_lookahead_dist` 内で横・後方へ回る | 線速度を 0 にし、終端点の方向へその場旋回する。前進しながら終端点の周囲を回る軌道を作らない |
| 7 | `dt` が大きく `LINEAR_VEL_GAIN * dt > 1` | 1 次遅れが行き過ぎるが `[0, desired_linear_vel]` のクランプで収まる |
| 8 | ロボットが経路から大きく離れている | 保存済み index 以降の最近傍点へ向かう。距離上限による失敗は返さない |

### 7.1 向き合わせが完了した周期はそのまま前進する

navyu は `|alpha| < yaw_tolerance` で `adjust_yaw_angle_` を立てた**同じ周期でも旋回指令を返す** (`.cpp:122-127`)。
さらに `alpha == 0` のとき `sign = -1` になるため、整合した瞬間に**逆向きの旋回指令**を 1 周期出す。

eltanin はラッチを立てたらそのまま直進フェーズに落ちる。踏襲する契約は
「`yaw_tolerance` に入るまで旋回のみ」であり、これは満たしている。
100 Hz で 1 周期の差は挙動の同一性を損なわない一方、「整合した後に逆向きへ回る」指令は説明できない。

### 7.2 クランプの順序

`linear_vel_` のクランプを角速度の計算**より前**に行う。navyu は順序が逆で、クランプ前の `v_` から `w_` を
計算していた (`.cpp:137-139`)。また navyu の `std::clamp(v_, -limit_v_speed_, limit_v_speed_)` は
`v_` が 0 から単調増加するため負側が働かない。eltanin は `[0, desired_linear_vel]` にする。

---

## 8. `eltanin_core` に追加した 4 ユーティリティ

T1 から移送された汎用ユーティリティ。**`eltanin::control` には置かない。** 対応する既存関数
(`path_length` / `normalize_angle` / `closest_point_on_segment`) がすべて `core` にあり、
T6 (`eltanin_safety`) の多角形交差からも使える。`control` に置くと T6 が `control` に依存することになる。

既存の同種関数と同居させ、新しいヘッダは作っていない。

| 関数 | 場所 | 契約 |
|---|---|---|
| `interpolate_angle(from, to, t)` | `core/angle.hpp` (inline) | 最短方向を通り、結果は `(-pi, pi]`。`t` は `[0, 1]` にクランプ。`shortest_angular_distance` の上に組む |
| `interpolate_pose(from, to, t)` | `core/types.hpp` (inline) | 位置は線形、`yaw` は `interpolate_angle`。`t` のクランプ規約は同じ |
| `cumulative_arc_length(path)` | `core/path.hpp` / `src/core/path.cpp` | 要素数は `path.size()`、`[0] = 0`、単調非減少。空経路は空の `vector` |
| `segment_intersection(a1, a2, b1, b2)` | `core/geometry.hpp` / `src/core/geometry.cpp` | 閉じた線分の交点。平行・共線・退化はすべて `nullopt` |

`t` のクランプ規約は `closest_point_on_segment` (パラメータを `[0, 1]` にクランプする) に合わせた。

**対向 (角度差が厳密に `pi`) の扱い**: `shortest_angular_distance` の値域が `(-pi, pi]` なので `+pi` 側 =
反時計回りが選ばれる。決定的である。

**`cumulative_arc_length` は `path_length` と同じ加算順序・同じ式で計算する。** これにより
`path.size() >= 1` のとき `back() == path_length(path)` が `EXPECT_DOUBLE_EQ` で成立する。
`path_length` の実装をこちらに置き換えることはしない (確保を持ち込まないため)。

**値返しにした理由**: ホットループの利用者が現在いない。必要になったら
`sensor::project_scan(scan, filter, out)` と同じ out 引数版を足せる。

**`segment_intersection` の平行判定は相対しきい値で行う。** 外積を `|r| * |s|` で正規化し `1e-12` と比べる。
座標の絶対値に依存する生の比較にすると、1e6 倍にスケールした座標で判定が変わる。
退化線分 (`r` または `s` が零ベクトル) は外積 0 かつ右辺 0 となり平行分岐に落ちるので、独立検査を足していない
(分岐を 1 本に保つ)。

- **共線の重なり区間は返さない。** 単一点で表せないため `nullopt` にする。
- **`[0, 1]` は閉区間で厳密比較する。** 端点接触の判定は丸め誤差の影響を受けうるので、テストは二進で厳密に
  表せる座標 (整数・0.5) を使っている。実数座標での端点接触に依存する利用者が現れたら、許容値付きの別関数を
  検討すること。

**T5 時点の利用者はテストのみである** (`interpolate_pose` / `segment_intersection`)。
想定利用者は T6 の多角形交差。先行距離の円と経路線分の交点で目標点を選ぶ方式へ切り替える案もあるが、
実測で**点間隔 0.02〜0.20 m の間で定常誤差がほぼ変わらない**ため利得が小さい。移行は別タスク。

---

## 9. 検証しなかったこと

**`compute()` がヒープ確保をしないことを自動テストにしていない。** グローバル `operator new` を差し替えると
テスト実行ファイル全体 (GoogleTest 自身の確保を含む) に影響し、ASan の `new` インターセプトと同居させる
必要がある。得られる保証は「`compute()` が確保しない」だけで、これは実装を読めば確認できる
(`std::vector` を持たず、メンバはスカラ 2 個 + パラメータ構造体)。費用が保証を上回るため入れていない。

最近傍探索は距離の 2 乗で比較する (`squaredNorm`)。平方根を取らずに済み、同順位の解決 (厳密 `<` で最小 index)
も変わらない。目標点選択は先行距離との比較なので `norm()` を使う。

---

## 10. navyu の欠陥との対応表

| # | navyu の箇所 | 欠陥 | eltanin での扱い |
|---|---|---|---|
| 1 | `.hpp:96-97` | `v_` / `w_` が未初期化。初回 `process()` で読まれる (未定義動作) | 全メンバに既定メンバ初期化子 + `reset()` + 初回決定性テスト |
| 2 | `.cpp:98` | ゴール到達を `path_.poses.clear()` = 入力データの破壊で表現 | `Status::GoalReached` を返す。`path` は `const Path &` + 不変性テスト |
| 3 | `.cpp:78`, `.cpp:82-85` | 経路なし / 自己位置取得失敗で**指令を publish せず return**。駆動器に直前指令が残る | `NoPath` でも必ずゼロ指令を返す。自己位置取得は呼び出し側の責務 |
| 4 | `.cpp:134` | 角速度の曲率係数 2 が欠落し、分母が公称先行距離 | 教科書形に修正 (§3) + 円弧テストで固定 |
| 5 | `.cpp:137-139` | クランプ前の `v_` から `w_` を計算 | クランプを角速度計算の前に行う (§7.2) |
| 6 | `.cpp:104`, `.cpp:134` | 先行距離が 0 になりうるが除算前の検査がない | `min_lookahead_dist > 0` を `create()` で検証 + 分母の下限クランプ |
| 7 | `.cpp:130-133` | 刻み幅に実経過時間ではなくパラメータ `update_frequency` を使う | `dt` を引数で受ける。`compute()` は時刻を取らない |
| 8 | `.cpp:88-93` | 毎周期 `std::vector<double>` をヒープ確保 (100 Hz × 727 点) | 最小値と index のみ保持し確保しない |
| 9 | `.cpp:107-109` | `int` index と `size_t` の signed/unsigned 比較 | index はすべて `std::size_t`。`-Werror` で検証 |
| 10 | `.cpp:122-127` | 向き合わせの `0.5` 直書き。整合した周期に逆向き旋回を 1 周期出す | 非公開定数化 + 整合周期はそのまま直進へ (§7.1) |
| 11 | `.cpp:130` | 加速度ゲイン `1.0` 直書き | 非公開定数化。値は変えない |
| 12 | `navyu_utils.cpp:51-59` | `normalized_radian` は `±2π` の 1 回補正のみで `\|angle\| >= 3π` で壊れる | `core` の `normalize_angle` を使う (任意の大きさで 1 回で正しい) |
| 13 | `.cpp:113-121` | 経路点の `orientation` を一度も読まない。要求 goal 姿勢が捨てられる | T5 でも経路の `yaw` は使わない (C-4)。最終姿勢合わせは対象外 (C-3)。ただし T4 が末尾に載せた `goal.yaw` は残っているので後続タスクが使える |
| 14 | 全体 | ROS ノードと制御ロジックが 1 クラスに同居し、単体テストが書けない | `PurePursuit` は ROS を知らない値型。ノードは T7 |

---

## 11. テスト構成

| ファイル | 内容 |
|---|---|
| `test/control/tracking_fixture.hpp` | 差動二輪の運動学積分 (`simulate`)、経路生成 (`make_straight_path` / `make_arc_path`)、横方向誤差計測 (`lateral_error`)、`NavyuFormReference` |
| `test/control/test_pure_pursuit.cpp` | `create()` の検証、退化ケース、初回決定性、`path` の不変性、向き合わせ (22 件) |
| `test/control/test_pure_pursuit_tracking.cpp` | 直線・円弧の追従誤差、navyu 形との比較、ゴール残距離 (7 件) |
| `test/control/test_goal_approach.cpp` | `GoalApproach` (§12) の減速則 / yaw 収束 / 進捗検査 / 退化ケースとラッチ / `apply_linear_limit` (38 件) |
| `test/core/test_angle.cpp` / `test_types.cpp` / `test_path.cpp` / `test_geometry.cpp` | 4 ユーティリティ (§8) の契約 (25 件を追加) |

横方向誤差は経路の各線分に対する `distance_to_segment` の最小値で測る (再実装しない)。
シミュレータはテスト内に閉じており、T6 の `sim/` とは別物である。

### 11.0 実マップでの目視確認 (`examples/eltanin_track_on_real_map`)

単体テストは合成経路 (直線・円弧) しか使わない。実マップの経路は曲率がはるかに厳しく、
そこで初めて §5.1 の結果が出る。目視・外部プロット用の導線として例を 1 つ置いた。

```bash
cmake -B build -DELTANIN_BUILD_EXAMPLES=ON && cmake --build build -j
./build/examples/eltanin_track_on_real_map <map.yaml> <out_dir> [start_x start_y goal_x goal_y] [lateral_offset]
```

| 出力 | 内容 |
|---|---|
| `crop.pgm` | 膨張コストマップを経路と軌跡の外接矩形 + 30 セルで切り出した画像 |
| `path.csv` | `x,y,yaw` — A* + スムーザの経路 |
| `trajectory.csv` | `t,x,y,yaw,v,w,lateral_error,travelled,target_index,lookahead_x,lookahead_y` — 追従した実軌跡 |
| `meta.txt` | crop のジオメトリ、パラメータ、誤差の要約、`Circumscribed` / `Inscribed` 侵入数 |

`crop.pgm` の座標系は `meta.txt` の `origin_x` / `origin_y` / `resolution` で CSV と対応が取れる。
`lateral_offset` は経路始点から横に robot をずらす引数で、定常誤差ではなく過渡を見たいときに使う。

`trajectory.csv` は `Result` の観測用フィールド (`target_index` / `lookahead_point`) もそのまま出す。
**角で内側を切る理由はこの 2 列を見ないと分からない。** ロボット位置から `lookahead_x` / `lookahead_y` へ
線を引くと、曲率半径 0.133 m の折れ点に対して先行距離 0.35 m が長すぎ、目標点が角の向こう側に置かれる
ことが見える。§5.1 の結論はこの観察から出ている。

**この例は回帰テストではない。** 実マップの存在に依存し、自動選択の start / goal が地図の内容で変わるため、
`ctest` には入れていない (実マップ依存のテストは `test/planner/test_planner_real_map.cpp` と同じく
`ELTANIN_TEST_MAP_DIR` の枠組みに載せる方が筋だが、追従は 7500 ステップ回るので既定のテスト時間に対して重い)。
§5.1 / §5.2 の数値はこの例の出力である。

### 11.1 未初期化バグの回帰テストは 2 本立てである

`linear_vel_` が 0 で始まることの検出は、単独では弱い。2 通りで固定した。

1. **厳密値テスト (主検出器)**: 整合済みの直線経路で初回を呼ぶと `command.linear.x()` は
   `LINEAR_VEL_GAIN * (desired_linear_vel - 0.0) * dt` に厳密一致する (既定値・`dt = 0.01` で 0.005)。
   `linear_vel_` が 0 以外なら一致しない。
2. **構造テスト**: 先行距離の式が `linear_vel_` を実際に読んでいることを見る。
   `lookahead_distance` が `min_lookahead_dist` のときと、それより大きいときで**目標点が変わり
   `alpha` の符号が反転する**探査用の合成経路を使う。

```
p0 (0.00,  0.00)  距離 0.0000
p1 (0.30,  0.10)  距離 0.3162  方位 +0.3218
p2 (0.36, -0.15)  距離 0.3900  方位 -0.3948
p3 (0.80, -0.40)  距離 0.8944
p4 (1.20, -0.70)  距離 1.3892
```

`linear_vel_ == 0` なら先行距離 0.300 で p1 を選び `command.angular = +0.5`。
`linear_vel_ == 0.5` (未初期化のゴミの代表値) なら先行距離 0.350 で p1 を飛ばして p2 を選び `-0.5` になる。
ただし**この経路が検出できるのは `linear_vel_ > 0.162` のゴミだけ**である (それ以下では先行距離が
0.3162 未満で p1 が選ばれ続ける)。だから 1 と 2 の両方が必要である。
この経路は追従の現実性を意図しない合成経路である。

---

## 12. ゴール最終接近 (`GoalApproach`)

対象は `include/eltanin/control/goal_approach.hpp` / `src/control/goal_approach.cpp` の
`eltanin::control::GoalApproach` と `eltanin::control::detail::apply_linear_limit()`。

`docs/integration-design.md` §13-2 / §15.1 が記録した **R-7 (ゴール手前で角速度が ±0.4 rad/s で振動する)**
への対策である。移植元はない。navyu の `navyu_path_tracker` はゴール最終接近の概念を持たない
(欠陥 13 / C-3)。

### 12.1 責務境界と合成の契約

**`GoalApproach` は `PurePursuit` を知らない。** 速度上限と状態を返すだけで、合成は呼び出し側が行う。

```
pp = pursuit.compute(robot, path, dt);
ga = approach.compute(robot, path, dt);

switch (ga.state) {
  Aligning:                    cmd = ga.command;                                       // その場旋回
  Reached / AlignmentTimeout:  cmd = {};                                               // ゼロ指令
  Inactive / Approaching:      cmd = detail::apply_linear_limit(pp.command, ga.linear_vel_limit);
}
// pp.status が Tracking 以外ならゼロ指令 (§1)
```

`Inactive` では上限が `+inf` で `apply_linear_limit()` が恒等写像になるため、
**`Inactive` と `Approaching` は分岐なしで同じ経路を通せる。**

#### 12.1.1 `linear.x()` だけをクランプすると R-7 は直らない

これが `apply_linear_limit()` を `eltanin` 側に置いた理由である。
§1.1 の機構によりゴール手前では角速度の分母が `min_lookahead_dist` に張り付き、
`w` の振幅は `v` にほぼ比例する。したがって `v` を絞れば `w` も縮む —
**ただし `PurePursuit` が返す `w` は自分の内部状態 `linear_vel_` から計算されている**
(`pure_pursuit.cpp:132-134`)。呼び出し側が `command.linear.x()` だけを `std::min` で潰しても、
`w` は絞られていない `linear_vel_` のまま出てくる。振動はそのまま残る。

合成は **Twist 全体への比率スケール**でなければならない。曲率 `w/v` を保つ形である。

```
ratio = v_out / v_in      (|v_in| <= MIN_LINEAR_VEL なら 1.0)
out   = { v_out, 0.0, w_in * ratio }
```

同じ形の先例が `collision::detail::limit_command()` (`velocity_limiter.cpp:60-76`) にある。
減速則の式まで同一である。**この関数を呼び出し側 (ROS ノード) に書かせると、正しさが微妙な割に
`eltanin` の単体テストでは誤りを検出できない。** 責務境界は変えずに実装だけ `eltanin` 側へ移した。
`Twist2D` と上限値しか受けない純関数なので、`PurePursuit` への依存は生じない。

`MIN_LINEAR_VEL = 1e-9` は `collision::detail::MIN_LINEAR_VEL` と同値だが**参照しない。**
`control` が `collision` に依存してはいけない (`AGENTS.md` の依存規則)。

### 12.2 パラメータと既定値

| パラメータ | 既定値 | 意味・根拠 |
|---|---|---|
| `xy_goal_tolerance` | 0.10 [m] | 到達とみなす位置誤差。nav2 `SimpleGoalChecker` の語彙 |
| `yaw_goal_tolerance` | 0.10 [rad] | 到達とみなす方位誤差。同上 |
| `approach_distance` | 0.5 [m] | 減速フェーズに入る残弧長。制動距離 `v²/(2a) = 0.25 m` の 2 倍 |
| `approach_decel` | 0.5 [m/s²] | 減速則の減速度。実機の加減速限界が未計測のため保守値 |
| `yaw_align_timeout` | 5.0 [s] | 向き合わせの進捗検査。最悪 (`pi`) の 3.45 s が予算の 69 % |
| `max_angular_vel` | 1.0 [rad/s] | 向き合わせ時の角速度の対称上限 |

`approach_distance` を制動距離の 2 倍に取った理由は、**帯に入った瞬間に指令が段差にならない**ことである。
`approach_decel = 0.5` での減速則の値:

| `remaining_arc` [m] | 0.5 | 0.4 | 0.3 | 0.25 | 0.2 | 0.15 | 0.10 | 0.05 | 0.01 | 0 |
|---|---|---|---|---|---|---|---|---|---|---|
| `sqrt(2 * 0.5 * s)` [m/s] | **0.707** | 0.632 | 0.548 | **0.500** | 0.447 | 0.387 | **0.316** | 0.224 | 0.100 | **0** |

- 接近帯の入口 (0.5 m) で上限 0.707 m/s は `desired_linear_vel` の既定 0.5 を上回る。
  実際に減速が効き始めるのは残弧長 0.25 m からである。
- 到達帯の境界 (0.10 m) でも 0.316 m/s ある。**クリープ不足でゴールに入れない事態は起きない。**
- `Inactive` ↔ `Approaching` の境界でのチャタリングは無害である。
  上限が `+inf` と 0.707 の間で往復するだけで、どちらも 0.5 を上回るので指令は変わらない。

**`max_angular_vel` は `PurePursuitParams` と同名・同既定値になる。これは意図した重複である。**
ROS 側のパラメータは 1 つで、どちらの生成器が指令を出しても同じ上限に従う。

`create()` の検証は §10 の分類 B (`std::optional`)。6 値の有限性 → `yaw_goal_tolerance` 以外の正値性 →
`yaw_goal_tolerance ∈ (0, pi)` → **`approach_distance >= xy_goal_tolerance`** の順に見る。
最後の条件を課す理由は 12.3 にある。

### 12.3 状態遷移と評価順序

| `state` | 条件 | `command` | `linear_vel_limit` |
|---|---|---|---|
| `Inactive` | `remaining_arc > approach_distance` (空経路を含む) | ゼロ | `+inf` |
| `Approaching` | 接近帯の中 かつ `position_error > xy_goal_tolerance` | ゼロ | `sqrt(2 * approach_decel * remaining_arc)` |
| `Aligning` | 接近帯の中 かつ 位置到達 かつ `|yaw_error| > yaw_goal_tolerance` | **その場旋回** | **0** |
| `Reached` | 接近帯の中 かつ 位置到達 かつ 方位到達 | ゼロ | **0** |
| `AlignmentTimeout` | `align_elapsed > yaw_align_timeout` | ゼロ | **0** |

**終端 3 状態の上限を 0 にすることが「ゴールで止まる」ことの本体である。**
`Reached` の時点で残弧長は `xy_goal_tolerance` 前後残っているので、減速則を素直に当てると
0.316 m/s が出る。一方 `PurePursuit` の `GoalReached` は経路点間隔基準 (C-2 / 実測 0.025 m 手前) なので
まだ `Tracking` を返して 0.5 m/s を要求している。**合成結果が前進指令になり、ゴールで止まらない。**

`compute()` の評価順序は次に固定してある。

```
1. assert (dt > 0 かつ有限、robot が有限)                       ← 分類 A
2. 経路が空でなければ観測量 (remaining_arc / position_error / yaw_error) を計算する
3. latched_ があれば その状態 + ゼロ指令 + 上限 0 を返す           ← ラッチ最優先
4. remaining_arc > approach_distance なら Inactive + ゼロ指令 + 上限 +inf
5. position_error > xy_goal_tolerance なら Approaching + ゼロ指令 + 減速則の上限
6. |yaw_error| <= yaw_goal_tolerance なら Reached をラッチして返す
7. Aligning: align_elapsed_ += dt。timeout を厳密 > で超えたら AlignmentTimeout をラッチする
8. 4 / 5 / 6 のいずれかを通ったら align_elapsed_ = 0 に戻す
```

**接近帯 (`remaining_arc <= approach_distance`) を `Approaching` / `Aligning` / `Reached` 全部の
前提条件にしてある。** これが「迂回路が自分の終端付近を通ったときに早期 `Reached` になる」問題への
対策である。迂回路の途中でロボットがゴール位置の 10 cm 以内を通っても、**その時点の最近傍点は
経路の途中**なので残弧長は大きく `Inactive` のままになる。判定が「ゴールとの距離」ではなく
「経路上のどこを走っているか」で行われるためである。

**`approach_distance >= xy_goal_tolerance` を `create()` で要求する理由もここにある。**
接近帯が到達帯より内側だと、到達帯に入っても `Inactive` のままで永久に `Reached` にならない。

**ラッチが空経路より優先する。** 一過性の空経路で消えるラッチはラッチではない。
新しい経路を渡すときは呼び出し側が `reset()` を呼ぶ契約 (G-5) なので正常なフローでは差が出ない。
差が出るのは「経路が途切れた」異常時だけで、そこではラッチを保つ方が安全側である。

**ラッチ中も観測量は実測値を返す** (手順 2 が手順 3 より前にある)。
ラッチしたことで `remaining_arc` が `+inf` に見えるのは「ゴールから遠い」を意味してしまい、
`follower_state` に載せたときに読み手を誤らせる。ラッチが支配するのは
`state` / `command` / `linear_vel_limit` の 3 つだけである。

空経路が `Inactive` に落ちるのは**特別扱いではなく式の帰結**である。観測量の既定値が `+inf` なので
`remaining_arc > approach_distance` が自然に成立する。

### 12.4 向き合わせは比例則にした (`PurePursuit` の bang-bang を踏襲しない)

旋回則は `w = clamp(YAW_ALIGN_GAIN * yaw_error, ±max_angular_vel)`、`YAW_ALIGN_GAIN = 2.0 [1/s]` は
`.cpp` の無名名前空間の非公開定数である (`LINEAR_VEL_GAIN` と同じ扱い / §2)。

`PurePursuit` の初期向き合わせと同じ bang-bang (`ALIGNMENT_ANGULAR_VEL_RATIO = 0.5`) を**却下した。**
`dt = 0.05 s`、`max_angular_vel = 1.0`、`yaw_goal_tolerance = 0.10` で初期方位誤差から
許容帯に入るまでの時間 [s]:

| 初期誤差 [rad] | **比例 `k=2.0`** | 比例 `k=1.0` | bang-bang `0.5 w_max` | bang-bang `w_max` |
|---|---|---|---|---|
| 0.50 | 0.80 | 1.60 | 0.80 | 0.45 |
| 1.00 | 1.30 | 2.25 | 1.80 | 0.90 |
| 2.00 | 2.30 | 3.25 | 3.85 | 1.90 |
| **pi (最悪)** | **3.45** | 4.40 | **6.10** | 3.05 |

**`0.5 w_max` 形は既定値で `pi` の回転に 6.10 s かかり、`yaw_align_timeout` の 5.0 s を超える。**
つまり既定設定のまま対向姿勢のゴールを与えると必ず `AlignmentTimeout` になる。
`k = 1.0` も 4.40 s で余裕が 0.6 s しかない。

`max_angular_vel` を丸ごと使う bang-bang は 3.05 s で時間は足りるが、**`dt` に対して脆い。**
初期誤差 `pi` から収束させたときの符号反転回数 (許容帯を飛び越えて振動した回数):

| `dt` [s] | 比例 `k=2.0` | 反転 | bang-bang `w_max` | 反転 |
|---|---|---|---|---|
| 0.05 | 3.45 s | 0 | 3.05 s | 0 |
| 0.15 | 3.45 s | 0 | 3.15 s | **1** |
| 0.30 | 3.30 s | 0 | **収束せず** | **90** |
| 0.50 | 3.50 s | **1** | **収束せず** | 54 |

bang-bang の破綻条件は `dt > yaw_goal_tolerance / max_angular_vel = 0.10 s` で、
20 Hz 運用の 0.05 s に対して余裕が 2 倍しかない。**ROS タイマのジッタで 0.1 s を超えることは現実にある。**
比例則の破綻条件は `YAW_ALIGN_GAIN * dt <= 1` すなわち `dt <= 0.5 s` で、余裕が 10 倍ある。
実測で反転が出るのもちょうど `dt = 1/k` からで、解析と一致する。

副産物として **`sign()` 分岐が消える。** 符号は比例項が持つため、navyu 欠陥 10
(`alpha == 0` で `sign = -1` になり逆向きに回る) と同種の穴が原理的に作れない。
最短方向を通ることは `normalize_angle` の値域 `(-pi, pi]` がそのまま保証する。
対向 (厳密に `pi`) では `+pi` 側 = 反時計回りが選ばれ、決定的である (§8 と同じ規約)。

### 12.5 C-3 / C-4 との関係

- **C-3 (最終姿勢合わせを持たない) は `GoalApproach` が引き取った。** §6 の C-3 の行は
  「T5 の `PurePursuit` は持たない」という意味に限定される。
- **C-4 (経路点の `yaw` を使わない) の唯一の例外である。** `GoalApproach` は経路末尾の `yaw` を
  ゴール方位として読む。T4 が経路末尾に載せた `goal.yaw` (navyu 欠陥 13 で捨てられていた値) が
  ここで初めて使われる。**中間点の `yaw` は依然読まない。**

### 12.6 `PurePursuit::Status` に `Approaching` を追加しなかった

`eltanin_ros` の詳細設計 (`docs/design/eltaninnavyuros.md` の D-17 / §8.2 / §10 S4) は
「`PurePursuit::Status` に `Approaching` を追加する」と書いている。**実装はこれに従っていない。**

- 接近の状態語彙は `GoalApproach::State` の 5 値に閉じている。`Status` に `Approaching` を足すと
  **同じ概念が 2 箇所に出て、どちらが真かが決まらない。**
- `PurePursuit` の `nearest_index_` は追従進捗だけを表し、接近帯
  (`approach_distance` / 残弧長) の状態語彙は `GoalApproach` に閉じる。
- `Status` を触らなければ既存 432 件・`examples` の実測値・§4.1 の閾値表がそのまま生きる。

`Status` の `switch` は製品コード・テスト・`examples` を通じて **0 件**であり、
網羅性警告のリスクは元から存在しなかった。**追加を避けた理由は警告ではなく概念の重複である。**
`eltaninnavyuros.md` 側の記述の修正は `path_follower` の実装タスクに送っている。

### 12.7 進捗検査は `Aligning` の `dt` 累積だけである

`align_elapsed_` に `dt` を積み、`yaw_align_timeout` を**厳密 `>`** で超えたら `AlignmentTimeout` を
ラッチする。既定値 5.0 / `dt = 0.05` なら 100 周期目で `elapsed == 5.0` (まだ `Aligning`)、
101 周期目の 5.05 で打ち切る。時計は持たない (G-1)。

**これは C-1 に対する第二の役目を持つ。** C-1 は「`dt` が大きいと 1 周期の旋回量が許容帯を
飛び越え続けて永久に前進しない」という上限を記録しているが、`create()` は `dt` を知らないので
検証できない。`GoalApproach` では同じ状況が有限時間で打ち切られる。
12.4 で比例則を選んだのは破綻条件を `dt <= 0.5 s` まで押し出すためで、
進捗検査はそれでも破れたときの最後の網である。

`AlignmentTimeout` を `reset()` までラッチし、同時に指令をゼロ・上限を 0 にしてある。
**呼び出し側が失敗を取りこぼしても、ロボットは回り続けも進み続けもしない** (N-11 への構造的対策)。

**`Approaching` に進捗検査はない。** 壁に引っかかって物理的に停滞した場合は打ち切れない。
これは意図した範囲外である。停滞の検出は上位 (`navigator` の停止トリガ / `path_follower` の
`trajectory_timeout`) の担当であり、`GoalApproach` に時計や実測速度を持ち込む (G-1 / G-10 に反する)
理由にはならない。12.8 でマージン減算を却下した理由でもある。

### 12.8 実装上の判断

| 判断 | 理由 |
|---|---|
| **`cumulative_arc_length()` を使わない** | 値返しでヒープ確保が入る (G-2 / navyu 欠陥 8)。`compute()` の中で「最近傍 index の走査 `O(n)`」→「そこから末尾までの線分長の総和 `O(n-i)`」を確保なしで回す。20 Hz × 727 点 = 14.5 k 距離評価/秒で実害はない |
| **最近傍 index を受けるオーバーロードを足さない** | `PurePursuit` と重複して走査するが、`costmap-design.md` §9.2「利用者が現に存在しないものを public にしない」。必要になったら実測を根拠に足す |
| **`nearest_index()` を複製した** | `core` へ昇格させると `pure_pursuit.cpp` を書き換えることになり、`PurePursuit` を 1 行も変えない方針を破る。複製は 12 行で、同順位解決 (厳密 `<` で最小 index / `squaredNorm` 比較) まで揃えてある。統合は `PurePursuit` に触る後続タスクの判断に送る |
| **`approach_decel` を `VelocityLimiterParams::max_deceleration` と分ける** | 既定値は同じ 0.5 だが関心が違う。前者は**ゴール**までの制動則 (到達品質)、後者は**障害物**までの制動則 (安全)。片方を詰めたい要求は片方に波及すべきでない |
| **角速度に独立した減速則を持たない** | §1.1 の機構により、ゴール手前では `w` の振幅が `v` にほぼ比例する。比率スケール (12.1.1) で合成すれば `w` は自動的に同じ比率で縮む |
| **マージン減算を却下した** | `max(0, remaining_arc - xy_goal_tolerance)` を `sqrt` に入れれば到達帯の境界で上限がちょうど 0 になり `Aligning` への遷移が滑らかになる。しかし**上限が 0 に漸近するので到達帯に入る手前で停滞しうる**。`Approaching` に進捗検査はない (12.7) ので打ち切れない。「必ず帯に入る」を優先し、境界での 0.316 → 0 の段差を受け入れる (`dt = 0.05 s` の行き過ぎは 1.6 cm で `xy_goal_tolerance` の内側) |
| **ラッチを 2 つの `bool` にせず `std::optional<State>` 1 個にした** | `Reached` と `AlignmentTimeout` は排他であり、優先順位が 1 行で書ける |

内部状態は `latched_` と `align_elapsed_` の **2 個だけ**である。§1 の「内部状態は 2 つのみ」と同じ規律で、
**経路の identity に依存する状態 (前回 index など) を持たない。**

### 12.9 制約と退化ケース

制約 ID は **`G-*`** を使う。§6 の `C-1` 〜 `C-11` は `PurePursuit` の制約であり、
そこに追記すると「どちらのクラスの制約か」が読めなくなる。

| # | 制約 | `PurePursuit` の対応 |
|---|---|---|
| G-1 | 時計を持たない。`dt` を引数で受ける | §1 / navyu 欠陥 7 |
| G-2 | `compute()` はヒープ確保をしない | §1 / navyu 欠陥 8 |
| G-3 | コストマップを見ない。障害物由来の減速は `VelocityLimiter` の担当 | C-11 |
| G-4 | 後退しない。オーバーシュートしても戻らず旋回のみで合わせる | C-7 |
| G-5 | **経路の identity を検知できない。新しい経路では呼び出し側が `reset()` を呼ぶ。** ラッチがあるため本クラスでは必須である | C-6 |
| G-6 | スレッド安全性は呼び出し側の責務 | C-9 |
| G-7 | `dt` に上限がある。`create()` は検証できない。打ち切りは進捗検査が行う (12.7) | C-1 |
| G-8 | 経路は前進のみでカスプを含まず自己交差しないことを前提とする | C-5 |
| G-9 | ROS / Rerun / matplotlib-cpp / Python を知らない | `AGENTS.md` |
| G-10 | 実測速度を受け取らない。状態推定・遅延補償は範囲外 | C-8 |
| G-11 | 経路点の座標は有限であることを前提とし、検査しない | C-10 |
| G-12 | `PurePursuit` を知らない。合成は呼び出し側が行う | 12.1 |

| # | 状況 | 挙動 |
|---|---|---|
| 1 | `path` が空 | `Inactive` / ゼロ指令 / 上限 `+inf` / 観測量 `remaining_arc` `position_error` は `+inf`、`yaw_error` は 0 |
| 2 | `path` が 1 点 | `remaining_arc == 0`。状態は位置・方位誤差から決まり、`Approaching` なら上限は 0 になる |
| 3 | ゴール上で開始 (位置・方位ともに到達) | 初回 `compute()` で `Reached` |
| 4 | ゴール上で開始・方位誤差が許容値より大きい | 初回から `Aligning`。既定パラメータ・`dt = 0.05 s` で有限周期で `Reached` に収束する |
| 5 | 初期方位誤差が厳密に `pi` | 反時計回りに回って収束する (12.4) |
| 6 | `remaining_arc` が `approach_distance` をまたぐ | 上限が `+inf` から有限値へ落ちる。`Approaching` の範囲で単調性を保つ |
| 7 | `Reached` 後にロボットが外乱で帯を出る | ラッチにより `Reached` のまま。帯の境界で状態が振動すること自体が R-7 の再発になる |
| 8 | `apply_linear_limit()` に `v ≈ 0` の指令 | 比率が定義できないので `w` をそのまま通す。その場旋回を上限で殺さない |

### 12.10 単調性の但し書き

`linear_vel_limit` が `remaining_arc` に対して単調非減少なのは **`state == Approaching` の範囲に限る。**
終端 3 状態の上限を 0 にした (12.3) ため、`remaining_arc = 0.08` では
「`Approaching` なら 0.28 / `Reached` なら 0」となり、`remaining_arc` だけの関数ではなくなる。
テストも `Approaching` の範囲で掃いている。
