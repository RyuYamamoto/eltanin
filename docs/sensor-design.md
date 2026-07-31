# eltanin センサ処理設計方針 (2D スキャンの点列投影)

本書は `eltanin_sensor` (`include/eltanin/sensor/` + `src/sensor/`) の設計判断の記録である。`docs/costmap-design.md` は冒頭で対象をコストマップ / 衝突判定に限定しているため、センサ処理の決定は本書に置く。命名規約 (§9)・公開範囲 (§9.2)・エラー通知方式 (§10)・依存規則 (§11)・モジュール構成 (§12) は `costmap-design.md` の記述に従う。

参照実装は navyu の `navyu_costmap_2d/include/navyu_costmap_2d/plugins/dynamic_layer.hpp` である。同クラスに同居していた「scan フィルタ + 投影 + 座標変換 + セル書き込み + origin 更新」のうち、**センサ処理部だけ**を切り出した。

---

## 1. 対象と範囲

対象は「1 枚の 2D レーザスキャンを、フィルタしたうえで平面上の点列にする」ことのみである。

公開 API は次の 4 つだけである。

| 要素 | 場所 |
|---|---|
| `ScanData` / `ScanFilter` | `include/eltanin/sensor/scan_projection.hpp` |
| `project_scan()` 2 オーバーロード (マーキング用) | 同上 |
| `project_scan_for_clearing()` 2 オーバーロード (クリアリング用、§11) | `include/eltanin/sensor/scan_clearing.hpp` |
| `AngleRange` / `angle_in_range()` | `include/eltanin/core/angle.hpp` (角度の概念は core に閉じる) |

対象外とし、利用者が現れてから足すもの:

| 項目 | 移送先 / 理由 |
|---|---|
| `sensor_msgs::msg::LaserScan` ⇄ `ScanData` の変換、`frame_id` / `stamp` | ROS ブリッジ層 |
| PointCloud2 / 3D 点群の入力、z フィルタ | 2D スキャンに用途が閉じている。3D → 2D の縮約はブリッジ層かセンサモデルが決まってから |
| 傾いたセンサ (ロール / ピッチ非零) の投影 | `Transform2D` は SE(2)。3D 姿勢を扱うならブリッジ層で 2D へ落とす |
| `intensities` / `time_increment` / `scan_time` | 利用者が存在しない |
| ビームごとの時刻補正 (スキャン中のロボット運動による歪み補正) | 姿勢の時系列補間が前提。`Transform2D` 1 個を受ける現契約の外 |
| ダウンサンプリング / voxel grid、スキャンの蓄積 (nav2 の `ObservationBuffer` 相当) | 観測モデルの話。本モジュールは 1 スキャンを 1 回投影するだけ |
| レイトレースのセル塗り (レイが通過したセルの列挙) | セル座標を扱わない (§6)。塗るのは `eltanin::map::RaytraceLayer` である。センサ側が出すのはレイの終点だけ (§11) |
| 複数センサの点列統合 | 呼び出し側が 2 回呼んで連結すればよい。追記モードの API を持たせない (§8) |
| ロボット自身のフットプリント形状によるビーム除外 | 角度セクタで足りない幾何判定は `eltanin_safety` |
| 極座標のまま扱うプランナ向け API | 利用者が存在しない |
| スレッド安全性 / ロック | 呼び出し側の責務。既存モジュールと同じ立場 |

---

## 2. `ScanData` の契約

```cpp
struct ScanData
{
  double angle_min{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{std::numeric_limits<double>::infinity()};
  std::vector<float> ranges;
};
```

不変条件を持たない素の集約である。`create()` / コンストラクタを作らない。値の妥当性は投影関数の入口で扱う (§9)。

**ビーム i の角度は `angle_min + static_cast<double>(i) * angle_increment` で毎回計算する。逐次加算しない。** 720 ビーム程度では実害が出にくいが、加算誤差の累積は後から発見しづらい欠陥である。`angle_increment` は負値 (時計回りスキャン) を許す。

**`angle_max` を持たない。** ビーム角は `angle_min` + i × `angle_increment` で一意に決まる。`angle_max` を持つと `ranges.size()` との整合性検証という責務が増え、不整合時にどちらを信じるかを決めなければならなくなる。ROS メッセージとの照合はブリッジ層の責務である。

**スカラは `double`、`ranges` は `std::vector<float>`。** `ranges` の型は ROS の wire format と一致するのでブリッジ層でコピーが増えない。スカラを `double` にするのは `Transform2D` / `Eigen::Vector2d` / 角度ユーティリティがすべて `double` であり、API 境界で型を混ぜないためである。`float` → `double` の変換は投影ループ内の 1 箇所だけで明示的に行う。

**`ranges` を `std::span<const float>` にしない。** ブリッジ層のメッセージ寿命に依存する。`ObstacleLayer` が `span` を保持しないのと同じ理由である。

既定値は `range_min = 0.0` / `range_max = +inf` とした。`0.0` は `max` の、`+inf` は `min` の単位元であり、既定構築が「センサ側は何も制約しない」を意味する。`ScanFilter` と単位元の規則が揃うので、読み手が 2 つの規則を覚えなくてよい。

---

## 3. レンジ判定

`ScanData` 側の諸元と `ScanFilter` 側のパラメータの**交差を 1 本の閉区間述語に畳む**。有効境界はビームループの外で 1 回だけ算出する。

```cpp
lower = filter.min_range; if (isfinite(scan.range_min)) lower = max(lower, scan.range_min);
upper = filter.max_range; if (isfinite(scan.range_max)) upper = min(upper, scan.range_max);
```

- 判定は**閉区間** `lower <= range <= upper`。両端の値は残る。
- `scan.range_*` の**非有限値は「制約なし」**を意味する。`optional` の対で持つ案は述語を分岐で汚すので採らない。
- **有効下限 > 有効上限 の場合に専用の早期 return を置かない。** `range < lower || range > upper` が全ビームを落とすので、出力が空になるのは述語の帰結である。分岐を足すと同じ判定が 2 本になる。この状態はセンサ諸元とパラメータの組み合わせ次第で起こるので `assert` しない (§9)。

navyu は「パラメータによるゲート」と「`laser_geometry` 内部の `scan.range_*` ゲート」の 2 段構成で、有効範囲がコードのどこを読んでも分からなかった。参照 LiDAR の実測値 (`scan` = [0.1, 30.0] / `filter` = [0.0, 5.0]) では**下限がセンサ側、上限がパラメータ側から来る**ため、片方だけを見る設計では両方を再現できない。`effective_bounds()` が「有効範囲はどこに書いてあるのか」に対する唯一の答えになる。

**畳み込みは `detail::effective_bounds()` の 1 箇所しかない。** クリアリングのパス (§11) も同じ関数を呼ぶため、`detail` 名前空間に出して 2 つの翻訳単位で共有している。センサ諸元との交差規則をパスごとに書き分けると、非有限値の扱いや閉区間の端が食い違ってもテストが両方を跨いで検出できない。

`ScanFilter` のフィールド名を `min_range` / `max_range` とし、`ScanData` の `range_min` / `range_max` と**あえて別語順**にしている。両構造体が同じ関数に渡るため、`std::max(filter.min_range, scan.range_min)` の一方を書き間違えると**コンパイルエラーになる**。同名にすると `std::max(scan.range_min, scan.range_min)` が黙って通り、畳み込みが 1 箇所しかないだけにそこを取り違えると全ビームの判定が壊れる。

---

## 4. 非有限値の除外

**`std::isfinite(range)` を明示的に、レンジ比較より先に評価する。「範囲外でない」を「有効」と読み替えない。**

navyu の `if (r < min or max < r)` は NaN に対して両方の比較が偽になるため NaN を通し、実際には `projectLaser` 内部の除外に暗黙に依存していた。除外の責務が 2 箇所に分かれた状態で片方を差し替えると NaN が下流へ漏れる。

`MapGeometry::to_int_saturating()` は NaN / 範囲外 double を安全に扱うので、仮に NaN 座標の点を渡しても未定義動作にはならず `nullopt` になる。しかし**NaN 座標の点を出力しないことはセンサ側の責務**である。下流の防御に依存しない。

`ranges` に非有限値・レンジ外の値・負値が含まれるのは**正常なセンサ出力**である。`assert` してはならない (§9)。

---

## 5. 角度範囲フィルタ

判定は `eltanin/core/angle.hpp` に置く。角度の概念を 2 モジュールに散らさないためである。

```cpp
struct AngleRange { double from{0.0}; double to{0.0}; };

inline bool angle_in_range(double angle, double from, double to)
{
  return normalize_angle_positive(angle - from) <= normalize_angle_positive(to - from);
}
```

意味は **`from` から `to` へ反時計回り (CCW) にたどった閉じた弧に `angle` が含まれるか**である。CCW の弧として定義すると、ラップアラウンドが特別扱いではなく定義そのものに含まれる。

| 性質 | 成立の根拠 |
|---|---|
| 両端を含む閉じた弧 | `<=` 比較。`angle == from` で左辺 0、`angle == to` で両辺一致 |
| ±π を跨ぐ範囲を特別扱いしない | `from` を原点に移してから `[0, 2π)` へ正規化するので、ラップアラウンドが定義に内包される |
| 入力角の大きさに制限がない | 正規化が入口で行われる |
| 非有限入力に対して `false` | `normalize_angle_positive(NaN)` / `(±inf)` は NaN (`fmod` の規定)。NaN を含む `<=` は `false` |
| `from == to` (mod 2π) は「その 1 角度のみ」 | 右辺が 0 になる |
| **全周は表現できない** | `from = -π, to = π` のとき `to - from` はちょうど `2π` (同一の `double` π の減算なので厳密) で、`std::fmod(2π, 2π) = 0` により右辺が 0 になる |
| 除外セクタは弧の反転で表現する | `angle_in_range(a, to, from)` が (両端を除いて) 補集合になる |

**全周が表現できないことが、`ScanFilter::angle_range` を `std::optional` にしている理由である。** 「`[-π, π]` を渡せば全周」ではなく、それはほぼ空の範囲に化ける。全周は「範囲がない」ことなので `optional` の不在が意味的にも正しい。この罠はテストで固定してある (`Angle.InRangeFromMinusPiToPiIsNotAFullTurn` / `ScanProjection.UnsetAngleRangeKeepsEveryBeamButMinusPiToPiDoesNot`)。

除外セクタ (例: ロボット自身を見る後方のビームを捨てる) は**弧の反転**で表現する。専用の除外 API を作らない。フィルタのフィールドが 2 つになると「両方設定されたとき」の意味を決める責務が増える。

`angle_in_range` に `AngleRange` を取るオーバーロードを併設していない。呼び出し側が投影ループの 1 箇所しかないためである (`costmap-design.md` §9.2)。

navyu には角度フィルタがなく、ロボット自身を見るビームや端の信頼できないビームを落とせなかった。

---

## 6. `Transform2D` の向きの規約と、角度フィルタを掛ける座標系

**`project_scan` が受け取る `Transform2D` は `T_world_sensor`** (センサ座標系の点を world へ写す) である。`Transform2D::from_pose(world 座標系でのセンサ姿勢)` と一致する。逆向きを渡すのは呼び出し側の誤りであり、ライブラリは検出できない。

world 変換は **`Transform2D::operator*` を各点に適用する**。ビーム角に yaw を畳み込む単一パス方式は却下した。畳み込み方式は同じ回転に対して 2 本目の浮動小数点式を作る。`costmap-design.md` §14.4 で「膨張半径外の判定を `cost_at_distance()` と別の式にしない」として却下したのと同じ種類の欠陥であり、「センサ座標系版の出力を写した結果 == world 版の出力」が浮動小数点として厳密に閉じる利益も失う。

そのため world 版は**センサ座標系版を呼んでから `out` を写す 2 パス**で実装してある。追加のヒープ確保は発生しない。720 点 × 40 Hz でのコサイン再計算は許容範囲であり、必要になれば `Transform2D` 側で扱う話になる。既定ビルドは `-O0` なので、実測の前提が揃うまで最適化しない。

**角度フィルタはセンサ座標系のビーム角に対して適用する。生存ビーム集合は `Transform2D` に依存しない。** 角度フィルタの用途は「ロボット自身を見るビーム / センサの端で信頼できないビームを落とす」であり、これはセンサ取り付けに固定された性質でロボット姿勢とは無関係である。world 方位でフィルタすると、ロボットが回転するたびに落ちるビームが変わるという別の意味になる。

**座標変換の実装を増やさない。** 回転・並進は `Transform2D` のみ。world ↔ cell 変換は `MapGeometry` のみであり、`eltanin_sensor` はセル座標を扱わない。navyu は `dynamic_layer` 内で world → cell を自前実装し、`static_cast<int>` により負値を誤判定していた。

---

## 7. tf を持たない理由

**センサ姿勢は呼び出し側が `Transform2D` で渡す。** これは単なる依存削減ではなく、**ライブラリから「変換の取得失敗」という失敗経路を消すための設計**である。navyu は毎周期 tf lookup が失敗しうる `update()` を持っていた。

`ScanData` に `frame_id` / タイムスタンプを持たせないのも同じ理由である。`frame_id` を持っても照合する相手がライブラリ内に存在せず、検証されない文字列は誤った安心を与える。

---

## 8. 出力の契約

```cpp
void project_scan(const ScanData &, const ScanFilter &, std::vector<Eigen::Vector2d> & out);
void project_scan(const ScanData &, const ScanFilter &, const Transform2D & sensor_to_world,
                  std::vector<Eigen::Vector2d> & out);
```

| 規約 | 理由 |
|---|---|
| 出力は `std::vector<Eigen::Vector2d> &` の出力引数 | 呼び出し側がバッファを再利用すれば定常状態でヒープ確保が 0 になる。戻り値で返すと 40 Hz で毎周期確保が起きる |
| 先頭で `out.clear()`。**追記モードを持たない** | 追記モードを持つと「クリアし忘れ」が観測の重複として現れ、原因追跡が難しい種類のバグになる。複数センサの統合は呼び出し側が連結すればよい |
| `out.reserve(scan.ranges.size())` | `clear()` は容量を保持するので、再利用時の `reserve` は no-op になる |
| 出力は**生存ビームの index 昇順** | |
| フィルタで落ちたビームの**穴埋めをしない** (NaN プレースホルダ等) | |
| **ビーム index と出力点の対応は保持されない** | `ObstacleLayer` は必要としない。将来必要になったら出力に index を持たせるか穴埋めするかの設計判断が要る |
| `noexcept` を付けない | `out` への追加が `std::bad_alloc` を投げうる |
| 入力 `ScanData` を書き換えない (`const &`) | navyu は受信メッセージの `scan_->ranges[i]` を NaN で破壊的に書き換えていた。同じ `SharedPtr` を他が参照していれば影響が漏れる |
| `Eigen::Vector2d` に短縮別名を付けない | `Polygon2D` の前例。`Eigen::aligned_allocator` は C++17 以降の over-aligned new により不要 |

出力の `std::vector<Eigen::Vector2d>` は `ObstacleLayer::set_points(std::span<const Eigen::Vector2d>)` に**そのまま**渡せる (暗黙変換)。中間バッファやコンテナ変換を挟まない。`test/sensor/test_scan_to_costmap.cpp` がこの結線を `LayeredCostmap::update()` 後のセル値まで含めて検証している。

`eltanin_sensor` は `eltanin_map` に依存しない (`ObstacleLayer` を知る必要がない)。結線の検証はテストターゲット `eltanin_test_sensor` が `eltanin::map` にリンクして行う。依存規則はライブラリに対する制約であり、テストは optional 層である。

---

## 9. 前提条件違反と正常な異常値の切り分け

`costmap-design.md` §10 のエラー通知方式に従う。**前提条件違反 (呼び出し側のプログラミングエラー) は `assert`、センサデータ由来の異常値は正常処理として除外する。** `eltanin_sensor` は例外を投げない。

`assert` するもの (`project_scan` の入口):

| `assert` | 捕まえるもの |
|---|---|
| `isfinite(scan.angle_min) && isfinite(scan.angle_increment)` | ブリッジ層がメッセージの妥当性を保証しなかった場合 |
| `isfinite(filter.min_range) && filter.min_range >= 0.0` | 下限の NaN / ±inf / 負値 |
| `filter.min_range <= filter.max_range` | 上限の NaN (比較が偽になる)、上限の `-inf`、下限 > 上限。**`+inf` は通す** |
| `angle_range` が有効なら `from` / `to` が有限 | NaN セクタによる「全ビームが黙って消える」を早期に暴く |

`assert` しないもの (いずれも正常な LiDAR 出力またはセンサ諸元の帰結):

- `ranges` の NaN / ±inf / レンジ外 / 負値。
- `scan.range_min` / `range_max` の非有限値 (「制約なし」の表現)。
- 有効下限 > 有効上限 (§3)。出力が空になるのが正しい振る舞いである。

上限の `+inf` を許すのは `ScanFilter::max_range` の既定値がそれだからである。「境界が非有限なら `assert`」を字面どおり実装すると既定構築の時点で発火する。上の 2 本で上限の NaN / `-inf` / 下限 > 上限はすべて捕まる。

**`core` の `angle_in_range` 自体は非有限入力に対して `false` を返す全域関数のままにする。** `assert` は sensor の入口だけに置く。core の述語に前提条件を持たせると単体テストで NaN を渡せなくなる。

`assert` のために一時変数を作らない。Release (`NDEBUG`) で `-Wunused-variable` / `-Wunused-but-set-variable` を出さないためである。

---

## 10. navyu との対応

| navyu の問題 | 箇所 (`dynamic_layer.hpp`) | 本モジュールでの対応 |
|---|---|---|
| レンジフィルタが受信メッセージ (`scan_->ranges[i]`) を破壊的に書き換える | 70-73 | `ScanData` は `const &` で受け、書き換えない (§8) |
| 非有限値の判定が `r < min or max < r` のみで、NaN が両方の比較で偽になるため通り抜ける | 71 | `std::isfinite()` を明示 (§4) |
| レンジゲートが 2 段 (パラメータと `scan.range_*`) に分かれ、有効範囲がコード上のどこにも書かれていない | 71 + `laser_geometry` 内部 | 交差を 1 本の閉区間述語に (§3) |
| 2D の点列を得るために `PointCloud2` → PCL → 4x4 行列変換を経由し、`laser_geometry` / `pcl_ros` / `tf2_eigen` に依存 | 76-110 | 三角関数と `Transform2D` のみ (§6) |
| tf lookup が `update()` の毎周期の失敗経路になっている | 84-87 | 変換は引数。失敗経路がライブラリに存在しない (§7) |
| センサ処理・costmap 書き込み・origin 更新が同一クラスに同居 | 全体 | 投影は `eltanin_sensor`、セル書き込みは `ObstacleLayer`、origin は `LayeredCostmap` |
| world → cell 変換の自前実装 (`static_cast<int>` で負値を誤判定) | 122-123 | `eltanin_sensor` はセル座標を扱わない (§6) |
| 角度フィルタがなく、ロボット自身を見るビームや端の信頼できないビームを落とせない | — | 角度範囲フィルタ (§5) |

参照 LiDAR の諸元 (`navyu_simulator/urdf/sample_robot.urdf` の `hokuyo_link`): 720 ビーム / 角度範囲 ±1.57 rad / `angle_increment ≈ 3.14/719` / レンジ 0.1〜30.0 m / 40 Hz。`navyu_navigation/config/navyu_params.yaml` の `dynamic_layer`: `min_laser_range = 0.0` / `max_laser_range = 5.0`。

---

## 11. clearing 用の投影 (`project_scan_for_clearing`)

`project_scan` はフィルタで落ちたビームを出力に残さないため、ray trace のクリアリングには使えない。**これは意図した契約であり、欠陥ではない。** そこで**マーキング用の `project_scan` を変えず、clearing 用の投影を別関数として足す 2 パス**にした (`costmap-design.md` §15.1 / §15.5 / §16 と対応する)。

```cpp
void project_scan_for_clearing(const ScanData &, const ScanFilter & marking_filter,
                               double clearing_max_range, std::vector<Eigen::Vector2d> & out);
void project_scan_for_clearing(const ScanData &, const ScanFilter & marking_filter,
                               double clearing_max_range, const Transform2D & sensor_to_world,
                               std::vector<Eigen::Vector2d> & out);
```

出力は**各ビームの自由区間の遠端 (レイの終点) の点列**である。セル座標も通過セルの列も含まない。セルを塗るのは `eltanin::map::RaytraceLayer` の責務であり、§6 の「`eltanin_sensor` はセル座標を扱わない」を維持する。出力の契約 (出力引数 / 先頭で `clear()` / 追記モードなし / index 昇順 / 穴埋めなし) は §8 と同一である。

**レイの始点 = センサ原点は出力に含めない。** 呼び出し側が `sensor_to_world.translation()` として既に持っている値であり、720 点に同じ値を複製する意味がない。

### 11.1 マーキング用の `ScanFilter` をそのまま受け取る

clearing 専用のフィルタ型を作らず、**マーキングに渡した `ScanFilter` そのものと、クリアリング最大レンジ 1 個**を受ける。

- **罠: 角度フィルタがマーキングとクリアリングで食い違うと、信用しないビームで自由空間を宣言してしまう。** 「同じ角度範囲を渡すこと」を文書上の規約にするだけでは、2 箇所に別の値を書ける API を残すことになる。角度セクタと近傍側の下限をマーキングと同じオブジェクトから読むので、**食い違いが型の上で起こりえない。**
- 分けたのは遠方側の上限だけである (詳細設計 D-27)。遠方ビームは角度分解能が粗く、自由空間の宣言には信頼できない。**既定値はライブラリに持たない** (ROS 層のパラメータ `clearing_max_range`、既定 3.0 m)。
- `marking_filter.max_range` は clearing パスでは**使わない**。同じ構造体の一部を無視する API になるが、角度セクタと近傍側下限が一致する利益がそれを上回る。

### 11.2 マーキングとクリアリングでビームの扱いが変わる箇所

| 読み値 | marking (`project_scan`) | clearing |
|---|---|---|
| 遠方側の上限より遠い | **捨てる** | **上限まで切り詰める。** そこまでは自由と分かっている |
| `+inf` (無反射) | 捨てる | `scan.range_max` まで自由とみなし、そのうえで上限で切り詰める |
| `+inf` かつ `scan.range_max` が非有限 | 捨てる | **捨てる。** どこまで自由なのかを決める情報がない |
| `NaN` / `-inf` / 負値 | 捨てる | 捨てる |
| 近傍側の下限より近い | 捨てる | 捨てる。信頼できない短い読み値でレイを引くと実在する障害物を消しうる |

`range_max` を `ScanData` 側に持たせた判断 (§2) がここで効く。inf ビームの置換に必要な情報が `ScanData` に揃っている。nav2 も同じ構成であり、`inf_is_valid` で inf を `range_max - ε` に置換し、`obstacle_max_range` と `raytrace_max_range` を分けている。

**有効区間の畳み込みは §3 の `effective_bounds()` 1 箇所を共有する。** clearing パスは「マーキングのフィルタの `max_range` だけをクリアリング最大レンジに差し替えた `ScanFilter`」を渡して同じ関数を呼ぶ。非有限は「制約なし」・閉区間、という交差規則がマーキングと一致することが構造的に保証される。切り詰めたあとの距離が下限を下回るビームを落とす判定 1 本だけを持ち、「有効区間が空」(例: クリアリング上限 < `range_min`) も同じ述語の帰結として空出力になる。専用の早期 return を置かない (§3 と同じ方針)。

### 11.3 引き続き持たないもの

| 項目 | 理由 |
|---|---|
| ビーム index と出力点の対応 | 2 パス方式では不要である (§8)。1 パスで両方を作る設計を選ぶ場合にのみ必要になる |
| 「このビームは無反射だった」というフラグ | 終点セルを塗るか否かの判断に使えるが、`RaytraceLayer` は**終点セルを常に塗らない**保守側の規則を採ったので利用者がいない (`costmap-design.md` §16.3) |
| 自由区間の始点を出力に含める (点対の列にする) | レイは常にセンサ原点から引く (§11.4)。出力が 2 倍になり `ObstacleLayer` への直渡し (§8) と非対称になる |
| クリアリング専用の角度範囲 / 最小レンジ | §11.1 のとおり、食い違いを作れる API を持たない |

### 11.4 レイはセンサ原点から引く (`range_min` の内側も自由になる)

`RaytraceLayer` はセンサ原点のセルから終点セルの手前までを塗るので、**`range_min` より近い区間 (参照 LiDAR で 0.1 m、`resolution` 0.05 m なら 2 セル) も自由と宣言される。** ここは構造的に未観測な区間である。

- 受け入れた理由: その 2 セルはロボット自身が今いる場所であり、静的地図の `LETHAL_OBSTACLE` は既定で保護される (`costmap-design.md` §16.2)。nav2 も同じ挙動である。
- 代案 (始点を `range_min` の位置にする) は出力を点対にする必要があり、§11.3 の理由で採らない。

---

## 12. 依存

| 対象 | 依存 |
|---|---|
| `eltanin_sensor` (ライブラリ) | C++ 標準ライブラリ / Eigen / `eltanin_core` のみ。clearing 用の投影を足しても変わらない |
| `eltanin_test_sensor` (テスト) | 上記 + `eltanin::map` (`ObstacleLayer` 結線の検証) + GoogleTest |

`laser_geometry` / PCL / tf2 / ROS 2 / yaml-cpp / `eltanin_map` をライブラリに持ち込まない。`cmake/eltaninConfig.cmake.in` は新しい外部依存がないため変更していない。

STATIC ライブラリであり、ヘッダオンリー (INTERFACE) にしない。宣言と定義の分離が実際にビルドされることを常に検証するためである。

前提: ビーム角は厳密に等間隔 (ROS `LaserScan` と同じ前提。不等間隔のスキャンは扱わない)。センサは水平に取り付けられておりロール / ピッチは 0。単位は [m] / [rad]。
