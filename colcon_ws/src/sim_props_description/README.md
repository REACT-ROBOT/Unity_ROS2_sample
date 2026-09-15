# sim_props_description

シミュレータ v1.4.0 の新機能を試すための「置物 (prop)」を集めたパッケージです。
prop はどれも URDF エンティティなので、ロボットと同じように spawn / delete /
reset でき、`get_entities` にも並びます。

```bash
ros2 launch sim_props_description spawn_prop.launch.py prop:=weeds
ros2 launch sim_props_description spawn_prop.launch.py prop:=magnetic_course
ros2 launch sim_props_description spawn_prop.launch.py prop:=gnss_canyon
```

`x` / `y` / `z` / `Y` で置く位置と向きを変えられます。`name` を与えると
エンティティ名を指定できます (既定は URDF の robot 名)。

| prop | 何のため | 置かれるもの |
|---|---|---|
| `weeds` | LiDAR には映るが衝突しない物体 (`sensor_only`) | x=1.5〜4.0 / y=±0.9 に草 56 株 (草丈 0.45〜0.75 m) |
| `magnetic_course` | 磁気ラインセンサ (`magnetic_guide`) | 1 周 約21 m の楕円コース + マーカ 3 箇所 |
| `gnss_canyon` | GNSS の劣化 (`gnss_sky_view` / `gnss`) | 幅 4 m・高さ 12 m のビル街 (x=4〜26、横道 2 本) |

## sensor_only (weeds)

`<collision_material><sensor_only value="true"/>` を付けた `<collision>` は Unity の
トリガコライダになります。レイキャストには当たるので LiDAR には映り、接触解決には
入らないのでロボットはすり抜けます。`get_contact_events` にも残りません。

collision を持たないリンクでは LiDAR のレイが当たらず「何も無い」のと同じになるので、
collision は普通に書いて `sensor_only` でトリガにするのが正しい書き方です。

株ごとにリンクを作らないでください。リンクは全て ArticulationBody になり、PhysX は
1 articulation あたり 64 body までしか持てません。リンクは 1 つにして `<collision>` を
並べます (リンクあたりのコライダ数に上限はありません)。トリガだけのボディは何にも
支えられないので、`world` リンクに fixed で吊るす必要もあります。

`urdf/weeds.urdf` の `<sensor_only value="true"/>` を `false` にして spawn し直すと、
同じ草が普通のコライダになってロボットを止めます。効果を確かめるのに使えます。

## 磁気テープコース (magnetic_course)

`courses/*.json` に折れ線でコースを書き、`scripts/gen_tape_urdf.py` で URDF に
変換したものです。テープは `<collision_material><magnetic_tape polarity="track|marker"/>`
を付けた薄い box で、`magnetic_tape` は `sensor_only` を含意します (踏んで走れる)。

```bash
# コースを書き換えたら生成し直す
ros2 run sim_props_description gen_tape_urdf.py \
  courses/oval_with_markers.json urdf/magnetic_course.urdf
```

`urdf/magnetic_course.urdf` の楕円は直線部が x=0〜6 の y=0 と y=3、両端が半径 1.5 m の
半円です。原点で +x を向いて spawn したロボットはそのまま軌道の上に乗ります。
マーカはテープ端から 15〜30 mm 外側 (データシートの決まり) に置いてあります。

## アーバンキャニオン (gnss_canyon)

衛星を遮るのは普通のコライダだけなので、ビルには `sensor_only` を付けていません
(雑草や磁気テープはトリガなので、既定では衛星を遮りません)。街路は幅 4 m・
壁高 12 m で、途中に 2 m の横道が 2 本あります。そこを通るたびに空が開けるので、
測位品質が一度回復する様子が見られます。

街路軸 (+x) 方向の空は塞げないため、どれだけ深くしても衛星が全て消えることはありません。
幅 8 m・高さ 10 m で試したときは使用衛星が 20 → 11 に減るだけで RTK Fix が保たれたので、
等級まで落とすにはこのくらいの狭さが要ります。逆に劣化を弱めたいときは街路を広げます。
壁を高く / 街路を狭くするほど反射 (NLOS) の余分な行程も伸びます。
