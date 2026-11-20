# Town04_square_parking — 按点云几何在方框左侧布的一排车位

`carla_maps/Town04_square/`（按 `type=square` 裁出来的子图）加了一排垂直车位，
位置和朝向由**点云实测的场地边界**决定，不是随手摆的。

## 一、点云给了哪些依据

在方框内对 46.27M 点做 0.5 m / 0.25 m 占用栅格（`z > 0.35 m` 记为障碍），得到的实测结构：

| 判据 | 点云实测值 | 用途 |
|---|---|---|
| 场地西侧围挡 | **x ≈ 263.5**，从 y 177.2 到 y 241.7 连续不断（0.5 m 栅格上是一条 64 m 不中断的线） | 车位"背靠"的边界 |
| 场地南侧墙 | y ≈ 177.0（x 264.0–275.5 连续） | 车位排的南端 |
| 场地北侧墙 | y ≈ 241.0（x 265.5–275.5 与 x 291.5–296.0 两段） | 车位排的北端 |
| 场地东侧路缘 | x ≈ 300 | `parking_lot` 东边界 |
| 场内已停车辆 | 4.5 m × 2.0 m 连通域，**长轴沿 x**（x 跨度 4.5、y 跨度 2.0） | 证明这里就是**垂直式停车**，开口朝 ±x |
| 车辆分布 | 最靠西一列车在 x 269.5–275.0（车心 x≈272.5）；x 269 以西为空地 | 车位排放在 x 264–269，正好避开已停车 |

结论：**西侧第一排 = 贴着 x≈263.5 围挡、沿 y 方向排一列、车尾朝西、开口朝东（+x）**。

## 二、生成结果

```
parking_lot  : (264.0,178.0) (300.0,178.0) (300.0,241.0) (264.0,241.0)   36.0 x 63.0 m
车位排       : 22 个，x 264.0–269.0（深 5.0 m），宽 2.5 m，间距 2.8 m
第一个车位   : y 179.25–181.75（沿边 0.50 m 起）
```

`parking_space` 全部按 lanelet2 规则写成 **2 个点 + `width` 标签**的 lineString，
中线从车位底部指向通道口；`parking_lot` 是闭合 `area=yes` 多边形。

```
GOAL   x=266.500  y=210.550  yaw=+π/2 (90°, 车头朝 −x)
EGO    x=272.000  y=210.550        （车位开口外 3 m 的通道内）
CARLA  spawn = (272.000, -210.550)  # CARLA 的 y 轴取反
```

## 三、逐车位点云校验

每个车位 2.5×5.0 m 的足迹按 0.10 m 栅格统计 `z>0.35 m` 的占用比例：

```
#1 1.3%   #2..#18 0.0%   #19 0.3%   #20 0.0%  #21 0.0%  #22 0.6%
```

最大 1.3%，说明整排都落在干净地面上，没有压在车辆、树、隔离墩上。
（若换成 x 269–274 那一带，占用率会到 20–50%——那里是已经停着的车。）

## 四、用法

```bash
mkdir -p ~/autoware_data/maps/Town04_square_parking
cp lanelet2_map.osm pointcloud_map.pcd map_projector_info.yaml \
   ~/autoware_data/maps/Town04_square_parking/
# map_path:=$HOME/autoware_data/maps/Town04_square_parking
# goal 用 parking_goal.txt 里的 goal_x / goal_y / goal_yaw_rad
```

自检（`draw_parking_row.py` 内置）全部 OK：lot 闭合、22 个 space 都是 2 点且带 width、
每个车位在 lot 内、goal 与 ego 在 lot 内、goal 距最近 lanelet 6.20 m（>0.01 m，
满足 `scenario_selector`「终点不在车道内」的判定）。

## 五、必须说清楚的三件事

1. **点云里没有车位线**。这份 `pointcloud_map.pcd` 只有 `x y z`，没有 intensity/颜色；
   CARLA 的路面标线是贴图，不产生几何点。所以上面这排车位是**按场地边界 + 车辆停放
   方向推出来的**，"依据点云"指的是依据可测的墙体/围挡/已停车辆几何，不是识别标线。
2. **这不是 CARLA 世界里画好的停车场**。它在点云里是一片被围挡圈起来的水泥场地，
   场内散停着若干辆车、两个集装箱；补上车位图元只是让 Autoware 的 PARKING 场景有地方可停。
3. **未实机验证**。本机没有 ROS 2 / CARLA，以上是几何、拓扑与点云占用三重静态校验。

## 六、重新生成 / 回滚

```bash
python tools/draw_parking_row.py \
  --input  carla_maps/Town04_square/lanelet2_map.osm \
  --output carla_maps/Town04_square_parking/lanelet2_map.osm \
  --lot 264 178 300 241 --row left \
  --width 2.5 --depth 5.0 --pitch 2.8 --margin 0.5 \
  --aisle-offset 3.0 --cloud-check --preview
```

回滚：删除本目录即可，`carla_maps/Town04_square/` 与 `carla_maps/Town04/` 都不会被改动。
