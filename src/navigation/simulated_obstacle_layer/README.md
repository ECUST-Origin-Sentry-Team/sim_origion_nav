# simulated_obstacle_layer

Nav2 二维代价地图插件。它订阅一个带坐标系的圆心和一个启停状态，在本层中绘制圆形致命障碍物。圆心会从消息的 `frame_id` 自动转换到当前代价地图坐标系，因此同一个 `map` 坐标可同时供全局 `map` 代价地图和局部 `odom` 代价地图使用。

## 话题

- `/simulated_obstacle/center`：`geometry_msgs/msg/PointStamped`，圆心；GUI 默认发布 `frame_id=map`。
- `/simulated_obstacle/enabled`：`std_msgs/msg/Bool`，`true` 显示、`false` 移除。

两个订阅都使用 Reliable + Volatile QoS，既兼容持续发布的 GUI，也兼容普通的 `ros2 topic pub`。GUI 发布端使用 Transient Local，并默认持续发布。

## 参数

- `enabled`：是否启用整个插件，默认 `true`。
- `center_topic`：圆心话题。
- `enabled_topic`：启停话题。
- `radius`：原始致命圆半径，单位 m，必须大于 0。
- `cost`：圆内栅格代价值，范围 1～254，默认 254。
- `transform_tolerance`：查询最新 TF 的最长等待时间，单位 s。

插件必须放在 `inflation_layer` 之前，例如：

```yaml
plugins: ["static_layer", "simulated_obstacle_layer", "inflation_layer"]
simulated_obstacle_layer:
  plugin: "simulated_obstacle_layer::SimulatedObstacleLayer"
  center_topic: /simulated_obstacle/center
  enabled_topic: /simulated_obstacle/enabled
  radius: 0.40
  cost: 254
  transform_tolerance: 0.10
```

`radius` 是致命圆本体半径；最终代价地图还会被后续 `inflation_layer` 再膨胀。

## 构建

```bash
colcon build --symlink-install --packages-select simulated_obstacle_layer
source install/setup.bash
```
