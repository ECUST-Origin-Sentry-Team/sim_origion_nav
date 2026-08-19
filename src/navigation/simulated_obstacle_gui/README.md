# simulated_obstacle_gui

用于输入地图坐标并持续发布 Nav2 圆形模拟障碍物的 Qt 界面。

## 构建与启动

```bash
colcon build --symlink-install --packages-select simulated_obstacle_gui
source install/setup.bash
ros2 launch simulated_obstacle_gui simulated_obstacle_gui.launch.py
```

实车未使用 `/clock` 时：

```bash
ros2 launch simulated_obstacle_gui simulated_obstacle_gui.launch.py use_sim_time:=false
```

界面输入 `map` 坐标 X/Y，勾选“在代价地图中启用障碍物”，点击“应用并发布”。默认以 10 Hz 持续发布，修改坐标后会在下一周期更新。点击“移除障碍物”会发布 `enabled=false`。默认关闭窗口时也会移除障碍物，可用 `clear_on_exit:=false` 改变该行为。

## 话题

- `/simulated_obstacle/center`：`geometry_msgs/msg/PointStamped`
- `/simulated_obstacle/enabled`：`std_msgs/msg/Bool`

话题名、坐标系和频率均可通过 launch 参数覆盖。
