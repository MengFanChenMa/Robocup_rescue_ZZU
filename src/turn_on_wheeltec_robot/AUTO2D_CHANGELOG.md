# Auto2D Changelog

## auto2d-v0.1.3 (2026-04-14)
- 更新 `launch/turn_on_wheeltec_robot.launch`：新增 `local_planner` 与 `pure3d_local_planner` 参数，支持 `teb/dwa` 切换。
- 更新 `launch/mapping.launch`：新增并透传 `local_planner` 参数到底层启动链路。
- 更新 `launch/rrt_slam.launch`：新增 `mapping_mode` 与 `local_planner` 参数，支持建图/导航算法组合测试。
- 更新 `launch/auto_2d_mapping.launch`：透传 `mapping_mode` 与 `local_planner` 参数，实现一键切换算法。

## auto2d-v0.1.2 (2026-04-14)
- 更新 `launch/include/algorithm_gmapping.launch`：针对 TX2 做保守提速调参，降低更新频率与地图分辨率（`map_update_interval: 0.2 -> 0.5`，`linearUpdate: 0.05 -> 0.12`，`angularUpdate: 0.1 -> 0.2`，`lskip: 1 -> 2`，`delta: 0.05 -> 0.08`）。
- 更新 `launch/simple.launch`：将 `rrt_rate: 20 -> 12`，降低 `filter.py` 与 `assigner.py` 频率，减轻 CPU 占用。

## auto2d-v0.1.1 (2026-04-14)
- 新增 `rviz/auto_2d_mapping.rviz`：预置自主建图可视化话题显示（`clicked_point`、`detected_points`、`frontiers`、`centroids`、`global_detector_shapes`、`local_detector_shapes`）。
- 更新 `launch/auto_2d_mapping.launch`：新增 `open_rviz` 与 `rviz_config` 参数，支持一键打开RViz并加载配置。

## auto2d-v0.1.0 (2026-04-14)
- 新增 `launch/auto_2d_mapping.launch`：提供一键自主2D建图总入口，复用现有 `rrt_slam.launch`。
- 新增 `scripts/auto_init_explore.py`：自动向 `/clicked_point` 发布RRT初始化5点，替代RViz手工打点。
- 新增参数化配置：探索边界、起点、启动等待、最小订阅数、发布轮次与间隔。

## 备注
- 版本标注格式：`[auto2d-vX.Y.Z][YYYY-MM-DD]`。
