# GO2 平层高精度二维建图设计

## 目标

在无楼梯的室内平层环境中，用 Unitree GO2 的倒装 L2 点云和
`/utlidar/robot_odom` 建立适合 AMCL 定位与 Nav2 导航的二维占用栅格地图。

地图优先表达墙壁、固定柜体等长期不变结构。桌椅等可能移动的家具不是主要定位
锚点，导航阶段由实时局部代价地图处理。第一版不追求完整三维重建，也不允许
`z`、`roll` 或 `pitch` 进入建图位姿。

成功标准：

- 静止 30 秒时，已有墙线不持续抖动或变厚；
- 转向后不出现明显的扇形拖影；
- 完成闭环后，主要墙体保持约 1--2 个 5 cm 栅格宽；
- 不出现间距超过约 0.10 m 的持久双墙；
- 过滤后的点云中没有随 GO2 一起移动的机身点簇；
- 建图过程不启动 Nav2、速度桥或任何软件运动命令；
- 一次实机路线产生足够的 rosbag 数据，可在 GO2 关机后继续复现与调参。

## 选择的方案

采用“清洁二维扫描 + 平面 odom + SLAM Toolbox + 同步录包”。

没有选择自由六自由度 RTAB-Map，因为 GO2 行走时机身高度、横滚和俯仰的真实变化
会进入其位姿图，而本项目的最终定位和导航状态只需要 `x`、`y`、`yaw`。也不在
第一版采用强制三自由度 RTAB-Map；它保留为清洁二维方案仍缺乏环境辨识度时的后续
备选。

地图分辨率保持 0.05 m。把分辨率直接提高到 0.025 m 只会放大点云、时间同步和
里程计误差，并不会自动得到更准确的墙线。

## 数据流与坐标系

```text
/utlidar/cloud_deskewed (base_link)
  -> pcl_ros CropBox：负裁剪 GO2 机身
  -> /cloud_self_filtered (base_link)
  -> pointcloud_to_laserscan：转换到 base_footprint 并做高度投影
  -> /scan
  -> SLAM Toolbox

/utlidar/robot_odom
  -> planar_odom：仅保留 x、y、yaw
  -> /odom + odom -> base_footprint
  -> SLAM Toolbox 的短时运动约束

SLAM Toolbox
  -> map -> odom
  -> /map
```

`planar_odom` 继续发布 `base_footprint -> base_link` 的瞬时机身高度与残余姿态，以便
点云能正确重力对齐。但是 SLAM Toolbox 的 `base_frame` 必须保持
`base_footprint`，因此这些机身运动不会进入二维地图位姿。

SLAM Toolbox 同时使用 odom 预测和激光扫描匹配。odom 提供连续的短时运动估计，
扫描匹配和闭环负责修正长期漂移；任何一方都不单独决定最终地图。

## 点云清理与二维投影

公共传感器启动文件 `sensors.launch.py` 增加一个 `pcl_ros::CropBox` 组件。它位于
`pointcloud_to_laserscan` 之前，所以建图与后续导航使用完全相同的清洁 `/scan`。

CropBox 在 `base_link` 下采用负裁剪，初始边界复用当前三维链路已使用的机身范围：

- `x`: `[-0.45, 0.45]` m；
- `y`: `[-0.32, 0.32]` m；
- `z`: `[-0.45, 0.30]` m；
- `negative: true`。

该裁剪专门处理倒装 L2 扫到 GO2 自身的问题。输出保留 360 度环境信息，不只保留
前向点，因为闭环和转向后的重定位需要机器人周围的固定结构。

`pointcloud_to_laserscan` 在 `base_footprint` 下采用：

- 高度 `0.12--0.45` m；
- 距离 `0.25--6.0` m；
- 角度 `[-pi, pi]`；
- 角分辨率 `0.5` 度；
- 队列深度 1，优先使用最新扫描。

0.12 m 下限用于排除地面以及机身起伏时穿过零高度附近的闪烁点；0.45 m 上限减少
桌面、椅面等高度结构对静态地图的主导。墙壁和固定柜体仍会在该高度带内提供连续
轮廓。第一轮实机检查若发现柜体底部完全缺失，只调整高度上限；若地面仍成环，则
只提高高度下限。一次只改变一个边界。

二维投影前不加入 VoxelGrid。`pointcloud_to_laserscan` 本身会为每个角度选取最近
有效点，额外体素化可能减少细柜边和墙角的角向覆盖。原有三维建图链路保持不变。

## SLAM Toolbox 精度策略

保持 Ceres 默认求解器和 0.05 m 栅格，调整扫描接收密度与回环门槛：

- `map_update_interval: 1.0`；
- `max_laser_range: 6.0`；
- `minimum_time_interval: 0.15`；
- `minimum_travel_distance: 0.05`；
- `minimum_travel_heading: 0.05`；
- `check_min_dist_and_heading_precisely: true`；
- `scan_buffer_size: 10`；
- `scan_buffer_maximum_scan_distance: 6.0`；
- `link_match_minimum_response_fine: 0.20`；
- `link_scan_maximum_distance: 1.5`；
- `loop_search_maximum_distance: 2.0`；
- `loop_match_minimum_chain_size: 10`；
- `loop_match_minimum_response_coarse: 0.45`；
- `loop_match_minimum_response_fine: 0.55`。

其它相关扫描匹配参数先保持现值，避免在没有实机录包证据时同时改变过多变量。
第一版偏向拒绝可疑回环：漏掉一次回环可以用同一 rosbag 放宽阈值重跑，错误回环
却可能把整张地图永久拉坏。

## 录包与 RViz 可观测性

`mapping.launch.py` 增加：

- `record_bag` 启动参数，默认 `true`；
- `bag_output_root` 参数，默认 `~/go2_mapping_bags`；
- 每次启动创建 `YYYYMMDD_HHMMSS` 命名的唯一子目录；
- RViz 显示 `/cloud_self_filtered`、`/scan`、`/map` 和 TF。

录制以下话题：

- `/utlidar/cloud_deskewed`；
- `/utlidar/robot_odom`；
- `/cloud_self_filtered`；
- `/scan`；
- `/map`；
- `/tf`；
- `/tf_static`。

原始点云和原始 odom 允许离线重新运行过滤、平面化与 SLAM；处理后点云、扫描和
地图用于比较每一级输出。回放时关闭新录包，并避免同时回放已录 TF 和由处理节点
重新发布的同名 TF；文档提供明确的回放话题选择命令。

录包失败只应打印清晰错误，不得启动运动功能，也不得让建图节点静默退出。磁盘
空间应在实机测试前检查；录包目录不会自动删除或覆盖。

## 建图操作流程

1. 在能同时看到墙壁与固定柜体的位置启动，而不是从桌椅密集或长直空走廊开始。
2. 静止 30 秒，检查过滤后点云无机身点簇，`/scan` 与固定结构对齐。
3. 使用实体遥控器，以约 `0.3--0.4 m/s` 平滑移动；转向保持在已确认可行的
   `0.4--0.6 rad/s`，避免狭窄处快速原地旋转。
4. 先沿空间外围完成一条闭环并回到起点。
5. 确认地图没有突然整体跳动或形成双墙，再覆盖内部通道。
6. 最后再次回到起点附近，观察闭环结果后保存地图。
7. 正常停止 launch，等待 rosbag 写完索引；不得通过电脑或 GO2 突然断电结束录制。

## 测试与验收

自动测试应覆盖：

- CropBox 参数、负裁剪标志和话题连接；
- LaserScan 高度、距离、角度及目标坐标系；
- SLAM Toolbox 的平面坐标系、分辨率、更新阈值和回环阈值；
- mapping launch 的录包参数、话题集合和唯一输出路径；
- mapping launch 不包含 Nav2、`go2_cmd_vel_bridge` 或软件运动请求发布者；
- 所有 Python 启动文件可导入，YAML 可解析；
- 现有 2D 导航与 3D 建图测试不被破坏。

无硬件验证包括 `colcon build`、项目测试、launch 参数展开以及短时间无传感器启动
检查。实机验收按目标中的静止、转向、闭环、双墙和自体点簇标准执行。

若第一轮失败，排查顺序固定为：

1. 过滤后点云是否仍含机身或地面；
2. `/scan` 与点云、TF 的时间和方向是否一致；
3. odom 在静止和闭环时的平面漂移；
4. 相邻扫描匹配；
5. 最后才调整闭环阈值或栅格参数。

## 非目标与后续选项

本次不修改 AMCL、Nav2 控制器参数或三维 RTAB-Map 链路；它们在新地图通过建图
验收后单独处理。也不加入只保留前向视野、自由六自由度位姿或 2.5 cm 栅格。

若清洁后的二维地图通过验收但环境仍因长直墙或重复柜体而无法稳定辨识，下一阶段
才评估强制三自由度的 RTAB-Map 点云配准，并将其结果投影为 Nav2 使用的二维地图。
