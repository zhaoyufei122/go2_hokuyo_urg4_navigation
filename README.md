# GO2 实机二维导航与三维建图

本工作空间有两条互不混用的管线：

- 二维管线把 GO2 的 `/utlidar/cloud_deskewed` 按高度切片投影为
  `/scan`，用 SLAM Toolbox 建图，再用 AMCL + Nav2 导航；
- 三维管线裁掉 GO2 机身点、做 8 cm 体素降采样，再用 RTAB-Map 生成带回环
  优化的三维数据库和可交互点云地图。

`cloud_deskewed` 是运动去畸变后的当前点云，不是已经累计好的地图。

安全配置固定为：前进/后退最大 `0.4 m/s`、非零转向绝对值为
`0.4-0.6 rad/s`、横移为 0；零转向仍保持为零，RPP 原地转向目标速度是
`0.6 rad/s`，有效移动下限是 `0.3 m/s`。碰撞监测使用 StopZone，只停车，不把
速度降到机器狗无法行走的 0.3 以下。桥接只调用 Move(1008) 和 StopMove(1003)，
不会自动站立、切步态或执行特技。

## 1. 安装、编译

首次安装二维点云投影和三维建图依赖：

```bash
sudo apt install ros-jazzy-pointcloud-to-laserscan
sudo apt install ros-jazzy-rtabmap-ros ros-jazzy-pcl-ros
```

每个新终端都必须先加载 Unitree 环境，再加载本工作空间。首次编译执行：

```bash
source /home/yufei/Desktop/unitree_ros2/setup.sh
cd /home/yufei/Desktop/go2_base_navi
colcon build --symlink-install
source /home/yufei/Desktop/go2_base_navi/install/setup.bash
```
每次开机或重新插线后，先检查 GO2 有线接口：

```bash
ip -brief address show enp130s0
```

开始建图或导航前应显示 `UP`，并有 `192.168.123.222/24`。若显示 `DOWN`，先确认
GO2 已开机且网线链路正常，不要启动导航。

## 2. 只读传感器检查

先让 GO2 静止，启动不含运动桥接的传感器管线：

```bash
ros2 launch go2_base_nav sensors.launch.py
```

在另一个按上述顺序 source 的终端检查：

```bash
timeout 10s ros2 topic hz /scan
timeout 10s ros2 run tf2_ros tf2_echo odom base_footprint
timeout 10s ros2 topic echo /odom --once --no-arr
```

应持续收到 `/scan`；TF 应能解析；`/odom` 的 child frame 应为
`base_footprint`，位置 z 为 0。检查完成后 Ctrl-C 停止 sensors，避免下一步重复
启动同名节点。

## 3. 建图

确保实体遥控器在手边，机器狗周围留出安全空间。建图启动文件不会发送速度，
走动完全由实体遥控器控制：

```bash
ros2 launch go2_base_nav mapping.launch.py
```

缓慢走遍平层环境，桌椅边缘至少从两个方向扫到，最后回到起点附近以触发回环。
在另一个正确 source 的终端保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f /home/yufei/Desktop/go2_base_navi/maps/room_map
```

确认同时生成 `maps/room_map.yaml` 与 `maps/room_map.pgm` 后，再在建图终端
Ctrl-C。地图文件默认不提交到 Git。

## 4. 三维建图（RTAB-Map，只建图）

这个启动文件只运行里程计 TF 适配、机身裁剪、体素滤波、RTAB-Map 和可视化。
它不会启动 Nav2、速度桥或任何软件遥控节点；机器狗走动只允许使用实体遥控器。

L2 虽然倒装并会扫到地面和机身，但三维模式不只保留前向点：房间四周、地面、
桌面和椅腿都是回环与三维配准需要的几何信息。点云先变换到 `base_link`，再
删除机身框 x `[-0.45, 0.45]` m、y `[-0.32, 0.32]` m、
z `[-0.45, 0.30]` m 内的点，框外的 360 度环境点会保留。

第一次建新图使用 `new_map:=true`。它只会清空
`database_path` 指定的这个数据库，所以路径必须确认无误：

```bash
ros2 launch go2_base_nav mapping_3d.launch.py \
  database_path:=/home/yufei/Desktop/go2_base_navi/maps/room_3d.db \
  new_map:=true
```

先不要移动。在另一个按前述顺序 source 的终端检查过滤点云和完整三维 TF：

```bash
timeout 10s ros2 topic echo /cloud_3d_filtered --once --no-arr
timeout 10s ros2 run tf2_ros tf2_echo odom base_link
```

RTAB-Map 窗口中应看到房间、地面和家具，但机器狗身体附近不应有持续跟随的点簇。
若仍能看到机身，先调整机身裁剪框，不要先改 ICP 参数。确认点云正确后，用实体
遥控器缓慢走一条闭环，从两个方向扫到桌椅，并回到起点附近等待回环修正。

结束时在启动终端按 Ctrl-C，让 RTAB-Map 完整写盘，再检查数据库：

```bash
ls -lh /home/yufei/Desktop/go2_base_navi/maps/room_3d.db
```

要继续同一张图，保持相同路径并改用 `new_map:=false`：

```bash
ros2 launch go2_base_nav mapping_3d.launch.py \
  database_path:=/home/yufei/Desktop/go2_base_navi/maps/room_3d.db \
  new_map:=false
```

本版不做 3D 定位或自主导航；它先用于比较三维地图与回环效果。现有导航仍使用
第 3 节保存的二维地图，确认三维数据库质量后再单独接入 RTAB-Map 定位与 Nav2。

## 5. 二维地图导航

先用实体遥控器让 GO2 正常站立，并把它放到地图中的已知、开阔位置。地图参数
必须使用绝对路径：

```bash
ros2 launch go2_base_nav navigation.launch.py map:=/home/yufei/Desktop/go2_base_navi/maps/room_map.yaml
```

RViz 中先点 `2D Pose Estimate`，给出机器狗在地图上的初始位姿；等待激光与地图
重合后，再用 `Nav2 Goal` 发送一个 1–2 m、无遮挡的短目标。第一轮始终握住实体
遥控器，不要直接测试狭窄通道或贴近桌椅的目标。

## 紧急停止

实体遥控器是第一优先级的紧急接管手段。发现方向错误、地图错位、激光消失或
将要碰撞时，立即用实体遥控器停止/接管，不要只依赖软件。随后在导航终端
Ctrl-C；桥接关闭时会再次尝试发送 StopMove。完整实机验收顺序见
`docs/TESTING.md`。
