# GO2 实机测试清单

本清单必须有人全程看护。清空测试区，确保人员、宠物和易倒物品离开运动范围，
实体遥控器始终握在手中，并先确认你熟悉已经验证过的硬件停止/接管操作。

固定安全边界：线速度不超过 `0.4 m/s`，非零角速度绝对值为
`0.4-0.8 rad/s`，横移为 0；正常有效移动速度不低于 0.3 m/s。任何一项失败都
先停止，不继续下一项。

## 自动检查

```bash
source /home/yufei/Desktop/unitree_ros2/setup.sh
cd /home/yufei/Desktop/go2_base_navi
colcon build --symlink-install
source /home/yufei/Desktop/go2_base_navi/install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

通过标准：构建成功，测试结果无 failure/error。

## A. 传感器与 TF（不启动运动桥接）

启动 `sensors.launch.py` 后依次检查：

1. `timeout 10s ros2 topic hz /cloud_self_filtered` 有连续输出；RViz 中 GO2 身体
   附近没有随机器人一起移动的机身点簇，地面不形成近距离圆环。
2. `timeout 10s ros2 topic hz /scan` 有连续稳定频率；无消息、频率间歇或大量 TF
   丢帧均为失败。
3. `ros2 run tf2_ros tf2_echo odom base_footprint` 能连续解析，且
   `base_footprint` 的 z 为 0。
4. 静止 30 秒，RViz 中固定墙壁不持续移动、旋转或加粗。
5. 进入 mapping 模式后，用实体遥控器做一次受控转向；墙壁和柜体不应出现扇形拖影、
   方向相反或整帧跳动。

任一项失败都先停止，不进入建图路线。

## B. 建图与回环

1. 启动 `mapping.launch.py record_bag:=true`，确认终端显示新的 rosbag 输出目录；
   进程中必须没有软件运动指令、Nav2 或速度桥。
2. 全程只用实体遥控器移动：先走外围闭环，确认地图未错误跳动，再走内部通道，
   最后回到起点附近。
3. 回环后主要墙体应保持 1--2 个 5 cm 栅格厚度，不应出现间距超过 0.10 m 的
   持久双墙。
4. 保存 `room_map`，确认 YAML 和 PGM 同时存在且 YAML 正确引用图像文件。
5. 在建图终端按 Ctrl-C，等待 rosbag 正常结束；`ros2 bag info` 必须能读取元数据，
   并列出原始点云、原始 odom、`/cloud_self_filtered`、`/scan`、`/map` 和 TF
   话题。

## C. AMCL 与短距离导航

1. 启动 navigation 并用 `2D Pose Estimate` 设置初始位姿。
2. AMCL 粒子应收敛，`/scan` 与静态地图家具轮廓重合；明显错层时禁止发目标。
3. 发送 1–2 m 的开阔短目标。通过标准：线速度不超过 `0.4 m/s`，Unitree
   Move 请求中非零 `z` 的绝对值处于 `0.4-0.8 rad/s`，无横移，进入
   0.25 m/0.25 rad 容差后停车。
4. Ctrl-C 关闭 launch；监听 `/api/sport/request` 时应看到 StopMove（API 1003），
   机器狗不应继续运动。

## D. 椅子障碍测试

选择地图中已有的一把椅子，让目标路径经过椅子附近但不要故意指向碰撞点。实体
遥控器保持可立即接管。通过标准：椅子出现在局部代价地图中，Nav2 绕行；若椅子
进入 StopZone，则机器人直接停止而不是以低于 0.3 m/s 的速度爬行。发生接触、
代价地图没有椅子或继续向障碍运动均为失败。

## E. `/scan` 丢失与 StopMove

只在开阔区域、短目标、实体遥控器在手时测试。先用
`pgrep -af pointcloud_to_laserscan_node` 找到投影节点 PID；机器狗开始短距离运动后，
用 `kill -STOP <PID>` 暂停投影，模拟 `/scan` 丢失。通过标准：0.5 秒源超时后
Collision Monitor 输出零速，安全桥接发布一次 StopMove（API 1003），机器狗停止。

立即执行 `kill -CONT <PID>` 恢复进程，再确认 `/scan` 恢复；若未停止，立即用
实体遥控器接管并终止测试。不要通过拔网线做第一轮断流测试，因为它会同时影响
点云、里程计和控制链。

最后再验证整个 Nav2 输出链停止：在安全位置终止 navigation launch。即使没有新
`/cmd_vel`，桥接的 0.5 秒看门狗或关闭路径也必须发 StopMove。任一场景仍持续运动
都视为失败，禁止继续自动导航。

## F. 3D 建图验收（没有软件运动指令）

这一项与二维导航分开测试。关闭所有 mapping、sensors 和 navigation 启动实例后，
只启动 `mapping_3d.launch.py`；整个过程中只用实体遥控器移动，不发送 Nav2 目标。

默认 RViz 的 Fixed Frame 必须为 `map_3d`。确认四个预配置显示均正常：

- `Live Filtered Cloud` 随机器狗移动并只短暂保留当前扫描；
- `Accumulated 3D Map` 随行走不断增长；
- `Projected 2D Map` 同步显示平面占用区域；
- `Mapping Path` 记录已经走过的轨迹。

回环发生时累计地图和轨迹可能整体小幅重新对齐，这是正常的图优化结果。

1. 静止检查 `/cloud_3d_filtered`：应保留地面、墙面、桌面和椅腿，机器狗身体
   周围不应有随机器人一起移动的机身点簇。若有，先调整 CropBox 边界，再考虑
   ICP 参数。
2. 用实体遥控器缓慢走动并轻微转向。桌面和椅腿的高度应稳定，墙面不应分层，
   点云不应随着机身俯仰被重复画成多层。
3. 走一条闭环并回到起点附近。通过标准是回环后重叠区域收紧，不出现持久双墙，
   也不发生整张图跳到错误位置。
4. 在建图终端按 Ctrl-C，等待 RTAB-Map 完成保存；确认
   `maps/room_3d.db` 存在且大小非零。不要直接断电结束数据库写入。
5. 检查启动节点和话题：不得有 `go2_cmd_vel_bridge`、Nav2 或
   `/api/sport/request` 发布者。通过标准是全程没有软件运动指令。

## G. 头部思岚二维雷达

这一项使用 start_slamtec_lidar.sh、slamtec_mapping.launch.py 和
slamtec_navigation.launch.py，不得同时运行旧 L2 的 sensors.launch.py、
mapping.launch.py 或 navigation.launch.py。

1. 雷达启动日志必须包含健康状态 OK 和 Sensitivity 模式；电脑端
   timeout 10s ros2 topic hz /scan 应稳定收到约 10 Hz 以上数据。
2. 使用 ros2 topic echo /scan sensor_msgs/msg/LaserScan --once --no-arr
   检查 frame_id 为 laser、角度覆盖约 -pi 到 pi，并存在有限距离值。
3. tf2_echo base_link laser 应显示安装变换。原地不动时，RViz 的橙色扫描轮廓
   必须稳定；前方实物应显示在机器人前方。若整体方向固定偏转，先校准
   laser_yaw，不要靠 SLAM 参数掩盖安装误差。
4. 思岚建图启动实例中不得有 pointcloud_to_laserscan_node、cloud_self_filter
   或 go2_cmd_vel_bridge；全程只用实体遥控器移动。
5. 保存 slamtec_room 后关闭建图，再启动思岚导航。先设置初始位姿并确认扫描与
   墙壁、固定柜体重合，再做 1--2 m 短目标测试；沿用 A--E 的速度、障碍物、
   /scan 断流和 StopMove 验收标准。
6. 停止雷达脚本时日志必须出现 Stop motor，随后串口不应再被 sllidar_node
   占用。
