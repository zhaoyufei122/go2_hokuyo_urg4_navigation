# GO2 实机测试清单

本清单必须有人全程看护。清空测试区，确保人员、宠物和易倒物品离开运动范围，
实体遥控器始终握在手中，并先确认你熟悉已经验证过的硬件停止/接管操作。

固定安全边界：线速度不超过 `0.4 m/s`，角速度不超过 `0.4 rad/s`，横移为 0；
正常有效移动速度不低于 0.3 m/s。任何一项失败都先停止，不继续下一项。

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

1. `/scan`：`timeout 10s ros2 topic hz /scan` 有连续、稳定频率。无消息、频率间歇
   或大量 TF 丢帧均为失败。
2. TF：`ros2 run tf2_ros tf2_echo odom base_footprint` 能连续解析；
   `base_footprint` 的 z 为 0。
3. 静止对齐：RViz 中 `/scan` 应贴合 `/utlidar/cloud_deskewed` 的家具边缘，静止
   30 秒不应明显漂移或旋转。
4. 旋转对齐：进入 mapping 模式后，用实体遥控器低速原地转一圈；墙面和桌椅
   在 map 中应保持固定。出现扇形拖影、整帧跳动或方向相反为失败。

## B. 建图与回环

1. 用实体遥控器走一条闭环，覆盖桌子、椅子和通道两侧。
2. 回到起点时，重复区域应重合，地图不应出现双墙或整体突然错位。
3. 保存 `room_map`，确认 YAML 和 PGM 均存在且 YAML 能引用图像文件。
4. 关闭 mapping，重新打开 PGM/YAML；轮廓、尺度和原点应合理。

## C. AMCL 与短距离导航

1. 启动 navigation 并用 `2D Pose Estimate` 设置初始位姿。
2. AMCL 粒子应收敛，`/scan` 与静态地图家具轮廓重合；明显错层时禁止发目标。
3. 发送 1–2 m 的开阔短目标。通过标准：速度不超过 `0.4 m/s` 和
   `0.4 rad/s`，无横移，进入 0.25 m/0.25 rad 容差后停车。
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
