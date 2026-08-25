# Hokuyo 管线速查手册

每条命令都在 `/home/yufei/Desktop/go2_base_navi` 目录下执行。
新开终端先 source:

```bash
source /home/yufei/Desktop/unitree_ros2/setup.sh
source /home/yufei/Desktop/go2_base_navi/install/setup.bash
```

## 1. 雷达（终端 1，保持开着）

```bash
# 雷达插在狗的 Jetson 上(推荐,自动对时):
GO2_HOKUYO_SSH_TARGET=unitree@192.168.123.18 ./scripts/start_hokuyo_lidar.sh

# 雷达插在本机 USB:
./scripts/start_hokuyo_lidar.sh
```

验证:另一个终端 `timeout 10s ros2 topic hz /scan` 应约 10 Hz。

## 2. 建图(终端 2)

```bash
./scripts/start_hokuyo_mapping.sh

# 续建：在旧图基础上继续建（房间格局微调时用，狗停在原建图起点）：
./scripts/start_hokuyo_mapping.sh \
  continue_from:=/home/yufei/Desktop/go2_base_navi/maps/hokuyo_door_open
# 存的时候换个新名字（如 hokuyo_door_open_v2），别覆盖原图。
```

站立、慢速、转弯分段;量程 4 m,别离墙太远。门开着建图可覆盖两侧。

## 3. 保存地图(终端 3,建图终端保持开着!)

```bash
# SLAM Toolbox 定位用的图结构
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: /home/yufei/Desktop/go2_base_navi/maps/hokuyo_room}"
# AMCL/显示用的占据栅格图
ros2 run nav2_map_server map_saver_cli -f /home/yufei/Desktop/go2_base_navi/maps/hokuyo_room
```

确认 maps/ 下有 `.posegraph .data .yaml .pgm` 四个文件后再 Ctrl-C 建图。

## 4. 导航(终端 2)

```bash
# 与 Final_Work 联用（task_planner 巡逻/拖人区）的完整命令——
# 注意 use_cmd_vel_bridge:=false（关内置桥，统一用 Final_Work 的桥，
# 保留 vy 横移）：
cd ~/Desktop/go2_base_navi
./scripts/start_hokuyo_navigation.sh maps/hokuyo_room.yaml \
  localization:=slam_toolbox \
  slam_posegraph:=maps/hokuyo_room \
  map_start_x:=-0.905 map_start_y:=0.162 map_start_yaw:=-0.089 \
  use_cmd_vel_bridge:=false

# 2026-08-24 起：slam_posegraph 给相对路径也行，脚本自动转绝对路径；
# 文件不存在会直接报错退出（此前相对路径静默失败 → 空图定位 →
# 看起来像重新建图，rviz 里位置很不对）。

# 独立用（自带 cmd_vel 桥，不接 task_planner）:
./scripts/start_hokuyo_navigation.sh localization:=slam_toolbox \
  slam_posegraph:=maps/hokuyo_room

# AMCL 模式(需 RViz 里点 2D Pose Estimate):
./scripts/start_hokuyo_navigation.sh
```

## 5. 点位(航点)

```bash
# 狗停在某点,读出当前 x/y/yaw(只打印不保存):
    ./scripts/record_waypoint.py

# 追加存到文件(攒任务点位):
./scripts/record_waypoint.py missions/my_task.yaml

# 按文件顺序自动走一遍(Nav2 需已启动;第二个参数=每点停顿秒数):
./scripts/follow_waypoints.py missions/my_task.yaml 3

# 相对移动(不走 Nav2,抓取后退场等用;负数=倒退,只允许短距离):
./scripts/drive_relative.py -0.5        # 倒退 0.5 m
./scripts/drive_relative.py 0.3 0.35    # 以 0.35 m/s 前进 0.3 m

# slam_toolbox 模式下可用启动参数指定初始位姿(yaw 为弧度),免去手点。
# 两套地图(两种模式)完整命令:

# A. 老房间图(hokuyo_room) + room_mission 任务:
./scripts/start_hokuyo_navigation.sh maps/hokuyo_room.yaml \
  localization:=slam_toolbox \
  slam_posegraph:=/home/yufei/Desktop/go2_base_navi/maps/hokuyo_room \
  map_start_x:=-0.905 map_start_y:=0.162 map_start_yaw:=-0.089
./scripts/follow_waypoints.py missions/room_mission.yaml 3

# B. 开门版图(hokuyo_door_open) + door_open_mission 任务:
./scripts/start_hokuyo_navigation.sh maps/hokuyo_door_open.yaml \
  localization:=slam_toolbox \
  slam_posegraph:=/home/yufei/Desktop/go2_base_navi/maps/hokuyo_door_open \
  map_start_x:=1.465 map_start_y:=0.084 map_start_yaw:=2.178
./scripts/follow_waypoints.py missions/door_open_mission.yaml 3
```

RViz 里读坐标:工具栏 `+` 加 Publish Point,点地图,
`ros2 topic echo /clicked_point --once`。2D Pose Estimate 拖拽可以连朝向一起量,
但注意 slam_toolbox 模式下点它会真的搬动定位,不要乱点。

## 6. 常用参数覆盖(追加在脚本命令后)

| 参数 | 默认 | 含义 |
|---|---|---|
| `laser_x:=` `laser_z:=` | 0.20 / 0.10 | 雷达安装位姿 |
| `GO2_HOKUYO_ANGLE_MIN/MAX` | ±2.0944 rad | 扫描角度窗口(环境变量) |
| `GO2_HOKUYO_SERIAL` | /dev/ttyACM0 | 雷达串口(环境变量) |

## 7. 排错速查

| 现象 | 处理 |
|---|---|
| 串口打不开 | 停 ModemManager + `sudo chmod 666 /dev/ttyACM0` |
| /scan 无数据 | 检查雷达终端还在;`ros2 topic info /scan_raw` 应有 1 发布者 |
| 设完位姿站着不动 10 秒后丢失 | AMCL 特性,换 slam_toolbox 定位模式 |
| 转身卡住不动 | 已修(max_angular_accel 3.5);还卡看 controller 日志 |
| 导航中止 aborted | 看 RViz 是否被活障碍包围;`ros2 topic info /map` 应为 1 个发布者 |
| 残留进程捣乱 | `pkill -f go2_base_nav; pkill -f slam_toolbox; pkill -f rviz2` |

## 8. 安全

前进/后退 ≤0.4 m/s,转向 0.4-0.8 rad/s,不横移;StopZone 只停车。
实体遥控器是第一优先级急停手段。



















  ~/Desktop/Final_Work/scripts/grip_lidar_compare.py

  用法（需要 Cyclone 环境，因为要收 /scan）：

  cd ~/Desktop/Final_Work
  source ~/Desktop/unitree_ros2/setup.sh

  # A = 夹着 walker 的状态：把 walker 夹好，录 10 秒
  python3 scripts/grip_lidar_compare.py record A

  # B = 没夹/脱手的状态：拿走 walker，录 10 秒
  python3 scripts/grip_lidar_compare.py record B

  # 出统计 + 对比图
  python3 scripts/grip_lidar_compare.py compare

  数据存在 record/grip_A.csv、grip_B.csv，对比图 record/grip_compare.png。跑完把 compare 输出的均值/std/最小值发我，我来
  调 grip_watch 的 hold_max_m 阈值（现在拍脑袋的 0.9m）——这组数据 also 是你论文"失败恢复"章节的素材。
