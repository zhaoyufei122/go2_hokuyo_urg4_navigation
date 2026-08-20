# GO2 实机二维导航与三维建图

本工作空间有四条互不混用的管线：

- 二维管线先裁掉 GO2 自身点,再把 `/utlidar/cloud_deskewed` 的稳定高度层
  投影为 `/scan`,用 SLAM Toolbox 建图,再用 AMCL + Nav2 导航;
- 三维管线裁掉 GO2 机身点、做 8 cm 体素降采样,再用 RTAB-Map 生成带回环
  优化的三维数据库和可交互点云地图。
- 头部思岚管线直接使用 Jetson 发布的二维 /scan,不再把 L2 三维点云压成二维;
  它继续使用 GO2 的平面 odom、SLAM Toolbox、AMCL 和 Nav2。
- Hokuyo URG-04LX-UG01 管线用 USB 接在本机上的 Hokuyo 二维雷达替换思岚雷达,
  其余平面 odom、SLAM Toolbox、AMCL 和 Nav2 与思岚管线完全相同。

`cloud_deskewed` 是运动去畸变后的当前点云,不是已经累计好的地图。

安全配置固定为:前进/后退最大 `0.4 m/s`、非零转向绝对值为
`0.4-0.8 rad/s`、横移为 0;小于 0.1 rad/s 的转向请求视为零(死区,防止
来回抖动),RPP 原地转向目标速度是 `0.6 rad/s`,有效移动下限是 `0.3 m/s`。
碰撞监测使用 StopZone,只停车,不把速度降到机器狗无法行走的 0.3 以下。桥接
只调用 Move(1008) 和 StopMove(1003),不会自动站立、切步态或执行特技。

## 0. 最重要的纪律:每个终端的环境加载

**"RViz 空白/话题有数据但节点收不到"基本都是因为没有按顺序 source。**
每个新终端都必须:

```bash
source /home/yufei/Desktop/unitree_ros2/setup.sh
source /home/yufei/Desktop/go2_base_navi/install/setup.bash
```

启动任何 launch 前先自检,必须输出 `rmw_cyclonedds_cpp`:

```bash
echo $RMW_IMPLEMENTATION
```

如果输出为空:当前终端用的是默认 FastDDS,能发现话题但收不到狗的数据,
**不要启动**,重新按上面两行 source。

如果之前启动过又没退干净,先清残留进程(多个同名节点会互相抢话题):

```bash
pkill -f go2_base_nav; pkill -f pointcloud_to_laserscan; pkill -f slam_toolbox; pkill -f component_container_mt
```

## 1. 安装、编译

首次安装二维点云投影和三维建图依赖:

```bash
sudo apt install ros-jazzy-pointcloud-to-laserscan
sudo apt install ros-jazzy-rtabmap-ros ros-jazzy-pcl-ros
sudo apt install ros-jazzy-ros2bag
```

每个新终端都必须先加载 Unitree 环境,再加载本工作空间。首次编译执行:

```bash
source /home/yufei/Desktop/unitree_ros2/setup.sh
cd /home/yufei/Desktop/go2_base_navi
colcon build --symlink-install
source /home/yufei/Desktop/go2_base_navi/install/setup.bash
```

每次开机或重新插线后,先检查 GO2 有线接口:

```bash
ip -brief address show enp130s0
```

开始建图或导航前应显示 `UP`,并有 `192.168.123.222/24`。若显示 `DOWN`,先确认
GO2 已开机且网线链路正常,不要启动导航。

## 头部思岚雷达:以后按这个顺序启动

本机已验证这颗雷达可使用思岚 A3 配置:Jetson 串口为 CP2102,驱动参数为
256000 波特率、Sensitivity 扫描模式,发布 /scan,坐标系为 laser。实测健康状态
为 OK、最大量程 25 m,电脑端接收频率约 11.3 Hz。

新思岚管线与旧 L2 投影管线互斥。使用下面命令时,不要同时启动
sensors.launch.py、mapping.launch.py 或 navigation.launch.py,否则会有两个节点
同时发布 /scan。

第一次使用新代码只需在电脑上编译一次:

    source /home/yufei/Desktop/unitree_ros2/setup.sh
    cd /home/yufei/Desktop/go2_base_navi
    colcon build --symlink-install

连接 GO2 后打开终端 1,启动 Jetson 上的雷达驱动:

    cd /home/yufei/Desktop/go2_base_navi
    ./scripts/start_slamtec_lidar.sh

首次连接可能要求确认新的 SSH 主机指纹,随后输入 Jetson 登录密码。这个终端必须
保持打开;在这里按 Ctrl-C 会让驱动执行 Stop motor。脚本不会把密码保存在代码中,
并使用单独的 go2-slamtec-jetson 主机别名,避免和以前同 IP 的机器冲突。

等雷达转动后,在终端 2 验证数据。Humble Jetson 与 Jazzy 电脑之间的话题类型发现
偶尔较慢,因此单帧检查显式写出消息类型:

    source /home/yufei/Desktop/unitree_ros2/setup.sh
    source /home/yufei/Desktop/go2_base_navi/install/setup.bash
    timeout 10s ros2 topic hz /scan
    ros2 topic echo /scan sensor_msgs/msg/LaserScan --once --no-arr
    timeout 10s ros2 run tf2_ros tf2_echo base_link laser

确认 /scan 连续后,仍在终端 2 启动建图:

    cd /home/yufei/Desktop/go2_base_navi
    ./scripts/start_slamtec_mapping.sh

这个命令只启动平面 odom、base_link 到 laser 的静态变换、SLAM Toolbox 和 RViz,
不会启动速度桥或让 GO2 自动行走。默认安装位姿沿用机器中已有配置的近似值:
x=0.20 m、y=0、z=0.25 m、roll=pitch=yaw=0。用实体遥控器缓慢走完整个平层,
先走外围闭环,再覆盖家具通道,最后回到起点等待回环。

如果实际安装位置不同,可直接临时覆盖,例如:

    ./scripts/start_slamtec_mapping.sh laser_x:=0.20 laser_z:=0.25 laser_yaw:=0.0

地图满意后,在终端 3 保存:

    source /home/yufei/Desktop/unitree_ros2/setup.sh
    source /home/yufei/Desktop/go2_base_navi/install/setup.bash
    ros2 run nav2_map_server map_saver_cli -f /home/yufei/Desktop/go2_base_navi/maps/slamtec_room

确认 maps/slamtec_room.yaml 和 maps/slamtec_room.pgm 都已生成,再在建图终端
Ctrl-C。需要录包时,建图命令增加 record_bag:=true;它只录思岚扫描、odom、地图
和 TF,不录体积很大的 L2 点云。

以后导航时,终端 1 仍先启动雷达,然后在另一个终端执行:

    cd /home/yufei/Desktop/go2_base_navi
    ./scripts/start_slamtec_navigation.sh

脚本默认使用 maps/slamtec_room.yaml;也可把另一张地图作为第一个参数传入。RViz
打开后先用 2D Pose Estimate 设置初始位置,确认橙色激光与地图墙壁、柜体重合,
再发送 1--2 m 的开阔短目标。速度安全边界仍为 0.4 m/s、转向
0.4--0.6 rad/s、横移为零。结束时先 Ctrl-C 关闭导航,再到终端 1 Ctrl-C
停止雷达。

## Hokuyo URG-04LX-UG01 管线:与思岚管线互斥

Hokuyo 管线用 URG-04LX-UG01 替换思岚雷达,发布同样的 /scan(frame 为
laser),建图与导航流程和思岚管线一致。它与思岚管线互斥:不要同时启动
start_slamtec_lidar.sh 或任何 slamtec launch,否则两个驱动会同时发布
/scan。

雷达有两种接法(任选其一):

- **插在狗的 Jetson USB 口上**(与思岚一致,推荐):驱动通过 SSH 在 Jetson
  上启动,/scan 经 DDS 发到电脑,狗身上不用拖 USB 线到电脑;
- **插在本机 USB 口上**:驱动在本机启动,雷达 USB 线直接连电脑。

URG-04LX-UG01 特性:240° 视野(正前方左 120° + 右 120°,后方 120° 是物理
盲区)、10 Hz、有效量程约 0.06--4.0 m,比思岚 A3 短得多;建图配置已相应调低
max_laser_range 到 4.0 m、分辨率 0.05 m。

雷达后方装有机械臂:安装时让雷达的物理盲区(屁股方向)对准机械臂,臂就
不会出现在扫描里。如果臂仍出现在 ±120° 窗口内,可用 GO2_HOKUYO_ANGLE_MIN/
GO2_HOKUYO_ANGLE_MAX(弧度,0 为雷达正前方)收紧扫描范围。

首次使用先安装驱动并确认串口权限(只需一次,加完组要重新登录)。驱动装在哪
里取决于雷达插在哪里:插 Jetson(Foxy)就在 Jetson 上装 ros-foxy-urg-node,插本机
就在本机装 ros-jazzy-urg-node:

    # 本机(Jazzy)
    sudo apt install ros-jazzy-urg-node ros-jazzy-laser-filters
    sudo usermod -aG dialout $USER

    # 或者 Jetson(Foxy, Ubuntu 20.04):ssh unitree@192.168.123.18 后执行
    sudo apt install ros-foxy-urg-node
    sudo usermod -aG dialout unitree

如果 Jetson 无法上网(默认就是),则与思岚驱动一样在 Jetson 上源码编译:
把电脑端准备好的 ~/Desktop/hokuyo_jetson_src.tar.gz scp 到 Jetson,解压到
/home/unitree/demo_ws/src,再在 demo_ws 里 source /opt/ros/foxy/setup.bash
后 colcon build 即可。

把雷达 USB 插到选定的机器上,确认设备出现:

    ls /dev/ttyACM*

如果不是 /dev/ttyACM0,用 GO2_HOKUYO_SERIAL 环境变量指定实际端口。然后按
思岚管线相同的顺序启动。终端 1 启动雷达驱动:

    cd /home/yufei/Desktop/go2_base_navi

    # 雷达插在狗的 Jetson 上(与思岚相同的启动方式,启动前会自动同步 Jetson 时钟):
    GO2_HOKUYO_SSH_TARGET=unitree@192.168.123.18 ./scripts/start_hokuyo_lidar.sh

    # 或者雷达插在本机 USB 上:
    ./scripts/start_hokuyo_lidar.sh

这个终端必须保持打开,Ctrl-C 停止驱动。

终端 2 验证 /scan 连续(约 10 Hz)后启动建图:

    source /home/yufei/Desktop/unitree_ros2/setup.sh
    source /home/yufei/Desktop/go2_base_navi/install/setup.bash
    timeout 10s ros2 topic hz /scan
    ./scripts/start_hokuyo_mapping.sh

建图纪律与思岚管线相同;量程只有 4 m,行走时保持在墙体和家具可见的范围内。
地图满意后在终端 3 保存:

    ros2 run nav2_map_server map_saver_cli -f /home/yufei/Desktop/go2_base_navi/maps/hokuyo_room

确认 maps/hokuyo_room.yaml 和 maps/hokuyo_room.pgm 生成后,以后导航在终端 2
执行(终端 1 的雷达驱动保持运行):

    ./scripts/start_hokuyo_navigation.sh

脚本默认使用 maps/hokuyo_room.yaml;也可把另一张地图作为第一个参数传入。
速度安全边界与思岚管线完全相同。安装位姿默认为 x=0.20 m、z=0.10 m
(头顶前方实测近似值),
实际安装位置不同可用 laser_x:=、laser_z:= 等参数临时覆盖。

### Hokuyo 可选:SLAM Toolbox 定位(默认 AMCL)

AMCL 在狗静止超过 10 秒后会停止发布 map->odom 并被 TF 缓存清除,导致
导航卡死;步态抖动也会让粒子滤波在行走中拒绝更新。如果 AMCL 表现不
稳,可换 SLAM Toolbox 定位(连续发布 TF,不受静止影响):

建图结束前(建图终端还在运行时)先序列化一次 posegraph:

    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: /home/yufei/Desktop/go2_base_navi/maps/hokuyo_room}"

导航时改用(注意 slam_posegraph 不带 .posegraph 后缀):

    ./scripts/start_hokuyo_navigation.sh localization:=slam_toolbox slam_posegraph:=/home/yufei/Desktop/go2_base_navi/maps/hokuyo_room

此模式下启动两个部分:SLAM Toolbox 只负责发布 map->odom 定位(它自己重建的
局部地图被重定向到 /map_slam,不影响导航),完整地图由 map_server 从
maps/hokuyo_room.yaml 加载,RViz 和 Nav2 看到的就是你保存的原图。初始位姿
不用手点,SLAM Toolbox 会自行匹配;若启动位置离原点远,可在
config/hokuyo_slam_toolbox_localization.yaml 里设 map_start_pose。

## 2. 只读传感器检查

先让 GO2 静止,启动不含运动桥接的传感器管线:

```bash
ros2 launch go2_base_nav sensors.launch.py
```

在另一个按上述顺序 source 的终端检查:

```bash
timeout 10s ros2 topic hz /scan
timeout 10s ros2 run tf2_ros tf2_echo odom base_footprint
timeout 10s ros2 topic echo /odom --once --no-arr
```

应持续收到 `/scan`;TF 应能解析;`/odom` 的 child frame 应为
`base_footprint`,位置 z 为 0。检查完成后 Ctrl-C 停止 sensors,避免下一步重复
启动同名节点。

## 3. 高精度二维建图

二维传感器链先在 `base_link` 中删除 GO2 机身点,输出
`/cloud_self_filtered`;随后在水平的 `base_footprint` 中保留高度
`0.12--0.45 m`、距离 `0.25--6.0 m` 的 360 度墙壁和固定柜体结构,再生成
`/scan`。SLAM Toolbox 使用平面 odom 预测和扫描匹配,地图位姿始终只包含
x、y、yaw。

建图启动文件不会发送速度,走动完全由实体遥控器控制。默认同步录包;启动前先确认
录包磁盘空间:

```bash
mkdir -p ~/go2_mapping_bags
df -h ~/go2_mapping_bags
ros2 launch go2_base_nav mapping.launch.py record_bag:=true
```

每次录包保存在 `~/go2_mapping_bags/YYYYMMDD_HHMMSS`,已有目录不会被覆盖。

建图纪律(直接影响地图质量):

- 启动后先在 RViz 检查 `Filtered Cloud` 和 `/scan`:不应有随 GO2 移动的
  机身点簇或地面圆环;**不要长时间站着不动**(站立时腿部残点位置固定,
  会变成地图上去不掉的黑点),确认显示正常后就开始慢速行走;
- 用实体遥控器先走房间外围闭环,确认墙体没有重影,再覆盖内部通道,最后回到
  起点附近;
- 转角处放慢,给回环检测留出特征重叠。

在另一个正确 source 的终端保存地图:

```bash
ros2 run nav2_map_server map_saver_cli -f /home/yufei/Desktop/go2_base_navi/maps/room_map
```

确认 `maps/room_map.yaml` 与 `maps/room_map.pgm` 已生成后,在建图终端按
Ctrl-C,等待 rosbag 完成索引,再把实际时间戳代入检查:

```bash
ros2 bag info ~/go2_mapping_bags/YYYYMMDD_HHMMSS
```

少量去不掉的孤立黑点是正常的(射线清除覆盖不到的位置),直接用 GIMP 打开
`room_map.pgm` 涂白即可(黑=占据、白=空闲、灰=未知),涂完不用改 yaml。

GO2 关机后可以重放原始点云和 odom。终端一启动离线建图节点:

```bash
ros2 launch go2_base_nav mapping.launch.py \
  use_sim_time:=true use_rviz:=true record_bag:=false
```

另一个已 source 的终端播放选定输入:

```bash
ros2 bag play ~/go2_mapping_bags/YYYYMMDD_HHMMSS \
  --clock \
  --topics /utlidar/cloud_deskewed /utlidar/robot_odom /tf_static
```

回放时故意不播放 `/tf`,因为 `planar_odom` 与 SLAM Toolbox 会重新生成动态
TF;同时播放旧 `/tf` 会造成同名坐标变换冲突。

### 3.1 可选:实验性多帧累积(默认关闭)

`sensors.launch.py` 有一个 `use_accumulator` 开关(默认 `false`)。开启后会在
CropBox 和投影之间插入 `scan_accumulator` 节点:在静态 `odom` 系做 0.35 s 滑窗
累积 + 体素一致性滤波,输出更稠密的 `/cloud_accumulated` 再投影成 `/scan`。
对 L1 非重复扫描的稀疏单帧有增益,但会引入少量延迟,**效果因场景而异,先用
默认管线确认能导航,再实验性尝试**:

```bash
ros2 launch go2_base_nav mapping.launch.py use_accumulator:=true
```

旋钮在 `config/scan_accumulator.yaml`:`min_voxel_hits`(2->3 更干净)、
`window_sec`(0.35->0.25 更低延迟)。

## 4. 三维建图(RTAB-Map,只建图)

这个启动文件只运行里程计 TF 适配、机身裁剪、体素滤波、RTAB-Map 和可视化。
它不会启动 Nav2、速度桥或任何软件遥控节点;机器狗走动只允许使用实体遥控器。

L2 虽然倒装并会扫到地面和机身,但三维模式不只保留前向点:房间四周、地面、
桌面和椅腿都是回环与三维配准需要的几何信息。点云先变换到 `base_link`,再
删除机身框 x `[-0.45, 0.45]` m、y `[-0.32, 0.32]` m、
z `[-0.45, 0.30]` m 内的点,框外的 360 度环境点会保留。

第一次建新图使用 `new_map:=true`。它只会清空
`database_path` 指定的这个数据库,所以路径必须确认无误:

```bash
ros2 launch go2_base_nav mapping_3d.launch.py \
  database_path:=/home/yufei/Desktop/go2_base_navi/maps/room_3d.db \
  new_map:=true
```

这个命令默认打开配置好的 RViz。Fixed Frame 已设为 `map_3d`:

- `Live Filtered Cloud`(橙色)显示当前帧 `/cloud_3d_filtered`;
- `Accumulated 3D Map`(青色)显示不断增长和回环优化的 `/cloud_map`;
- `Projected 2D Map` 显示同步生成的 `/map`;
- `Mapping Path` 显示 `/mapPath` 建图轨迹。

只做无界面诊断时增加 `use_rviz:=false`。需要检查 RTAB-Map 节点图和
回环细节时增加 `use_rtabmap_viz:=true`;默认不打开这个专业调试窗口。

先不要移动。在另一个按前述顺序 source 的终端检查过滤点云和完整三维 TF:

```bash
timeout 10s ros2 topic echo /cloud_3d_filtered --once --no-arr
timeout 10s ros2 run tf2_ros tf2_echo odom base_link
```

RViz 中应看到房间、地面和家具,但机器狗身体附近不应有持续跟随的点簇。
若仍能看到机身,先调整机身裁剪框,不要先改 ICP 参数。确认点云正确后,用实体
遥控器缓慢走一条闭环,从两个方向扫到桌椅,并回到起点附近等待回环修正。

结束时在启动终端按 Ctrl-C,让 RTAB-Map 完整写盘,再检查数据库:

```bash
ls -lh /home/yufei/Desktop/go2_base_navi/maps/room_3d.db
```

要继续同一张图,保持相同路径并改用 `new_map:=false`:

```bash
ros2 launch go2_base_nav mapping_3d.launch.py \
  database_path:=/home/yufei/Desktop/go2_base_navi/maps/room_3d.db \
  new_map:=false
```

本版不做 3D 定位或自主导航;它先用于比较三维地图与回环效果。现有导航仍使用
第 3 节保存的二维地图,确认三维数据库质量后再单独接入 RTAB-Map 定位与 Nav2。

## 5. 二维地图导航

**先同步狗的时钟**(狗的时钟比 PC 慢约 2 分半,不同步会导致导航时 TF 跨
时钟外推报错)。PC 上执行(地址若不通就换 192.168.123.13,密码 123):

```bash
ssh unitree@192.168.123.161 "sudo date -s \"$(date '+%Y-%m-%d %H:%M:%S')\""
```

验证同步成功(两条命令输出的秒数应一致):

```bash
date +%s; ssh unitree@192.168.123.161 date +%s
```

先用实体遥控器让 GO2 正常站立,并把它放到地图中的已知、开阔位置。地图参数
必须使用绝对路径:

```bash
ros2 launch go2_base_nav navigation.launch.py map:=/home/yufei/Desktop/go2_base_navi/maps/room_map.yaml
```

RViz 中先点 `2D Pose Estimate`,给出机器狗在地图上的初始位姿;等待激光与地图
重合后,再用 `Nav2 Goal` 发送一个 1--2 m、无遮挡的短目标。第一轮始终握住实体
遥控器,不要直接测试狭窄通道或贴近桌椅的目标。

### 5.1 可选:实验性 SLAM Toolbox localization(默认关闭)

默认定位是 AMCL(已知可用)。如果要尝试 SLAM Toolbox localization 模式,
建图结束时需要额外序列化 posegraph:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: /home/yufei/Desktop/go2_base_navi/maps/room_map}"
```

导航时不传 `map`,改传 posegraph 路径(**不带 .posegraph 后缀**):

```bash
ros2 launch go2_base_nav navigation.launch.py \
  localization:=slam_toolbox \
  slam_posegraph:=/home/yufei/Desktop/go2_base_navi/maps/room_map
```

## 6. 常见问题速查

| 现象 | 原因 | 处理 |
|---|---|---|
| RViz 全空、话题有但节点收不到 | 终端没 source unitree 环境,跑在 FastDDS 上 | Ctrl-C,按第 0 节重新 source 再启动 |
| `echo $RMW_IMPLEMENTATION` 为空 | 同上 | 同上 |
| 节点越启越多、行为诡异 | 旧 launch 没退干净 | 用第 0 节的 pkill 命令清场 |
| `/scan` 有数据但 RViz 不显示扫描线 | RViz 显示的 Reliability 是 Reliable | 该显示项的 Reliability Policy 改 Best Effort |
| 地图黑点不消失 | 射线清除覆盖不到/站立时腿影 | 建图时保持慢速移动;残余用 GIMP 涂白 |
| 导航时 TF 报 extrapolation | 狗时钟没同步 | 见第 5 节开头的 date 同步 |

## 紧急停止

实体遥控器是第一优先级的紧急接管手段。发现方向错误、地图错位、激光消失或
将要碰撞时,立即用实体遥控器停止/接管,不要只依赖软件。随后在导航终端
Ctrl-C;桥接关闭时会再次尝试发送 StopMove。完整实机验收顺序见
`docs/TESTING.md`。
