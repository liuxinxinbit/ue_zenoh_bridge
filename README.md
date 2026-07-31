# UE Zenoh 到 ROS2 C++ Bridge 使用说明

这个包把 Windows/UE 通过 zenoh 发布的 ROS2 CDR payload 转发成 ROS2 topic。UE 侧的
`PublishCompressedImage`、`PublishPointCloud2`、`PublishLivoxPointCloud2`、`PublishImu`
和 `PublishUniRtkPvh` 已经在 payload 里写入了 XCDR1 little-endian 序列化字节，因此
bridge 端使用 `rclcpp::GenericPublisher` 直接发布序列化消息，不再重复解析和拷贝成具体
消息对象。

## 1. 前置条件

- ROS2 Humble 或更新版本
- `colcon`
- UE 端已连接到同一个 zenoh router 或 peer 网络

包内已经固定携带 zenoh-c 1.9.0 的头文件和 Linux x86_64 动态库。构建和运行均不读取
系统 zenoh-c、`zenoh_cpp_vendor`、`ZENOHC_ROOT`、`ZENOH_C_ROOT` 或 UESim 源码目录。
安装时 `libzenohc.so` 会放在 bridge 可执行文件旁边，并通过 `$ORIGIN` 强制加载。

当前内置二进制只支持 Linux x86_64；其他系统或架构会在 CMake 配置阶段明确报错，需在
`ThirdParty/zenoh-c` 中加入同版本的平台库后再扩展 CMake。

## 2. 构建

在 `ros_ws` 根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ue_zenoh_bridge
source install/setup.bash
```

## 3. 启动

连接本机 zenoh router：

```bash
ros2 run ue_zenoh_bridge ue_zenoh_bridge --endpoint tcp/127.0.0.1:7447
```

如果 UE 和 bridge 都使用 zenoh peer/default 配置，可以把 endpoint 置空：

```bash
ros2 run ue_zenoh_bridge ue_zenoh_bridge --endpoint ""
```

订阅指定 key 表达式：

```bash
ros2 run ue_zenoh_bridge ue_zenoh_bridge \
  --endpoint tcp/127.0.0.1:7447 \
  --key-expr 'rt/**'
```

## 4. 默认 key 到 topic 和自动类型识别

默认去掉 key 前缀 `rt/`，再在前面加 `/`。topic 名称只决定 ROS topic，不再决定传感器消息类型：

```text
rt/camera/front/image/compressed  ->  /camera/front/image/compressed
rt/front_depth/image              ->  /front_depth/image/compressed
rt/front_depth/image/compressed   ->  /front_depth/image/compressed
rt/front_lidar                    ->  /front_lidar
rt/front_lidar/lidar              ->  /front_lidar/lidar
rt/lidar/front/points             ->  /lidar/front/points
rt/imu                            ->  /imu
rt/gps                            ->  /gps
rt/odom/mujoco_odom               ->  /odom/mujoco_odom
rt/odom/mujoco_gps                ->  /odom/mujoco_gps
```

收到 Zenoh payload 后，bridge 会先解析 CDR 结构自动识别 ROS 类型：

- `sensor_msgs/msg/PointCloud2`
- `sensor_msgs/msg/CompressedImage`
- `sensor_msgs/msg/Imu`
- `nav_msgs/msg/Odometry`
- `robots_dog_msgs/msg/UniRtkPvh`

识别顺序是：显式 `--topic-type` 覆盖 > CDR payload 自动识别 > 旧 topic 后缀规则兜底。通常直接启动即可自动转发 UE 默认传感器 key：

```bash
ros2 run ue_zenoh_bridge ue_zenoh_bridge --key-expr 'rt/**'
```

仍可为非默认 key 自定义映射：

```bash
ros2 run ue_zenoh_bridge ue_zenoh_bridge \
  --topic-type rt/livox/points:=sensor_msgs/msg/PointCloud2 \
  --topic-type /rtk/pvh:=robots_dog_msgs/msg/UniRtkPvh
```

## 5. 预声明 topic

没有收到第一帧之前，publisher 还不会创建。调试时可以先预声明：

```bash
ros2 run ue_zenoh_bridge ue_zenoh_bridge \
  --endpoint tcp/127.0.0.1:7447 \
  --predeclare-topic /camera/front/image/compressed:sensor_msgs/msg/CompressedImage
```

如果 `--predeclare-topic` 不带类型，默认按 topic 后缀推断；无法推断时按
`sensor_msgs/msg/CompressedImage` 处理。

## 6. QoS 和队列

传感器数据默认使用 best-effort：

```bash
ros2 run ue_zenoh_bridge ue_zenoh_bridge --best-effort
```

需要可靠传输时：

```bash
ros2 run ue_zenoh_bridge ue_zenoh_bridge --reliable --qos-depth 10
```

bridge 内部 zenoh callback 到 ROS 发布线程之间使用有界“最新帧”队列，默认最多保留 64 个
不同 key 的待发消息。同一 key 在等待发布期间只保留最新一帧；队列满时会淘汰最旧消息，避免
过载时持续转发陈旧传感器数据。

默认使用 10 个发布 worker：其中最多 4 个用于小于等于 64 KiB 的 IMU、里程计和 GNSS，
其余用于较大的图像和点云。消息按 key 稳定分配，既保持同 topic 顺序，又避免大消息的 DDS
发布阻塞 IMU。每个 worker 都使用独立的有界最新帧队列，`max_queue_depth` 会在 worker
之间均分，使总容量维持在配置值附近：

```bash
ros2 run ue_zenoh_bridge ue_zenoh_bridge \
  --worker-count 10 \
  --max-queue-depth 64
```

对低延迟传感器转发，建议不要把该值调大；`1` 到 `64` 通常比大队列更合适。
常见双雷达、RGB、深度、IMU、里程计和 GNSS 组合建议保持默认 10 个 worker。参数范围会限制
在 1 到 64 之间。

Zenoh payload 会通过 bytes reader 直接复制到 ROS2 serialized buffer，不再先展平到临时
slice，因此即使 Zenoh payload 是分片存储，大消息接收路径也只需要一次显式复制。由于
`rclcpp::GenericPublisher` 发布的是序列化 CDR，且不支持 intra-process 通信，这不是 DDS
loaned-message 意义上的端到端零拷贝。

## 7. 验证

另开终端并 source 工作空间：

```bash
source install/setup.bash
ros2 topic list
ros2 topic type /camera/front/image/compressed
ros2 topic echo /camera/front/image/compressed --no-arr
```

预期类型：

```text
sensor_msgs/msg/CompressedImage
```

PointCloud2 和 IMU 可分别验证：

```bash
ros2 topic type /lidar/front/points
ros2 topic type /imu
```

## 8. 常见问题

### CMake 报 bundled zenoh-c 文件缺失

确认源码包包含 `ThirdParty/zenoh-c/include/zenoh.h` 和
`ThirdParty/zenoh-c/lib/linux-x86_64/libzenohc.so`。bridge 不会回退使用系统 zenoh-c。

### ROS2 topic 看不到

- 确认 bridge 日志出现 `subscribed Zenoh 'rt/**' via endpoint ...`
- 确认 UE 端 key 被 `--key-expr` 覆盖
- 调试时用 `--predeclare-topic` 先创建 ROS publisher
- WSL 场景不要误用 Windows 侧不可达的 `127.0.0.1`

### rqt 查看深度图报 encoding 为空

深度相机通过 `sensor_msgs/msg/CompressedImage` 发布 PNG 压缩图。ROS topic 应为
`/front_depth/image/compressed`，不要把 `CompressedImage` 当作普通 `sensor_msgs/msg/Image`
直接打开。若 UE 旧配置仍发布 `rt/front_depth/image`，bridge 会自动转成
`/front_depth/image/compressed`。

### 自定义 UniRtkPvh publisher 创建失败

`rclcpp::GenericPublisher` 仍然需要本机 ROS 环境里能找到对应 type support。请先构建并
source 包含 `robots_dog_msgs/msg/UniRtkPvh` 的工作空间。
