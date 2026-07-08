# UE Zenoh 到 ROS2 C++ Bridge 使用说明

这个包把 Windows/UE 通过 zenoh 发布的 ROS2 CDR payload 转发成 ROS2 topic。UE 侧的
`PublishCompressedImage`、`PublishPointCloud2`、`PublishLivoxPointCloud2`、`PublishImu`
和 `PublishUniRtkPvh` 已经在 payload 里写入了 XCDR1 little-endian 序列化字节，因此
bridge 端使用 `rclcpp::GenericPublisher` 直接发布序列化消息，不再重复解析和拷贝成具体
消息对象。

## 1. 前置条件

- ROS2 Humble 或更新版本
- `colcon`
- ROS 环境中的 `zenoh_cpp_vendor`，或系统安装的 zenoh-c 开发库
- UE 端已连接到同一个 zenoh router 或 peer 网络

在 ROS2 Humble 环境下，包会优先使用 `/opt/ros/humble` 提供的 `zenoh_cpp_vendor`，通常不需要手动设置 `ZENOHC_ROOT`。

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

bridge 内部 zenoh callback 到 ROS 发布线程之间有队列，默认深度 1024：

```bash
ros2 run ue_zenoh_bridge ue_zenoh_bridge --max-queue-depth 4096
```

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

### CMake 找不到 zenoh-c

先确认已经 source ROS 环境：

```bash
source /opt/ros/humble/setup.bash
```

Humble 下默认使用 `zenoh_cpp_vendor` 提供的 zenoh-c：

```text
/opt/ros/humble/opt/zenoh_cpp_vendor/include/zenoh.h
/opt/ros/humble/opt/zenoh_cpp_vendor/lib/libzenohc.so
```

如果使用非 ROS 安装的 zenoh-c，再检查系统是否能通过 `pkg-config --modversion zenohc`
发现它。

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
