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

默认使用自动 QoS：`PointCloud2` 使用 reliable、depth 10，避免大点云的任一 DDS 分片丢失后
整帧作废；图像、IMU、里程计和 GNSS 继续使用 best-effort、depth 1。启动日志会打印每个
publisher 的实际 QoS。需要强制所有 topic 使用 best-effort：

```bash
ros2 run ue_zenoh_bridge ue_zenoh_bridge --best-effort
```

需要强制所有 topic 使用 reliable：

```bash
ros2 run ue_zenoh_bridge ue_zenoh_bridge --reliable --qos-depth 10
```

命令行 `--reliable` 或 `--best-effort` 会关闭自动 QoS；也可通过 ROS 参数
`auto_qos:=false` 配合 `reliable:=true/false` 控制。

ROS 2 Humble 的 Fast DDS 默认同步发送。大点云分片发送时，单次 publish 可能阻塞数百毫秒，
表现为长时间停顿后多帧密集到达。bridge 会在未设置
`RMW_FASTRTPS_PUBLICATION_MODE` 时自动选择 `ASYNCHRONOUS`。可在排查兼容性问题时恢复同步：

```bash
ros2 run ue_zenoh_bridge ue_zenoh_bridge --sync-publish
```

命令行 `--async-publish` 可显式覆盖外部环境变量并强制使用异步模式。该设置只影响 Fast DDS，
使用其他 RMW 时会被忽略。启动日志会打印当前 RMW 和 publication mode。

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

Zenoh 回调只对 payload 做浅克隆并立即返回，不再在 Zenoh 接收线程中逐帧分配、复制大点云。
发布 worker 对连续 payload 直接借用 Zenoh 缓冲区调用 ROS2 serialized publish；仅当 Zenoh
payload 由多个 slice 组成时，才通过 bytes reader 复制到每个 worker 复用的缓冲区。因此这能
消除常见路径上的一次大消息复制，但 DDS/RMW 仍可能在发布内部复制，并不是端到端零拷贝。

退出时日志会输出 `received`、`published`、`coalesced`、`queue_dropped`、`borrowed` 和
`fragmented_copies`。若运行中出现 stale sample 警告且退出时 `coalesced > 0`，说明 ROS/DDS
发布速度低于 Zenoh 输入速度；若 `received` 本身只有约 7 Hz，则瓶颈在 UE/Zenoh 上游。

## 7. 开发稳定的 PointCloud2 订阅程序

### 7.1 发布端和订阅端必须同时使用 reliable

PointCloud2 通常会被 DDS 拆成大量分片。best-effort 订阅端只要丢失一个分片，整帧点云就会
作废；reliable 发布端不能强制 best-effort 订阅端请求重传。因此，bridge 和业务订阅程序必须
同时使用 reliable，才能稳定接收完整帧。

| 发布端 | 订阅端 | 结果 |
| --- | --- | --- |
| reliable | reliable | 推荐；支持丢失分片重传 |
| reliable | best-effort | 可以匹配，但订阅端不会请求重传，大点云可能降频 |
| reliable | `SensorDataQoS()` | 等同 best-effort，可能出现约 7 Hz 和突发到达 |

bridge 默认已经为 `sensor_msgs/msg/PointCloud2` 设置 `reliable + depth 10`，业务程序不要再用
`rclcpp::SensorDataQoS()` 订阅点云。

### 7.2 C++ 订阅示例

```cpp
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;

// 构造函数中创建订阅。显式指定 reliability，不依赖 RMW 默认值。
auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(10));
lidar_qos.reliable();
lidar_qos.durability_volatile();

lidar_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
  "/front_lidar",
  lidar_qos,
  std::bind(&LidarNode::lidar_callback, this, _1));
```

回调建议接收 `ConstSharedPtr`，避免复制整帧点云：

```cpp
void LidarNode::lidar_callback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  // latest_message_、latest_mutex_ 和 work_cv_ 是 LidarNode 的成员。
  // 新帧直接替换尚未处理的旧帧，耗时处理由 worker 完成。
  {
    std::lock_guard<std::mutex> lock(latest_mutex_);
    latest_message_ = std::move(message);
  }
  work_cv_.notify_one();
}
```

不要在订阅回调里执行可能超过 100 ms 的点云配准、保存 PCD、可视化或同步网络请求。推荐采用
“订阅回调 -> 有界最新帧队列 -> 处理 worker”的结构；队列只保留最新帧，避免处理能力不足时
不断累积旧数据。多个相机、雷达和 IMU 同时工作时，可为不同传感器分配 callback group，并使用
`rclcpp::executors::MultiThreadedExecutor`，避免一个耗时回调阻塞其他 topic。

### 7.3 Python 订阅示例

```python
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2

lidar_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

self.lidar_subscription = self.create_subscription(
    PointCloud2,
    "/front_lidar",
    self.lidar_callback,
    lidar_qos,
)
```

Python 回调同样应尽快返回；CPU 密集型点云处理建议放到独立进程或原生扩展中，避免阻塞
`rclpy` executor。

### 7.4 检查订阅程序的实际 QoS

启动 bridge 和业务订阅程序后执行：

```bash
ros2 topic info /front_lidar --verbose
```

确认 bridge 的 publisher 和业务程序的 subscription 都显示：

```text
Reliability: RELIABLE
Durability: VOLATILE
```

如果 subscription 显示 `BEST_EFFORT`，即使 publisher 是 reliable，业务程序仍可能丢失大点云。
RViz 的 PointCloud2 Display 也需要把 Reliability Policy 设置为 `Reliable`。

### 7.5 频率和抖动的区别

bridge 是收到即转发，不会伪造或重复点云。平均 10 Hz 表示长期没有丢帧，但 UE、Zenoh、DDS
和系统调度仍可能让单帧间隔偏离 100 ms。如果业务要求严格每 100 ms 触发一次，应在算法层用
固定周期 timer 读取“最新点云”；不要通过重复发布旧点云来制造表面上的 10 Hz。

## 8. 验证和故障定位

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

Humble 的 `ros2 topic hz` 固定使用 sensor-data best-effort QoS，不能用于验证 reliable 点云
链路。它可能只显示约 7 Hz，即使业务程序能用 reliable 稳定收到 10 Hz。请使用包内的 reliable
测频工具：

```bash
ros2 run ue_zenoh_bridge reliable_lidar_hz /front_lidar
```

推荐按以下顺序定位：

1. 启动日志应显示 `Fast DDS publication mode 'ASYNCHRONOUS'`。
2. `/front_lidar` publisher 日志应显示 `qos=reliable depth=10`。
3. 使用 `reliable_lidar_hz` 测量真实 reliable 接收率。
4. 停止 bridge，检查 shutdown summary。

统计含义：

| 现象 | 判断 |
| --- | --- |
| `received` 本身低于目标频率 | UE、传感器或 Zenoh 上游没有稳定输入 |
| `received == published` 且 `coalesced == 0` | bridge 完整转发，没有内部丢帧 |
| `coalesced > 0` | ROS/DDS publish 一度慢于 Zenoh 输入，只保留了最新帧 |
| `queue_dropped > 0` | 多 key 总负载超过 bridge 队列容量 |
| `ros2 topic hz` 约 7 Hz，但 reliable 工具约 10 Hz | best-effort 测量造成的 DDS 分片丢失 |
| reliable 工具也低于 10 Hz，但 bridge 完整发布 | 检查订阅主机负载、回调耗时和网络质量 |

本项目在 1 MiB、10 Hz 点云测试中，best-effort 订阅约为 7 Hz；发布端和订阅端都使用 reliable
后可完整接收 20/20 帧。真实 `/front_lidar` 测试中 bridge 为 `received=373, published=373`，
没有 coalesce 或 queue drop。

## 9. 常见问题

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
