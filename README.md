# UE Zenoh 到 ROS 2 C++ Bridge 使用说明

这个包把 Windows/UE 通过 Zenoh 发布的 ROS 2 CDR payload 转发成 ROS 2 topic。UE 侧的
`PublishCompressedImage`、`PublishPointCloud2`、`PublishLivoxPointCloud2`、`PublishImu`
和 `PublishUniRtkPvh` 已经在 payload 里写入了 XCDR1 little-endian 序列化字节，因此
bridge 端使用 `rclcpp::GenericPublisher` 直接发布序列化消息，不再重复解析和拷贝成具体
消息对象。

## 完整录制模式（2026-09-07）

使用 `--recording-mode` 时所有 topic 采用按 key 保序 FIFO，不覆盖高频 IMU/GPS/里程计。
普通队列每个 key 默认 `--recording-queue-depth 2048`；点云仍使用独立线程和
`--lidar-queue-depth 30`。所有队列共享 `--max-queue-mib` 字节上限，超限拒绝新消息、
记录 rejected 并使 data_integrity=FAILED，退出码为 2。它不能保证无限过载下不丢帧。

该模式强制所有 publisher 为 reliable；IMU、Odometry 和 UniRtkPvh 的 QoS depth
默认 `--recording-qos-depth 1024`，相机和点云仍使用 `--qos-depth`（默认 30）。
录包器也需要 reliable 和足够的队列容量。

配合 `--predeclare-topic` 和 `--wait-for-recorder-ms 20000`，bridge 会等待所有预声明
publisher 都发现名为 rosbag2_recorder 的 reliable 订阅后才订阅原始 Zenoh 数据。
超时明确失败；记录窗口从桥接开始接收数据算起，窗口前的上游数据不包含在录制中。
录制期间不要重启录包器或改变其 QoS。

完整录制还计算每个 topic 按发布顺序拼接的原始 CDR payload CRC32（初值 0），
输出字段 `payload_crc32`。停桥后 final_stats 应满足 received=published、
coalesced/rejected/failed/shutdown_dropped/queued/in_flight 全为 0。
停包后按每个 topic 的数据库插入顺序重算 CRC32 和消息数，应与 final_stats 相同。
CRC32 是传输一致性检查，不是密码学完整性证明；还应检查源时间戳/序号连续性。

停止顺序：停止 bridge 接收并等待队列排空及 DDS 确认，再正常停止 recorder 刷新缓存，
最后检查 metadata、数据库完整性、逐 topic 条数、CRC32 和源时间戳。
ROS publish/DDS ACK 本身不能证明磁盘落盘。原始源端未送达 bridge 的数据只能通过
源序号或时间戳连续性另行检查。

不传 `--recording-mode` 时仍保留下面描述的旧实时预览策略（非雷达 latest）。

## 1. 前置条件和 Zenoh 依赖

- ROS 2 Humble 或更新版本
- `colcon`
- UE 端已连接到同一个 zenoh router 或 peer 网络
- ROS 2 的 `zenoh_cpp_vendor`，或系统安装的 zenoh-c 开发包

包内不再携带 Zenoh 头文件和动态库。CMake 优先使用当前 ROS 2 环境中的
`zenoh_cpp_vendor`，没有该包时依次查找系统 `zenohc` CMake 包、`pkg-config` 元数据和标准
头文件/库路径。编译结果使用所选依赖的正常运行时搜索规则，不再复制或强制加载私有
`libzenohc.so`。

查找顺序如下：

1. 当前 ROS 2 环境中的 `zenoh_cpp_vendor`
2. 系统或 `CMAKE_PREFIX_PATH` 中的 `zenohcConfig.cmake`
3. `PKG_CONFIG_PATH` 中的 `zenohc.pc`
4. 标准搜索路径中的 `zenoh.h` 和 `libzenohc`

构建配置日志会打印实际选择，例如：

```text
-- Using ROS 2 zenoh_cpp_vendor via target zenohc::lib
```

或：

```text
-- Using system zenoh-c via target zenohc::lib
```

### 1.1 使用 ROS 2 提供的 Zenoh（推荐）

先 source 实际使用的 ROS 2 发行版，再安装对应软件包：

```bash
export ROS_DISTRO=humble  # 按实际发行版修改，例如 jazzy
source /opt/ros/$ROS_DISTRO/setup.bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-zenoh-cpp-vendor
```

构建脚本不硬编码某个 ROS 2 发行版或 Zenoh 安装路径，而是使用该发行版导出的 CMake target。

### 1.2 使用系统 Zenoh

如果没有安装 `zenoh_cpp_vendor`，CMake 会自动回退到系统 zenoh-c。自行安装到非标准路径时，
根据安装内容设置其中一个搜索路径：

```bash
export CMAKE_PREFIX_PATH=/path/to/zenoh/install:$CMAKE_PREFIX_PATH
# 或者仅提供 zenohc.pc 时：
export PKG_CONFIG_PATH=/path/to/zenoh/install/lib/pkgconfig:$PKG_CONFIG_PATH
```

如果 ROS 2 环境中已经存在 `zenoh_cpp_vendor`，但希望强制使用系统 zenoh-c，构建时增加：

```bash
colcon build --packages-select ue_zenoh_bridge \
  --cmake-args -DCMAKE_DISABLE_FIND_PACKAGE_zenoh_cpp_vendor=TRUE
```

头文件和动态库应来自同一个 zenoh-c 安装，避免混用不同版本。

## 2. 构建

在 `ros_ws` 根目录执行：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select ue_zenoh_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

切换 ROS 2 发行版后，应使用独立的 build/install/log 目录，或先清理旧发行版的构建产物，
避免 CMake 缓存继续引用上一发行版的头文件和动态库。

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

默认使用自动 QoS：`PointCloud2` 使用 reliable、depth 30，避免大点云的任一 DDS 分片丢失后
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

bridge 默认对 `sensor_msgs/msg/PointCloud2` 使用**按 topic 保序的 FIFO**，每个雷达最多
缓存 30 帧，不再用新帧覆盖待发旧帧。类型由首帧 CDR 或显式 `--topic-type` 确定，因此
`/front_lidar/imu` 仍按 IMU 处理。其他传感器继续只保留每个 key 最新的待发数据。

雷达有独立的发布线程池，默认 4 个线程，前 4 个雷达 key 各占一个线程；更多雷达会轮转共享，
并打印警告。普通消息默认另有 10 个线程，雷达不会与相机争用这些发布线程。单个 topic 始终
绑定一个线程，保证顺序；同一线程内的多个 key 轮流处理，避免饥饿。

```bash
ros2 run ue_zenoh_bridge ue_zenoh_bridge \
  --async-publish \
  --lidar-worker-count 4 \
  --lidar-queue-depth 30 \
  --max-queue-mib 512 \
  --qos-depth 30
```

| 参数 / 同名 ROS 参数（下划线） | 默认 | 含义 |
| --- | --- | --- |
| `--lidar-worker-count` | 4 | 独立雷达发布线程数，1～64 |
| `--lidar-queue-depth` | 30 | 每个雷达的待发 FIFO 帧数，不含正在发布的一帧 |
| `--max-queue-mib` | 512 | 所有线程共享的待发 payload 字节上限，MiB |
| `--qos-depth` | 30 | DDS 发布历史深度，独立于桥接 FIFO |
| `--worker-count` | 10 | 非雷达发布线程数，1～64 |
| `--max-queue-depth` | 64 | 普通线程分配待发 key 容量的基数，仍按线程向上均分；雷达线程采用相同每线程 key 上限 |
| `--drain-timeout-ms` | 10000 | 停止后的队列排空与 DDS 确认共用预算 |

512 MiB 只限制队列中的 payload 字节，不包括正在发布的帧、Zenoh 接收缓存、DDS 历史和
录包缓存。按帧数和字节数中先达到的上限拒绝**新帧**，不挤掉 FIFO 中已接受的帧。雷达发生
拒绝时会立即打印包含 key 的错误、把完整性状态标为失败，并在退出时返回状态码 2；不会
重复旧帧来补出 10 Hz。持续过载无法靠有限缓存解决，必须提高处理能力或让上游暂停/重试。

正常 SIGINT/SIGTERM 会先取消 Zenoh 订阅，保持 ROS 上下文有效，排空已接受的队列，再等待
reliable DDS 确认，最后关闭连接。超过排空预算的待发帧会计入 `shutdown_dropped`。正在执行的
一次 DDS publish 调用不能被此预算强制打断。DDS 确认不等于磁盘已经落盘，录包器必须保持运行，
待桥接退出后再正常停止录包器并等待其刷新缓存。SIGKILL 或系统崩溃不执行上述排空流程。

首帧会在 Zenoh 回调中解析类型并选择通道，分片首帧可能需要额外展开复制。后续帧的回调只对
payload 做浅克隆和入队，不在 Zenoh 接收线程中逐帧分配、复制大点云。
发布 worker 对连续 payload 直接借用 Zenoh 缓冲区调用 ROS 2 serialized publish；仅当 Zenoh
payload 由多个 slice 组成时，才通过 bytes reader 复制到每个 worker 复用的缓冲区。因此这能
消除常见路径上的一次大消息复制，但 DDS/RMW 仍可能在发布内部复制，并不是端到端零拷贝。

统计字段和完整录制步骤见第 8 节。注意 `coalesced` 只应发生在非点云话题；雷达低频要结合
逐 key 的 `rx_hz`、`pub_hz`、排队等待时间和 bag 内帧数判断。

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

bridge 默认已经为 `sensor_msgs/msg/PointCloud2` 设置 `reliable + depth 30`，业务程序不要再用
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

bridge 按源数据转发，不会伪造或重复点云。平均 10 Hz 本身不能证明没有缺帧或重复帧，
应同时核对帧数和源时间戳/帧标识。UE、Zenoh、DDS 和系统调度仍可能让单帧间隔偏离 100 ms。如果业务要求严格每 100 ms 触发一次，应在算法层用
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

桥接每 5 秒打印逐 key 的 `stats`，退出时打印 `final_stats` 和总计。`rx_hz` 和 `pub_hz`
采用本机 steady clock 的本周期计数差；首周期含发现时间，末周期含排空时间，不能单独用来判断
源的稳定频率。`published` 只表示 ROS publish 调用成功，不表示录包已经收到或落盘。

| 字段 | 判断 |
| --- | --- |
| `received` / `published` | 相同采集区间排空后对账 |
| `queued` / `in_flight` / `high_water` | 当前待发、正在发布和待发峰值 |
| `coalesced` | 非雷达最新帧替换数；雷达必须为 0 |
| `rejected` | 帧数、key 数或全局字节预算耗尽，拒绝的新帧数 |
| `failed` | 转换路由或发布失败数 |
| `shutdown_dropped` | 退出排空预算耗尽后丢弃的帧数 |
| `oldest_ms` / `max_wait_ms` | 当前最老帧等待时间 / 本周期最大排队时间 |
| `max_publish_ms` | 本周期最慢的转换/发布调用耗时 |

每个 key 应满足 `received = published + queued + in_flight + coalesced + rejected + failed +
shutdown_dropped`。排空后雷达要求 `received == published`，各类丢帧均为 0；还需对账 bag。
`lidar_integrity=OK` 表示桥接没有检测到雷达队列拒绝或交付/排空失败，不证明上游没丢帧、
存在匹配订阅者或磁盘已经写入成功。接收错误、发布失败和 DDS 确认失败采用保守策略，也会标记失败；接收错误另计入总计的 `receive_errors`。

### 完整录制双雷达

先 source 工作空间。推荐先启动 bridge 并预声明点云 publisher，再启动录包，确认 recorder
已经以 RELIABLE 匹配后，最后开始仿真器的采集。否则启动发现期间的帧不在录制范围内。

```bash
ros2 run ue_zenoh_bridge ue_zenoh_bridge \
  --async-publish --qos-depth 30 \
  --predeclare-topic /front_lidar:=sensor_msgs/msg/PointCloud2 \
  --predeclare-topic /rear_lidar:=sensor_msgs/msg/PointCloud2
```

本包安装 `config/lidar_record_qos.yaml`，对前后雷达显式设置 reliable、volatile、depth=30：

```bash
ros2 bag record -s sqlite3 -o lidar_bag \
  --qos-profile-overrides-path "$(ros2 pkg prefix ue_zenoh_bridge)/share/ue_zenoh_bridge/config/lidar_record_qos.yaml" \
  --max-cache-size 104857600 \
  /front_lidar /rear_lidar
```

此命令不启用压缩。更改雷达 topic 时也要更改 YAML 中的精确名称。通过
`ros2 topic info /front_lidar --verbose` 检查 recorder 的实际订阅 QoS。按所有录制 topic 的
`payload 大小 × Hz` 相加估算带宽；磁盘持续写入能力必须高于输入，缓存可按约 1～2 秒数据量
调整。Humble 的缓存为双缓冲，`--max-cache-size` 指每个缓冲区，峰值可接近设置值的两倍。
不要把录包缓存、DDS history 和桥接 FIFO 当作同一层缓存。

结束时先停止仿真器产生新帧、等待在途数据抵达，再 Ctrl+C 停止 bridge，检查逐雷达
`final_stats`；bridge 退出后再 Ctrl+C 停止 recorder。`ros2 bag info lidar_bag` 只能粗略查看
帧数和时长，准确完整性应比较同一采集区间内的源帧标识/时间戳；不要只用回放时的 `topic hz`
验收。可连续采集 10 分钟，检查每雷达约 6000 帧、无缺失/重复、队列无持续增长。

### 回归验证

```bash
colcon test --packages-select ue_zenoh_bridge
colcon test-result --verbose
```

`test/pending_queue_test.cpp` 覆盖 FIFO 顺序、帧数溢出、跨线程共享字节预算、最新帧替换、
多 key 公平调度及排空释放。端到端测试需要 ROS Humble、Python zenoh 和 rosbag2，使用独立
ROS domain 83、本机 Zenoh 端口 17447，不连接仿真器。输出目录必须不存在：

```bash
python3 test/recording_integration.py \
  --bridge "$(ros2 pkg prefix ue_zenoh_bridge)/lib/ue_zenoh_bridge/ue_zenoh_bridge" \
  --output /tmp/ue_bridge_recording_check
```

默认双雷达各 1 MiB、10 Hz、200 帧，再附加 20 帧突发；同时发送雷达 IMU。测试比较每帧
序列化内容的 SHA256，要求 reliable 订阅和 SQLite bag 中的全部帧内容及顺序与源完全一致。
可用 `--payload-mib`、`--frames`、`--burst` 增加负载；`--pause-recorder-frames 20` 可模拟录包器
暂停 2 秒后恢复，验证缓冲和重传；修改 `--domain` / `--port` 避免冲突。

2026-09-05 本机 Release 验证结果：双雷达各 1 MiB，200 帧 10 Hz + 20 帧突发，订阅与 bag 均为
220/220；双雷达各 4 MiB，120 帧 10 Hz + 20 帧突发，录包器中途暂停 2 秒，订阅与 bag 均为
140/140，稳定段 bag 接收率约 10.02 Hz。全部逐帧 SHA256 和顺序一致，桥接无丢帧/失败。
这是本机合成负载测试，真实 UE、网络和全传感器负载仍应按上述完整性指标验收。

## 9. 常见问题

### CMake 报 zenoh-c was not found

先 source 当前 ROS 2 环境并安装对应的 `zenoh_cpp_vendor`。如果使用系统 zenoh-c，确认开发包
包含 `zenoh.h` 和 `libzenohc`，并通过 `CMAKE_PREFIX_PATH`、`PKG_CONFIG_PATH` 或标准系统
路径使 CMake 能找到它。

### 构建时选中了错误的 Zenoh

查看 CMake 输出中的 `Using ... via target ...`。ROS 2 vendor 默认优先；需要强制使用系统版本时，
传入 `-DCMAKE_DISABLE_FIND_PACKAGE_zenoh_cpp_vendor=TRUE`，并重新使用空的 build 目录构建。

运行时还可以检查动态链接结果：

```bash
ldd install/ue_zenoh_bridge/lib/ue_zenoh_bridge/ue_zenoh_bridge | grep zenoh
```

结果应指向所选 ROS 2 或系统安装，不应指向项目源码目录。

### ROS 2 topic 看不到

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
