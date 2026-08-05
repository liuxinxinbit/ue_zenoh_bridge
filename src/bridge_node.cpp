#include <zenoh.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/exceptions/exceptions.hpp"
#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"

namespace
{

struct BridgeCli
{
  std::string endpoint{"tcp/127.0.0.1:7447"};
  std::string key_expr{"rt/**"};
  std::string strip_prefix{"rt"};
  std::vector<std::string> predeclare_topics;
  std::vector<std::string> topic_type_overrides;
  // This bounds the number of distinct keys waiting for ROS publication.  New
  // samples for a key that is already waiting replace the older sample.
  int max_queue_depth{64};
  int qos_depth{10};
  int worker_count{10};
  bool force_synchronous_publish{false};
  bool force_asynchronous_publish{false};
  bool auto_qos{true};
  bool reliable{false};
  bool verbose{false};
};

class OwnedZenohBytes
{
public:
  OwnedZenohBytes()
  {
    z_internal_null(&bytes_);
  }

  explicit OwnedZenohBytes(const z_loaned_bytes_t * bytes)
  {
    z_bytes_clone(&bytes_, bytes);
  }

  OwnedZenohBytes(const OwnedZenohBytes &) = delete;
  OwnedZenohBytes & operator=(const OwnedZenohBytes &) = delete;

  OwnedZenohBytes(OwnedZenohBytes && other) noexcept
  {
    z_internal_null(&bytes_);
    z_take(&bytes_, z_move(other.bytes_));
  }

  OwnedZenohBytes & operator=(OwnedZenohBytes && other) noexcept
  {
    if (this != &other) {
      if (z_internal_check(bytes_)) {
        z_drop(z_move(bytes_));
      }
      z_take(&bytes_, z_move(other.bytes_));
    }
    return *this;
  }

  ~OwnedZenohBytes()
  {
    if (z_internal_check(bytes_)) {
      z_drop(z_move(bytes_));
    }
  }

  const z_loaned_bytes_t * loan() const
  {
    return z_loan(bytes_);
  }

private:
  z_owned_bytes_t bytes_;
};

struct ZenohSample
{
  std::string key;
  OwnedZenohBytes payload;
  std::size_t payload_size{0};
};

struct PublishRoute
{
  std::string topic;
  std::string type;
  std::shared_ptr<rclcpp::GenericPublisher> publisher;
};

struct WorkerLane
{
  std::mutex mutex;
  std::condition_variable cv;
  std::deque<std::string> pending_keys;
  std::unordered_map<std::string, ZenohSample> pending_samples;
  // Each key is pinned to one lane, so only this lane's worker accesses these
  // cached routes. The hot path therefore avoids global publisher-map locks
  // and repeated topic/type string construction.
  std::unordered_map<std::string, PublishRoute> routes;
  std::thread thread;
};

// Up to four lanes are reserved for small real-time messages. Larger payloads
// are assigned to the remaining lanes. Stable per-key routing preserves order
// while preventing slow image/point-cloud publications from blocking IMU.
constexpr std::size_t kRealtimePayloadMaxBytes = 64u * 1024u;
constexpr std::size_t kMaxPayloadBytes = 256u * 1024u * 1024u;

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.size() >= prefix.size() &&
         value.compare(0, prefix.size(), prefix) == 0;
}

bool ends_with(const std::string & value, const std::string & suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string trim_slashes(std::string value)
{
  while (!value.empty() && value.front() == '/') {
    value.erase(value.begin());
  }
  while (!value.empty() && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

std::string sanitize_ros_topic(std::string topic)
{
  std::replace(topic.begin(), topic.end(), '-', '_');
  std::replace(topic.begin(), topic.end(), '.', '_');
  while (topic.find("//") != std::string::npos) {
    topic.replace(topic.find("//"), 2, "/");
  }
  if (topic.empty() || topic.front() != '/') {
    topic.insert(topic.begin(), '/');
  }
  return topic;
}

std::string normalize_ros_topic_for_type(std::string topic)
{
  topic = sanitize_ros_topic(std::move(topic));
  if (ends_with(topic, "/image")) {
    topic += "/compressed";
  }
  return topic;
}

std::string topic_from_key(const std::string & key, const std::string & strip_prefix)
{
  const std::string prefix = trim_slashes(strip_prefix);
  std::string topic = key;
  if (!prefix.empty() && topic == prefix) {
    topic.clear();
  } else if (!prefix.empty() && starts_with(topic, prefix + "/")) {
    topic.erase(0, prefix.size() + 1);
  }
  return normalize_ros_topic_for_type(topic);
}

std::pair<std::string, std::string> split_mapping(const std::string & mapping)
{
  auto pos = mapping.find(":=");
  const std::size_t sep_len = pos == std::string::npos ? 1 : 2;
  if (pos == std::string::npos) {
    pos = mapping.find('=');
  }
  if (pos == std::string::npos) {
    pos = mapping.find(':');
  }
  if (pos == std::string::npos) {
    return {mapping, ""};
  }
  return {mapping.substr(0, pos), mapping.substr(pos + sep_len)};
}

class CdrReader
{
public:
  CdrReader(const uint8_t * data, std::size_t size)
  : data_(data), size_(size), pos_(4), ok_(data != nullptr && size >= 4)
  {
    if (!ok_ || data_[0] != 0x00 || data_[1] != 0x01) {
      ok_ = false;
    }
  }

  bool ok() const { return ok_; }
  bool end() const { return ok_ && pos_ == size_; }

  bool read_u8(uint8_t & value)
  {
    if (!align(1) || !require(1)) {
      return false;
    }
    value = data_[pos_++];
    return true;
  }

  bool read_bool()
  {
    uint8_t value = 0;
    return read_u8(value);
  }

  bool read_i32(int32_t & value)
  {
    uint32_t raw = 0;
    if (!read_u32(raw)) {
      return false;
    }
    value = static_cast<int32_t>(raw);
    return true;
  }

  bool read_u32(uint32_t & value)
  {
    if (!align(4) || !require(4)) {
      return false;
    }
    value = static_cast<uint32_t>(data_[pos_]) |
      (static_cast<uint32_t>(data_[pos_ + 1]) << 8) |
      (static_cast<uint32_t>(data_[pos_ + 2]) << 16) |
      (static_cast<uint32_t>(data_[pos_ + 3]) << 24);
    pos_ += 4;
    return true;
  }

  bool read_f32(float & value)
  {
    uint32_t raw = 0;
    if (!read_u32(raw)) {
      return false;
    }
    std::memcpy(&value, &raw, sizeof(value));
    return true;
  }

  bool read_f64(double & value)
  {
    if (!align(8) || !require(8)) {
      return false;
    }
    uint64_t raw = 0;
    for (int i = 0; i < 8; ++i) {
      raw |= static_cast<uint64_t>(data_[pos_ + i]) << (8 * i);
    }
    std::memcpy(&value, &raw, sizeof(value));
    pos_ += 8;
    return true;
  }

  bool read_string(std::string * value = nullptr, std::size_t max_length = 4096)
  {
    uint32_t length = 0;
    if (!read_u32(length) || length == 0 || length > max_length || !require(length)) {
      return false;
    }
    if (data_[pos_ + length - 1] != 0) {
      ok_ = false;
      return false;
    }
    if (value) {
      value->assign(reinterpret_cast<const char *>(data_ + pos_), length - 1);
    }
    pos_ += length;
    return align(4);
  }

  bool skip(std::size_t count)
  {
    if (!require(count)) {
      return false;
    }
    pos_ += count;
    return true;
  }

private:
  bool align(std::size_t alignment)
  {
    if (!ok_ || alignment == 0) {
      ok_ = false;
      return false;
    }
    const std::size_t relative = pos_ >= 4 ? pos_ - 4 : 0;
    const std::size_t padding = (alignment - (relative % alignment)) % alignment;
    return skip(padding);
  }

  bool require(std::size_t count)
  {
    if (!ok_ || count > size_ - pos_) {
      ok_ = false;
      return false;
    }
    return true;
  }

  const uint8_t * data_;
  std::size_t size_;
  std::size_t pos_;
  bool ok_;
};

struct PointFieldInfo
{
  std::string name;
  uint32_t offset{0};
  uint8_t datatype{0};
  uint32_t count{0};
};

bool read_header(CdrReader & reader)
{
  int32_t sec = 0;
  uint32_t nanosec = 0;
  return reader.read_i32(sec) && reader.read_u32(nanosec) && nanosec < 1000000000u &&
         reader.read_string(nullptr, 1024);
}

bool skip_f64_array(CdrReader & reader, std::size_t count)
{
  double ignored = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    if (!reader.read_f64(ignored)) {
      return false;
    }
  }
  return true;
}

std::size_t point_field_datatype_size(uint8_t datatype)
{
  switch (datatype) {
    case 1:  // INT8
    case 2:  // UINT8
      return 1;
    case 3:  // INT16
    case 4:  // UINT16
      return 2;
    case 5:  // INT32
    case 6:  // UINT32
    case 7:  // FLOAT32
      return 4;
    case 8:  // FLOAT64
      return 8;
    default:
      return 0;
  }
}

bool has_point_field(const std::vector<PointFieldInfo> & fields, const std::string & name)
{
  return std::any_of(fields.begin(), fields.end(), [&](const PointFieldInfo & field) {
    return field.name == name;
  });
}

bool looks_like_pointcloud2(const uint8_t * payload, std::size_t payload_size)
{
  CdrReader reader(payload, payload_size);
  if (!reader.ok() || !read_header(reader)) {
    return false;
  }

  uint32_t height = 0;
  uint32_t width = 0;
  uint32_t field_count = 0;
  if (!reader.read_u32(height) || !reader.read_u32(width) || !reader.read_u32(field_count)) {
    return false;
  }
  if (height == 0 || width == 0 || field_count == 0 || field_count > 64) {
    return false;
  }

  std::vector<PointFieldInfo> fields;
  fields.reserve(field_count);
  for (uint32_t i = 0; i < field_count; ++i) {
    PointFieldInfo field;
    if (!reader.read_string(&field.name, 256) || !reader.read_u32(field.offset) ||
        !reader.read_u8(field.datatype) || !reader.read_u32(field.count)) {
      return false;
    }
    if (field.name.empty() || field.count == 0 || point_field_datatype_size(field.datatype) == 0) {
      return false;
    }
    fields.push_back(std::move(field));
  }

  if (!has_point_field(fields, "x") || !has_point_field(fields, "y") ||
      !has_point_field(fields, "z")) {
    return false;
  }

  if (!reader.read_bool()) {
    return false;
  }
  uint32_t point_step = 0;
  uint32_t row_step = 0;
  uint32_t data_len = 0;
  if (!reader.read_u32(point_step) || !reader.read_u32(row_step) || !reader.read_u32(data_len)) {
    return false;
  }
  if (point_step == 0 || point_step > 4096) {
    return false;
  }

  const uint64_t expected_row_step = static_cast<uint64_t>(width) * point_step;
  const uint64_t expected_data_len = expected_row_step * height;
  if (expected_row_step > UINT32_MAX || expected_data_len > UINT32_MAX ||
      row_step != expected_row_step || data_len != expected_data_len) {
    return false;
  }

  for (const auto & field : fields) {
    const uint64_t extent = static_cast<uint64_t>(field.offset) +
      static_cast<uint64_t>(point_field_datatype_size(field.datatype)) * field.count;
    if (extent > point_step) {
      return false;
    }
  }

  return reader.skip(data_len) && reader.read_bool() && reader.end();
}

bool looks_like_compressed_image(const uint8_t * payload, std::size_t payload_size)
{
  CdrReader reader(payload, payload_size);
  std::string format;
  uint32_t data_len = 0;
  return reader.ok() && read_header(reader) && reader.read_string(&format, 128) &&
         !format.empty() && reader.read_u32(data_len) && data_len > 0 &&
         reader.skip(data_len) && reader.end();
}

bool looks_like_imu(const uint8_t * payload, std::size_t payload_size)
{
  CdrReader reader(payload, payload_size);
  return reader.ok() && read_header(reader) && skip_f64_array(reader, 37) && reader.end();
}

bool looks_like_odometry(const uint8_t * payload, std::size_t payload_size)
{
  CdrReader reader(payload, payload_size);
  return reader.ok() && read_header(reader) && reader.read_string(nullptr, 1024) &&
         skip_f64_array(reader, 85) && reader.end();
}

bool looks_like_unirtk_pvh(const uint8_t * payload, std::size_t payload_size)
{
  CdrReader reader(payload, payload_size);
  double f64 = 0.0;
  float f32 = 0.0f;
  uint8_t u8 = 0;

  if (!reader.ok() || !read_header(reader) || !read_header(reader) || !reader.read_f64(f64) ||
      !reader.read_u8(u8) || !reader.read_u8(u8)) {
    return false;
  }
  for (int i = 0; i < 5; ++i) {
    if (!reader.read_f32(f32)) {
      return false;
    }
  }
  if (!reader.read_u8(u8) || !reader.read_u8(u8) || !read_header(reader) ||
      !reader.read_f64(f64) || !reader.read_u8(u8) || !reader.read_u8(u8)) {
    return false;
  }
  for (int i = 0; i < 3; ++i) {
    if (!reader.read_f64(f64)) {
      return false;
    }
  }
  for (int i = 0; i < 5; ++i) {
    if (!reader.read_f32(f32)) {
      return false;
    }
  }
  for (int i = 0; i < 4; ++i) {
    if (!reader.read_u8(u8)) {
      return false;
    }
  }
  for (int i = 0; i < 3; ++i) {
    if (!reader.read_f64(f64)) {
      return false;
    }
  }
  for (int i = 0; i < 2; ++i) {
    if (!reader.read_f32(f32)) {
      return false;
    }
  }
  return reader.end();
}

std::string detect_type_from_payload(const uint8_t * payload, std::size_t payload_size)
{
  if (looks_like_pointcloud2(payload, payload_size)) {
    return "sensor_msgs/msg/PointCloud2";
  }
  if (looks_like_compressed_image(payload, payload_size)) {
    return "sensor_msgs/msg/CompressedImage";
  }
  if (looks_like_imu(payload, payload_size)) {
    return "sensor_msgs/msg/Imu";
  }
  if (looks_like_odometry(payload, payload_size)) {
    return "nav_msgs/msg/Odometry";
  }
  if (looks_like_unirtk_pvh(payload, payload_size)) {
    return "robots_dog_msgs/msg/UniRtkPvh";
  }
  return "";
}

std::string default_type_for_topic(const std::string & topic)
{
  if (ends_with(topic, "/image/compressed") || ends_with(topic, "/image")) {
    return "sensor_msgs/msg/CompressedImage";
  }
  if (ends_with(topic, "/pointcloud2") || ends_with(topic, "/points") ||
      ends_with(topic, "/points_raw") || ends_with(topic, "/lidar") ||
      topic.find("lidar") != std::string::npos ||
      topic.find("/pointcloud/") != std::string::npos) {
    return "sensor_msgs/msg/PointCloud2";
  }
  if (ends_with(topic, "/imu") || topic.find("/imu/") != std::string::npos) {
    return "sensor_msgs/msg/Imu";
  }
  if (ends_with(topic, "/gps") || ends_with(topic, "/mujoco_gps")) {
    return "robots_dog_msgs/msg/UniRtkPvh";
  }
  if (ends_with(topic, "/odom") || topic.find("/odom/") != std::string::npos ||
      ends_with(topic, "/mujoco_odom")) {
    return "nav_msgs/msg/Odometry";
  }
  if (ends_with(topic, "/unirtk_pvh") || ends_with(topic, "/rtk/pvh") ||
      ends_with(topic, "/pvh")) {
    return "robots_dog_msgs/msg/UniRtkPvh";
  }
  return "";
}

std::string default_type_for_key_or_topic(const std::string & name)
{
  std::string topic = sanitize_ros_topic(name);
  if (topic == "/rt") {
    topic = "/";
  } else if (starts_with(topic, "/rt/")) {
    topic.erase(0, 3);
  }
  return default_type_for_topic(topic);
}

std::string json_endpoint_array(const std::string & endpoint)
{
  std::ostringstream out;
  out << "[\"";
  for (const char ch : endpoint) {
    if (ch == '"' || ch == '\\') {
      out << '\\';
    }
    out << ch;
  }
  out << "\"]";
  return out.str();
}

bool take_arg(
  const std::vector<std::string> & args, std::size_t & index, const std::string & name,
  std::string * value)
{
  const std::string & arg = args[index];
  const std::string prefix = name + "=";
  if (starts_with(arg, prefix)) {
    *value = arg.substr(prefix.size());
    return true;
  }
  if (arg == name && index + 1 < args.size()) {
    *value = args[++index];
    return true;
  }
  return false;
}

BridgeCli parse_bridge_cli(int argc, char ** argv, std::vector<std::string> * ros_args)
{
  BridgeCli cli;
  ros_args->clear();
  ros_args->push_back(argv[0]);

  std::vector<std::string> args;
  args.reserve(static_cast<std::size_t>(argc));
  for (int i = 0; i < argc; ++i) {
    args.emplace_back(argv[i]);
  }

  for (std::size_t i = 1; i < args.size(); ++i) {
    std::string value;
    if (take_arg(args, i, "--endpoint", &value)) {
      cli.endpoint = value;
    } else if (take_arg(args, i, "--key-expr", &value)) {
      cli.key_expr = value;
    } else if (take_arg(args, i, "--strip-prefix", &value)) {
      cli.strip_prefix = value;
    } else if (take_arg(args, i, "--predeclare-topic", &value)) {
      cli.predeclare_topics.push_back(value);
    } else if (take_arg(args, i, "--topic-type", &value) || take_arg(args, i, "--map", &value)) {
      cli.topic_type_overrides.push_back(value);
    } else if (take_arg(args, i, "--max-queue-depth", &value)) {
      cli.max_queue_depth = std::max(1, std::stoi(value));
    } else if (take_arg(args, i, "--qos-depth", &value)) {
      cli.qos_depth = std::max(1, std::stoi(value));
    } else if (take_arg(args, i, "--worker-count", &value)) {
      cli.worker_count = std::max(1, std::stoi(value));
    } else if (args[i] == "--sync-publish") {
      cli.force_synchronous_publish = true;
      cli.force_asynchronous_publish = false;
    } else if (args[i] == "--async-publish") {
      cli.force_synchronous_publish = false;
      cli.force_asynchronous_publish = true;
    } else if (args[i] == "--reliable") {
      cli.auto_qos = false;
      cli.reliable = true;
    } else if (args[i] == "--best-effort") {
      cli.auto_qos = false;
      cli.reliable = false;
    } else if (args[i] == "--verbose") {
      cli.verbose = true;
    } else {
      ros_args->push_back(args[i]);
    }
  }

  return cli;
}

int declare_positive_int_parameter(
  rclcpp::Node & node, const std::string & name, int default_value)
{
  const auto value =
    node.declare_parameter<int64_t>(name, static_cast<int64_t>(default_value));
  return static_cast<int>(std::clamp<int64_t>(
      value, 1, static_cast<int64_t>(std::numeric_limits<int>::max())));
}

void zenoh_sample_handler(z_loaned_sample_t * sample, void * arg);

}  // namespace

class UeZenohBridgeNode : public rclcpp::Node
{
public:
  explicit UeZenohBridgeNode(const BridgeCli & cli)
  : Node("ue_zenoh_bridge"),
    endpoint_(declare_parameter<std::string>("endpoint", cli.endpoint)),
    key_expr_(declare_parameter<std::string>("key_expr", cli.key_expr)),
    strip_prefix_(declare_parameter<std::string>("strip_prefix", cli.strip_prefix)),
    max_queue_depth_(
      declare_positive_int_parameter(*this, "max_queue_depth", cli.max_queue_depth)),
    qos_depth_(declare_positive_int_parameter(*this, "qos_depth", cli.qos_depth)),
    worker_count_(
      std::clamp(declare_positive_int_parameter(*this, "worker_count", cli.worker_count), 1, 64)),
    lane_queue_depth_(
      std::max(1, (max_queue_depth_ + worker_count_ - 1) / worker_count_)),
    auto_qos_(declare_parameter<bool>("auto_qos", cli.auto_qos)),
    reliable_(declare_parameter<bool>("reliable", cli.reliable)),
    verbose_(declare_parameter<bool>("verbose", cli.verbose))
  {
    z_internal_null(&session_);
    z_internal_null(&subscriber_);

    const auto param_predeclare =
      declare_parameter<std::vector<std::string>>("predeclare_topics", cli.predeclare_topics);
    const auto param_overrides =
      declare_parameter<std::vector<std::string>>("topic_type_overrides", cli.topic_type_overrides);

    for (const auto & mapping : param_overrides) {
      add_type_override(mapping);
    }

    lanes_.reserve(static_cast<std::size_t>(worker_count_));
    for (int i = 0; i < worker_count_; ++i) {
      lanes_.push_back(std::make_unique<WorkerLane>());
    }

    try {
      open_session();
      declare_subscriber();
      for (const auto & item : param_predeclare) {
        predeclare_topic(item);
      }
    } catch (...) {
      close_zenoh();
      throw;
    }

    running_.store(true);
    for (std::size_t i = 0; i < lanes_.size(); ++i) {
      lanes_[i]->thread = std::thread([this, i]() { process_loop(i); });
    }

    RCLCPP_INFO(
      get_logger(), "subscribed Zenoh '%s' via endpoint '%s' with %d publish workers",
      key_expr_.c_str(), endpoint_.empty() ? "<peer/default>" : endpoint_.c_str(), worker_count_);
    const char * publication_mode = std::getenv("RMW_FASTRTPS_PUBLICATION_MODE");
    RCLCPP_INFO(
      get_logger(), "RMW '%s', Fast DDS publication mode '%s'",
      rmw_get_implementation_identifier(), publication_mode ? publication_mode : "<default>");
  }

  ~UeZenohBridgeNode() override
  {
    // Stop Zenoh callbacks before stopping the worker so no sample can be
    // enqueued after the worker has exited.
    close_zenoh();
    running_.store(false);
    for (auto & lane : lanes_) {
      lane->cv.notify_all();
    }
    for (auto & lane : lanes_) {
      if (lane->thread.joinable()) {
        lane->thread.join();
      }
    }
    RCLCPP_INFO(
      get_logger(),
      "shutdown summary: received=%zu, published=%zu, coalesced=%zu, queue_dropped=%zu, "
      "borrowed=%zu, fragmented_copies=%zu",
      received_samples_.load(), published_samples_.load(), coalesced_samples_.load(),
      dropped_samples_.load(), borrowed_publishes_.load(), fragmented_copies_.load());
  }

  void enqueue(ZenohSample sample)
  {
    ++received_samples_;
    const std::size_t lane_index = lane_for(sample.key, sample.payload_size);
    auto & lane = *lanes_[lane_index];
    {
      std::lock_guard<std::mutex> lock(lane.mutex);
      const auto existing = lane.pending_samples.find(sample.key);
      if (existing != lane.pending_samples.end()) {
        // Sensor data is time-sensitive: while a frame is waiting, only the
        // newest frame for that key is useful.
        existing->second = std::move(sample);
        ++coalesced_samples_;
        if (coalesced_samples_ == 1 || coalesced_samples_ % 100 == 0) {
          RCLCPP_WARN(
            get_logger(), "Zenoh bridge replaced %zu stale samples with newer samples",
            coalesced_samples_.load());
        }
        return;
      }

      if (lane.pending_keys.size() >= static_cast<std::size_t>(lane_queue_depth_)) {
        const std::string oldest_key = std::move(lane.pending_keys.front());
        lane.pending_keys.pop_front();
        lane.pending_samples.erase(oldest_key);
        ++dropped_samples_;
        if (dropped_samples_ == 1 || dropped_samples_ % 100 == 0) {
          RCLCPP_WARN(
            get_logger(), "Zenoh bridge queue full, discarded %zu oldest samples",
            dropped_samples_.load());
        }
      }

      const std::string key = sample.key;
      lane.pending_samples.emplace(key, std::move(sample));
      lane.pending_keys.push_back(key);
    }
    lane.cv.notify_one();
  }

private:
  std::size_t lane_for(const std::string & key, std::size_t payload_size)
  {
    if (lanes_.size() == 1) {
      return 0;
    }

    std::lock_guard<std::mutex> lock(route_mutex_);
    const auto existing = key_lanes_.find(key);
    if (existing != key_lanes_.end()) {
      return existing->second;
    }

    const std::size_t realtime_lanes = std::min<std::size_t>(4, lanes_.size() - 1);
    std::size_t lane = 0;
    if (payload_size <= kRealtimePayloadMaxBytes) {
      lane = next_realtime_lane_++ % realtime_lanes;
    } else {
      const std::size_t bulk_lanes = lanes_.size() - realtime_lanes;
      lane = realtime_lanes + (next_bulk_lane_++ % bulk_lanes);
    }
    key_lanes_.emplace(key, lane);
    return lane;
  }

  void open_session()
  {
    z_owned_config_t config;
    z_config_default(&config);

    if (!endpoint_.empty()) {
      const std::string endpoints_json = json_endpoint_array(endpoint_);
      zc_config_insert_json5(z_loan_mut(config), "connect/endpoints", endpoints_json.c_str());
    }

    const int rc = z_open(&session_, z_move(config), nullptr);
    if (rc < 0) {
      throw std::runtime_error("z_open failed, rc=" + std::to_string(rc));
    }
  }

  void declare_subscriber()
  {
    z_view_keyexpr_t keyexpr;
    if (z_view_keyexpr_from_str(&keyexpr, key_expr_.c_str()) < 0) {
      throw std::runtime_error("invalid zenoh key expression: " + key_expr_);
    }

    z_owned_closure_sample_t closure;
    z_closure(&closure, zenoh_sample_handler, nullptr, static_cast<void *>(this));

    std::lock_guard<std::mutex> lock(zenoh_mutex_);
    const int rc = z_declare_subscriber(
      z_loan(session_), &subscriber_, z_loan(keyexpr), z_move(closure), nullptr);
    if (rc < 0) {
      throw std::runtime_error("z_declare_subscriber failed, rc=" + std::to_string(rc));
    }
  }

  void close_zenoh()
  {
    std::lock_guard<std::mutex> lock(zenoh_mutex_);
    if (z_internal_check(subscriber_)) {
      z_drop(z_move(subscriber_));
      z_internal_null(&subscriber_);
    }
    if (z_internal_check(session_)) {
      z_drop(z_move(session_));
      z_internal_null(&session_);
    }
  }

  void add_type_override(const std::string & mapping)
  {
    const auto [name, type] = split_mapping(mapping);
    if (name.empty() || type.empty()) {
      RCLCPP_WARN(get_logger(), "ignoring invalid topic type mapping '%s'", mapping.c_str());
      return;
    }
    type_overrides_[name] = type;
    type_overrides_[sanitize_ros_topic(name)] = type;
    type_overrides_[normalize_ros_topic_for_type(name)] = type;
  }

  void predeclare_topic(const std::string & item)
  {
    const auto [topic_part, type_part] = split_mapping(item);
    const std::string topic = normalize_ros_topic_for_type(topic_part);
    std::string type = type_part.empty() ? default_type_for_topic(topic) : type_part;
    if (type.empty()) {
      type = "sensor_msgs/msg/CompressedImage";
    }
    get_or_create_publisher(topic, type);
  }

  void process_loop(std::size_t lane_index)
  {
    auto & lane = *lanes_[lane_index];
    // Fragmented Zenoh payloads need one flattening copy. Keep that allocation
    // for the lifetime of the lane instead of allocating a point-cloud-sized
    // buffer for every frame.
    rclcpp::SerializedMessage scratch;
    while (rclcpp::ok() && running_.load()) {
      ZenohSample sample;
      {
        std::unique_lock<std::mutex> lock(lane.mutex);
        lane.cv.wait(lock, [this, &lane]() {
          return !running_.load() || !lane.pending_keys.empty();
        });
        if (!running_.load()) {
          break;
        }
        const std::string key = std::move(lane.pending_keys.front());
        lane.pending_keys.pop_front();
        auto sample_it = lane.pending_samples.find(key);
        sample = std::move(sample_it->second);
        lane.pending_samples.erase(sample_it);
      }
      publish_sample(sample, lane, scratch);
    }
  }

  const uint8_t * contiguous_payload(const ZenohSample & sample) const
  {
    auto iterator = z_bytes_get_slice_iterator(sample.payload.loan());
    z_view_slice_t slice;
    if (!z_bytes_slice_iterator_next(&iterator, &slice)) {
      return nullptr;
    }
    const auto * loaned_slice = z_loan(slice);
    if (z_slice_len(loaned_slice) != sample.payload_size) {
      return nullptr;
    }
    return z_slice_data(loaned_slice);
  }

  const uint8_t * materialize_payload(
    const ZenohSample & sample, rclcpp::SerializedMessage & scratch)
  {
    if (scratch.capacity() < sample.payload_size) {
      scratch.reserve(sample.payload_size);
    }
    auto & serialized = scratch.get_rcl_serialized_message();
    z_bytes_reader_t reader = z_bytes_get_reader(sample.payload.loan());
    const std::size_t copied =
      z_bytes_reader_read(&reader, serialized.buffer, sample.payload_size);
    if (copied != sample.payload_size || z_bytes_reader_remaining(&reader) != 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "failed to copy complete Zenoh payload: copied %zu of %zu bytes",
        copied, sample.payload_size);
      serialized.buffer_length = 0;
      return nullptr;
    }
    serialized.buffer_length = sample.payload_size;
    ++fragmented_copies_;
    return serialized.buffer;
  }

  PublishRoute * resolve_route(
    const ZenohSample & sample, WorkerLane & lane, const uint8_t * data)
  {
    if (const auto it = lane.routes.find(sample.key); it != lane.routes.end()) {
      return &it->second;
    }

    const std::string topic = topic_from_key(sample.key, strip_prefix_);
    std::string type = resolve_type(sample.key, topic, data, sample.payload_size);
    if (type.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no ROS type mapping for Zenoh key '%s' -> topic '%s'; use --topic-type %s:=TYPE",
        sample.key.c_str(), topic.c_str(), sample.key.c_str());
      return nullptr;
    }

    auto pub = get_or_create_publisher(topic, type);
    if (!pub) {
      return nullptr;
    }

    const auto [it, inserted] = lane.routes.emplace(
      sample.key, PublishRoute{topic, type, std::move(pub)});
    (void)inserted;
    return &it->second;
  }

  void publish_borrowed(
    const std::shared_ptr<rclcpp::GenericPublisher> & publisher,
    const uint8_t * data, std::size_t size)
  {
    rcl_serialized_message_t serialized = rmw_get_zero_initialized_serialized_message();
    serialized.buffer = const_cast<uint8_t *>(data);
    serialized.buffer_length = size;
    serialized.buffer_capacity = size;
    serialized.allocator = rcl_get_default_allocator();
    const rcl_ret_t rc = rcl_publish_serialized_message(
      publisher->get_publisher_handle().get(), &serialized, nullptr);
    if (rc != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(rc, "failed to publish serialized message");
    }
  }

  void publish_sample(
    const ZenohSample & sample, WorkerLane & lane, rclcpp::SerializedMessage & scratch)
  {
    if (sample.payload_size == 0) {
      return;
    }

    const uint8_t * data = contiguous_payload(sample);
    const bool borrowed = data != nullptr;
    if (!borrowed) {
      data = materialize_payload(sample, scratch);
      if (!data) {
        return;
      }
    }

    PublishRoute * route = resolve_route(sample, lane, data);
    if (!route) {
      return;
    }

    try {
      if (borrowed) {
        // The shallow-cloned Zenoh Bytes object keeps this slice alive until
        // the synchronous rcl publish call returns. DDS may still copy it.
        publish_borrowed(route->publisher, data, sample.payload_size);
        ++borrowed_publishes_;
      } else {
        route->publisher->publish(scratch);
      }
      ++published_samples_;
      if (verbose_) {
        RCLCPP_INFO(
          get_logger(), "published %zu bytes: %s -> %s [%s]",
          sample.payload_size, sample.key.c_str(), route->topic.c_str(), route->type.c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "failed to publish '%s' as '%s': %s",
        route->topic.c_str(), route->type.c_str(), e.what());
    }
  }

  std::string resolve_type(
    const std::string & key, const std::string & topic, const uint8_t * payload,
    std::size_t payload_size)
  {
    if (const auto it = type_overrides_.find(key); it != type_overrides_.end()) {
      return it->second;
    }
    if (const auto it = type_overrides_.find(topic); it != type_overrides_.end()) {
      return it->second;
    }

    {
      std::lock_guard<std::mutex> lock(type_mutex_);
      if (const auto it = detected_types_.find(key); it != detected_types_.end()) {
        return it->second;
      }
    }

    std::string type = detect_type_from_payload(payload, payload_size);
    if (!type.empty()) {
      std::lock_guard<std::mutex> lock(type_mutex_);
      detected_types_.emplace(key, type);
      return type;
    }

    type = default_type_for_key_or_topic(key);
    if (type.empty()) {
      type = default_type_for_key_or_topic(topic);
    }
    if (!type.empty()) {
      std::lock_guard<std::mutex> lock(type_mutex_);
      detected_types_.emplace(key, type);
    }
    return type;
  }

  std::shared_ptr<rclcpp::GenericPublisher> get_or_create_publisher(
    const std::string & topic, const std::string & type)
  {
    std::lock_guard<std::mutex> lock(publisher_mutex_);
    const std::string key = topic + "|" + type;
    if (const auto it = publishers_.find(key); it != publishers_.end()) {
      return it->second;
    }

    const bool pointcloud = type == "sensor_msgs/msg/PointCloud2";
    const bool use_reliable = reliable_ || (auto_qos_ && pointcloud);
    const int depth = auto_qos_ && !pointcloud ? 1 : std::max(1, qos_depth_);
    rclcpp::QoS qos{rclcpp::KeepLast(depth)};
    if (use_reliable) {
      qos.reliable();
    } else {
      qos.best_effort();
    }
    qos.durability_volatile();

    try {
      auto pub = create_generic_publisher(topic, type, qos);
      publishers_[key] = pub;
      RCLCPP_INFO(
        get_logger(), "created ROS2 publisher: %s [%s], qos=%s depth=%d",
        topic.c_str(), type.c_str(), use_reliable ? "reliable" : "best_effort", depth);
      return pub;
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "cannot create generic publisher for %s [%s]: %s",
        topic.c_str(), type.c_str(), e.what());
      return nullptr;
    }
  }

  std::string endpoint_;
  std::string key_expr_;
  std::string strip_prefix_;
  int max_queue_depth_;
  int qos_depth_;
  int worker_count_;
  int lane_queue_depth_;
  bool auto_qos_;
  bool reliable_;
  bool verbose_;

  z_owned_session_t session_;
  z_owned_subscriber_t subscriber_;
  std::mutex zenoh_mutex_;

  std::atomic<bool> running_{false};
  std::vector<std::unique_ptr<WorkerLane>> lanes_;
  std::mutex route_mutex_;
  std::unordered_map<std::string, std::size_t> key_lanes_;
  std::size_t next_realtime_lane_{0};
  std::size_t next_bulk_lane_{0};
  std::atomic<std::size_t> received_samples_{0};
  std::atomic<std::size_t> published_samples_{0};
  std::atomic<std::size_t> dropped_samples_{0};
  std::atomic<std::size_t> coalesced_samples_{0};
  std::atomic<std::size_t> borrowed_publishes_{0};
  std::atomic<std::size_t> fragmented_copies_{0};

  std::unordered_map<std::string, std::string> type_overrides_;
  std::mutex type_mutex_;
  std::unordered_map<std::string, std::string> detected_types_;
  std::mutex publisher_mutex_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> publishers_;
};

namespace
{

void zenoh_sample_handler(z_loaned_sample_t * sample, void * arg)
{
  auto * bridge = reinterpret_cast<UeZenohBridgeNode *>(arg);
  if (!bridge || !sample) {
    return;
  }

  z_view_string_t key_string;
  z_keyexpr_as_view_string(z_sample_keyexpr(sample), &key_string);
  const char * key_data = z_string_data(z_loan(key_string));
  const std::size_t key_len = z_string_len(z_loan(key_string));

  const auto * payload = z_sample_payload(sample);
  const std::size_t payload_len = z_bytes_len(payload);
  if (payload_len == 0 || payload_len > kMaxPayloadBytes) {
    RCLCPP_WARN_THROTTLE(
      bridge->get_logger(), *bridge->get_clock(), 5000,
      "discarding invalid Zenoh payload size: %zu bytes", payload_len);
    return;
  }

  try {
    ZenohSample out;
    if (key_data && key_len > 0) {
      out.key.assign(key_data, key_len);
    }
    // Cloning Zenoh Bytes is shallow. Keep the transport buffer alive for the
    // publish worker and return this callback immediately; this avoids a large
    // allocation and memcpy on Zenoh's receive thread for every lidar frame.
    out.payload = OwnedZenohBytes(payload);
    out.payload_size = payload_len;
    bridge->enqueue(std::move(out));
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      bridge->get_logger(), *bridge->get_clock(), 2000,
      "failed to receive Zenoh sample: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR_THROTTLE(
      bridge->get_logger(), *bridge->get_clock(), 2000,
      "failed to receive Zenoh sample: unknown error");
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  std::vector<std::string> ros_arg_strings;
  const BridgeCli cli = parse_bridge_cli(argc, argv, &ros_arg_strings);

  // Fast DDS defaults to synchronous publication on ROS 2 Humble. A large
  // fragmented point cloud can then block the only worker assigned to that
  // lidar key for hundreds of milliseconds. Use its asynchronous writer by
  // default, while respecting an environment value explicitly set by the user.
  if (cli.force_synchronous_publish) {
    (void)setenv("RMW_FASTRTPS_PUBLICATION_MODE", "SYNCHRONOUS", 1);
  } else if (cli.force_asynchronous_publish) {
    (void)setenv("RMW_FASTRTPS_PUBLICATION_MODE", "ASYNCHRONOUS", 1);
  } else if (std::getenv("RMW_FASTRTPS_PUBLICATION_MODE") == nullptr) {
    (void)setenv("RMW_FASTRTPS_PUBLICATION_MODE", "ASYNCHRONOUS", 0);
  }

  std::vector<char *> ros_argv;
  ros_argv.reserve(ros_arg_strings.size());
  for (auto & arg : ros_arg_strings) {
    ros_argv.push_back(arg.data());
  }
  int ros_argc = static_cast<int>(ros_argv.size());

  rclcpp::init(ros_argc, ros_argv.data());
  try {
    rclcpp::spin(std::make_shared<UeZenohBridgeNode>(cli));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("ue_zenoh_bridge"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
