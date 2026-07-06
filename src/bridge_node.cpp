#include <zenoh.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

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
  int max_queue_depth{1024};
  int qos_depth{10};
  bool reliable{false};
  bool verbose{false};
};

struct ZenohSample
{
  std::string key;
  std::vector<uint8_t> payload;
};

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
    } else if (args[i] == "--reliable") {
      cli.reliable = true;
    } else if (args[i] == "--best-effort") {
      cli.reliable = false;
    } else if (args[i] == "--verbose") {
      cli.verbose = true;
    } else {
      ros_args->push_back(args[i]);
    }
  }

  return cli;
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
    max_queue_depth_(std::max<int>(
      1, static_cast<int>(declare_parameter<int>("max_queue_depth", cli.max_queue_depth)))),
    qos_depth_(std::max<int>(
      1, static_cast<int>(declare_parameter<int>("qos_depth", cli.qos_depth)))),
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
    worker_ = std::thread([this]() { process_loop(); });

    RCLCPP_INFO(
      get_logger(), "subscribed Zenoh '%s' via endpoint '%s'",
      key_expr_.c_str(), endpoint_.empty() ? "<peer/default>" : endpoint_.c_str());
  }

  ~UeZenohBridgeNode() override
  {
    running_.store(false);
    queue_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }

    close_zenoh();
  }

  void enqueue(ZenohSample sample)
  {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (queue_.size() >= static_cast<std::size_t>(max_queue_depth_)) {
        ++dropped_samples_;
        if (dropped_samples_ == 1 || dropped_samples_ % 100 == 0) {
          RCLCPP_WARN(
            get_logger(), "Zenoh bridge queue full, dropped %zu samples",
            dropped_samples_.load());
        }
        return;
      }
      queue_.push_back(std::move(sample));
    }
    queue_cv_.notify_one();
  }

private:
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

  void process_loop()
  {
    while (rclcpp::ok() && running_.load()) {
      ZenohSample sample;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() { return !running_.load() || !queue_.empty(); });
        if (!running_.load() && queue_.empty()) {
          break;
        }
        sample = std::move(queue_.front());
        queue_.pop_front();
      }
      publish_sample(sample);
    }

    while (true) {
      ZenohSample sample;
      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (queue_.empty()) {
          break;
        }
        sample = std::move(queue_.front());
        queue_.pop_front();
      }
      publish_sample(sample);
    }
  }

  void publish_sample(const ZenohSample & sample)
  {
    if (sample.payload.empty()) {
      return;
    }

    const std::string topic = topic_from_key(sample.key, strip_prefix_);
    std::string type = resolve_type(sample.key, topic);
    if (type.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no ROS type mapping for Zenoh key '%s' -> topic '%s'; use --topic-type %s:=TYPE",
        sample.key.c_str(), topic.c_str(), sample.key.c_str());
      return;
    }

    auto pub = get_or_create_publisher(topic, type);
    if (!pub) {
      return;
    }

    rclcpp::SerializedMessage serialized(sample.payload.size());
    auto & ros_msg = serialized.get_rcl_serialized_message();
    std::memcpy(ros_msg.buffer, sample.payload.data(), sample.payload.size());
    ros_msg.buffer_length = sample.payload.size();

    try {
      pub->publish(serialized);
      if (verbose_) {
        RCLCPP_INFO(
          get_logger(), "published %zu bytes: %s -> %s [%s]",
          sample.payload.size(), sample.key.c_str(), topic.c_str(), type.c_str());
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "failed to publish '%s' as '%s': %s", topic.c_str(), type.c_str(), e.what());
    }
  }

  std::string resolve_type(const std::string & key, const std::string & topic) const
  {
    if (const auto it = type_overrides_.find(key); it != type_overrides_.end()) {
      return it->second;
    }
    if (const auto it = type_overrides_.find(topic); it != type_overrides_.end()) {
      return it->second;
    }
    std::string type = default_type_for_key_or_topic(key);
    if (!type.empty()) {
      return type;
    }
    return default_type_for_key_or_topic(topic);
  }

  std::shared_ptr<rclcpp::GenericPublisher> get_or_create_publisher(
    const std::string & topic, const std::string & type)
  {
    const std::string key = topic + "|" + type;
    if (const auto it = publishers_.find(key); it != publishers_.end()) {
      return it->second;
    }

    rclcpp::QoS qos(rclcpp::KeepLast(std::max(1, qos_depth_)));
    if (reliable_) {
      qos.reliable();
    } else {
      qos.best_effort();
    }
    qos.durability_volatile();

    try {
      auto pub = create_generic_publisher(topic, type, qos);
      publishers_[key] = pub;
      RCLCPP_INFO(get_logger(), "created ROS2 publisher: %s [%s]", topic.c_str(), type.c_str());
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
  bool reliable_;
  bool verbose_;

  z_owned_session_t session_;
  z_owned_subscriber_t subscriber_;
  std::mutex zenoh_mutex_;

  std::atomic<bool> running_{false};
  std::thread worker_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<ZenohSample> queue_;
  std::atomic<std::size_t> dropped_samples_{0};

  std::unordered_map<std::string, std::string> type_overrides_;
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

  z_owned_slice_t slice;
  z_bytes_to_slice(z_sample_payload(sample), &slice);
  const uint8_t * payload_data = z_slice_data(z_loan(slice));
  const std::size_t payload_len = z_slice_len(z_loan(slice));

  ZenohSample out;
  if (key_data && key_len > 0) {
    out.key.assign(key_data, key_len);
  }
  if (payload_data && payload_len > 0) {
    out.payload.assign(payload_data, payload_data + payload_len);
  }

  z_drop(z_move(slice));
  bridge->enqueue(std::move(out));
}

}  // namespace

int main(int argc, char ** argv)
{
  std::vector<std::string> ros_arg_strings;
  const BridgeCli cli = parse_bridge_cli(argc, argv, &ros_arg_strings);

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
