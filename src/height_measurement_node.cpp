#include "height_measurements/npy_reader.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace height_measurements {
namespace {

constexpr int kNumX = 12;
constexpr int kNumY = 11;
constexpr int kNumHeights = kNumX * kNumY;
constexpr std::array<double, kNumX> kMeasuredPointsX = {
    -0.45, -0.3, -0.15, 0.0, 0.15, 0.3,
    0.45,  0.6, 0.75, 0.9, 1.05, 1.2};
constexpr std::array<double, kNumY> kMeasuredPointsY = {
    -0.75, -0.6, -0.45, -0.3, -0.15, 0.0,
    0.15,  0.3, 0.45,  0.6,  0.75};

struct Metadata {
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::size_t width = 0;
  std::size_t height = 0;
  std::string frame_id = "map";
  std::string z_mode = "unknown";
};

struct PoseState {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  std::string frame_id;
};

enum class HeightFormula {
  TerrainMinusBase,
  BaseMinusTerrain,
  LeggedGym,
};

double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

HeightFormula parse_formula(const std::string &value) {
  if (value == "terrain_minus_base") {
    return HeightFormula::TerrainMinusBase;
  }
  if (value == "base_minus_terrain") {
    return HeightFormula::BaseMinusTerrain;
  }
  if (value == "legged_gym") {
    return HeightFormula::LeggedGym;
  }
  throw std::runtime_error(
      "height_formula must be terrain_minus_base, base_minus_terrain, or "
      "legged_gym");
}

Metadata load_metadata(const std::string &path) {
  const YAML::Node root = YAML::LoadFile(path);
  Metadata metadata;
  metadata.resolution = root["resolution"].as<double>();
  metadata.origin_x = root["origin_x"].as<double>();
  metadata.origin_y = root["origin_y"].as<double>();
  metadata.width = root["width"].as<std::size_t>();
  metadata.height = root["height"].as<std::size_t>();
  if (root["frame_id"]) {
    metadata.frame_id = root["frame_id"].as<std::string>();
  }
  if (root["z_mode"]) {
    metadata.z_mode = root["z_mode"].as<std::string>();
  }
  if (metadata.resolution <= 0.0 || metadata.width == 0 ||
      metadata.height == 0) {
    throw std::runtime_error("metadata has invalid resolution/width/height");
  }
  return metadata;
}

} // namespace

class HeightMeasurementNode : public rclcpp::Node {
public:
  HeightMeasurementNode() : Node("height_measurement_node") {
    heightmap_path_ = declare_parameter<std::string>("heightmap_path", "");
    metadata_path_ = declare_parameter<std::string>("metadata_path", "");
    odom_topic_ =
        declare_parameter<std::string>("odom_topic", "/aft_mapped_in_map");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_topic_ =
        declare_parameter<std::string>("publish_topic", "/heightmap_points");
    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
    height_formula_name_ =
        declare_parameter<std::string>("height_formula", "legged_gym");
    measured_height_offset_ =
        declare_parameter<double>("measured_height_offset", 0.3);
    base_to_odom_x_ = declare_parameter<double>("base_to_odom_x", 0.16266);
    base_to_odom_y_ = declare_parameter<double>("base_to_odom_y", 0.0);
    base_to_odom_z_ = declare_parameter<double>("base_to_odom_z", 0.11703);
    clip_min_ = declare_parameter<double>("clip_min", -1.0);
    clip_max_ = declare_parameter<double>("clip_max", 1.0);
    use_bilinear_ = declare_parameter<bool>("use_bilinear", true);
    default_height_ = declare_parameter<double>("default_height", 0.0);

    if (heightmap_path_.empty()) {
      throw std::runtime_error("heightmap_path parameter is required");
    }
    if (metadata_path_.empty()) {
      throw std::runtime_error("metadata_path parameter is required");
    }
    if (publish_rate_ <= 0.0) {
      throw std::runtime_error("publish_rate must be positive");
    }
    if (clip_min_ > clip_max_) {
      throw std::runtime_error("clip_min must be <= clip_max");
    }

    formula_ = parse_formula(height_formula_name_);
    metadata_ = load_metadata(metadata_path_);
    heightmap_ = load_npy_2d_float(heightmap_path_);
    if (heightmap_.rows != metadata_.height || heightmap_.cols != metadata_.width) {
      throw std::runtime_error("metadata width/height do not match npy shape");
    }

    build_local_points();

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();
    qos.durability_volatile();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, qos,
        std::bind(&HeightMeasurementNode::odom_callback, this,
                  std::placeholders::_1));
    publisher_ =
        create_publisher<std_msgs::msg::Float32MultiArray>(publish_topic_, qos);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_));
    timer_ = create_wall_timer(period, std::bind(&HeightMeasurementNode::on_timer, this));
    last_stats_time_ = now();

    RCLCPP_INFO(get_logger(),
                "Loaded heightmap shape=(%zu, %zu), resolution=%.4f, "
                "origin=(%.4f, %.4f), metadata_frame=%s, z_mode=%s",
                heightmap_.rows, heightmap_.cols, metadata_.resolution,
                metadata_.origin_x, metadata_.origin_y,
                metadata_.frame_id.c_str(), metadata_.z_mode.c_str());
    RCLCPP_INFO(get_logger(),
                "Subscribing odom=%s, publishing %s at %.2f Hz, map_frame=%s, "
                "base_frame=%s, formula=%s, bilinear=%s, "
                "base_to_odom=(%.5f, %.5f, %.5f)",
                odom_topic_.c_str(), publish_topic_.c_str(), publish_rate_,
                map_frame_.c_str(), base_frame_.c_str(),
                height_formula_name_.c_str(), use_bilinear_ ? "true" : "false",
                base_to_odom_x_, base_to_odom_y_, base_to_odom_z_);
    if (metadata_.frame_id != map_frame_) {
      RCLCPP_WARN(get_logger(),
                  "metadata frame_id (%s) differs from map_frame parameter (%s). "
                  "Make sure the PCD and SLAM map frames are aligned.",
                  metadata_.frame_id.c_str(), map_frame_.c_str());
    }
  }

private:
  void build_local_points() {
    local_points_.clear();
    local_points_.reserve(kNumHeights);
    for (int ix = 0; ix < kNumX; ++ix) {
      const double x = kMeasuredPointsX[ix];
      for (int iy = 0; iy < kNumY; ++iy) {
        const double y = kMeasuredPointsY[iy];
        local_points_.emplace_back(x, y);
      }
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    PoseState pose;
    pose.yaw = quaternion_to_yaw(msg->pose.pose.orientation);
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    pose.x = msg->pose.pose.position.x -
             (c * base_to_odom_x_ - s * base_to_odom_y_);
    pose.y = msg->pose.pose.position.y -
             (s * base_to_odom_x_ + c * base_to_odom_y_);
    pose.z = msg->pose.pose.position.z - base_to_odom_z_;
    pose.frame_id = msg->header.frame_id;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      latest_pose_ = pose;
      have_pose_ = true;
    }

    if (!pose.frame_id.empty() && pose.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Odometry frame_id is %s, expected map_frame %s. Height sampling "
          "assumes odometry is already expressed in the heightmap frame.",
          pose.frame_id.c_str(), map_frame_.c_str());
    }
  }

  bool world_to_grid(double world_x, double world_y, std::int64_t &row,
                     std::int64_t &col) const {
    col = static_cast<std::int64_t>(
        std::floor((world_x - metadata_.origin_x) / metadata_.resolution));
    row = static_cast<std::int64_t>(
        std::floor((world_y - metadata_.origin_y) / metadata_.resolution));
    return row >= 0 && col >= 0 &&
           row < static_cast<std::int64_t>(metadata_.height) &&
           col < static_cast<std::int64_t>(metadata_.width);
  }

  float at(std::int64_t row, std::int64_t col) const {
    const std::size_t idx = static_cast<std::size_t>(row) * metadata_.width +
                            static_cast<std::size_t>(col);
    return heightmap_.data[idx];
  }

  float query_nearest(double world_x, double world_y) const {
    std::int64_t row = 0;
    std::int64_t col = 0;
    if (!world_to_grid(world_x, world_y, row, col)) {
      return static_cast<float>(default_height_);
    }
    const float z = at(row, col);
    return std::isfinite(z) ? z : static_cast<float>(default_height_);
  }

  float query_bilinear(double world_x, double world_y) const {
    const double gx = (world_x - metadata_.origin_x) / metadata_.resolution;
    const double gy = (world_y - metadata_.origin_y) / metadata_.resolution;
    const std::int64_t col0 = static_cast<std::int64_t>(std::floor(gx));
    const std::int64_t row0 = static_cast<std::int64_t>(std::floor(gy));
    const std::int64_t col1 = col0 + 1;
    const std::int64_t row1 = row0 + 1;

    if (row0 < 0 || col0 < 0 || row1 >= static_cast<std::int64_t>(metadata_.height) ||
        col1 >= static_cast<std::int64_t>(metadata_.width)) {
      return query_nearest(world_x, world_y);
    }

    const float q00 = at(row0, col0);
    const float q10 = at(row0, col1);
    const float q01 = at(row1, col0);
    const float q11 = at(row1, col1);
    if (!std::isfinite(q00) || !std::isfinite(q10) || !std::isfinite(q01) ||
        !std::isfinite(q11)) {
      return query_nearest(world_x, world_y);
    }

    const double dx = gx - static_cast<double>(col0);
    const double dy = gy - static_cast<double>(row0);
    const double z = static_cast<double>(q00) * (1.0 - dx) * (1.0 - dy) +
                     static_cast<double>(q10) * dx * (1.0 - dy) +
                     static_cast<double>(q01) * (1.0 - dx) * dy +
                     static_cast<double>(q11) * dx * dy;
    return static_cast<float>(z);
  }

  float apply_formula(float terrain_z, double robot_z) const {
    double h = 0.0;
    switch (formula_) {
    case HeightFormula::TerrainMinusBase:
      h = static_cast<double>(terrain_z) - robot_z;
      break;
    case HeightFormula::BaseMinusTerrain:
      h = robot_z - static_cast<double>(terrain_z);
      break;
    case HeightFormula::LeggedGym:
      h = robot_z - measured_height_offset_ - static_cast<double>(terrain_z);
      break;
    }
    h = std::clamp(h, clip_min_, clip_max_);
    return static_cast<float>(h);
  }

  std::vector<float> sample(const PoseState &pose) const {
    std::vector<float> heights;
    heights.reserve(kNumHeights);
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    for (const auto &[local_x, local_y] : local_points_) {
      const double world_x = pose.x + c * local_x - s * local_y;
      const double world_y = pose.y + s * local_x + c * local_y;
      const float terrain_z =
          use_bilinear_ ? query_bilinear(world_x, world_y)
                        : query_nearest(world_x, world_y);
      heights.push_back(apply_formula(terrain_z, pose.z));
    }
    return heights;
  }

  void on_timer() {
    PoseState pose;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      if (!have_pose_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Waiting for odometry on %s", odom_topic_.c_str());
        return;
      }
      pose = latest_pose_;
    }

    std::vector<float> heights = sample(pose);

    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "x";
    msg.layout.dim[0].size = kNumX;
    msg.layout.dim[0].stride = kNumHeights;
    msg.layout.dim[1].label = "y";
    msg.layout.dim[1].size = kNumY;
    msg.layout.dim[1].stride = kNumY;
    msg.layout.data_offset = 0;
    msg.data = std::move(heights);
    publisher_->publish(msg);

    const rclcpp::Time current = now();
    if ((current - last_stats_time_).seconds() >= 2.0) {
      const auto minmax = std::minmax_element(msg.data.begin(), msg.data.end());
      double sum = 0.0;
      for (const float v : msg.data) {
        sum += static_cast<double>(v);
      }
      const double mean = sum / static_cast<double>(msg.data.size());
      RCLCPP_INFO(get_logger(),
                  "height measurements stats: min=%.4f max=%.4f mean=%.4f "
                  "shape=(%d, %d) flat=%zu",
                  static_cast<double>(*minmax.first),
                  static_cast<double>(*minmax.second), mean, kNumX, kNumY,
                  msg.data.size());
      last_stats_time_ = current;
    }
  }

  std::string heightmap_path_;
  std::string metadata_path_;
  std::string odom_topic_;
  std::string map_frame_;
  std::string base_frame_;
  std::string publish_topic_;
  double publish_rate_ = 50.0;
  std::string height_formula_name_;
  HeightFormula formula_ = HeightFormula::TerrainMinusBase;
  double measured_height_offset_ = 0.3;
  double base_to_odom_x_ = 0.16266;
  double base_to_odom_y_ = 0.0;
  double base_to_odom_z_ = 0.11703;
  double clip_min_ = -1.0;
  double clip_max_ = 1.0;
  bool use_bilinear_ = true;
  double default_height_ = 0.0;

  Metadata metadata_;
  NpyArray2D heightmap_;
  std::vector<std::pair<double, double>> local_points_;

  std::mutex pose_mutex_;
  PoseState latest_pose_;
  bool have_pose_ = false;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_stats_time_;
};

} // namespace height_measurements

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<height_measurements::HeightMeasurementNode>());
  } catch (const std::exception &exc) {
    RCLCPP_ERROR(rclcpp::get_logger("height_measurement_node"), "%s", exc.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
