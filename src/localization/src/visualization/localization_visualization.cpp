#include "localization_visualization.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace mando_localization {
namespace {

template <typename T>
T requireParameter(const ros::NodeHandle& node, const std::string& name) {
  T value;
  if (!node.getParam(name, value)) {
    throw std::runtime_error("필수 파라미터가 없습니다: " + node.resolveName(name));
  }
  return value;
}

void validateColor(const std::vector<double>& color,
                   const std::string& name) {
  if (color.size() != 4) {
    throw std::runtime_error(name + "는 RGBA 길이 4 배열이어야 합니다.");
  }
  for (const double value : color) {
    if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
      throw std::runtime_error(name + " 값은 [0, 1] 범위여야 합니다.");
    }
  }
}

}  // namespace

LocalizationVisualization::LocalizationVisualization(ros::NodeHandle nh,
                                                     ros::NodeHandle private_nh)
    : nh_(std::move(nh)), private_nh_(std::move(private_nh)) {
  const std::string odometry_topic =
      requireParameter<std::string>(private_nh_, "topics/output_odometry");
  const std::string valid_topic = requireParameter<std::string>(private_nh_, "topics/valid");
  const std::string path_topic = requireParameter<std::string>(private_nh_, "topics/path");
  const std::string marker_topic =
      requireParameter<std::string>(private_nh_, "topics/markers");
  const std::string gps_pose_topic =
      requireParameter<std::string>(private_nh_, "topics/gps_map_pose");
  map_frame_ = requireParameter<std::string>(private_nh_, "fixed_frame");
  const double publish_rate_hz =
      requireParameter<double>(private_nh_, "publish_rate_hz");
  const int max_path_points = requireParameter<int>(private_nh_, "path/max_pose_count");
  clear_on_invalid_ = requireParameter<bool>(private_nh_, "path/clear_on_invalid");
  min_translation_m_ =
      requireParameter<double>(private_nh_, "path/min_translation_m");
  min_rotation_rad_ =
      requireParameter<double>(private_nh_, "path/min_rotation_rad");
  path_line_width_m_ =
      requireParameter<double>(private_nh_, "path/line_width_m");
  path_color_ =
      requireParameter<std::vector<double>>(private_nh_, "path/color_rgba");
  vehicle_scale_ = requireParameter<std::vector<double>>(
      private_nh_, "markers/vehicle_scale_m");
  gps_scale_m_ =
      requireParameter<double>(private_nh_, "markers/gps_scale_m");
  gps_color_ = requireParameter<std::vector<double>>(
      private_nh_, "markers/gps_color_rgba");
  fault_color_ = requireParameter<std::vector<double>>(
      private_nh_, "markers/fault_color_rgba");
  if (map_frame_.empty() || max_path_points <= 0 ||
      !std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0 ||
      !std::isfinite(min_translation_m_) || min_translation_m_ < 0.0 ||
      !std::isfinite(min_rotation_rad_) || min_rotation_rad_ < 0.0 ||
      !std::isfinite(path_line_width_m_) || path_line_width_m_ <= 0.0 ||
      vehicle_scale_.size() != 3 ||
      std::any_of(vehicle_scale_.begin(), vehicle_scale_.end(),
                  [](double value) { return !std::isfinite(value) || value <= 0.0; }) ||
      !std::isfinite(gps_scale_m_) || gps_scale_m_ <= 0.0) {
    throw std::runtime_error("시각화 frame 또는 max_path_points가 유효하지 않습니다.");
  }
  validateColor(path_color_, "path/color_rgba");
  validateColor(gps_color_, "markers/gps_color_rgba");
  validateColor(fault_color_, "markers/fault_color_rgba");
  max_path_points_ = static_cast<std::size_t>(max_path_points);
  path_.header.frame_id = map_frame_;
  path_publisher_ = nh_.advertise<nav_msgs::Path>(path_topic, 1, true);
  marker_publisher_ =
      nh_.advertise<visualization_msgs::MarkerArray>(marker_topic, 1, true);
  odometry_subscriber_ = nh_.subscribe(
      odometry_topic, 50, &LocalizationVisualization::odometryCallback, this);
  valid_subscriber_ =
      nh_.subscribe(valid_topic, 10, &LocalizationVisualization::validCallback, this);
  gps_pose_subscriber_ = nh_.subscribe(
      gps_pose_topic, 10, &LocalizationVisualization::gpsPoseCallback, this);
  publish_timer_ = nh_.createTimer(
      ros::Duration(1.0 / publish_rate_hz),
      &LocalizationVisualization::publishTimerCallback, this);
  ROS_INFO_STREAM("LocalizationVisualization 설정: path=" << path_topic
                  << ", 최대 점=" << max_path_points_);
}

void LocalizationVisualization::odometryCallback(const nav_msgs::OdometryConstPtr& message) {
  if (!valid_ || message->header.frame_id != map_frame_ || message->header.stamp.isZero() ||
      !std::isfinite(message->pose.pose.position.x) ||
      !std::isfinite(message->pose.pose.position.y)) {
    return;
  }
  if (!path_.poses.empty() && message->header.stamp <= path_.poses.back().header.stamp) {
    path_.poses.clear();
  }
  odometry_ = *message;
  have_odometry_ = true;
  if (!path_.poses.empty()) {
    const geometry_msgs::Pose& previous = path_.poses.back().pose;
    const double translation = std::hypot(
        message->pose.pose.position.x - previous.position.x,
        message->pose.pose.position.y - previous.position.y);
    const double rotation = std::abs(
        std::atan2(std::sin(tf2::getYaw(message->pose.pose.orientation) -
                            tf2::getYaw(previous.orientation)),
                   std::cos(tf2::getYaw(message->pose.pose.orientation) -
                            tf2::getYaw(previous.orientation))));
    if (translation < min_translation_m_ && rotation < min_rotation_rad_) {
      return;
    }
  }
  geometry_msgs::PoseStamped pose;
  pose.header = message->header;
  pose.pose = message->pose.pose;
  path_.poses.push_back(pose);
  if (path_.poses.size() > max_path_points_) {
    const auto remove_count = path_.poses.size() - max_path_points_;
    path_.poses.erase(path_.poses.begin(), path_.poses.begin() + remove_count);
  }
  path_.header.stamp = message->header.stamp;
  path_publisher_.publish(path_);
}

void LocalizationVisualization::gpsPoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& message) {
  if (message->header.frame_id == map_frame_ && !message->header.stamp.isZero()) {
    gps_pose_ = *message;
    have_gps_pose_ = true;
  }
}

visualization_msgs::Marker LocalizationVisualization::makePoseMarker(
    const int id, const std::string& name, const geometry_msgs::Pose& pose,
    const double scale, const std::vector<double>& color) const {
  visualization_msgs::Marker marker;
  marker.header.frame_id = map_frame_;
  marker.header.stamp = ros::Time::now();
  marker.ns = name;
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  return marker;
}

void LocalizationVisualization::publishTimerCallback(const ros::TimerEvent&) {
  visualization_msgs::MarkerArray output;
  if (have_odometry_) {
    visualization_msgs::Marker vehicle = makePoseMarker(
        0, "vehicle", odometry_.pose.pose, 1.0,
        valid_ ? path_color_ : fault_color_);
    vehicle.type = visualization_msgs::Marker::CUBE;
    vehicle.pose.orientation = odometry_.pose.pose.orientation;
    vehicle.scale.x = vehicle_scale_[0];
    vehicle.scale.y = vehicle_scale_[1];
    vehicle.scale.z = vehicle_scale_[2];
    output.markers.push_back(vehicle);
  }
  if (have_gps_pose_) {
    output.markers.push_back(makePoseMarker(
        1, "gps", gps_pose_.pose.pose, gps_scale_m_, gps_color_));
  }
  visualization_msgs::Marker path_marker;
  path_marker.header = path_.header;
  path_marker.ns = "path";
  path_marker.id = 3;
  path_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::Marker::ADD;
  path_marker.pose.orientation.w = 1.0;
  path_marker.scale.x = path_line_width_m_;
  path_marker.color.r = path_color_[0];
  path_marker.color.g = path_color_[1];
  path_marker.color.b = path_color_[2];
  path_marker.color.a = path_color_[3];
  for (const geometry_msgs::PoseStamped& pose : path_.poses) {
    path_marker.points.push_back(pose.pose.position);
  }
  output.markers.push_back(path_marker);
  marker_publisher_.publish(output);
}

void LocalizationVisualization::validCallback(const std_msgs::BoolConstPtr& message) {
  if (valid_ && !message->data && clear_on_invalid_) {
    path_.poses.clear();
    path_.header.stamp = ros::Time::now();
    path_publisher_.publish(path_);
  }
  valid_ = message->data;
}

}  // namespace mando_localization
