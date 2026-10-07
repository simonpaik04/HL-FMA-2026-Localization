#include "common/localization_interface_adapter.hpp"

#include <set>
#include <stdexcept>

#include "common/parameter_utils.hpp"

namespace mando_localization {
namespace {

constexpr char kDriverImuTopic[] =
    "/mando_localization/internal/driver/imu";
constexpr char kDriverGpsTopic[] =
    "/mando_localization/internal/driver/gps_fix";
constexpr char kDriverGpsNavPvtTopic[] =
    "/mando_localization/internal/driver/gps_navpvt";
constexpr char kEkfImuTopic[] = "/mando_localization/internal/ekf/imu";
constexpr char kEkfTwistTopic[] = "/mando_localization/internal/ekf/twist";
constexpr char kEkfGpsPoseTopic[] =
    "/mando_localization/internal/ekf/gps_pose";
constexpr char kEkfLocalOdometryTopic[] =
    "/mando_localization/internal/ekf/local_odometry";
constexpr char kEkfGlobalOdometryTopic[] =
    "/mando_localization/internal/ekf/global_odometry";

}  // namespace

LocalizationInterfaceAdapter::LocalizationInterfaceAdapter(
    const ros::NodeHandle& node, const ros::NodeHandle& private_node)
    : node_(node), private_node_(private_node) {
  const std::string imu_data =
      requireParameter<std::string>(private_node_, "topics/imu_data");
  const std::string gps_fix =
      requireParameter<std::string>(private_node_, "topics/gps_fix");
  public_gps_navpvt_topic_ =
      requireParameter<std::string>(private_node_, "topics/gps_navpvt");
  const std::string imu_normalized =
      requireParameter<std::string>(private_node_, "topics/imu_normalized");
  const std::string imu_calibrated =
      requireParameter<std::string>(private_node_, "topics/imu_calibrated");
  const std::string encoder_twist =
      requireParameter<std::string>(private_node_, "topics/encoder_twist");
  const std::string gps_map_pose =
      requireParameter<std::string>(private_node_, "topics/gps_map_pose");
  const std::string local_odometry =
      requireParameter<std::string>(private_node_, "topics/local_odometry");
  const std::string global_odometry =
      requireParameter<std::string>(private_node_, "topics/global_odometry");
  const std::string public_topics[] = {
      imu_data,       gps_fix,       public_gps_navpvt_topic_,
      imu_normalized, imu_calibrated, encoder_twist, gps_map_pose,
      local_odometry, global_odometry};
  std::set<std::string> unique_public_topics;
  for (const std::string& topic : public_topics) {
    requireAbsoluteRosName(topic, "public localization topic");
    if (!unique_public_topics.insert(topic).second) {
      throw std::runtime_error("공개 Localization 토픽이 중복됩니다: " + topic);
    }
  }
  const std::pair<std::string, std::string> public_internal_pairs[] = {
      {imu_data, kDriverImuTopic},
      {gps_fix, kDriverGpsTopic},
      {public_gps_navpvt_topic_, kDriverGpsNavPvtTopic},
      // 보정 노드의 upstream도 EKF 입력에 alias되면 보정 결과가 다시 입력된다.
      {imu_normalized, kEkfImuTopic},
      {imu_calibrated, kEkfImuTopic},
      {encoder_twist, kEkfTwistTopic},
      {gps_map_pose, kEkfGpsPoseTopic},
      {local_odometry, kEkfLocalOdometryTopic},
      {global_odometry, kEkfGlobalOdometryTopic}};
  for (const auto& pair : public_internal_pairs) {
    if (pair.first == pair.second) {
      throw std::runtime_error("공개 토픽과 내부 토픽이 같아 relay loop가 발생합니다: " +
                               pair.first);
    }
  }

  public_imu_publisher_ = node_.advertise<sensor_msgs::Imu>(imu_data, 30);
  public_gps_publisher_ =
      node_.advertise<sensor_msgs::NavSatFix>(gps_fix, 20);
  internal_imu_publisher_ =
      node_.advertise<sensor_msgs::Imu>(kEkfImuTopic, 30);
  internal_twist_publisher_ =
      node_.advertise<geometry_msgs::TwistWithCovarianceStamped>(
          kEkfTwistTopic, 30);
  internal_gps_pose_publisher_ =
      node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
          kEkfGpsPoseTopic, 20);
  public_local_odometry_publisher_ =
      node_.advertise<nav_msgs::Odometry>(local_odometry, 30);
  public_global_odometry_publisher_ =
      node_.advertise<nav_msgs::Odometry>(global_odometry, 30);

  imu_driver_subscriber_ = node_.subscribe(
      kDriverImuTopic, 50,
      &LocalizationInterfaceAdapter::imuDriverCallback, this);
  gps_driver_subscriber_ = node_.subscribe(
      kDriverGpsTopic, 20,
      &LocalizationInterfaceAdapter::gpsDriverCallback, this);
  gps_navpvt_driver_subscriber_ = node_.subscribe(
      kDriverGpsNavPvtTopic, 20,
      &LocalizationInterfaceAdapter::gpsNavPvtDriverCallback, this);
  imu_subscriber_ = node_.subscribe(
      imu_calibrated, 50, &LocalizationInterfaceAdapter::imuCallback, this);
  twist_subscriber_ = node_.subscribe(
      encoder_twist, 50, &LocalizationInterfaceAdapter::twistCallback, this);
  gps_pose_subscriber_ = node_.subscribe(
      gps_map_pose, 20, &LocalizationInterfaceAdapter::gpsPoseCallback, this);
  local_odometry_subscriber_ = node_.subscribe(
      kEkfLocalOdometryTopic, 50,
      &LocalizationInterfaceAdapter::localOdometryCallback, this);
  global_odometry_subscriber_ = node_.subscribe(
      kEkfGlobalOdometryTopic, 50,
      &LocalizationInterfaceAdapter::globalOdometryCallback, this);

  ROS_INFO_STREAM("LocalizationInterfaceAdapter 설정: IMU=" << imu_data
                  << " -> " << imu_normalized
                  << " -> " << imu_calibrated
                  << ", Twist=" << encoder_twist
                  << ", Local/Global=" << local_odometry << "/"
                  << global_odometry);
}

void LocalizationInterfaceAdapter::imuDriverCallback(
    const sensor_msgs::ImuConstPtr& message) {
  public_imu_publisher_.publish(message);
}

void LocalizationInterfaceAdapter::gpsDriverCallback(
    const sensor_msgs::NavSatFixConstPtr& message) {
  public_gps_publisher_.publish(message);
}

void LocalizationInterfaceAdapter::gpsNavPvtDriverCallback(
    const topic_tools::ShapeShifter::ConstPtr& message) {
  if (!public_gps_navpvt_publisher_) {
    public_gps_navpvt_publisher_ = message->advertise(
        node_, public_gps_navpvt_topic_, 10, false);
  }
  public_gps_navpvt_publisher_.publish(message);
}

void LocalizationInterfaceAdapter::imuCallback(
    const sensor_msgs::ImuConstPtr& message) {
  internal_imu_publisher_.publish(message);
}

void LocalizationInterfaceAdapter::twistCallback(
    const geometry_msgs::TwistWithCovarianceStampedConstPtr& message) {
  internal_twist_publisher_.publish(message);
}

void LocalizationInterfaceAdapter::gpsPoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& message) {
  internal_gps_pose_publisher_.publish(message);
}

void LocalizationInterfaceAdapter::localOdometryCallback(
    const nav_msgs::OdometryConstPtr& message) {
  public_local_odometry_publisher_.publish(message);
}

void LocalizationInterfaceAdapter::globalOdometryCallback(
    const nav_msgs::OdometryConstPtr& message) {
  public_global_odometry_publisher_.publish(message);
}

}  // namespace mando_localization
