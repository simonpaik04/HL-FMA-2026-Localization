#include "transform_configuration.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <cmath>
#include <functional>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace mando_localization {
namespace {

template <typename T>
T requiredMember(const XmlRpc::XmlRpcValue& value, const std::string& name);

template <>
std::string requiredMember<std::string>(const XmlRpc::XmlRpcValue& value,
                                        const std::string& name) {
  if (!value.hasMember(name) || value[name].getType() != XmlRpc::XmlRpcValue::TypeString) {
    throw std::runtime_error("TF 항목의 문자열 필드가 없거나 타입이 다릅니다: " + name);
  }
  return static_cast<std::string>(value[name]);
}

template <>
bool requiredMember<bool>(const XmlRpc::XmlRpcValue& value, const std::string& name) {
  if (!value.hasMember(name) || value[name].getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
    throw std::runtime_error("TF 항목의 bool 필드가 없거나 타입이 다릅니다: " + name);
  }
  return static_cast<bool>(value[name]);
}

double numericValue(const XmlRpc::XmlRpcValue& value) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  throw std::runtime_error("TF translation/rotation 값은 숫자여야 합니다.");
}

void readTriple(const XmlRpc::XmlRpcValue& entry, const std::string& name, double* first,
                double* second, double* third) {
  if (!entry.hasMember(name) || entry[name].getType() != XmlRpc::XmlRpcValue::TypeArray ||
      entry[name].size() != 3) {
    throw std::runtime_error("TF 필드는 길이 3 배열이어야 합니다: " + name);
  }
  *first = numericValue(entry[name][0]);
  *second = numericValue(entry[name][1]);
  *third = numericValue(entry[name][2]);
}

bool validFrame(const std::string& frame) {
  return !frame.empty() && frame.front() != '/' && frame.find(' ') == std::string::npos;
}

}  // namespace

std::vector<StaticTransformSpec> TransformConfiguration::load(const ros::NodeHandle& node,
                                                              const std::string& parameter_name) {
  XmlRpc::XmlRpcValue entries;
  if (!node.getParam(parameter_name, entries)) {
    throw std::runtime_error("필수 TF 파라미터가 없습니다: " +
                             node.resolveName(parameter_name));
  }
  if (entries.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw std::runtime_error("static_transforms는 배열이어야 합니다.");
  }

  std::vector<StaticTransformSpec> result;
  result.reserve(entries.size());
  for (int index = 0; index < entries.size(); ++index) {
    const XmlRpc::XmlRpcValue& entry = entries[index];
    if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      throw std::runtime_error("각 static transform은 YAML map이어야 합니다.");
    }
    StaticTransformSpec transform;
    transform.parent_frame = requiredMember<std::string>(entry, "parent_frame");
    transform.child_frame = requiredMember<std::string>(entry, "child_frame");
    readTriple(entry, "translation_m", &transform.x_m, &transform.y_m, &transform.z_m);
    readTriple(entry, "rotation_rpy_deg", &transform.roll_deg, &transform.pitch_deg,
               &transform.yaw_deg);
    transform.enabled = requiredMember<bool>(entry, "enabled");
    transform.calibration_state =
        requiredMember<std::string>(entry, "calibration_state");
    transform.measured_at = requiredMember<std::string>(entry, "measured_at");
    transform.source = requiredMember<std::string>(entry, "source");
    result.push_back(transform);
  }
  validate(result);
  return result;
}

std::vector<DynamicTransformSpec> TransformConfiguration::loadDynamic(
    const ros::NodeHandle& node, const std::string& parameter_name) {
  XmlRpc::XmlRpcValue entries;
  if (!node.getParam(parameter_name, entries) ||
      entries.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw std::runtime_error("dynamic_transforms는 필수 배열이어야 합니다.");
  }
  std::vector<DynamicTransformSpec> result;
  result.reserve(entries.size());
  for (int index = 0; index < entries.size(); ++index) {
    const XmlRpc::XmlRpcValue& entry = entries[index];
    if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      throw std::runtime_error("각 dynamic transform은 YAML map이어야 합니다.");
    }
    DynamicTransformSpec transform;
    transform.parent_frame = requiredMember<std::string>(entry, "parent_frame");
    transform.child_frame = requiredMember<std::string>(entry, "child_frame");
    transform.owner_node = requiredMember<std::string>(entry, "owner_node");
    transform.source = requiredMember<std::string>(entry, "source");
    if (!validFrame(transform.parent_frame) ||
        !validFrame(transform.child_frame) ||
        transform.parent_frame == transform.child_frame ||
        transform.owner_node.empty() || transform.source.empty()) {
      throw std::runtime_error("동적 TF 선언이 유효하지 않습니다.");
    }
    result.push_back(transform);
  }
  return result;
}

void TransformConfiguration::validate(const std::vector<StaticTransformSpec>& transforms) {
  std::set<std::string> child_frames;
  std::unordered_map<std::string, std::string> parent_by_child;
  const std::set<std::string> states = {"unmeasured", "measured", "verified"};

  for (const StaticTransformSpec& transform : transforms) {
    if (!validFrame(transform.parent_frame) || !validFrame(transform.child_frame) ||
        transform.parent_frame == transform.child_frame) {
      throw std::runtime_error("유효하지 않은 TF frame: " + transform.parent_frame + " -> " +
                               transform.child_frame);
    }
    if (!child_frames.insert(transform.child_frame).second) {
      throw std::runtime_error("중복 TF child frame: " + transform.child_frame);
    }
    if (states.count(transform.calibration_state) == 0U) {
      throw std::runtime_error("알 수 없는 calibration_state: " +
                               transform.calibration_state);
    }
    const double values[] = {transform.x_m,       transform.y_m,      transform.z_m,
                             transform.roll_deg, transform.pitch_deg, transform.yaw_deg};
    for (const double value : values) {
      if (!std::isfinite(value)) {
        throw std::runtime_error("TF에 finite가 아닌 값이 있습니다: " +
                                 transform.child_frame);
      }
    }
    parent_by_child[transform.child_frame] = transform.parent_frame;
  }

  for (const StaticTransformSpec& transform : transforms) {
    std::set<std::string> visited;
    std::string current = transform.child_frame;
    while (parent_by_child.count(current) > 0U) {
      if (!visited.insert(current).second) {
        throw std::runtime_error("TF 순환 구조가 발견되었습니다: " + current);
      }
      current = parent_by_child.at(current);
    }
  }
}

void TransformConfiguration::validateTree(
    const std::vector<StaticTransformSpec>& static_transforms,
    const std::vector<DynamicTransformSpec>& dynamic_transforms) {
  std::vector<StaticTransformSpec> combined = static_transforms;
  for (const DynamicTransformSpec& dynamic : dynamic_transforms) {
    StaticTransformSpec placeholder;
    placeholder.parent_frame = dynamic.parent_frame;
    placeholder.child_frame = dynamic.child_frame;
    placeholder.calibration_state = "verified";
    combined.push_back(placeholder);
  }
  validate(combined);
}

bool TransformConfiguration::shouldPublish(const StaticTransformSpec& transform) {
  return transform.enabled &&
         (transform.calibration_state == "measured" ||
          transform.calibration_state == "verified");
}

geometry_msgs::TransformStamped TransformConfiguration::toMessage(
    const StaticTransformSpec& transform, const ros::Time& stamp) {
  constexpr double kDegreeToRadian = 3.14159265358979323846 / 180.0;
  geometry_msgs::TransformStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = transform.parent_frame;
  message.child_frame_id = transform.child_frame;
  message.transform.translation.x = transform.x_m;
  message.transform.translation.y = transform.y_m;
  message.transform.translation.z = transform.z_m;
  tf2::Quaternion quaternion;
  quaternion.setRPY(transform.roll_deg * kDegreeToRadian,
                    transform.pitch_deg * kDegreeToRadian,
                    transform.yaw_deg * kDegreeToRadian);
  quaternion.normalize();
  message.transform.rotation.x = quaternion.x();
  message.transform.rotation.y = quaternion.y();
  message.transform.rotation.z = quaternion.z();
  message.transform.rotation.w = quaternion.w();
  return message;
}

}  // namespace mando_localization
