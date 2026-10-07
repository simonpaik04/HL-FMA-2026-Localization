#!/usr/bin/env bash
# 지도 수집과 같은 실센서/Localization/RViz 구성을 rosbag 기록 없이 실행한다.

set -Eeo pipefail

readonly DEFAULT_WORKSPACE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
readonly WORKSPACE_ROOT="${MANDO_LOCALIZATION_WS:-${DEFAULT_WORKSPACE_ROOT}}"
readonly GPS_DEVICE="${MANDO_GPS_DEVICE:-/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00}"
readonly ENCODER_DEVICE="${MANDO_ENCODER_DEVICE:-/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_11254501101131313365-if00}"
readonly IMU_DEVICE="${MANDO_IMU_DEVICE:-/dev/imu}"
readonly LIDAR_DEVICE="${MANDO_LIDAR_DEVICE:-/dev/lidar}"

fail() {
  echo "[localization] 오류: $*" >&2
  exit 1
}

[[ -f /opt/ros/noetic/setup.bash ]] || fail "ROS Noetic이 없습니다."
[[ -f "${WORKSPACE_ROOT}/devel/setup.bash" ]] || \
  fail "${WORKSPACE_ROOT}가 빌드되지 않았습니다. catkin_make를 먼저 실행하세요."

source /opt/ros/noetic/setup.bash
source "${WORKSPACE_ROOT}/devel/setup.bash"

GPS_ARG=false
if [[ -r "${GPS_DEVICE}" && -w "${GPS_DEVICE}" ]]; then
  GPS_ARG=true
  rospack find ublox_gps >/dev/null 2>&1 || \
    fail "ublox_gps가 작업공간에 없습니다. 작업공간을 다시 빌드하세요."
else
  echo "[localization] GPS: 입력 없음 (${GPS_DEVICE})"
fi

ENCODER_ARG=false
if [[ -r "${ENCODER_DEVICE}" && -w "${ENCODER_DEVICE}" ]]; then
  ENCODER_ARG=true
else
  echo "[localization] Encoder: 입력 없음 (${ENCODER_DEVICE})"
fi

IMU_ARG=false
if [[ -r "${IMU_DEVICE}" && -w "${IMU_DEVICE}" ]]; then
  IMU_ARG=true
  rospack find xsens_mti_driver >/dev/null 2>&1 || \
    fail "xsens_mti_driver가 작업공간에 없습니다."
else
  echo "[localization] IMU: 입력 없음 (${IMU_DEVICE})"
fi

LIDAR_ARG=false
if [[ -r "${LIDAR_DEVICE}" && -w "${LIDAR_DEVICE}" ]]; then
  LIDAR_ARG=true
  rospack find rplidar_ros >/dev/null 2>&1 || \
    fail "rplidar_ros가 작업공간에 없습니다."
else
  echo "[localization] LiDAR: 입력 없음 (${LIDAR_DEVICE})"
fi

if [[ "${ENCODER_ARG}" == true ]]; then
  rospack find rosserial_python >/dev/null 2>&1 || \
    fail "rosserial_python이 작업공간에 없습니다."
fi

if running_nodes="$(rosnode list 2>/dev/null)"; then
  for node_name in \
    /mando_rplidar_s2 \
    /mando_encoder_serial \
    /xsens_mti_node \
    /ublox_gps_node \
    /mapping_rosbag_recorder \
    /localization_interface_adapter \
    /imu_encoder_local_ekf \
    /odometry_gps_global_ekf \
    /mando_localization_rviz; do
    if grep -Fxq "${node_name}" <<<"${running_nodes}"; then
      fail "이미 실행 중인 노드가 있습니다: ${node_name}"
    fi
  done
fi

echo "[localization] Mando 실센서 + Localization + RViz 시작"
echo "[localization] rosbag record: OFF / 종료: Ctrl+C"
echo "[localization] 공통 RViz: Local 초록 / Global 빨강 / GPS 보라 / RDDF 파랑 / ±10초 화면 탐색"
echo "[localization] GPS로 RDDF 시작 위치 선택 / GPS가 없으면 RViz 시작 위치 선택 → 미리보기 → 클릭"
exec roslaunch mando_localization map_data_collection.launch \
  start_encoder_driver:="${ENCODER_ARG}" \
  start_imu_driver:="${IMU_ARG}" \
  start_gps_driver:="${GPS_ARG}" \
  start_lidar_driver:="${LIDAR_ARG}" \
  start_rviz:=true \
  lidar_serial_port:="${LIDAR_DEVICE}" \
  "$@" \
  start_recording:=false
