#!/usr/bin/env bash
# 지도 제작용 실센서, RViz와 원본 ROS 토픽 기록을 한 명령으로 실행한다.

set -Eeo pipefail

readonly DEFAULT_WORKSPACE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
readonly WORKSPACE_ROOT="${MANDO_LOCALIZATION_WS:-${DEFAULT_WORKSPACE_ROOT}}"
readonly DEFAULT_RECORD_ROOT="${MANDO_RECORD_ROOT:-${WORKSPACE_ROOT}/rosbag/용인운전면허장 로스백}"
readonly GPS_DEVICE="${MANDO_GPS_DEVICE:-/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00}"
readonly ENCODER_DEVICE="${MANDO_ENCODER_DEVICE:-/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Uno_11254501101131313365-if00}"
readonly IMU_DEVICE="${MANDO_IMU_DEVICE:-/dev/imu}"
readonly LIDAR_DEVICE="${MANDO_LIDAR_DEVICE:-/dev/lidar}"
readonly CAPTURE_SCRIPT="${MANDO_CAPTURE_SCRIPT:-/home/stier/Mando/arduino/BROON_T870_Uno_Controller/logger/capture_and_diagnose.py}"
readonly CAPTURE_PORT="${MANDO_CAPTURE_PORT:-/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0}"
readonly CAPTURE_NAME="${MANDO_CAPTURE_NAME:-rc_test01}"
readonly CAPTURE_PYTHON="${MANDO_CAPTURE_PYTHON:-python3}"
readonly TOPIC_WAIT_TIMEOUT_SEC="${MANDO_RECORD_TOPIC_TIMEOUT_SEC:-30}"
readonly BAG_EXCLUDE_REGEX='^(/molit/localization/(path|markers)|/mando_localization/visualization/debug/scene)$'

fail() {
  echo "[localization-record] 오류: $*" >&2
  exit 1
}

usage() {
  printf '%s\n' \
    "사용법: localization-record [--output-root DIR] [--no-rviz] [--allow-missing-sensors]" \
    "" \
    "기본 동작:" \
    "  - RPLIDAR S2 + Encoder + Xsens IMU + u-blox GPS 실행" \
    "  - IMU·엔코더·GPS Localization/RViz 실행" \
    "  - 누적 Path/Marker를 제외한 활성 ROS 토픽을 LZ4, 2 GB 분할 bag으로 저장" \
    "  - CAN 로거 동시 수집 및 종료 후 자동 진단 (can_capture/)" \
    "  - 저장 위치: ${DEFAULT_RECORD_ROOT}/<timestamp>/" \
    "" \
    "환경변수로 장치 경로를 바꿀 수 있습니다:" \
    "  MANDO_LIDAR_DEVICE, MANDO_IMU_DEVICE, MANDO_GPS_DEVICE," \
    "  MANDO_ENCODER_DEVICE, MANDO_RECORD_ROOT, MANDO_LOCALIZATION_WS," \
    "  MANDO_RECORD_TOPIC_TIMEOUT_SEC" \
    "  MANDO_CAPTURE_PORT, MANDO_CAPTURE_NAME (기본 rc_test01), MANDO_CAPTURE_SCRIPT"
}

record_root="${DEFAULT_RECORD_ROOT}"
start_rviz=true
allow_missing_sensors=false

while (($#)); do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --output-root)
      (($# >= 2)) || fail "--output-root 뒤에 저장 경로가 필요합니다."
      record_root="$2"
      shift 2
      ;;
    --output-root=*)
      record_root="${1#--output-root=}"
      shift
      ;;
    --no-rviz)
      start_rviz=false
      shift
      ;;
    --allow-missing-sensors)
      allow_missing_sensors=true
      shift
      ;;
    *)
      fail "알 수 없는 인자입니다: $1"
      ;;
  esac
done

[[ -n "${record_root}" ]] || fail "저장 경로가 비어 있습니다."
[[ -f "${CAPTURE_SCRIPT}" ]] || fail "CAN capture 스크립트가 없습니다: ${CAPTURE_SCRIPT}"
[[ -r "${CAPTURE_PORT}" && -w "${CAPTURE_PORT}" ]] || fail "CAN 로거 포트 접근 불가: ${CAPTURE_PORT} (MANDO_CAPTURE_PORT로 지정)"
for sensor_port in "${GPS_DEVICE}" "${ENCODER_DEVICE}" "${IMU_DEVICE}" "${LIDAR_DEVICE}"; do
  if [[ -e "${sensor_port}" && "$(readlink -f -- "${CAPTURE_PORT}")" == "$(readlink -f -- "${sensor_port}")" ]]; then
    fail "CAN 로거 포트 ${CAPTURE_PORT}가 센서 ${sensor_port}와 같습니다. 별도 로거 포트를 MANDO_CAPTURE_PORT로 지정하세요."
  fi
done
"${CAPTURE_PYTHON}" -c 'import serial, cantools, ast, pathlib, sys; ast.parse(pathlib.Path(sys.argv[1]).read_text())' "${CAPTURE_SCRIPT}" || fail "CAN 로거 Python에 pyserial이 필요합니다."

[[ -f /opt/ros/noetic/setup.bash ]] || fail "ROS Noetic이 없습니다."
[[ -f "${WORKSPACE_ROOT}/devel/setup.bash" ]] || \
  fail "${WORKSPACE_ROOT}가 빌드되지 않았습니다. catkin_make를 먼저 실행하세요."

source /opt/ros/noetic/setup.bash
source "${WORKSPACE_ROOT}/devel/setup.bash"

for required_package in mando_localization rosbag rplidar_ros; do
  rospack find "${required_package}" >/dev/null 2>&1 || \
    fail "필수 ROS 패키지가 없습니다: ${required_package}"
done

start_lidar_driver=true
start_imu_driver=true
start_gps_driver=true
start_encoder_driver=true
missing_devices=()

check_device() {
  local label="$1"
  local path="$2"
  local flag_name="$3"
  if [[ -r "${path}" && -w "${path}" ]]; then
    printf '[localization-record] %s: %s -> %s\n' \
      "${label}" "${path}" "$(readlink -f -- "${path}")"
    return
  fi

  printf -v "${flag_name}" '%s' false
  missing_devices+=("${label}=${path}")
}

check_device "LiDAR" "${LIDAR_DEVICE}" start_lidar_driver
check_device "IMU" "${IMU_DEVICE}" start_imu_driver
check_device "GPS" "${GPS_DEVICE}" start_gps_driver
check_device "Encoder" "${ENCODER_DEVICE}" start_encoder_driver

if ((${#missing_devices[@]})); then
  if [[ "${allow_missing_sensors}" != true ]]; then
    printf '[localization-record] 연결되지 않은 필수 센서:\n' >&2
    printf '  - %s\n' "${missing_devices[@]}" >&2
    fail "모든 센서를 연결하거나, 불완전한 시험 기록만 허용할 때 --allow-missing-sensors를 사용하세요."
  fi
  printf '[localization-record] 경고: 다음 센서를 제외하고 기록합니다:\n' >&2
  printf '  - %s\n' "${missing_devices[@]}" >&2
fi

if [[ "${start_imu_driver}" == true ]]; then
  rospack find xsens_mti_driver >/dev/null 2>&1 || fail "xsens_mti_driver가 없습니다."
fi
if [[ "${start_gps_driver}" == true ]]; then
  rospack find ublox_gps >/dev/null 2>&1 || fail "ublox_gps가 없습니다."
fi
if [[ "${start_encoder_driver}" == true ]]; then
  rospack find rosserial_python >/dev/null 2>&1 || fail "rosserial_python이 없습니다."
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

session_stamp="$(date +%Y%m%d_%H%M%S)"
session_dir="${record_root%/}/${session_stamp}"
if [[ -e "${session_dir}" ]]; then
  session_dir="${session_dir}_$$"
fi
mkdir -p -- "${session_dir}/config_snapshot"
session_dir="$(readlink -m -- "${session_dir}")"
bag_prefix="${session_dir}/mapping"

package_path="$(rospack find mando_localization)"
cp -a -- "${package_path}/config/." "${session_dir}/config_snapshot/"
cp -a -- "${package_path}/launch/map_data_collection.launch" "${session_dir}/"

{
  printf 'mode=map_data_collection\n'
  printf 'started_at=%s\n' "$(date --iso-8601=seconds)"
  printf 'workspace=%s\n' "${WORKSPACE_ROOT}"
  printf 'package=%s\n' "${package_path}"
  printf 'use_sim_time=false\n'
  printf 'record_all_topics=true\n'
  printf 'bag_exclude_regex=%s\n' "${BAG_EXCLUDE_REGEX}"
  printf 'bag_compression=lz4\n'
  printf 'bag_split_size_mb=2048\n'
  printf 'bag_min_space=10G\n'
  printf 'lidar_device=%s\n' "${LIDAR_DEVICE}"
  printf 'imu_device=%s\n' "${IMU_DEVICE}"
  printf 'gps_device=%s\n' "${GPS_DEVICE}"
  printf 'encoder_device=%s\n' "${ENCODER_DEVICE}"
} >"${session_dir}/session_info.txt"

# GNSS UTC를 PC 시간에 결합하기 전에 동기화 근거를 세션과 함께 남긴다.
# 실패 시 CAN/센서 프로세스를 아직 시작하지 않은 상태에서 종료한다.
if [[ "${start_gps_driver}" == true ]]; then
  rosrun mando_localization check_time_sync.py \
    --config "${package_path}/config/time_sync.yaml" \
    --gps-config "${package_path}/config/gps_driver.yaml" \
    >"${session_dir}/time_sync_preflight.json" || \
    fail "PC 시계 동기화 검사 실패: ${session_dir}/time_sync_preflight.json"
fi

echo "[localization-record] 저장 경로: ${session_dir}"
echo "[localization-record] 누적 Path·Marker 제외 기록: ON"
echo "[localization-record] RViz: Local 초록 / GPS 보정 Global 빨강 / 차량 1.35 x 0.85 m"
echo "[localization-record] 필수 토픽을 확인하는 중입니다. 준비 완료 전에는 출발하지 마세요."

set +e
# 별도 세션으로 분리해 터미널 Ctrl+C와 부모의 전달 신호가 중복되지 않게 한다.
# SIGINT는 stop_launch에서 한 번 전달하고 CSV/진단 저장이 끝날 때까지 기다린다.
"${CAPTURE_PYTHON}" -u -c 'import os, runpy, signal, sys; os.setsid(); signal.signal(signal.SIGINT, signal.default_int_handler); sys.argv=sys.argv[1:]; runpy.run_path(sys.argv[0], run_name="__main__")'   "${CAPTURE_SCRIPT}" --port "${CAPTURE_PORT}" --name "${CAPTURE_NAME}"   --output-root "${session_dir}/can_capture" >"${session_dir}/can_capture.log" 2>&1 &
capture_pid=$!
printf 'capture_port=%s\ncapture_name=%s\n' "${CAPTURE_PORT}" "${CAPTURE_NAME}" >>"${session_dir}/session_info.txt"
echo "[localization-record] CAN 기록: ${CAPTURE_PORT}, name=${CAPTURE_NAME}"
roslaunch mando_localization map_data_collection.launch \
  start_lidar_driver:="${start_lidar_driver}" \
  start_imu_driver:="${start_imu_driver}" \
  start_gps_driver:="${start_gps_driver}" \
  start_encoder_driver:="${start_encoder_driver}" \
  start_rviz:="${start_rviz}" \
  start_recording:=true \
  lidar_serial_port:="${LIDAR_DEVICE}" \
  bag_exclude_regex:="${BAG_EXCLUDE_REGEX}" \
  bag_prefix:="${bag_prefix}" &
launch_pid=$!
capture_stop_requested=false

stop_launch() {
  if [[ "${capture_stop_requested}" == false ]] && kill -0 "${capture_pid}" 2>/dev/null; then
    capture_stop_requested=true
    kill -INT "${capture_pid}" 2>/dev/null || true
  fi
  if kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT "${launch_pid}" 2>/dev/null || true
  fi
}
trap stop_launch INT TERM

master_ready=false
for _ in $(seq 1 80); do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    break
  fi
  if rosnode list >/dev/null 2>&1; then
    master_ready=true
    break
  fi
  sleep 0.25
done

required_live_topics=(/tf_static /diagnostics)
if [[ "${start_lidar_driver}" == true ]]; then
  required_live_topics+=(/molit/sensors/lidar/scan)
fi
if [[ "${start_imu_driver}" == true ]]; then
  required_live_topics+=(/molit/sensors/imu/data)
fi
if [[ "${start_gps_driver}" == true ]]; then
  required_live_topics+=(/molit/sensors/gps/fix /molit/sensors/gps/navpvt)
fi
if [[ "${start_encoder_driver}" == true ]]; then
  required_live_topics+=(/erp42_serial/feedback)
fi
if [[ "${start_imu_driver}" == true && "${start_encoder_driver}" == true ]]; then
  required_live_topics+=(/tf)
fi

preflight_missing_topics=()
if [[ "${master_ready}" != true ]]; then
  preflight_missing_topics+=(ROS_MASTER)
else
  preflight_dir="${session_dir}/.preflight"
  mkdir -p -- "${preflight_dir}"
  topic_wait_pids=()
  topic_wait_names=()
  for topic in "${required_live_topics[@]}"; do
    topic_key="${topic#/}"
    topic_key="${topic_key//\//_}"
    timeout --signal=INT "${TOPIC_WAIT_TIMEOUT_SEC}" \
      rostopic echo -n 1 "${topic}" \
      >"${preflight_dir}/${topic_key}.out" \
      2>"${preflight_dir}/${topic_key}.err" &
    topic_wait_pids+=("$!")
    topic_wait_names+=("${topic}")
  done

  for index in "${!topic_wait_pids[@]}"; do
    if ! wait "${topic_wait_pids[index]}"; then
      preflight_missing_topics+=("${topic_wait_names[index]}")
    fi
  done
  find -P "${preflight_dir}" -depth -delete
fi

if ! kill -0 "${capture_pid}" 2>/dev/null; then
  preflight_missing_topics+=(CAN_CAPTURE_PROCESS)
fi
if ((${#preflight_missing_topics[@]})); then
  {
    printf 'preflight=FAIL\n'
    printf 'preflight_missing_topic=%s\n' "${preflight_missing_topics[@]}"
  } >>"${session_dir}/session_info.txt"
  printf '[localization-record] 오류: 데이터가 들어오지 않는 필수 토픽:\n' >&2
  printf '  - %s\n' "${preflight_missing_topics[@]}" >&2
  echo "[localization-record] 출발 전 검사 실패로 기록을 종료합니다." >&2
  stop_launch
  wait "${launch_pid}" 2>/dev/null
  launch_status=1
else
  printf 'preflight=PASS\n' >>"${session_dir}/session_info.txt"
  echo "[localization-record] 수집 준비 완료: 이제 주행을 시작하세요."
  echo "[localization-record] 정상 종료: Ctrl+C 한 번"
  wait -n "${launch_pid}" "${capture_pid}"
  launch_status=$?
  stop_launch
  if kill -0 "${launch_pid}" 2>/dev/null; then
    wait "${launch_pid}"
    launch_status=$?
  fi
fi
stop_launch
wait "${capture_pid}"
capture_status=$?
printf 'capture_exit_code=%s\n' "${capture_status}" >>"${session_dir}/session_info.txt"
if [[ "${capture_status}" != 0 ]]; then
  echo "[localization-record] CAN 기록/진단 실패: ${session_dir}/can_capture.log" >&2
  launch_status=1
fi
trap - INT TERM
set -e

shopt -s nullglob
bag_files=("${session_dir}"/*.bag)
active_files=("${session_dir}"/*.active)
shopt -u nullglob

if ((${#bag_files[@]})); then
  : >"${session_dir}/bag_info.txt"
  for bag_file in "${bag_files[@]}"; do
    rosbag info "${bag_file}" >>"${session_dir}/bag_info.txt" 2>&1 || true
  done

  expected_topics=(
    /molit/sensors/lidar/scan
    /molit/sensors/imu/data
    /molit/sensors/gps/fix
    /molit/sensors/gps/navpvt
    /erp42_serial/feedback
    /tf
    /tf_static
  )
  missing_topics=()
  for topic in "${expected_topics[@]}"; do
    if ! grep -Fq -- "${topic}" "${session_dir}/bag_info.txt"; then
      missing_topics+=("${topic}")
    fi
  done

  if ((${#missing_topics[@]})); then
    {
      printf 'result=INCOMPLETE\n'
      printf 'missing_topic=%s\n' "${missing_topics[@]}"
    } >"${session_dir}/record_validation.txt"
    printf '[localization-record] 경고: bag에 없는 필수 토픽:\n' >&2
    printf '  - %s\n' "${missing_topics[@]}" >&2
  else
    printf 'result=PASS\n' >"${session_dir}/record_validation.txt"
    echo "[localization-record] 필수 지도 수집 토픽: 모두 기록됨"
  fi
  echo "[localization-record] 저장 완료: ${#bag_files[@]}개 bag"
  echo "[localization-record] 요약: ${session_dir}/bag_info.txt"
else
  echo "[localization-record] 경고: 완성된 .bag 파일이 없습니다." >&2
fi

if ((${#active_files[@]})); then
  printf '[localization-record] 경고: 비정상 종료된 .active 파일이 남았습니다:\n' >&2
  printf '  - %s\n' "${active_files[@]}" >&2
fi

{
  printf 'finished_at=%s\n' "$(date --iso-8601=seconds)"
  printf 'roslaunch_exit_status=%s\n' "${launch_status}"
} >>"${session_dir}/session_info.txt"

exit "${launch_status}"
