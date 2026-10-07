# 센서 시각과 지연 GPS 처리

GPS는 **GNSS UTC 측정 시각**으로 융합하고 PC 수신 시각을 별도로 기록한다.
PC 시계 상태를 검사한 뒤 GPS 측정 시각의 Local Odometry를 보간하고,
Global EKF가 과거 측정을 반영해 현재 시각까지 예측한다.
IMU 내부 Heading 오류나 엔코더 물리 측정 시각을 보정하는 변경은 아니다.

추가된 [CalibratedIMU](calibrated_imu.md)도 같은 원칙으로 NAV-PVT UTC와 IMU 이력을 정합한다.
원본 시각 진단은 raw IMU를 유지하고, 실제 Local/Global EKF는 보정된 공통 IMU 출구를 사용한다.

## 1. PC 시계 검사

`systemd-timesyncd` 또는 `chrony`의 호스트 동기화 상태를 검사한다. 검사 프로그램은 시계나 서비스를 변경하지 않는다.
`time_sync.yaml`의 잠정 기준은 NTP 보고 오프셋 절댓값 50ms 이하,
root distance 50ms 이하, 최신 표본 3600초 이내다. 실제 허용 위치 오차와
주행 속도를 기준으로 별도 조정해야 하며 이 기준이 측량 정확도를 보장하지 않는다.

```bash
source /opt/ros/noetic/setup.bash
source ~/work/HL-FMA-2026-Localization/devel/setup.bash
rosrun mando_localization check_time_sync.py \
  --config ~/work/HL-FMA-2026-Localization/src/mando_localization/config/time_sync.yaml
```

`sensors.launch`는 GPS 프로세스를 실행하기 전에 같은 검사를 실행한다.
검사 실패 시 GPS 드라이버를 실행하지 않는다. `localization-record`는 검사 결과를
세션의 `time_sync_preflight.json`에도 저장한다.
운행 중 monitor가 5초마다 호스트를 검사하고 1Hz `clock_ready` heartbeat를 보낸다.
검사 실패·검사 지연·heartbeat 누락은 GPS 및 reanchor 승인을 차단한다.
GPS gate의 heartbeat 한계는 3초이며 경과 시간은 단조 증가 시계로 잰다.

`use_sim_time=true`인 재생은 `REPLAY_UNVERIFIED`로 명시한다. 이 경우의 readiness는
실장치 시계 동기화 증명이 아니다. 실제 GPS 드라이버는 rosbag 재생 시 끈다.
PPS 장치가 있다는 사실만으로 PPS 동기화를 성공으로 표시하지 않는다.

## 2. 측정 시각과 수신 시각 보존

운영 `gps_driver.yaml`은 `use_ros_time: false`, `require_valid_utc: true`다.
UTC 유효 비트·달력·나노초 범위를 통과한 NAV-PVT의 UTC를 NavSatFix와 속도
헤더에 사용한다. UTC가 유효하지 않으면 fix/velocity를 발행하지 않으며,
원본 NAV-PVT와 시각 진단은 계속 발행한다. 서로 다른 시계로 자동 전환하지 않는다.

명시적인 수신 시각 시험은 별도 GPS 설정 파일에서 `use_ros_time: true`로 선택할
수 있다. 이 방식은 GNSS/USB 전달 지연을 보정하지 않는다. 옵션 변경 후에는
드라이버를 재시작해야 한다.

| 내부 토픽 | 메시지 | 내용 |
|---|---|---|
| `/mando_localization/internal/driver/gps_timing` | `diagnostic_msgs/DiagnosticArray` | NAV-PVT UTC ns, callback 수신 ns, 선택한 헤더 ns, 유효성, 시각 출처, tAcc |
| `/mando_localization/internal/timing/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 호스트 동기화, GPS/IMU/encoder 시각 분포와 누락·역행·중복 |
| `/mando_localization/internal/timing/clock_ready` | `std_msgs/Bool` | 신선한 호스트 시계 검사 결과; GPS gate 내부 입력 |

monitor는 30초·최대 6000표본의 제한된 이력에서 p50/p95/p99/최댓값을 계산한다.
`receipt_minus_utc`에는 **PC 시계 오차와 전송·callback 지연이 함께 포함**된다.
NTP 상태와 함께 해석해야 하며 수신기 `tAcc`는 PC 시계 정확도가 아니다.
IMU와 encoder 헤더는 드라이버/adapter 수신 시각이므로 작은 헤더 차이가
장치 내부 지연이 작다는 증거는 아니다. 원래 측정 시각을 얻으려면 센서 시각
매핑 또는 하드웨어 동기화가 별도로 필요하다.

## 3. GPS 측정 시각의 Local Odometry

GPS gate는 2초·최대 200개 Local Odometry를 저장한다. GPS 시각을 둘러싼 두
표본의 간격이 0.10초 이내일 때 위치·자세·공분산을 보간한다. quaternion은
최단 회전으로 보간하며 공분산은 PSD를 유지하는 가중합을 사용한다.
외삽과 큰 데이터 공백을 가로지르는 보간은 하지 않는다.

GPS가 바로 뒤의 Local 표본보다 먼저 도착하면 최대 0.10초 동안 기다린다.
대기열은 최대 20개이며 대기 시간에는 단조 증가 시계를 쓴다.
최신 Local 수신 신선도 검사는 별도로 유지한다.

레버암 보정, innovation 검사, prediction anchor 및 reanchor의 Local 위치는
모두 GPS 측정 시각에 맞춘다. reanchor는 명령과 동일 timestamp의 후보 이력에
대응하는 Local pose를 사용한다. 중복·순서 역전 GPS는 앵커를 변경하지 않고
폐기한다. ROS 시각이 뒤로 이동하면 이력·대기열·앵커·복구 카운터를 초기화한다.

기존 `lever_arm.max_yaw_stamp_skew_sec` 키는 호환성을 위해 유지하지만 의미는
**GPS를 둘러싼 두 Local 표본 사이의 최대 간격**이다. 최신 Local과 과거 GPS의
시간차 제한으로 사용하지 않는다. GPS 최대 나이 1초와 미래 허용 0.10초는 유지한다.

## 4. Global EKF 지연 재처리

`ekf_global.yaml`:

```yaml
smooth_lagged_data: true
history_length: 2.0
predict_to_current_time: true
permit_corrected_publication: false
```

GPS가 늦게 도착하면 필터 이력의 해당 측정 시점부터 재처리하고 현재 상태를
예측한다. 이미 발행한 과거 시각의 출력/TF를 다시 발행하지 않는다.
Local EKF 입력 계약과 `map -> odom -> base_link` 소유 관계는 유지한다.
GPS 이력 한계와 EKF 이력 길이는 함께 설정해야 한다.

두 EKF의 `transform_timeout`은 0으로 두어 30Hz 주기보다 긴 TF 대기를 피한다.
Local도 `predict_to_current_time: true`로 현재 필터 시각의 상태/TF를 발행한다.
이는 TF의 timestamp에 임의 오프셋을 더하는 설정이 아니다.

예를 들어 보정되지 않은 시간오차 0.10초는 5m/s에서 약 0.5m의 위치 차이를 만든다.
지연 보정 이력이 충분하다면 수신 지연 자체보다 남아 있는 측정 시각 오차가
중요하므로, 단순히 timeout만 늘리지 않는다.

## 회귀 검사와 실장치 확인

`delayed_gps_alignment.test`는 지연·대기·중복·공백·시각 역행을,
`gps_clock_ready.test`는 heartbeat 누락·false·stale 차단을 검사한다.
`delayed_gps_ekf.test`는 300ms 지연 GPS를 실제 Local/GPS gate/Coordinator/Global
노드에 넣고 측정 시각 유지, 회전 레버암, 최종 출력 위치를 확인한다.

```bash
cd ~/work/HL-FMA-2026-Localization
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2
catkin_make -j4 -l4 run_tests_mando_localization
# 수정 ublox_gps 소스를 별도로 빌드한 workspace에서만:
# catkin_make -j2 run_tests_ublox_gps
catkin_test_results build/test_results
```

GPS·IMU 연결 후 원본 UTC/수신 시각 분포와 승인 GPS/최종 Odometry를 함께
확인해야 한다. 기존 bag의 PC–GPS 시계 차이는 기록마다 변하므로 일정한 0.35초를
일괄 빼지 않는다. 원본 bag을 수정하지 않고 시각 매핑 가정이 명시된 복사본으로
재생한다. 합성 지연 시험은 실제 장치 지연이나 IMU Heading 성능 검증을 대신하지 않는다.

근거: [robot_localization Noetic 문서](https://github.com/cra-ros-pkg/robot_localization/blob/noetic-devel/doc/state_estimation_nodes.rst),
[chrony 공식 예제](https://chrony-project.org/examples.html).
