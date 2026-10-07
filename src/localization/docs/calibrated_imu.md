# CalibratedIMU

`CalibratedIMU`는 `sensor_msgs/Imu`를 사용하는 위치 추정 공통 입력이다.
원본 → ImuNormalizer → CalibratedIMU → InterfaceAdapter → Local/Global EKF 순서다.
StatusManager도 보정 출구를 읽는다. 원본 relay, normalizer와 센서 시각 진단만 원본을 읽는다.

- 입력: `topics/imu_normalized`, `topics/gps_navpvt`, `topics/encoder_twist`, `topics/encoder_state`.
- 출력: `topics/imu_calibrated`, 기본 `/molit/localization/imu/calibrated`.
- 진단: `internal_topics/imu_calibration_status`, 기본
  `/mando_localization/internal/imu/calibration_status` (`DiagnosticArray`).

## 동작

1. 기본 `initialize_heading:=true`: 첫 유효 IMU와 장착 TF를 기다린 뒤 차량 yaw를
   RDDF 출발 방향에 맞추고 `RDDF_INITIALIZED`로 출력한다. GNSS를 기다리지 않는다.
   초기화 대기 중에는 보정 IMU를 발행하지 않는다. 초기화를 끈 경우에만
   `UNCALIBRATED` 상태에서 시작 방향 offset 없이 전달한다. 정지 제약은 별도 적용한다.
2. 이후 유효한 전진 직진 구간에서 `target_offset = wrap(GNSS_body_yaw - IMU_base_yaw)`를
   승인한다. RDDF 초기화는 GNSS one-shot 횟수를 소비하지 않는다.
3. `ALIGNING`: 승인 offset을 초당 최대 90°씩 적용한다. 실제 적용 속도는
   입력 간격과 `max_imu_gap_sec` 제한에 영향을 받는다. 직진 선별용 yaw rate 상한 3°/s와 구분한다.
4. `CALIBRATED`: 다음 유효 직진 구간까지 적용된 offset을 유지한다. 기본 반복 모드에서는
   새 구간이 모든 조건을 다시 통과하면 갱신한다. 정지·GPS 소실·clock_ready=false에도 원본으로 돌아가지 않는다.

출력은 `q_world_z(applied_offset) * q_stationary_corrected`다. 정지 제약과 GNSS 정합이
같은 자세 이력을 사용한다. Roll/pitch, Header의 stamp/frame, acceleration과 벡터의
공분산은 보존한다. gyro는 정지 제약 중 차량 Z 성분만 0으로 만든다. 장착 회전은
`base_link -> imu_link` TF로 차량 yaw를 계산할 때 한 번 사용하며 TF 자체는 수정하지 않는다.
입력 IMU가 없거나 잘못되면 이전 메시지를 새 시각으로 재발행하지 않는다.
`Header.seq`는 ROS 발행기가 자신의 발행 순번으로 다시 매기므로 원본 비교 키는 `stamp`를 사용한다.

Orientation 공분산의 yaw 대각에는 초기 배치 또는 GNSS 추정 불확실성, 아직 적용하지 않은 보정각²,
마지막 정합 이후 시간에 따른 잠정 불확실성을 더한다. 10° floor 및 0.1°/s 불확실성 증가는
운영용 잠정 모델이며 실측 정확도나 제조사 드리프트 사양이 아니다. 상관된 연속 GNSS 표본 수로
분산을 나누지 않는다. GPS 위치와 이동 방향도 같은 수신기에서 나오는 상관된 정보다.

## RDDF 시작 방향

[`initial_heading.yaml`](../config/initial_heading.yaml)은 현재 용인
`rddf/yongin_1_right.csv`의 index 0 방향을 담는다: `2.8181825146 rad`, 약 `161.47°`.
동쪽을 0°로 보는 ENU 값이며 RDDF 출발선과 나란한 방향이다. 차량을 이 방향으로
배치하고 실행하는 전제이며, 현재 위치나 출발 경로를 자동 판별하지 않는다.

첫 발행 전에 `offset = wrap(start_yaw - first_imu_body_yaw)`를 한 번 적용한다.
이후 이동 중 IMU 회전 변화는 반영하고, 정지 중에는 아래 엔코더 제약을 적용한다.
roll/pitch는 0으로 만들지 않는다. 같은 YAML의 `initial_state`를 Local·Global EKF에도 적용하여
엔코더가 IMU보다 먼저 도착해도 같은 yaw로 시작한다. 기존 x/y 시작값은 유지한다.

YAML anchor로 세 입력의 yaw 값을 공유한다. 출발 경로를 바꾸면 해당 RDDF의
`path_yaw_rad`와 `source`를 함께 갱신한다. RDDF 원본을 수정하거나 실행 시 경로를
맞춰 움직이지 않는다. `standard_deviation_deg: 10.0`은 초기 배치의 잠정 불확실성이다.

다른 시작 방향이나 중간 지점에서 재생할 때는 초기 설정 파일을 교체하거나
`initialize_heading:=false`로 RDDF 초기화를 끈다. 이 경우 기존 GNSS 정합은 유지된다.

```bash
localization
localization initialize_heading:=false
roslaunch mando_localization replay.launch bag:=/absolute/source.bag initialize_heading:=false
```

`initial_heading_config`와 `initialize_heading`은 bringup·수집·재생 진입점에서
세 입력에 함께 전달한다. 진단의 `initial_heading_source`, `initial_yaw_deg`,
`initialization_stamp`는 초기 방향의 출처를, `calibration_stamp`와 `correction_count`는
이후 GNSS 정합을 나타낸다. `RDDF_INITIALIZED`는 실측 방향이 아니라 시작 배치 가정이다.

## GNSS 보정 기준

기준은 [`imu_heading_calibration.yaml`](../config/imu_heading_calibration.yaml)에 모았다.
현재 저속 차량에서 시작할 잠정 기준이며 실차 검증에 따라 조정해야 한다.

| 항목 | 기본 조건 |
|---|---|
| 방향 | 첫 보정까지 전진 직진, 이후 기존 보정 방향과 30° 이내인 새 직진 구간에서 반복 정합 |
| GNSS | fixType=3, GNSS_FIX_OK, 위성 ≥6개 |
| 이동 | GPS ≥1m/s, encoder ≥0.5m/s |
| 정확도 추정치 | headAcc ≤10°, hAcc ≤2m, sAcc ≤0.5m/s, tAcc ≤50ms |
| 속도 일치 | 차이 ≤max(0.75m/s, GNSS 속도의 40%) |
| 연속성 | ≥3초·6표본·원본 GNSS 변위 3m, GNSS 간격 ≤0.5초 |
| 직진 | course 범위 ≤8°, offset 범위 ≤5°, IMU yaw rate ≤3°/s |
| 기울기 | 차량 roll/pitch 및 IMU pitch 절댓값 ≤15° |
| 교차 확인 | 위치 변위 방향과 course 차이 ≤15°, velN/E 방향과 course 차이 ≤5° |
| 시각 | 유효 GNSS UTC, clock_ready, age ≤1초, 미래 ≤500ms |
| IMU 정합 | 2초 이력에서 측정 시각 SLERP, bracket ≤100ms, 외삽 금지 |
| 엔코더 정합 | 측정 시각 이전 마지막 속도, 최대 age 300ms |
| 정렬 대기 | 미래 측정에 대응하는 IMU·엔코더를 최대 600ms 대기 (실시간/1배속) |

NAV-PVT는 Header가 없으므로 UTC와 `valid`/`flags2`를 검사한다. 북쪽 0°, 시계 방향인
`heading`을 `wrap(90° - heading × 1e-5)`로 ENU 변환한다. 수신 시각으로 측정 시각을 대체하거나
백을 통과시키려고 UTC에 상수를 더하지 않는다. IMU/encoder의 Header는 기존 PC 수신 시각 기반이므로
하드웨어 측정 동기화까지 검증된 것은 아니다. Global EKF 경로를 보정 기준으로 되먹이지 않는다.

현재 `Gear`는 방향 증거가 아니다. Arduino 참고 코드가 갱신하지 않는 필드다.
`forward_start`는 **첫 정합까지 전진하는 운용 조건**이다. 처음부터 후진할 수 있다면 실제 encoder
속도 부호를 검증한 뒤 `signed_encoder`를 사용한다. 그 모드는 음수 속도에서 GNSS 방향에 π를 더한다.
기본 `one_shot: false`에서는 첫 정렬 이후에도 새 직진 구간으로 반복 정합한다.
반복 후보 방향은 기존 보정 방향과 `max_repeat_heading_difference_deg: 30.0` 이내여야 한다.
반대 방향이나 큰 불일치는 `REPEAT_HEADING_DIFFERENCE_TOO_LARGE`로 보류하고,
직전 보정각 적용 중에는 `WAITING_FOR_PREVIOUS_ALIGNMENT`로 새 후보를 모으지 않는다.
이는 기존 정렬과의 일관성 검사이며, 검증되지 않은 encoder 부호를 전·후진 증거로 사용하지 않는다.
이미 잘못된 초기 정렬을 이 검사만으로 판별할 수는 없다. `one_shot: true`로 되돌리면 최초 정렬만 수행한다.

저속 course는 위치 잡음과 수신기 저속/정지 처리의 영향을 받으므로 정지한 GPS로 yaw를 구하지 않는다.
[u-blox 공식 Course Over Ground 설명](https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf)

## 엔코더 정지 yaw 제약

`imu_heading_calibration.yaml`의 `encoder_yaw_hold.enabled: true`가 기본이다.
`SerialFeedBack.encoder == 0`이고 `speed == 0`이며 alive가 갱신될 때,
마지막으로 출력한 차량 yaw를 유지한다. 첫 IMU부터 정지했다면 그 자세에서 시작한 뒤
RDDF 초기 offset을 적용한다. `±1`을 포함한 비영점 encoder는 이동으로 처리한다.

Header 없는 feedback은 PC 수신 시각으로 최대 500개를 보관한다. IMU stamp 이전의
마지막 표본만 사용하며 IMU/현재 ROS 시각 모두에서 나이가 0.30초 이하여야 한다.
중복 alive, NaN 속도, encoder=0/speed!=0 모순, 수신 누락·만료는 정지 증거로 쓰지 않는다.
정지 중 GNSS offset 적용도 미루며, 처음 움직인 IMU는 고정 yaw에서 이어 붙이고
그 다음 표본부터 회전 변화량을 반영한다. `/clock` 역행과 노드 재시작은 정지 이력도 초기화한다.

보정 IMU에서만 yaw를 고정하고 차량 Z축 gyro를 0으로 만든다. 기울어진 장착 TF에서는
차량 Z축을 센서 좌표로 변환하여 해당 성분만 제거한다. 원시/정규화 IMU는 수정하지 않는다.
진단의 `encoder_yaw_held`, `encoder_yaw_hold_reason`, `encoder_yaw_hold_age_sec`,
`stationary_yaw_offset_deg`, `stationary_held_samples`로 적용 여부를 확인한다.

이는 엔코더 0을 차량 정지로 보는 모델 제약이다. EKF 입력 yaw를 고정하며 Local/Global
Odometry나 TF를 나중에 덮어쓰지 않는다. 필터 수렴과 Global 절대 관측에 의한 작은 yaw
변화까지 수학적으로 차단하는 것은 아니다. 센서가 끊기면 마지막 0값으로 무한 고정하지 않는다.

## 범위와 실행

**원본 AHRS 자체와 이동 중 드리프트는 수정하지 않는다.** 정지 중에만 위 제약으로
yaw 변화를 제외하며, gyro bias 추정 필터를 추가한 것은 아니다.

IMU 보정은 현재 프로세스 세션에서 유지한다. 노드 재시작 또는 `/clock` 역행은 새 세션으로 보고
첫 유효 IMU에서 초기 방향을 다시 정한다. 중간 지점의 시각 역행은 새 차량 시작 방향과 같다는 뜻이
아니므로 임의 지점 재생에는 해당 방향의 설정 또는 초기화 비활성 옵션을 사용한다.
일시정지·정상 지연·GPS 불량은 초기화하지 않는다. 장착/전원 변경 후 과거 offset을 적용하지 않도록
추정한 offset을 파일이나 IMU EEPROM에 저장하지 않는다. 설정에 기록하는 RDDF 목표 yaw와 구분한다.

```bash
roslaunch mando_localization bringup.launch
rostopic echo /mando_localization/internal/imu/calibration_status
rostopic echo -n 1 /molit/localization/imu/calibrated
```

`bringup.launch`, `local_fusion.launch`, `map_data_collection.launch`의 `imu_heading_config`로 정책을 지정한다.
`start_calibrated_imu:=false`는 외부 보정 노드가 같은 출구를 제공할 때만 사용한다. normalizer→EKF 우회는 없다.
기존 localization `valid`는 센서 상태이며 yaw 정합 완료 증명이 아니다. 위 진단의 상태와
`yaw_offset_deg`, `applied_yaw_offset_deg`로 보정 여부를 확인한다.

```bash
source /opt/ros/noetic/setup.bash
cd ~/work/HL-FMA-2026-Localization
catkin_make -j4 -l4 -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
rostest mando_localization calibrated_imu.test
rostest mando_localization initial_heading.test
rostest mando_localization encoder_yaw_hold.test
catkin_make run_tests_mando_localization -j4 -l4
catkin_test_results build/test_results/mando_localization
```

합성 ROS 시험은 실제 normalizer/CalibratedIMU/adapter/Local·Global EKF를 실행한다.
GNSS 전달 지연 250ms, 초기 yaw 오차 70°, 원본 필드 보존과 GNSS 소실 후 유지를 검사한다.
`initial_heading.test`는 기본 RDDF 초기화, 엔코더 선도착, 늦은 장착 TF와 장착 yaw,
실제 30° 선회와 후속 GNSS 5° 정합을 두 EKF로 검사한다.
`encoder_yaw_hold.test`는 정지 중 합성 AHRS 80° drift와 gyro 입력을 차단하고,
재출발 회전·엔코더 timeout·원본 필드 보존을 확인한다.
실차 주행이나 물리 센서의 절대 정확도 검증을 대체하지 않는다.

## RViz에서 확인

하단의 `Raw / Calibrated / Local / Global yaw`는 각각의 표시 시점 표본에서 계산한다.
차량 박스와 화살표는 해당 Local/Global Odometry의 orientation을 그대로 사용한다.
Global 표시를 Calibrated IMU 값으로 덮어쓰거나 GNSS 경로 방향으로 회전하지 않는다.
`RDDF 초기 정렬 · GNSS 미보정`과 `GNSS 보정 적용 완료`를 구분하고,
실제 GNSS 보정은 `correction_count > 0` 및 `calibration_stamp`로 확인한다.
현재 `one_shot: false`이므로 조건을 통과한 새로운 직진 구간마다 보정 횟수가 증가한다.
보정 직후 후보 구간을 비우며, 매회 최소 시간·거리·표본 조건을 새로 충족해야 한다.
