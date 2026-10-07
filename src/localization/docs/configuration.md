# 설정 가이드

설정의 원본은 `config/` YAML입니다. 파일을 변경한 뒤 해당 노드를 재시작해야 합니다.
문서에 모든 키를 복제하지 않고 파일별 책임과 함께 조정해야 하는 항목을 정리합니다.

## 설정 파일

| 파일 | 책임 |
|---|---|
| [localization_interfaces.yaml](../config/localization_interfaces.yaml) | 공개·내부 토픽, 서비스, frame과 기본 노드 이름 |
| [encoder_driver.yaml](../config/encoder_driver.yaml) | rosserial 장치와 baud |
| [encoder_calibration.yaml](../config/encoder_calibration.yaml) | 속도 변환, alive·범위 검사, vx·vy 공분산 |
| [imu_driver.yaml](../config/imu_driver.yaml) | Xsens 드라이버, IMU 정규화·공분산 |
| [imu_heading_calibration.yaml](../config/imu_heading_calibration.yaml) | GNSS 직진 구간의 최초 yaw 보정 |
| [rddf_initialization.yaml](../config/rddf_initialization.yaml) | RDDF datum, 시작 위치 선택과 적용 확인 |
| [rddf_tracking.yaml](../config/rddf_tracking.yaml) | 현재 경로 후보 추적 |
| [initial_heading.yaml](../config/initial_heading.yaml) | RDDF 출발 yaw, 보정 IMU 및 두 EKF의 공통 초기값 |
| [gps_driver.yaml](../config/gps_driver.yaml) | u-blox 장치, UTC 헤더, 출력 설정 |
| [time_sync.yaml](../config/time_sync.yaml) | 호스트 시계 검사와 센서 시각 진단 |
| [gps_reference.yaml](../config/gps_reference.yaml) | datum·레버암, 시각 이력, GPS 품질 gate |
| [lidar_driver.yaml](../config/lidar_driver.yaml) | RPLIDAR 장치와 원시 scan |
| [ekf_local.yaml](../config/ekf_local.yaml), [ekf_global.yaml](../config/ekf_global.yaml) | EKF 관측 구성, 공분산, 지연 재처리 |
| [status_policy.yaml](../config/status_policy.yaml) | freshness, 소스 일관성, 추측 항법 예산, 최종 출력 검사 |
| [relocalization_policy.yaml](../config/relocalization_policy.yaml) | 장기 GPS 단절 복구와 transaction ACK |
| [tf_configuration.yaml](../config/tf_configuration.yaml) | 차량 기준점, 동적 TF 소유자, 정적 장착값 |
| [visualization.yaml](../config/visualization.yaml) | 공개 Path·MarkerArray |
| [localization_viewer.yaml](../config/localization_viewer.yaml) | 공통 화면 토픽·색상·RDDF 경로·버퍼 |
| [lidar_front_visualization.yaml](../config/lidar_front_visualization.yaml) | 표시용 전방 scan 범위 |

## 현재 핵심 조건

- 기본 시작은 RDDF 경로에서 선택한 XY·차량 방향을 사용하는 초기화입니다. `initial_heading.yaml`의 고정 `1_right` yaw는 `start_rddf_initialization:=false`일 때 적용됩니다.
- GPS는 `minimum_fix_status: 0`이며 RTK Fixed만 허용하는 설정이 아닙니다.
  status 통과 후에도 시각·좌표·공분산·Local innovation 검사가 필요합니다.
- `gps_reference.yaml` 자체는 `first_fix`, `measured: false`입니다. 기본 RDDF 초기화 실행은 `rddf_initialization.yaml`의 `rddf_datum` 기준으로 덮어씁니다. 설계 경로 datum과 측량된 `manual_datum`은 구분합니다.
- GPS-only `automatic_reset_enabled: false`입니다. 장기 단절 복구 조건은
  [GPS 문서](gps_quality_and_recovery.md)에 있습니다.
- IMU는 `one_shot: true`, 보정각 적용 속도 상한 `90.0 deg/s`입니다.
  직진 구간 선별 조건의 `max_yaw_rate_degps: 3.0`과는 다른 값입니다.
- 같은 IMU YAML의 `encoder_yaw_hold.enabled: true`로 엔코더 0일 때 yaw를 유지합니다.
  alive가 갱신되는 0.30초 이내 feedback과 speed=0을 요구하며 비영점 tick은 고정하지 않습니다.
- 두 EKF는 보정 IMU와 vx·vy=0 제약을 사용합니다. Global GPS 관측은 x/y입니다.
- GPS 측정 시각의 Local 보간 이력과 Global 지연 재처리 이력은 각각 2초입니다.

## 실행 모드와 인자

| 진입점 | 기본 동작 |
|---|---|
| `localization` | 연결된 실센서·위치추정·공통 화면, 기록 끔 |
| `localization-record` | 센서 확인 후 rosbag 기록 |
| `map_data_collection.launch` | 실센서·기록 기본 켬 |
| `replay.launch bag:=...` | 원본 센서 백으로 현재 코드를 재계산, 실장치 드라이버 끔 |
| `bringup.launch` | IMU·엔코더·GPS 위치 추정 및 RDDF 초기화 |
| `viewer.launch` | 실행 중인 토픽 또는 지정한 결과 백의 공통 화면 |

센서를 외부 프로세스가 소유하면 `start_encoder_driver`, `start_imu_driver`,
`start_gps_driver`를 꺼 중복 연결을 막습니다. `start_rviz:=false`로 화면만 끌 수 있습니다.
`start_recording:=false`는 수집 launch의 기록만 끄며 `localization`은 이를 강제합니다.

`start_supervisor`는 내부 StatusManager와 Coordinator를 함께 제어합니다.
`start_status_manager`는 기존 호출 호환용 별칭입니다. Supervisor가 꺼지면
공개 GPS pose와 최종 상태 승인이 없어집니다. `start_calibrated_imu:=false`는
외부 노드가 같은 보정 IMU 출구를 제공할 때 사용합니다. 구현된 `motion_source`는 `encoder`입니다.

`rviz_config`는 공통 뷰어 YAML 경로를 받는 기존 인자명입니다. 공개 토픽을 바꾸면
`localization_interfaces.yaml`과 `localization_viewer.yaml`을 함께 조정합니다.
실제 노드 이름은 launch 인자가 결정합니다. EKF 이름을 바꾸면
`tf_configuration.yaml`의 동적 TF 소유자도 함께 변경합니다.

파일 교체 인자와 노드 이름 인자는 [bringup.launch](../launch/bringup.launch),
기록·센서 인자는 [map_data_collection.launch](../launch/map_data_collection.launch),
재생 입력은 [replay.launch](../launch/replay.launch)가 기준입니다.
장치 설치는 [udev 안내](../udev/README.md), 지도 준비는 [maps 안내](../maps/README.md)를 참고합니다.
