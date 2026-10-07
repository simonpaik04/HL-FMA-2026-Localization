# 추출 범위와 출처

이 저장소는 최종 차량 프로젝트의 Localization을 독립 Catkin 작업공간으로 정리했습니다.

## 최종 기준

- 원본: [Team-Stier/HL-FMA-1-5-2026](https://github.com/Team-Stier/HL-FMA-1-5-2026)
- branch: `main`
- 기준 commit: `fba9a5c12382a61a2711fe5685fe839e20c804c3`
- 추출 날짜: 2026-10-08 (Asia/Seoul)
- 구현·설정·launch·메시지·RDDF·테스트 자료는 위 commit의 `src/localization`을 기준으로 합니다. 원본의 로컬 미커밋 변경은 섞지 않았습니다.
- 이전 독립 레포의 `Team-Stier/HL-FMA2026-stier` 기반 구현은 Git 이력에 남습니다. 이전 키보드 IMU 테스트 파일은 최종 원본에 없어 현재 배치에서 제외했습니다.

## 포함과 제외

| 포함 | 원본 위치 |
|---|---|
| Localization 구현·설정·메시지·경로·테스트 | `src/localization` |
| 차량 feedback 지원 메시지 | `src/interfaces/vehicle_interface/erp42_msgs` |
| Route/RouteMap 등 기존 지원 메시지 | `src/interfaces/planning_interfaces` |

지원 메시지 패키지는 외부 통합 계약을 유지하기 위한 것입니다. 새로 구현한 localization 알고리즘으로 주장하지 않습니다. 다른 차량 모듈, 센서 드라이버 소스, rosbag·빌드 결과, 과거 archive, 기존 자동 생성 HTML·그림·검사와 모델 사용 집계 자료는 포함하지 않습니다.

독립 배치에 맞춘 변경은 패키지 README·상세 문서와 최상위 소개, 설정 테스트의 `erp42_msgs` 경로, 두 shell wrapper의 workspace 기본 경로입니다. 원본 RDDF README의 State Manager 그림 링크는 원본 GitHub 주소로 연결했습니다. 지원 메시지의 파일 끝 빈 줄만 정리했고 필드·상수는 유지했습니다. C++·Python 핵심 동작, 최종 YAML 값과 ROS 메시지 계약은 그대로 가져왔습니다. 새 방어 로직이나 정책을 추가하지 않았습니다.

## 드라이버와 외부 의존성

EKF 구현은 `robot_localization`, ROS 메시지·TF·시각화는 ROS 패키지를 사용합니다. `erp42_msgs`, `planning_interfaces`는 포함하고 `ublox_msgs`는 ROS 배포판/rosdep으로 설치합니다.

| 센서 | 필요한 패키지·설정 |
|---|---|
| Xsens IMU | `xsens_mti_driver`, `config/imu_driver.yaml` |
| u-blox GNSS | 프로젝트의 수정 `ublox_gps`, `config/gps_driver.yaml`, `config/time_sync.yaml` |
| Arduino encoder | `rosserial_python`, 기존 feedback firmware, `config/encoder_driver.yaml` |
| RPLIDAR, 선택 | `rplidar_ros`, 장착 TF, `map_data_collection.launch` |

원본 드라이버 위치는 `src/sensor_drivers/imu/xsens_ros_mti_driver`와 `src/sensor_drivers/gps/ublox`입니다. 필요한 센서 드라이버만 별도 ROS workspace에서 해당 README·의존성에 따라 빌드합니다. 별도 workspace를 쓰면 localization 빌드 전에 driver workspace의 `devel/setup.bash`도 source합니다. 같은 패키지를 apt와 소스 workspace에서 중복 배치하지 않습니다.

시각 모니터는 `ublox_gps: measurement timing`과 `receipt_minus_utc_clock_offset_plus_transport_ns` 등의 진단 필드를 읽습니다. 일반 배포판 드라이버에 같은 출력이 있다고 가정하지 않습니다. 최종 기본 설정은 호스트 시각에 의한 `clock_ready` 차단을 비활성화하지만 GPS 자체의 timestamp·이력 정합 검사는 유지됩니다. 진단 호환성과 운행 승인 조건을 구분합니다.

원본 [GPS 드라이버](https://github.com/Team-Stier/HL-FMA-1-5-2026/tree/fba9a5c12382a61a2711fe5685fe839e20c804c3/src/sensor_drivers/gps/ublox)와 [시각 모니터](../src/localization/scripts/sensor_timing_monitor.py)를 참고합니다.

현장 wrapper `localization_record_command.sh`는 별도 CAN capture 스크립트를 요구합니다. 이 파일은 포함하지 않으므로 사용할 때 `MANDO_CAPTURE_SCRIPT`로 지정합니다. 일반 bringup·rosbag 재계산은 이 wrapper 없이 실행할 수 있습니다. workspace 자동 탐지는 소스 트리 배치 기준이며 catkin install 배치에서는 `MANDO_LOCALIZATION_WS`를 명시합니다.

## 이미지와 Archify

대표 화면과 분석 그래프는 이 PC에 있던 `Rosbag/rddf_display_check_20260914`의 실제 과거 재생 자료입니다. 원본 bag 기록은 `20260906_123929`, 화면 검증은 2026-09-14입니다. 새 최종 commit의 실행을 캡처했다고 주장하지 않습니다. 이미지 출처·해시는 [기록](architecture/runtime-image-provenance.json)에 남깁니다.

Archify는 위 최종 commit의 코드·launch를 근거로 다시 작성했습니다. [JSON·HTML·검증 기록](architecture/README.md)을 함께 제공합니다. 그림의 검증과 localization 런타임 테스트 결과는 구분합니다.

## 기존 라이선스 표기

`mando_localization`의 manifest는 `BSD-3-Clause`, `planning_interfaces`는 `MIT`, `erp42_msgs`는 원본의 `TODO` 표기입니다. 저장소 전체에 새 라이선스를 일괄 부여하거나 외부 코드의 저작권·라이선스를 임의로 변경하지 않았습니다. 별도 LICENSE 본문과 지원 패키지 라이선스 확정은 현재 snapshot에 포함되지 않습니다.
