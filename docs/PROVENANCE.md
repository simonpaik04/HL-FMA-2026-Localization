# 추출 범위와 출처

이 저장소는 `Team-Stier/HL-FMA2026-stier`의 localization을 독립적인 Catkin 작업공간으로 정리한 것입니다. 패키지명 `mando_localization`과 기존 토픽·메시지 이름을 유지합니다.

## 추출 기준

- 원본: https://github.com/Team-Stier/HL-FMA2026-stier
- 기준 commit: `799f0c17c5f5e80967098a64ff19f30a9f1f09db`
- 정리 날짜: 2026-10-08 (Asia/Seoul)
- `src/localization/`은 당시 로컬 작업 트리의 수정·추가 파일을 포함합니다. 위 commit만으로 동일한 snapshot을 재현할 수는 없습니다.
- 기존 수정: 패키지 CMake, README, 뷰어와 뷰어 테스트.
- 기존 추가: 키보드 IMU 테스트의 launch·YAML·Python relay/encoder·설명 문서.
- 지원 패키지 `erp42_msgs`는 원본 `src/interfaces/vehicle_interface/erp42_msgs`에서 가져왔습니다. Localization이 사용하는 엔코더 입력 메시지 호환을 위한 것이며, 별도로 새로 구현한 메시지라고 주장하지 않습니다.
- 이번 정리에서는 최상위 README·문서 목차·검증 기록을 작성하고, 기존 운영 README를 `src/localization/docs/operations.md`로 옮겼습니다. 운영 wrapper의 작업공간 기본 경로를 현재 checkout 기준으로 변경했습니다. `launch.sh`의 오래된 패키지/launch 이름과 설정 테스트의 지원 패키지 경로를 독립 배치에 맞췄습니다. 기존 설명 중 고정 yaw·datum·LiDAR와 뷰어 좌표 모드의 오래된 문구를 현재 기본 실행에 맞게 정리했습니다.
- 다른 차량 모듈, 센서 드라이버 소스, bag·빌드 산출물, 과거 실험 archive, 자동 생성 HTML/이미지와 검사·모델 사용 집계 산출물은 제외했습니다.

## 드라이버와 외부 의존성

EKF 구현은 `robot_localization`, ROS 메시지·TF·시각화는 ROS 패키지를 사용합니다. 이 저장소의 핵심 빌드에는 `erp42_msgs`와 `ublox_msgs`가 필요합니다. 전자는 포함하고 후자는 rosdep/ROS 배포판으로 설치합니다.

실센서 실행에는 센서별 드라이버를 별도로 준비합니다.

| 센서 | 필요한 패키지/설정 |
|---|---|
| Xsens IMU | `xsens_mti_driver`, `config/imu_driver.yaml` |
| u-blox GNSS | 프로젝트에서 수정한 `ublox_gps`와 타이밍 진단, `config/gps_driver.yaml`, `config/time_sync.yaml` |
| Arduino encoder | `rosserial_python`, 기존 feedback firmware와 `config/encoder_driver.yaml` |
| RPLIDAR, 선택 | `rplidar_ros`, 장착 TF와 `map_data_collection.launch` |

원본 드라이버 위치는 `src/sensor_drivers/imu/xsens_ros_mti_driver`와 `src/sensor_drivers/gps/ublox`입니다. ROS workspace에 해당 패키지를 별도 배치하고 드라이버 자체의 README와 의존성에 따라 빌드합니다. 서로 다른 workspace를 사용하는 경우 localization 빌드 전에 driver workspace의 `devel/setup.bash`도 source합니다. apt로 설치한 같은 이름의 패키지와 중복 배치하지 않습니다.

현재 시각 모니터는 `ublox_gps: measurement timing` 진단과 `receipt_minus_utc_clock_offset_plus_transport_ns` 등의 필드를 읽습니다. 일반 `ublox_gps`의 동일한 동작을 가정하지 않습니다. 필요한 드라이버 출력은 원본 [수정 GPS 드라이버](https://github.com/Team-Stier/HL-FMA2026-stier/tree/799f0c17c5f5e80967098a64ff19f30a9f1f09db/src/sensor_drivers/gps/ublox)와 [시각 모니터](../src/localization/scripts/sensor_timing_monitor.py)를 참고합니다. `clock_ready`를 임의로 true로 바꾸어 검증을 우회하는 실행 방법은 제공하지 않습니다.

`localization_record_command.sh`는 별도의 CAN capture 스크립트도 요구하는 현장용 wrapper입니다. 그 스크립트는 이 저장소에 포함하지 않으며, 사용할 때 `MANDO_CAPTURE_SCRIPT`로 지정합니다. 일반 융합과 rosbag 재계산은 이 wrapper 없이 실행할 수 있습니다. command wrapper의 workspace 자동 탐지는 소스 트리 배치 기준이며, catkin install 배치에서는 `MANDO_LOCALIZATION_WS`를 명시합니다.

## 기존 라이선스 표기

`mando_localization/package.xml`의 기존 표기는 `BSD-3-Clause`입니다. 지원 패키지 `erp42_msgs/package.xml`은 원본의 `TODO` 표기를 유지합니다. 저장소 전체에 새 라이선스를 일괄 부여하거나 외부 코드의 저작권·라이선스를 임의로 변경하지 않았습니다. 별도 LICENSE 본문과 지원 패키지의 라이선스 확정은 현재 snapshot에 포함되지 않습니다.
