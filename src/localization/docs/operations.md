# mando_localization

현재 운영값·GPS 비활성 모드·반복 방향 보정·RDDF 연속 추적은 [최종 설정](final_configuration.md)을 기준으로 확인합니다. 아래 과거 실험 기록은 이번 snapshot의 검증 결과와 구분합니다.



현재 기본 시작 동작은 [GPS/RViz 기반 RDDF 초기화](rddf_startup.md)입니다. 코스 중간에서도 실제 차량을 RDDF 중심선·진행 방향에 놓고 실행합니다.
`mando_localization`은 ROS1 Noetic 차량에서 IMU·엔코더의 연속 움직임과 GPS의 절대 위치를 두 단계 EKF로 결합하는 패키지입니다. 상태 판단과 최종 위치 출력을 분리하고, 입력이 오래되거나 계약을 위반하면 공개 Odometry를 차단하는 fail-closed 구조입니다.

자세한 내용은 [문서 목차](README.md), [아키텍처](architecture.md),
[GPS 품질·재연결](gps_quality_and_recovery.md),
[센서 시각·지연 GPS 처리](sensor_timing.md),
[CalibratedIMU와 yaw 보정 조건](calibrated_imu.md),
[설정 가이드](configuration.md)를 참고하십시오.

## 공통 RViz와 로스백 재생

기본 실행은 RDDF 위치·방향 초기화입니다. 고정 `1_right` 출발 yaw는 초기화가 비활성인 기존 모드에서 사용합니다. 정지 yaw 유지와 GNSS 반복 정합은 [최종 설정](final_configuration.md)을 참고합니다.

실시간 bringup, `localization` 명령, 원본 로스백 재계산, 기록 결과 탐색은 모두
[`localization_viewer.py`](../scripts/localization_viewer.py)와
[`localization_viewer.yaml`](../config/localization_viewer.yaml)을 사용합니다.
RDDF 원본은 패키지의 `rddf/`에서 읽고 변경하지 않습니다. 다른 설치 경로에서는
YAML의 `rddf_directory`를 지정합니다. 상대 경로는 패키지 디렉터리 기준입니다.
기존 `rviz_config` launch 인자는 호환을 위해 이름을 유지하지만 이제 이 공통 YAML을 받습니다.

- 초록 Local, 빨강 Global, 보라 원시 GPS, 파랑 RDDF와 차량·LiDAR·상태·Raw/Calibrated yaw를 표시합니다.
- 현재 Global 위치에 해당하는 RDDF를 굵은 노란색으로 강조하고 차량 옆에 경로 이름과 경로까지의 거리를 표시합니다. 가까운 경로가 겹치면 주황색 후보로 표시하며, 5 m 밖은 `off route`로 표시합니다. Global 수신이 0.5초 넘게 끊기거나 valid가 false/1초 이상 미수신이면 강조를 해제합니다. 화면 탐색 중에는 선택 시점 기준이며, 과거 평행이동 모드는 `display coordinates`로 구분합니다. 거리 기준은 `current_rddf_*` YAML 설정, 표시 결과는 `/mando_localization/visualization/debug/status`의 `current_rddf`에서 확인할 수 있습니다.
- 첫 유효 GPS와 첫 Local 위치로 표시용 XY 이동을 한 번 정하고 Local/Global에 동일하게 적용합니다. **초기 yaw를 0으로 돌리거나 경로별로 정렬하지 않습니다.** GPS와 Local이 모두 오기 전에는 RDDF를 먼저 표시하며 기준점 대기 문구를 보여줍니다.
- ±10초, 슬라이더, 분:초 입력, 화면 재생/정지, 배속, `최신 / 끝` 버튼을 지원합니다. 실시간 모드의 탐색은 수신한 결과 버퍼만 다시 그립니다. 센서 재생·EKF·ROS 시계를 뒤로 돌리거나 정지하지 않습니다.
- 새 데이터는 화면 정지 중에도 수신합니다. `최신 / 끝`으로 실시간 화면에 복귀합니다. 노드를 재시작하면 실시간 버퍼가 초기화됩니다.
- 버퍼는 토픽당 최대 200,000개, LiDAR는 최대 6,000개를 보관합니다. 오래된 표본이 삭제된 시점에는 해당 데이터가 표시되지 않습니다. 진단 JSON에 누락·버퍼 삭제 수가 기록됩니다.
- Global EKF는 추정값이며 RDDF는 설계 경로입니다. 표시 기준은 측량된 map↔RDDF 변환이 아닙니다.

현재 실행 중인 ROS에 화면만 연결:

```bash
roslaunch mando_localization viewer.launch
```

원본 센서 백으로 **현재 코드**를 처음부터 실행하고 같은 RViz 열기:

```bash
roslaunch mando_localization replay.launch bag:='/절대경로/원본.bag' rate:=1.0
```

이 재생 진입점은 현재 기록 계약의 driver IMU/GPS/NavPVT, Arduino feedback, raw scan만 입력합니다.
기존 EKF 출력과 TF는 입력하지 않습니다. 다른 토픽 계약의 백은 토픽 구성을 먼저 확인해야 합니다.
`start:=530`으로 중간부터 재계산할 수 있지만, EKF 초기 상태도 그 시점에서 새로 시작하므로
처음부터 실행한 결과를 탐색하는 것과 다릅니다. 실센서 실행과 재생은 별도 ROS master에서 사용하십시오.

완료된 계산 결과를 탐색할 때는 파일을 명시합니다. 이전 분석 폴더를 자동 선택하지 않습니다:

```bash
roslaunch mando_localization viewer.launch mode:=recorded \
  processed_bag:='/절대경로/outputs.bag' source_bag:='/절대경로/원본.bag'
```

`source_bag`는 시간 원점과 길이에만 사용하며, 표시 데이터는 `processed_bag`에서 읽습니다.
원본을 생략하면 결과 백 자체의 시작·끝을 사용합니다. 이 모드도 같은 렌더러·설정을 사용합니다.
과거 `rosbag/.../viewer*.py`, `.rviz`와 zero-origin 시각화는 과거 실험 재현 자료이며 운영 진입점에서 사용하지 않습니다.

## 공개 ROS 인터페이스

| 용도 | 기본 토픽 | 메시지 |
|---|---|---|
| Arduino 피드백 | `/erp42_serial/feedback` | `erp42_msgs/SerialFeedBack` |
| 차량 속도 | `/molit/vehicle/twist` | `geometry_msgs/TwistWithCovarianceStamped` |
| IMU 원본 / 정규화 | `/molit/sensors/imu/data`, `/molit/localization/imu/normalized` | `sensor_msgs/Imu` |
| 공통 보정 IMU | `/molit/localization/imu/calibrated` | `sensor_msgs/Imu` |
| GPS 원본 | `/molit/sensors/gps/fix` | `sensor_msgs/NavSatFix` |
| GPS 상세 상태 | `/molit/sensors/gps/navpvt` | 드라이버의 NavPVT 타입을 그대로 relay |
| 원시 LaserScan | `/molit/sensors/lidar/scan` | `sensor_msgs/LaserScan` |
| Local / Global EKF | `/molit/localization/local/odometry`, `/molit/localization/global/odometry` | `nav_msgs/Odometry` |
| 현재 RDDF 경로 | `/molit/localization/rddf/current` | `mando_localization/RddfMatch` |
| GPS 승인 위치 | `/molit/localization/gps/map_pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| 최종 위치 | `/molit/localization/odometry` | `nav_msgs/Odometry` |
| 경로 / 마커 | `/molit/localization/path`, `/molit/localization/markers` | `nav_msgs/Path`, `visualization_msgs/MarkerArray` |
| 상태 / enum / 유효성 | `/molit/localization/status`, `/molit/localization/state`, `/molit/localization/valid` | `DiagnosticArray`, `String`, `Bool` |
| 장기 복구 단계 | `/molit/localization/recovery/state` | `std_msgs/String` |

`/molit/localization/gps/map_pose`의 단일 발행자는
`RelocalizationCoordinator`입니다. GPS gate의 정상 승인 결과와 장기 복구
quality candidate는 서로 다른 내부 토픽으로만 Coordinator에 전달되며 Global
EKF를 직접 우회하지 않습니다.

### 현재 RDDF 경로 구독

`rddf_tracker`가 Global Odometry와 `/molit/localization/valid`를 구독해 현재 위치를
RDDF CSV에 투영하고 **10 Hz**로 발행합니다. `bringup.launch`에서 기본 실행하므로
`localization`, 지도 수집, 원본 bag 재계산에서도 RViz 없이 사용할 수 있습니다.
RViz의 화면 정지·과거 시점 탐색은 이 공개 토픽에 영향을 주지 않습니다.
기존 RViz와 공통 매칭 함수를 사용하지만, 공개 토픽은 표시용 평행이동을 적용하지 않고
RDDF 원점과 같은 **map 좌표의 Global 위치**를 사용합니다.

첫 유효 위치에서는 전체 RDDF 중 위치와 차량 yaw가 맞는 구간을 활성화합니다. 이후에는
교차로에서 다른 RDDF가 더 가까워져도 현재 원본 RDDF를 유지하며, 현재 경로와 실제로
연결된 다음 번호 경로의 진입부에 도달했을 때만 전환합니다. 따라서 현재 RDDF의 선택·순서·
교차점 연속성은 Localization 한 곳에서 결정하며 State Manager는 이 결과를 소비만 합니다.
프로세스를 새로 시작하면 임의 구간에서도 최초 전역 매칭이 가능합니다.
주차 경로를 선택하거나 기어가 바뀌는 경계(4→5, 5→6, 9→10, 10→11)는
State Manager의 `/mission/rddf_successor` 요청에 지정된 정확한 원본 RDDF만 전환 후보로
사용합니다. 요청 경로가 현재 RDDF의 다음 번호이고 실제 진입부에 연결된
경우에만 활성화하며, 겹친 주차 RDDF의 기하 매칭만으로는 자동 전환하지 않습니다.
종료 분기도 같은 요청 방식으로 검증합니다.
최초 1번 RDDF의 좌·우가 겹치는 경우에도 같은 요청으로 지정된 시작 분기를 우선합니다.

T자와 평행주차는 같은 쪽의 진입·진출을 **하나의 RDDF 이름**으로 표시/발행합니다.
왼쪽과 오른쪽은 별개이며, 아래 통합으로 현재 RDDF 식별자는 총 **15개**입니다.

| 공개 `route_name` | 포함하는 원본 CSV 경로 |
|---|---|
| `T_left` | `5_T-left-in`, `6-T-left-out` |
| `T_right` | `5_T-right-in`, `6_T-right-out` |
| `parallel_left` | `10_parallel-left-in`, `11-parallel-left-out` |
| `parallel_right` | `10_parallel-right-in`, `11_parallel-right-out` |

같은 그룹 안의 진입·진출이 겹쳐도 `matched=true`로 그 그룹을 표시하며, RViz는 두 경로를
함께 강조합니다. 수동 초기화 클릭이 두 원본 RDDF에 걸리면 그룹 매칭이 성공했더라도
후보 메뉴를 열어 진입/진출 원본 경로와 차체 방향을 명시적으로 고르게 합니다.
왼쪽·오른쪽 또는 다른 경로까지 비슷하게 가까우면 `AMBIGUOUS_ROUTE`입니다.
원본 19개 CSV의 좌표와 후진 방향은 유지합니다. 초기 위치·방향 선택은 원본 경로를 사용하고,
현재 위치 표시에 사용할 그룹은 [`yongin_route_project.json`](../rddf/yongin_route_project.json)의
`route_groups`에서 관리합니다.

메시지를 추가했으므로 먼저 작업공간에서 `catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2` 후
`source devel/setup.bash`를 실행합니다. 별도 작업공간의 구독 패키지는 `mando_localization`을
의존성에 추가하고 이 메시지가 빌드된 작업공간을 source해야 합니다.
`source_route_name` 필드가 추가된 메시지 형식을 사용하므로 기존 구독 노드도 재빌드·재시작합니다.

```bash
rostopic type /molit/localization/rddf/current
rosmsg show mando_localization/RddfMatch
rostopic echo /molit/localization/rddf/current
rostopic hz /molit/localization/rddf/current
```

이미 실행 중인 localization에 발행 노드만 추가하려면 다음 명령을 사용합니다.
새 bringup에서는 자동 실행되므로 중복 실행하지 않습니다.

```bash
roslaunch mando_localization rddf_tracking.launch
```

| `RddfMatch` 필드 | 의미 |
|---|---|
| `header.stamp`, `header.frame_id` | 발행 ROS 시각과 RDDF 좌표 프레임(기본 `map`) |
| `pose_stamp` | 입력 Global Odometry의 측정 시각. 입력이 없으면 0 |
| `matched`, `reason` | 경로를 하나로 판별했는지와 아래 상태 코드 |
| `route_name` | 성공한 공개 RDDF 이름(주차는 좌우별 진입·진출 그룹). 실패하면 빈 문자열 |
| `source_route_name`, `segment_index` | 최근접 원본 CSV 경로명과 해당 CSV 선분 시작점 index. 실패하면 빈 문자열 / -1 |
| `has_nearest`, `nearest` | 최근 유효 위치에서 계산한 가장 가까운 투영 결과. `has_nearest=false`이면 `nearest`의 기본값을 사용하지 않음 |
| `candidates` | 허용 거리 안에서 최단 거리 + 모호성 여유에 들어온 후보 선분 배열. 같은 경로의 다른 선분도 들어갈 수 있음 |

`nearest`와 `candidates[]`의 타입은 [`RddfCandidate`](../msg/RddfCandidate.msg)입니다.
`route_name`(공개 이름), `source_route_name`(원본 CSV), `segment_index`, `projected_point`(map, m), `distance_m`(차량과 투영점 사이 거리),
`heading_rad`(경로의 차량 진행 방향, ENU), `segment_fraction`(해당 선분 내 0~1)을 제공합니다.
`segment_index`는 RDDF 조각 번호나 waypoint 전체 진행률이 아닙니다.
주차 그룹의 `matched=true`는 그룹 식별 성공을 뜻합니다. 진입/진출 선분이나 진행 방향까지
하나로 확정했다는 의미는 아니므로, 해당 정보가 필요하면 `candidates`의 원본 경로와 방향을 확인합니다.

| `reason` | 구독 측 해석 |
|---|---|
| `MATCHED_OFF_ROUTE` | 활성 원본 경로를 유지한 채 거리 제한 밖. 실제 거리 확인 필요 |
| `ROUTE_TRANSITION` | 연결된 다음 원본 경로로 전환 |
| `MATCHED` | `matched=true`; `route_name`과 `nearest` 사용 가능 |
| `AMBIGUOUS_ROUTE` | 경로가 겹치거나 여러 분기가 비슷하게 가까움. `candidates`를 확인하며 경로 하나를 확정하지 않음 |
| `TOO_FAR` | 가장 가까운 RDDF도 5 m 초과. `nearest`는 참고용이며 현재 경로로 사용하지 않음 |
| `NO_GLOBAL`, `STALE_GLOBAL` | Global 미수신 또는 헤더/수신 시각이 0.5초보다 오래됨 |
| `STALE_VALID`, `LOCALIZATION_INVALID` | valid 미수신/1초 초과 또는 false |
| `FRAME_MISMATCH`, `INVALID_STAMP`, `INVALID_INPUT` | map 프레임 불일치, 0/미래 timestamp 또는 비정상 좌표 |

실패 메시지도 계속 발행합니다. 활성 원본 경로는 거리 이탈만으로 재선택하지 않으며 `MATCHED_OFF_ROUTE`에서 유지합니다. ROS 시간이 뒤로 이동하면
저장한 pose와 valid를 비웁니다. 발행은 ROS 시간 기준이고 latch를 사용하지 않으므로,
bag의 `/clock`이 정지하면 발행도 정지합니다. 구독자는 발행 노드 종료·통신 단절에 대비해
자체 수신 timeout을 두고, `matched=true`인 새 메시지만 사용해야 합니다.
최초 위치·방향 매칭 뒤에는 연결 관계·연속 진행과 미션 successor 요청을 사용합니다. State Manager의 미션 결정을 이 노드가 대신하지는 않습니다.

Python 구독 예제:

```python
#!/usr/bin/env python3
import rospy
from mando_localization.msg import RddfMatch

def on_rddf(msg):
    if not msg.matched:
        rospy.loginfo_throttle(1.0, 'RDDF unavailable: %s', msg.reason)
        return
    rospy.loginfo_throttle(1.0, 'RDDF=%s source=%s segment=%d distance=%.2f m',
                           msg.route_name, msg.source_route_name,
                           msg.segment_index, msg.nearest.distance_m)

rospy.init_node('current_rddf_subscriber')
subscriber = rospy.Subscriber('/molit/localization/rddf/current', RddfMatch,
                              on_rddf, queue_size=1)
rospy.spin()
```

토픽 이름은 [`localization_interfaces.yaml`](../config/localization_interfaces.yaml)의
`topics.current_rddf`, 주기·거리·유효시간은 [`rddf_tracking.yaml`](../config/rddf_tracking.yaml)에서
설정합니다. RDDF 디렉터리는 초기화와 동일한 `rddf_initialization.yaml`의
`initialization.rddf_directory`를 사용합니다. 설정 변경 후 노드를 재시작합니다.
bringup에서 비활성화하려면 `start_rddf_tracking:=false`를 지정합니다.

### Arduino 엔코더 연결 계약

`rosserial_python/serial_node.py`가 Arduino USB와 연결해 펌웨어의
`/erp42_serial/feedback`을 ROS에 노출하고 localization이 이를 직접 구독합니다.
현재 포트와 baud는 [`config/encoder_driver.yaml`](../config/encoder_driver.yaml)에
serial 기반 `/dev/serial/by-id/...` 경로와 `57600`으로 고정돼 있습니다.
`SerialFeedBack`에는 ROS `Header`가 없으므로 `EncoderToTwistAdapter`가 받은 PC 시각을 출력 timestamp로 사용합니다.

| 입력 필드 | 타입 | localization 사용 방식 |
|---|---|---|
| `MorA`, `EStop`, `Gear` | `uint8` | 수신은 하지만 현재 EKF 관측에는 사용하지 않음 |
| `speed` | `float64`, m/s | `vx = speed × 1.0 × 1`; Local/Global EKF의 전진 속도 |
| `steer` | `float64`, ADC | finite·범위 검사와 진단만 수행; yaw 계산에는 사용하지 않음 |
| `brake` | `int16` | 상태 진단만 수행 |
| `encoder` | `int32`, 최근 0.10 s delta | 범위 검사와 선택적 속도 일관성 검사; CalibratedIMU에서 0이면 정지 yaw 제약; 현재 meter-per-tick 비활성 |
| `alive` | `uint8` | 직전 값과 같으면 중복/정지로 거부; `255 → 0` 변경 허용 |

통과한 입력은 `/molit/vehicle/twist`의 `TwistWithCovarianceStamped`로 변환됩니다. 두 EKF는 `linear.x`와 `linear.y=0` 제약을 관측하며 `Var(vx)=0.25 (m/s)^2`, `Var(vy)=0.01 (m/s)^2`입니다. 정확한 기본값은 [`config/encoder_calibration.yaml`](../config/encoder_calibration.yaml)이 기준입니다.

펌웨어는 같은 rosserial 연결에서 `/erp42_serial/drive` 구독도 선언합니다.
`erp42_msgs/DriveCmd`는 깨끗한 rosserial type 협상을 위해 포함하지만 localization은
이 토픽을 발행하지 않습니다. 별도 Controller나 외부 rosserial bridge가 이미 포트를
소유한다면 `start_encoder_driver:=false`로 중복 연결을 막아야 합니다.

드라이버, `robot_localization` 내부 토픽은 충돌과 설정 중복을 막기 위해 `/mando_localization/internal/...`로 고정되어 있습니다. 어댑터는 GPS NavPVT도 relay하므로 공개 이름은 `config/localization_interfaces.yaml`에서 관리합니다. 공통 RViz의 구독 토픽은 `config/localization_viewer.yaml`에서 함께 관리합니다.

## 빌드와 기본 실행

ROS1 bag 녹화 파일은 작업공간의 `~/work/HL-FMA-2026-Localization/rosbag/` 아래에 저장합니다.

```bash
cd ~/work/HL-FMA-2026-Localization
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2
source devel/setup.bash
localization
```

지도 수집 현장과 동일한 RPLIDAR S2·Encoder·IMU·GPS·메인 Localization·RViz를
rosbag 기록 없이 PATH 명령 하나로 실행할 수 있습니다. 연결되지 않은 센서는
`입력 없음`으로 표시하고 해당 드라이버만 건너뜁니다.

```bash
localization
localization start_rviz:=false  # 화면이 필요 없는 경우
```

현재 `localization` 명령은 `map_data_collection.launch`를 사용하되
`start_recording:=false`를 강제합니다. raw LiDAR와 정적 TF를 실행하며 위치 추정은 IMU·엔코더·GPS를 사용합니다.
GPS 장치가 없으면 스크립트가 `enable_gps_fusion:=false`로 실행하고, RViz에서 수동
RDDF 초기위치를 확정한 뒤 IMU·엔코더 기반 Local/Global EKF 결과를 최종 Odometry로
출력합니다. 이 모드는 절대 위치·방향 보정이 없으므로 이동할수록 drift가 누적됩니다.

LiDAR는 `lidar_front_scan_visualizer`가 만든
`/mando_localization/visualization/lidar/front_scan`을 청록색으로 표시합니다.
전방 마스크 설정은 `config/lidar_front_visualization.yaml`, 장착 TF는
`config/tf_configuration.yaml`이 기준입니다. 원시 스캔은 변경하지 않습니다.
선택 시점 직전 Global 자세와 `base_link -> laser_link` 정적 장착 변환을 사용하며,
스캔 또는 Global 표본이 0.5초보다 오래됐거나 장착 TF가 없으면 점을 숨깁니다.
이 시간 탐색 화면은 스캔 측정시각에 차량 자세를 보간한 정밀 정합 검증은 아닙니다.

뷰어는 독립적인 `localization_debug` 표시 프레임만 사용하며 메인 EKF 출력과
`map -> odom -> base_link` TF를 수정하지 않습니다. 화면을 뒤로 이동해도
추정 노드의 ROS 시계와 센서 입력은 계속 진행합니다.

외부에서 공개 센서 토픽을 발행하지 않으면 상태는 유효해지지 않고 최종
Odometry도 나오지 않습니다. Local drift가 누적된 장기 GPS 단절에서는 기본
설정이 자동 위치 점프를 하지 않고 `RELOCALIZING`, `valid=false`를 유지합니다.

## 용인운전면허장 지도 데이터 수집

동일한 센서와 화면을 실행하면서 rosbag까지 저장하려면 다음 수집 전용 명령을
사용합니다.

```bash
localization-record
```

이 명령은 RPLIDAR S2, Encoder, Xsens IMU, u-blox GPS, 메인 Localization과
동일한 Local/Global 비교 RViz를 함께 실행합니다. 원시 LiDAR와 활성화된 원본 ROS 토픽을 `LZ4`로 압축하고
2 GB 단위로 분할하며, 각 분할 파일에 `/tf_static`을 다시 기록합니다. Odometry로
재생성할 수 있는 누적 Local/Global Path와 누적 경로 Marker는 기록에서 제외합니다.

기본 저장 위치는 다음과 같으며 실행 시각별 하위 폴더를 자동 생성합니다.

```text
~/work/HL-FMA-2026-Localization/rosbag/용인운전면허장 로스백/<YYYYMMDD_HHMMSS>/
```

각 폴더에는 `mapping*.bag`, `bag_info.txt`, `record_validation.txt`,
`session_info.txt`, 실행 launch와 config 스냅샷이 남습니다. 디스크 여유 공간이
10 GB 아래로 내려가거나 LiDAR
노드가 종료되면 불완전한 기록을 계속하지 않고 launch를 종료합니다. 정상 종료는
터미널에서 `Ctrl+C`를 한 번 누릅니다.

실행 직후에는 각 센서 토픽과 `/tf`, `/tf_static`, `/diagnostics`의 실제 메시지를
최대 30초 동안 검사합니다. 터미널에 `수집 준비 완료: 이제 주행을 시작하세요.`가
표시된 뒤 출발합니다. 필수 토픽이 하나라도 없으면 자동으로 기록을 닫고 누락 토픽을
출력합니다.

기본적으로 네 센서 장치가 모두 연결되지 않으면 기록을 시작하지 않습니다. 센서가
일부 없는 배선 시험만 명시적으로 허용하려면 다음 옵션을 사용합니다.

```bash
localization-record --allow-missing-sensors
localization-record --no-rviz
localization-record --output-root /absolute/output/path
```

`-a` 기반 기록 방식이므로 별도로 실행 중인 카메라나 진단 토픽도 자동 포함되지만,
위 누적 시각화 토픽에는 exclude 정규식을 적용합니다. 이 명령 자체가 카메라
드라이버를 시작하지는 않습니다. 현장 출발 전 RViz에서
LaserScan을 확인하고, 종료 후 `bag_info.txt`에 최소한
`/molit/sensors/lidar/scan`, `/molit/sensors/imu/data`,
`/molit/sensors/gps/fix`, `/molit/sensors/gps/navpvt`,
`/erp42_serial/feedback`, `/tf`, `/tf_static`이 있는지 확인합니다.

장치 경로와 드라이버 설정은 [설정 가이드](configuration.md)와
[udev 설치 절차](../udev/README.md)를 참고합니다. tty 번호 대신 설정된
`/dev/serial/by-id` 또는 장치 별칭을 확인합니다.

`bringup.launch`는 IMU·엔코더·GPS 위치 추정과 RDDF 초기화를 실행합니다.

```bash
roslaunch mando_localization bringup.launch
```

## 운영 범위와 검증

- GPS gate는 현재 `minimum_fix_status: 0`입니다. RTK Fixed 전용 정책이 아니며,
  시각·공분산·Local innovation 검사를 함께 통과해야 합니다.
- Datum은 `first_fix`, `measured: false`이고 GPS-only 자동 reset은 꺼져 있습니다.
  실제 `maps/map.yaml`은 제공되지 않습니다. [지도 안내](../maps/README.md)를 참고합니다.
- IMU·LiDAR 장착 TF는 활성화되어 있고 GPS 6DoF TF는 비활성입니다.
  [TF 문서](tf_frames.md)의 현재 설정과 측정 근거를 확인합니다.
- 엔코더 속도 scale, 주행 중 IMU 불확실성, GPS 평면 레버암의 정밀 측정과
  지도 기준 정확도는 코드 실행 성공만으로 검증되지 않습니다.

```bash
cd ~/work/HL-FMA-2026-Localization
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make run_tests_mando_localization -j1
catkin_test_results build/test_results/mando_localization
```

회귀 테스트는 시각 지연·GPS gate·복구 transaction·상태 및 출력 계약을 검사합니다.
합성 ROS 시험 통과와 실차 GNSS 정확도·지도 정합·제어 복구 성능은 구분합니다.
과거 검증 수치와 실험 설정은 해당 `rosbag/` 또는 `.codex-runtime/` 결과에 보존합니다.
정리 전 소스는 작업공간 `snapshots/1차빌드완료/`에서 복원할 수 있습니다.
