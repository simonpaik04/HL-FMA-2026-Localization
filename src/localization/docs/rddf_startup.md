# RDDF 위치 기반 초기화

현재 운영값·GPS 비활성 모드·반복 방향 보정·RDDF 연속 추적은 [최종 설정](final_configuration.md)을 기준으로 확인합니다. 아래 과거 실험 기록은 이번 snapshot의 검증 결과와 구분합니다.


차량을 실제 RDDF 중심선 위에 **설정된 차량 방향에 맞춰** 놓고 실행한다. 코스 중간에서 프로그램 전체를 다시 실행해도 고정된 `1_right` 출발 yaw를 사용하지 않고 그 위치의 차량 방향(전진은 경로 접선, 후진은 접선 반대)으로 초기화한다. 이 가정은 작업 중 사용자에게 확인했다.

`rddf/yongin_route_project.json`의 `route_directions`에서 `5_T-left-in`, `5_T-right-in`, `10_parallel-left-in`을 `reverse`로 지정한다. 이 세 경로는 전체 구간에 걸쳐 차량 yaw를 경로 접선에서 180° 보정한다. 미지정 경로는 `forward`다. CSV 점 순서와 `path_yaw_rad`는 경로 진행 방향 그대로 유지한다. 편집기에서 프로젝트를 다시 내보낼 때 이 사용자 정의 설정이 보존되는지 확인한다. 이 설정은 초기 자세와 미리보기에 적용되며 기어 명령을 발행하지 않는다.

## 실행과 조작

기존 `localization` 명령과 `bringup.launch` / `replay.launch`는 기본 `start_rddf_initialization:=true`다.

1. GPS의 fix/frame/원래 timestamp/covariance와 clock readiness를 확인한다. 차량 정지는 fresh 엔코더 Twist로 확인한다.
2. GPS 위경도를 `rddf/yongin_route_project.json`의 origin 기준 ENU로 투영한다. RDDF 방향을 이용한 GPS 레버암 보정 후 가까운 선분에 투영한다. 안정된 3개 GPS 후보가 필요하다.
3. IMU 현재 차량 yaw를 선택된 RDDF 차량 yaw에 맞춘다. 이 초기 정합은 GNSS 주행 heading 보정의 one-shot을 소비하지 않는다.
4. 두 EKF의 set_pose에 선택된 x/y/yaw를 각자의 frame(Local=odom, Global=map)으로 보낸다. 서비스 반환만으로 성공 판정하지 않고 fresh Local/Global 결과 각각 3개의 위치·yaw를 확인한다.
5. 확인 후 `READY` heartbeat와 `committed_pose`를 발행한다. Manager와 Output Gate는 초기화 완료 전 최종 Odometry를 차단한다.

GPS가 없거나 경로가 겹쳐 자동 선택이 불가능하면 공통 RViz 상단 **시작 위치 선택**을 누른다. RDDF 위에 마우스를 올리면 중심선에 맞춘 차량 윤곽과 차량 앞쪽 방향, XY/yaw가 표시된다. 클릭하면 미리 본 위치로 초기화한다. 겹치는 지점을 클릭하면 경로·구간·차량 yaw 후보 메뉴가 열린다. 후보에 마우스를 올려 차량 윤곽과 방향을 확인하고 실제 경로를 선택한다. 같은 경로의 자가 교차도 구간으로 구분한다. 진행 경로 콤보로 후보 경로를 미리 제한할 수도 있다. 메뉴에서 Esc는 후보 선택만 취소하고, 지도에서 Esc 또는 선택 취소는 GPS 대기로 돌아간다. 초기화 완료 후에는 클릭으로 주행 중 위치를 바꾸지 않는다.

수동 선택은 실제 좌표를 측량한 GPS fix가 아니므로 `MANUAL_RDDF`로 기록한다. 초기 anchor는 한 번만 등록하고 GPS healthy를 만들지 않는다. `enable_gps_fusion:=false`로 시작한 세션은 GPS가 의도적으로 없는 운용 모드이므로, 이 anchor와 fresh IMU·엔코더 및 정상 Local/Global EKF가 유지되는 동안 `DEAD_RECKONING`, `valid=true`를 출력한다. GPS를 활성화한 세션에서 신호만 끊긴 경우에는 기존 2000초 또는 1000m 제한이 그대로 적용된다. GPS 없는 모드는 절대 위치 보정이 없어 이동할수록 위치·방향 오차가 누적된다.

## 좌표 기준

- `reference.mode=rddf_datum`: origin 위도 37.288731°, 경도 127.1072336°. X 동쪽, Y 북쪽, 미터, yaw 동쪽 0°·반시계 양수.
- RDDF는 제공된 설계 좌표이며 `reference.measured=false`를 유지한다. `manual_datum`의 측량 완료 의미와 구별한다.
- RDDF datum에서 GPS 안테나 좌표에 원점 기준 레버암을 더하지 않고 현재 레버암만 뺀다. 기존 first_fix/manual_datum 동작은 유지한다.
- GPS 게이트는 READY 전 GPS 승인을 만들지 않는다. READY 경계에서 이전 Local 이력·prediction anchor를 버려 Local set_pose 전후의 좌표가 섞이지 않게 한다.
- Local도 초기에는 RDDF XY로 시작하지만 이후 odom 기준 추측 항법이다. Global과 같은 정확도를 의미하지 않는다. 공통 RViz의 live 모드는 `rddf_map`으로 원래 XY를 표시하고 첫 GPS로 다시 평행이동하지 않는다.
- 예전 outputs.bag recorded 모드는 기존 평행이동 표시를 유지한다. 새 RDDF map 기준 outputs를 기록 모드로 열 때는 viewer YAML에 `frame_mode: rddf_map`을 지정한다.

## 실패와 경계

- GPS 미사용 모드: `enable_gps_fusion:=false`에서 현재 위치를 수동 선택한 뒤 IMU·엔코더로 계속 추정한다.
- GPS 활성 상태의 부재·불량: 자동 피팅하지 않고 대기하며, 초기화 후 단절에는 bounded dead reckoning 제한을 적용한다.
- 스냅 거리 초과·서로 다른 경로가 비슷하게 가까움: 임의 경로 선택 거부.
- fresh 정지 속도 없음: WAITING_FOR_STATIONARY. IMU/TF 준비 대기: WAITING_FOR_IMU.
- 서비스 미준비·fresh IMU 대기·EKF 위치/yaw 확인 지연: 준비와 확인을 계속 기다린다. 3초 초과는 지연 안내만 표시하며 FAULT로 고정하지 않는다. 초기화 완료 전 ready는 false다.
- 서비스 호출 실패 또는 확인 중 이동: FAULT, 최종 출력 차단. 전체 localization 재실행 후 다시 선택한다.
- ready heartbeat 중단: Manager와 Output Gate에서 독립 차단. 같은 pose/heartbeat를 반복해 DR 시간을 늘리지 않는다.
- ROS 시각 역행: 초기화 epoch를 바꾸고 새 위치 선택부터 다시 진행한다.
- 차량 제동은 Controller 책임이다. 이 패키지는 Ctrl_cmd를 발행하지 않는다.

## 코드 근거

| 책임 | 파일 |
|---|---|
| 경로 투영·접선·모호성 | `scripts/rddf_initialization_core.py` |
| GPS/수동 선택·두 EKF 확인 | `scripts/rddf_initializer_node.py` |
| 동적 IMU 초기 정합 | `scripts/calibrated_imu_core.py`, `scripts/calibrated_imu_node.py`, `srv/SetInitialHeading.srv` |
| GPS datum·초기화 시각 경계 | `src/odometry_gps_fusion/odometry_gps_fusion.cpp` |
| 초기 anchor·최종 사용 승인 | `src/status_manager/localization_status_manager.cpp`, `src/output_gate/localization_output_gate.cpp` |
| 선택 UI·hover 차량 pose | `scripts/localization_viewer.py` |
| 임계값 | `config/rddf_initialization.yaml`, 기존 `config/status_policy.yaml` |

추가한 통신은 `/mando_localization/internal/initialization/{status,ready,committed_pose,manual_active,manual_request}`와 CalibratedIMU의 `set_initial_heading` 서비스다. 기존 공개 센서·Local/Global·최종 Odometry 토픽 계약은 유지한다. 기존 고정 초기화 실험을 재현할 때만 `start_rddf_initialization:=false`를 사용한다.

## 검증 실행

```bash
source /opt/ros/noetic/setup.bash
cd ~/work/HL-FMA-2026-Localization
source devel/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
catkin_make run_tests_mando_localization
catkin_test_results build/test_results
rostest mando_localization rddf_initialization.test
rostest mando_localization rddf_initialization_manual.test
```

합성 입력·GUI 시험과 실제 차량 재시작 시험은 구분한다. 실제 GPS/PPS 동기화·IMU 절대방향·차량 배치 정확도는 별도 실차 확인이 필요하다.

## 원본 프로젝트의 과거 실행 기록

- Catkin 빌드 성공. 최종 Catkin 결과 269 tests, 0 errors, 0 failures.
- 실제 ROS 노드 기반 GPS/수동 초기화와 최종 출력 검증 통과(합성 입력).
- 실제 embedded RViz hover preview, 클릭 요청, READY 잠금과 recorded 입력 격리 검증 통과.
- 07 일곱번째 bag 첫 25초 센서 3324개를 원래 header 그대로 재생하여 자동 RDDF 1_right 초기화, READY, 최종 Odometry 708개 확인. 실차 검증은 아님.
- 재생에서 TF 시각 요청에 대한 `Using latest instead` 경고가 남았다. 전체 로그와 현재 소스 해시는 `.codex-runtime/rddf-startup-20260913/`에 기록했다.
