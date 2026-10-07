# 최종 snapshot의 설정과 모드

기준: `Team-Stier/HL-FMA-1-5-2026`의 `fba9a5c12382a61a2711fe5685fe839e20c804c3`.
기존 상세 문서의 과거 실험 설명과 현재 운영값을 구분하기 위한 안내입니다. YAML과 구현이 기준입니다.

## GPS 사용과 수동 anchor

`enable_gps_fusion:=true`가 기본입니다. GPS가 설정된 뒤 unhealthy가 되면
[상태 정책](../config/status_policy.yaml)의 DR 예산 2000초 또는 1000m 중 먼저 초과하는 기준을 사용합니다.
이 예산은 실제 위치 정확도를 검증한 수치가 아닙니다.

GPS를 명시적으로 끄고 수동/RDDF anchor를 적용하면 healthy Local motion과 Global 출력으로
`DEAD_RECKONING`, `valid=true`를 결정합니다. 이 모드는 활성 절대 소스가 0인 별도 분기이며
GPS 단절 예산을 적용하지 않습니다. Output Gate는 이 모드에서 위치 variance 제한 대신
`max_covariance_diagonal` 상한을 사용합니다. 유한성·음수 공분산·frame·freshness 등의 계약 검사는 유지합니다.
[상태 평가 코드](../src/status_manager/localization_state_evaluator.cpp), [출력 구성](../launch/safety_and_tf.launch)이 기준입니다.

## IMU 방향과 호스트 시각

- [IMU 보정 설정](../config/imu_heading_calibration.yaml)은 `one_shot: false`로 GNSS 직진 정합을 반복합니다.
- 설정된 S자·동적 장애물·T 주차 경로 진입에서는 RDDF 방향을 반영하는 보정 경로도 사용합니다. GNSS 기반 관측 보정과 구분합니다.
- [시각 설정](../config/time_sync.yaml)의 `enforce_host_clock_ready: false`로 호스트 NTP 검사 결과가 운행 입력을 차단하지 않습니다.
- 호스트 offset 허용은 3초, 센서별 진단과 GPS 자체 측정 시각·Local 이력 정합은 유지됩니다.
- 초기화의 3초 confirmation 값은 지연 안내 기준입니다. 초과해도 결과 확인을 계속 기다립니다.

## RDDF 제공과 연속 추적

[Provider](../scripts/rddf_route_provider.py)는 19개 원본 RDDF를 `/route/map`으로 제공합니다.
[Tracker](../scripts/rddf_tracking_core.py)는 공개 그룹 15개의 현재 경로를 발행합니다.
주차 진입·진출은 `T_left`, `T_right`, `parallel_left`, `parallel_right`로 묶고 원본 이름은 별도 필드로 보존합니다.

첫 매칭 뒤에는 현재 원본 경로를 유지하며, 연결된 다음 원본 경로와 차량 위치·방향을 확인해 전환합니다.
주차·종료 분기는 `/mission/rddf_successor` 요청을 이용합니다. State Manager 자체는 이 독립 저장소에 포함하지 않습니다.

최초 매칭의 거리 제한과 활성 경로 유지 동작은 다릅니다. 이미 활성화된 경로에서 멀어져도
`matched=true`, `reason=MATCHED_OFF_ROUTE`로 현재 경로를 유지할 수 있습니다.
따라서 `matched=true`는 설계 경로 위에 있다는 거리 보증이 아닙니다. `nearest.distance_m`과 reason을 함께 봅니다.
연결 전환 결과는 `ROUTE_TRANSITION`입니다.

[test_data](../test_data/)에는 홍익대 운동장 좌표로 옮긴 RDDF 사본이 있습니다. 실제 운동장 경계를 측량한 새 경로가 아닙니다.
각 사본의 README와 reference·viewer 설정을 함께 사용합니다.

## 검증의 구분

원본에 남아 있는 과거 GUI·bag 결과는 당시 기록입니다. 이번 독립 snapshot의 빌드·테스트는
[검증 기록](../../../docs/VALIDATION.md)을 기준으로 합니다. 원본 테스트와 현재 동작이 불일치하는 항목도 기록하며,
통과 결과를 만들기 위해 핵심 동작이나 테스트 기대값을 바꾸지 않습니다.
