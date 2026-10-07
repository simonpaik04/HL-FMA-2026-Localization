# TF 프레임과 장착 설정

차량 기준점은 뒷바퀴 축 중심(`rear_axle_center`)입니다. `base_link`의 +X는 전방,
+Y는 좌측, +Z는 위입니다. 위치·각도·각속도는 m·rad·rad/s이며,
설정 파일의 `rotation_rpy_deg`만 degree, roll/pitch/yaw 순서입니다.

## 현재 TF 소유자와 설정

기준 파일은 [tf_configuration.yaml](../config/tf_configuration.yaml)입니다.

| Transform | 유일 발행자 | 현재 설정 |
|---|---|---|
| `map -> odom` | Global EKF | 동적 TF |
| `odom -> base_link` | Local EKF | 동적 TF |
| `base_link -> imu_link` | `localization_static_tf_publisher` | XYZ=(0.20, 0, 0)m, RPY=(0, 0, 0)°, verified·활성 |
| `base_link -> gps_link` | 같은 노드 | XYZ=(0.65, 0, 0)m, RPY=(0, 0, 0)°, unmeasured·비활성 |
| `base_link -> laser_link` | 같은 노드 | XYZ=(1.05, 0, 0)m, RPY=(180, 0, 180)°, measured·활성 |

IMU는 2026-09-06 사용자 확인 장착값입니다. LiDAR 위치는 2026-09-05,
위아래 뒤집힘과 후방 장착 방향은 2026-09-07 사용자 확인값입니다.
GPS의 미측정 축 0은 자리표시자이며 검증된 6DoF 장착값이 아닙니다.

Xsens는 `pub_transform=false`입니다.
다른 URDF나 정적 TF 노드가 같은 child를 발행한다면 중복 소유를 해소해야 합니다.
공통 뷰어의 `localization_debug_world -> localization_debug`는 독립 표시용 TF입니다.

## 설정 검증

정적 TF 발행기는 schema·차량 축·유한 수치를 검사하고 중복 child, self edge,
잘못된 frame과 순환 구조를 거부합니다. `dynamic_transforms`는 값이 아닌 소유권 선언입니다.
launch의 `expected_local_owner`, `expected_global_owner`와 YAML의 `owner_node`가
일치해야 합니다. EKF 노드명을 바꿀 때 두 설정을 함께 갱신합니다.

정적 TF는 `enabled: true`이고 `calibration_state`가 `measured` 또는 `verified`인
항목만 발행합니다. 이 검증은 설정 일관성을 검사하며 ROS master 전체의 실제
중복 TF 발행자를 자동 탐색하는 기능은 아닙니다.

## 센서별 주의점

- IMU 장착 회전은 TF에 한 번 반영합니다. CalibratedIMU의 GNSS yaw 정합은
  별도 월드 방향 보정이며 장착 TF를 수정하지 않습니다.
- GPS gate는 [gps_reference.yaml](../config/gps_reference.yaml)의 평면 레버암
  X=0.65m, Y=0m와 GPS 측정 시각의 Local yaw를 사용합니다. 비활성 GPS 정적 TF를
  읽지 않습니다. 현재 레버암은 provisional이며 정밀 장착 측정을 대체하지 않습니다.
- LiDAR 장착 회전은 TF에만 반영합니다. 드라이버 `inverted: false`와 원시 scan
  각도·ranges·timestamp를 유지합니다. 센서 축은 차량에서 +X→−X, +Y→+Y, +Z→−Z입니다.
- 과거 bag 재생 시에는 기록 당시 장착 설정과 현재 설정을 구분합니다. 원본 bag을
  보존하고 필요한 TF 선택은 재생 구성에서 명시합니다.

장착을 바꾸면 같은 차량 기준점에서 위치·회전을 측정하고 `source`, `measured_at`을
기록합니다. 정지 중 중력축과 정면 장애물, 직진·좌회전 중 yaw 부호를 확인합니다.

## 런타임 확인

```bash
rosrun tf tf_echo map odom
rosrun tf tf_echo odom base_link
rosrun tf tf_echo base_link imu_link
rosrun tf tf_echo base_link laser_link
rosrun tf tf_monitor
rosrun tf view_frames
```

각 동적 TF의 발행자가 하나인지, timestamp가 갱신되는지, 센서 frame에서 필요한
목표 frame까지 연결되는지 확인합니다. GPS 정적 TF는 현재 비활성이므로 해당 edge의
부재 자체를 GPS 직접 투영 경로의 실패로 판단하지 않습니다.
