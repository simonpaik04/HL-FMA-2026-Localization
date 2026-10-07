# 실제 IMU + 키보드 엔코더 테스트

연결된 IMU의 원본 자세·각속도·가속도·시각을 그대로 사용하고, `W`로
가상의 Arduino `SerialFeedBack`을 생성한다. 현재 `encoder_to_twist_adapter`,
IMU 정규화·정지 yaw 고정, Local/Global EKF와 공통 RViz를 실행한다.
차량 제어 명령은 발행하지 않는다.

아래 예시는 IMU 드라이버 master가 `11351`, 테스트 master가 `11361`인 경우다. 실제 드라이버 master 주소에 맞춰 `source_master_uri`를 지정한다.
기존 IMU 실행을 유지한 상태에서 다음 명령을 사용한다.

```bash
cd ~/work/HL-FMA-2026-Localization
source /opt/ros/noetic/setup.bash
source devel/setup.bash
ROS_MASTER_URI=http://localhost:11361 roslaunch -p 11361 mando_localization keyboard_imu_test.launch \
  source_master_uri:=http://localhost:11351
```

- `IMU + Encoder Test | Hold W` 창을 클릭하고 **W를 누르는 동안 1 m/s 전진**한다.
- **W를 떼거나 Space/Esc를 누르거나 다른 창을 선택하면 정지**한다.
- 방향은 **실제 IMU를 회전**시켜 변경한다. 기존 코드대로 정지 중에는
  Calibrated yaw가 고정되므로, 이동 중 회전을 확인하려면 W를 누른 채 IMU를 돌린다.
- 실 IMU가 끊기거나 오래된 데이터만 오면 가상 전진을 해제한다. 재연결 후 W를 다시 누른다.
- 엔코더 창을 닫거나 launch에서 Ctrl+C를 누르면 테스트가 종료된다.
  원래 master의 IMU 드라이버와 장치의 VRU/AHS 설정은 변경하지 않는다.

엔코더는 20 Hz로 발행하며 `alive`가 매번 증가한다. 전진 `speed`는 m/s,
`encoder`는 최근 100 ms 증분이다. 이 테스트의 가상 1 tick은 0.01 m로 가정하여
1 m/s에서 10 tick을 넣는다. 실제 차량의 tick 보정값은 아니다.
현재 운영 설정은 `meter_per_tick_enabled: false`여서 속도는 `speed` 필드로 계산한다.

시작 위치·방향은 RDDF **`1_right`, index=0**으로 수동 지정한다.
Local·Global 모두 **X=37.006811 m, Y=26.908526 m, yaw=161.469964°**로 시작한다.
`keyboard_imu_test_initial_pose.yaml`을 두 EKF와 CalibratedIMU에 적용한다.
시작 순간 실 IMU의 방향을 RDDF 진행 방향에 맞추고, 이후 방향 변화는 실 IMU를 따른다.
원본 IMU 메시지와 장치 설정은 바꾸지 않는다.

GPS를 생성하지 않고 자동 RDDF 위치 선택도 실행하지 않는다. 지정한 출발점에서
상대 이동을 추정하는 테스트이며 GNSS 방향 보정은 없다.
Global도 IMU·엔코더만 사용하므로 Local과 겹쳐 보일 수 있다. 원래 상태 판단은
유지하며, 절대 위치가 확보되지 않았다는 상태를 정상 GPS 수신으로 바꾸지 않는다.

테스트용 `frame_mode: rddf_map`은 GPS 기준점을 기다리지 않고 **노드 메시지의
위치와 yaw를 그대로 표시**한다. RDDF도 함께 표시한다. 기존 현장·재생 뷰어의
기본 설정은 유지한다. 화면의 초록은 Local, 빨강은 Global이다.
경로가 화면 밖으로 나가면 공통 RViz의 화면 맞춤 버튼을 사용한다.
