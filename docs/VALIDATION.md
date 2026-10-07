# 최종 Localization 독립 검증

날짜: 2026-10-08 (Asia/Seoul)
원본: `Team-Stier/HL-FMA-1-5-2026`, commit `fba9a5c12382a61a2711fe5685fe839e20c804c3`.

## 환경과 실행

Ubuntu 20.04.6 LTS, ROS1 Noetic, 시스템 Python 3. ROS·외부 라이브러리는 이 PC에 설치된 패키지를 사용했습니다. 새 OS에서 rosdep 설치까지 재현한 검증은 아닙니다.

`mando_localization`, 최종 `erp42_msgs`, `planning_interfaces`를 독립 checkout에서 빌드했습니다.
기존 버전의 테스트 결과를 보관한 뒤 결과 디렉터리를 비우고 새 테스트를 실행했습니다.

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j4 -l4 -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
catkin_make -j4 -l4 run_tests_mando_localization
catkin_test_results build/test_results
```

- **빌드 성공**
- 최종 Catkin 집계: **290 tests, 0 errors, 5 failures, 0 skipped**
- C++ gtest 9개 target, Python nosetests 10개 target, ROS rostest 13개 구성 실행
- 290은 Catkin 집계이며 독립적인 실차 시나리오 수가 아닙니다.
- 예전 분리본의 271개 무실패 결과는 이 최종 snapshot의 결과로 사용하지 않습니다.

## 실패 항목

실패한 기대값을 바꾸어 통과 처리하거나 원본의 동작을 되돌리지 않았습니다. 다음은 이번 실행에서 관찰한 테스트와 현재 코드의 불일치입니다. 단순 기대값 노후화인지, 유지해야 할 동작의 회귀인지에 대한 판정·수정은 이 추출 작업에 포함하지 않았습니다.

| 항목 | 테스트 기대 | 관찰 결과 |
|---|---|---|
| 설정 `test_dead_reckoning_is_bounded` | DR 200초 | 최종 YAML은 2000초 |
| GPS RDDF 초기화 | 초기화 후 방향 재설정 서비스 거부 | `accepted=true` 반환 |
| 수동 RDDF 초기화 | 초기화 후 방향 재설정 서비스 거부 | `accepted=true` 반환 |
| RDDF 공개 토픽 | 활성 경로에서 먼 위치에 `TOO_FAR` | `MATCHED_OFF_ROUTE`, `matched=true`와 기존 경로 유지 |
| startup gate | 테스트 진행 후 `FAULT`, `valid=false` | 대기 종료 시 `DEAD_RECKONING`, `valid=true` |

근거 테스트는 [설정](../src/localization/test/test_configuration.py), [RDDF 초기화](../src/localization/test/test_rddf_initialization_ros.py), [현재 경로](../src/localization/test/test_rddf_tracking_ros.py), [startup gate](../src/localization/test/test_startup_gate.py)입니다.

실행 로그와 XML 결과는 이 PC의 `/home/paik/morai-artifacts/localization-final-export-20261008/`에 보관했습니다. 대용량 로컬 로그를 저장소에 올리지는 않았습니다.

## 추가 확인과 범위

- 원본과 비교하여 핵심 C++·Python 구현, YAML, launch·메시지 계약이 일치하는지 확인했습니다. 독립 workspace 경로·문서 변경은 [출처](PROVENANCE.md)에 구분했습니다.
- 센서 드라이버와 GUI를 끈 bringup 구성을 `roslaunch --nodes`로 확인했습니다.
- launch·rostest XML, shell wrapper 구문, README·추가 문서의 내부 파일 링크를 검사했습니다.
- RDDF Provider 로더가 포함된 원본·테스트 RDDF의 카탈로그를 읽는지 확인했습니다.
- Archify는 9/9 showcase 검사, 브라우저 4개 화면 크기와 실제 이미지 검토를 통과했습니다. [그림 검증](architecture/README.md)

시뮬레이션 시각 시작의 EKF update-rate 경고와 `Using latest instead` TF 경고가 남았습니다. 경고가 없는 런타임이라고 주장하지 않습니다.

실제 센서 연결·차량 주행, 최종 소스의 GUI 조작·실제 bag 재생, 수정 GPS 드라이버 빌드, PPS 하드웨어 동기화와 측량 ground truth RMSE는 이번 작업에서 검증하지 않았습니다. 대표 실행 화면은 과거 실제 rosbag 재생 기록이며 새 최종 소스를 실행한 증거가 아닙니다.
