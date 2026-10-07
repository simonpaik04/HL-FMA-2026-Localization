# 독립 저장소 검증 기록

검증 날짜: 2026-10-08 (Asia/Seoul)

## 환경

- Ubuntu 20.04.6 LTS, ROS1 Noetic, 시스템 Python 3
- 원본 차량 작업공간과 다른 checkout에서 별도의 `build/`·`devel/` 생성
- 이 저장소의 `mando_localization`과 `erp42_msgs`를 빌드
- ROS·외부 라이브러리는 이 PC에 설치된 패키지를 사용. 깨끗한 새 OS에서 rosdep 설치까지 재현한 검증은 아님

## 실행과 결과

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j4 -l4 -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
catkin_make -j4 -l4 run_tests_mando_localization
catkin_make -j4 -l4 run_tests_mando_localization_nosetests_test.test_configuration.py
catkin_test_results build/test_results
```

- 독립 Catkin 빌드: 성공
- 최종 `catkin_test_results` 출력: **271 tests, 0 errors, 0 failures, 0 skipped**
- C++ gtest 9개 target, Python nosetests 10개 target, ROS rostest 13개 구성의 결과를 확인
- 최초 설정 테스트가 원본 `interfaces/vehicle_interface/erp42_msgs` 경로를 참조해 실패함. 이 저장소의 `src/erp42_msgs` 경로로 수정한 뒤 해당 target을 재실행하여 24개 테스트 전부 통과. 최종 집계는 수정 후 결과임

271은 Catkin 결과 집계 수치이며 서로 다른 실차 시나리오 271개라는 의미가 아닙니다. 테스트 입력은 합성 ROS 메시지·시각과 테스트 fixture를 사용합니다.

추가 확인:

- 센서 드라이버·GUI를 끈 `bringup.launch`를 `roslaunch --nodes`로 해석하여 node 구성을 확인
- launch·rostest XML 파싱, shell wrapper의 `bash -n`, Markdown 내부 파일 링크 확인
- 원본 working-tree와 비교: 핵심 C++·Python 융합·상태·복구 구현은 동일. README·문서, wrapper 작업공간 경로, `launch.sh`의 ROS 진입점, 지원 메시지 경로 테스트만 독립 배치에 맞게 변경
- 원본 작업공간의 기존 수정·추가 파일 상태는 유지

## 범위와 남은 확인

시뮬레이션 시각 시작 시 EKF update-rate 경고, TF 준비 전 lookup 경고와 `Using latest instead` 경고가 발생했습니다. 테스트 assertion은 통과했으며, 경고가 없는 런타임이라고 주장하지 않습니다.

이번 정리에서 실제 센서 연결·차량 주행, GUI 조작, 원본 rosbag 재생, 수정 u-blox 드라이버 빌드, 하드웨어 PPS 동기화, 측량 ground truth 기반 위치 정확도는 검증하지 않았습니다. RDDF 초기화와 지연 측정 처리의 소프트웨어 동작을 확인한 결과입니다.

원본 상세 문서에 남아 있는 과거 bag·GUI 검증 결과는 원본 프로젝트의 기록입니다. 이번 독립 저장소의 실행 결과와 구분합니다.
