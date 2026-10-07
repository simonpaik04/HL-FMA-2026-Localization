# Localization 문서

[프로젝트 README](../../../README.md)에서 개요·설치·실행을 먼저 확인한 뒤 필요한 문서로 이동합니다. 설정값의 기준은 패키지 `config/` YAML입니다.

| 문서 | 내용 |
|---|---|
| [아키텍처](architecture.md) | 노드 책임, 데이터 흐름, TF·출력 소유권 |
| [설정 가이드](configuration.md) | 설정 파일 역할과 실행 모드 |
| [RDDF 초기화](rddf_startup.md) | GPS 자동 선택, 뷰어 수동 선택, 두 EKF 적용 확인 |
| [GPS 품질·복구](gps_quality_and_recovery.md) | 승인 조건과 장기 단절 복구 |
| [센서 시각](sensor_timing.md) | clock_ready, 지연 GPS 정합 |
| [CalibratedIMU](calibrated_imu.md) | 방향 보정과 정지 yaw 제약 |
| [상태와 복구](status_and_recovery.md) | Supervisor 우선순위와 최종 출력 승인 |
| [TF](tf_frames.md) | 차량 좌표계, 장착값, 검증 |
| [키보드 IMU 테스트](keyboard_imu_test.md) | 실제 IMU·가상 엔코더 입력 |
| [운영 참고](operations.md) | 기존 현장용 명령·기록·뷰어 상세 |

기존 현장 문서에는 원본 작업공간의 실험·장치 설정이 포함됩니다. 현재 독립 저장소의 검증은 [검증 기록](../../../docs/VALIDATION.md), 추출·외부 의존성은 [출처](../../../docs/PROVENANCE.md)를 기준으로 확인합니다.
