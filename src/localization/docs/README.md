# Localization 문서

[프로젝트 README](../../../README.md)에서 개요·설치·실행을 먼저 확인합니다.
현재 snapshot의 값과 모드 차이는 [최종 설정](final_configuration.md)이 기준이며 세부 값은 `config/` YAML에서 확인합니다.

| 문서 | 내용 |
|---|---|
| [최종 설정](final_configuration.md) | GPS 유무·DR 예산·시각 정책·반복 yaw 보정·RDDF 연속 추적 |
| [Archify](../../../docs/architecture/README.md) | 그림·HTML·소스 근거와 시각 검증 |
| [아키텍처](architecture.md) | 노드 책임, 데이터 흐름, TF·출력 소유권 |
| [설정 가이드](configuration.md) | 설정 파일 역할과 실행 모드 |
| [RDDF 초기화](rddf_startup.md) | GPS 자동 선택, 수동 원본 경로 선택과 두 EKF 적용 확인 |
| [GPS 품질·복구](gps_quality_and_recovery.md) | GPS 승인과 장기 단절 복구 |
| [센서 시각](sensor_timing.md) | 호스트·센서 진단과 지연 측정 정합 |
| [CalibratedIMU](calibrated_imu.md) | GNSS 반복 정합과 정지·경로 진입 yaw 처리 |
| [상태와 복구](status_and_recovery.md) | Supervisor 우선순위와 출력 검사 |
| [TF](tf_frames.md) | 차량 기준점과 장착 설정 |
| [운영·인터페이스](operations.md) | 현장 wrapper·뷰어·현재 RDDF 메시지 상세 |

현재 [검증 기록](../../../docs/VALIDATION.md)과 [출처·의존성](../../../docs/PROVENANCE.md)을 함께 확인합니다.
