# Localization Archify 그림

최종 원본 `Team-Stier/HL-FMA-1-5-2026`의 commit `fba9a5c12382a61a2711fe5685fe839e20c804c3`에서 코드·launch 근거를 검증했습니다. 책임별 개요이며 세부 센서 상태 입력·Local 이력 feedback과 RDDF 통신선은 [상세 아키텍처](../../src/localization/docs/architecture.md)·[현재 설정](../../src/localization/docs/final_configuration.md)에서 설명합니다.

![아키텍처 개요](../../assets/localization-architecture.png)

[독립 HTML 뷰어](localization.html)를 다운로드해 브라우저로 열면 확대·검색·Light/Dark·Export를 사용할 수 있습니다. GitHub 파일 보기에서는 HTML을 실행하지 않습니다. 한국어 설명이며 고정 Viewer UI와 HTML lang는 영어 fallback입니다.

- [JSON 원본](localization.architecture.json)
- [전달 검증](delivery.json)
- [브라우저 검증](localization.visual-check.json)
- [실행 화면 출처](runtime-image-provenance.json)

```text
diagram_type: architecture
output: localization.html
specification_sha256: 57e62e19138244cb3f6fdba83636311e64c100b315db52129ed3ab3e0adc62c5
artifact_sha256: 5a28666558261352b3ebaf9331bb64fa71d83b860e81afbcc8524dd10c20bd4a
validation: 9/9 showcase, 0 errors, 0 warnings
browser_evidence: passed
visual_review: passed
correction_rounds: 0
```

브라우저 자동 확인은 1440×900, 1600×1000, 1920×1080, 2048×1320에서 가로·세로 overflow가 없음을 확인했습니다. Codex는 최종 생성 이미지의 2048×1320 Light와 1440×900 Dark를 열어 라벨·연결선·노드·카드와 전체 여백을 별도로 검토했습니다. 자동 receipt의 `visualReview: pending`과 이 이미지 검토 기록은 구분합니다.

이 결과는 그림 검증입니다. Localization 런타임·정확도 결과는 [검증 기록](../VALIDATION.md)을 참고합니다. 대표 실행 이미지는 2026-09-14 기존 구현의 rosbag 재생 기록이며 이번 최종 소스의 새 캡처가 아닙니다.
