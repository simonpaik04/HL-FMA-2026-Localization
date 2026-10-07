# 센서 장치 별칭 설치

`99-mando-sensors.rules`는 현재 실장치의 vendor/product/serial을 함께 검사해 다음
별칭을 만듭니다.

- Xsens `DB8GG04M` -> `/dev/imu`
- C099 ZED-F9P UART1 `DBT042C1` -> `/dev/mando_gps`

설치 예시는 다음과 같습니다.

```bash
sudo install -m 0644 99-mando-sensors.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=tty
```

`ACTION=="add"` 조건을 사용하므로 이미 연결된 장치는 위 `--action=add` 명령을
실행하거나 한 번 분리 후 다시 연결해야 합니다.

규칙을 설치한 뒤 `readlink -f /dev/imu`와
`readlink -f /dev/mando_gps`로 실제 포트를 확인합니다. 장치를 교체하면 시리얼을
새 장치 값으로 갱신해야 하며, `/dev/ttyUSB*` 번호를 규칙에 직접 고정하지 않습니다.

장치 별칭 재생성은 serial transport가 다시 연결됐다는 뜻일 뿐 위치추정 준비
완료를 의미하지 않습니다. GPS 재연결 뒤에는 다음 순서로 확인합니다.

```text
readlink와 USB identity
→ raw serial 출력
→ GPS driver node
→ NavSatFix status·covariance·timestamp·rate
→ /molit/localization/recovery/state
→ /molit/localization/state와 /valid
```

장기 단절에서는 별칭이나 GPS callback이 돌아온 것만으로 localization-ready라고
판단하지 않습니다. 자세한 승인 조건은
[`docs/gps_quality_and_recovery.md`](../docs/gps_quality_and_recovery.md)를 참고하십시오.
