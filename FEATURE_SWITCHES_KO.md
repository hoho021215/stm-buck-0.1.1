# 제어 기능 ON/OFF와 측정 · 0.1.1

CubeIDE의 **Live Expressions**에서 `buck_feature_request`에 켜 둘 기능 값의 합을 입력합니다. 기동이 수락되면 펌웨어가 요청 마스크와 튜닝값을 그 운전에 적용합니다. [기능 비트 정의](Core/Inc/buck_app.h#L13-L16) · [적용 마스크 저장](Core/Src/buck_app.c#L136-L155) · [제어식 266~285행](Core/Src/buck_app.c#L266-L285)

| 비교할 기능 | 모두 ON | 해당 기능만 OFF | 동작 |
|---|---:|---:|---|
| 입력전압 피드포워드(값 1) | **7** | **6** | ON은 기준전압을 측정 Vin으로, OFF는 `nominal_vin_v`(기본 30 V)로 나눠 기본 듀티를 계산합니다. |
| 듀티 제한에 대한 적분 추적 보정(값 2) | **7** | **5** | ON은 적용 듀티와 요청 듀티의 차이를 `aw_gain_per_s`에 비례해 적분기에 반영합니다. 양쪽 모두 적분항의 범위는 ±`integral_limit`입니다. |
| 정상 운전 듀티 상승률 제한(값 4) | **7** | **3** | ON은 기동 확인 후 명령 듀티의 상승률을 `run_slew_per_s`로 제한합니다. 기본값 1 duty/s은 PWM 한 주기에 듀티 0.00002 증가에 해당합니다. 기동 확인 전에는 양쪽 모두 고정 1 duty/s가 적용됩니다. |

마스크 0은 세 비교 기능을 모두 끕니다. 소프트스타트, 전류에 따른 듀티 상한, 과전압·과전류·역전류 등 고장 차단은 모든 마스크에서 동작합니다. 정상 운전 중 50 ms 지속 저전압은 `buck_debug.uv_warning_latched`에 표시됩니다. [보호와 경고 판정](DESIGN_TUNING_KO.md#voltage)

## CubeIDE에서 A/B 비교

1. 배선과 보호 동작을 확인한 보드에서 시작합니다. 배포 기본값 `BUCK_BOARD_VERIFIED=0`이면 전력단 기동은 차단됩니다. 초기화 후 Live Expressions에서 `buck_debug.state == 0`(DISARMED), `buck_debug.calibrated == 1`을 확인합니다. [보드 설정](Core/Inc/buck_board.h)
2. A 조건으로 `buck_feature_request`에 7을 씁니다. `buck_tuning_request`를 펼쳐 이번 두 시험에서 사용할 이득과 제한값을 확인한 뒤, 모든 입력을 마치면 `buck_arm_request`에 1을 씁니다. `buck_debug.active_features`와 `buck_debug.active_tuning`이 실제 적용값입니다. [튜닝 변수 변경 방법](DESIGN_TUNING_KO.md#measured-pi)
3. `buck_debug.monitoring == 1`일 때 `buck_measure_request`에 1을 한 번 씁니다. 창이 완료되면 `buck_measurement.status == 2`, `samples == 2500`과 해당 마스크가 표시됩니다. `mae_v`, `vout_min_v`, `vout_max_v`, `peak_abs_il_a`, 듀티 범위를 기록합니다. [측정 변수](Core/Inc/buck_measurement.h) · [측정 계산](Core/Src/buck_measurement.c)
4. `buck_arm_request`에 2를 써서 정지하고 명령 처리 및 `state == 0`, Vout ≤0.5 V를 확인합니다. 표의 B 마스크(6·5·3 중 하나)를 입력하고 2~3단계를 반복합니다. 전압·부하와 변화 속도, 튜닝값, 측정 창을 동일하게 맞춰야 기능 하나의 차이를 볼 수 있습니다.
