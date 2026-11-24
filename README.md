# STM32 동기식 벅 컨버터 0.1.1

NUCLEO-F303ZE용 30 V 입력, 15 V / 1 A 출력, 50 kHz 동기식 벅 펌웨어입니다. 이 버전에서는 전압 PI 등 여섯 값을 `buck_tuning_request`에서 조정합니다. CubeIDE에서 영구 기본값을 바꾸거나, 같은 펌웨어로 실험하면서 다음 기동에 사용할 값을 입력할 수 있습니다. [CubeIDE 통합 방법](INTEGRATION_KO.md) · [설계·튜닝 가이드](DESIGN_TUNING_KO.md)

## Kp와 Ki 바꾸기

- **전원을 다시 켜도 사용할 기본값**은 `Core/Src/buck_app.c`에서 `volatile BuckTuning buck_tuning_request`를 검색해 초기값 블록을 수정하세요. 예를 들어 Kp만 비교하려면 `.kp = 0.005f`를 `.kp = 0.004f`로 바꾸고 다시 빌드·프로그래밍합니다. Ki의 단위는 duty/(V·s)이며 제어기가 100 µs 주기를 곱합니다. [기본값 소스 41~48행](Core/Src/buck_app.c#L41-L48)
- **한 번의 실험**에서는 정지 상태(`buck_debug.state == 0`)에서 CubeIDE의 **Live Expressions**에 `buck_tuning_request.kp`와 `buck_tuning_request.ki`를 추가해 원하는 값을 먼저 씁니다. 보드 기동 검증·영점 보정이 끝난 상태에서 `buck_arm_request = 1`을 마지막에 한 번 쓰세요. `buck_debug.active_tuning`에서 채택된 값을 확인합니다. 운전 중에는 현재 적용값을 유지하며, 요청값을 바꾸면 다음 기동에서 읽습니다. RAM으로 입력한 값은 리셋 후 펌웨어 기본값으로 돌아갑니다. [기능 전환·측정 절차](FEATURE_SWITCHES_KO.md)

| 요청 변수의 필드 | 의미 | 기본값 |
|---|---|---:|
| `kp` | 출력전압 오차의 비례 이득 (duty/V) | 0.005 |
| `ki` | 출력전압 오차의 적분 이득 (duty/(V·s)) | 0.25 |
| `nominal_vin_v` | 입력전압 피드포워드 OFF에서 계산에 쓰는 공칭 입력 (V) | 30 |
| `integral_limit` | 적분 출력의 대칭 상·하한 크기 (duty) | 0.25 |
| `aw_gain_per_s` | 듀티 제한 차이를 적분기에 되돌리는 추적 계수 (1/s) | 50 |
| `run_slew_per_s` | 기동 검증 후 듀티 상승률 제한 기능이 켜졌을 때의 상승 한도 (duty/s) | 1 |

입력전압 피드포워드, 적분기 추적 보정, 정상 운전 듀티 상승률 제한의 ON/OFF는 기존과 같이 `buck_feature_request`로 선택합니다. 기본 마스크는 7입니다. 실측 L·C·DCR·ESR·부하는 회로 시뮬레이션의 입력값입니다. [실측값 입력과 PC 실행](SIMULATION_KO.md)

## 어느 차단이 달라졌나?

**기동 검증을 통과한 RUN에서 출력 저전압이 50 ms 지속되면 경고를 기록하고 제어를 계속합니다.** 부하 스텝 중 전압이 떨어진 뒤 회복하는 과정을 관찰할 수 있습니다. 목표 15 V에서는 12 V 미만에서 저전압 구간에 들어가고, 12.75 V 초과에서 벗어납니다. `buck_debug.uv_active`는 현재 저전압 상태, `buck_debug.uv_samples`는 그 지속 표본 수, `buck_debug.uv_warning_latched`는 이번 기동에서 경고 기준에 도달한 적이 있는지를 보여줍니다. 출력이 회복되거나 정지해도 경고 이력은 남고 초기화 또는 다음 기동 수락 때 지워집니다.

지속 과부하로 저전압이 계속돼도 전류가 기존 차단 임계에 도달하지 않으면 스위칭이 이어질 수 있습니다. 부하가 저전압에서 반드시 정지해야 하는 설계에서는 이 동작 차이를 고려하세요. 입력전압 범위 검사, 출력 과전압, 과전류·역전류, 기동 실패·프리바이어스, 전류 듀티 상한, ADC·PWM 타이밍 감시, 비상 게이트 차단은 계속 적용됩니다. 배포 기본값은 `BUCK_BOARD_VERIFIED=0`입니다. [이전 차단 코드 원문](archive/README.md) · [보드 설정](Core/Inc/buck_board.h)

## 적용 및 검증

0.1.0에서 이전할 때는 새 `buck_tuning.h`를 포함하여 이 버전의 `buck_*` 사용자 파일을 같은 버전으로 맞춥니다. 기존 CubeMX 생성 코드의 `Buck_Init()`·`Buck_Poll()` 연결을 그대로 사용합니다. ADC와 TIM 설정을 추가로 생성할 필요는 없습니다. [프로젝트 파일·USER CODE 위치](INTEGRATION_KO.md)

```bash
python3 tests/run_host_tests.py
bash simulation/build_bridge.sh
python3 simulation/run.py simulation/out
python3 simulation/plot.py simulation/out
```

모델에서 Kp만 바꾸는 예시는 `python3 simulation/run.py simulation/out_kp004 --kp 0.004`입니다. `parameters.json`에 두 마스크에 실제로 적용된 조정값이 기록됩니다.

게시 전 호스트 앱 33개 시나리오와 하드웨어 설정 모의 검사 40개를 통과했습니다. 기본 모델의 파형 CSV와 통계 JSON은 0.1.0과 바이트 단위로 일치했습니다. 참조 프로젝트의 시작 코드·링커 스크립트와 결합한 Cortex-M4F ELF도 링크했습니다. 실물 보드의 파형·인터럽트 실행시간·부하 동작은 미검증입니다.

이 저장소는 0.1.0 커밋 `876db1b4186849ae9688c47f8ed3d5e323f306a6`에서 분리했습니다. 이전 제어·보호 코드를 빌드에서 제외되는 확장자로 보관했습니다. [기준 소스](https://github.com/hoho021215/stm-buck-0.1.0/tree/876db1b4186849ae9688c47f8ed3d5e323f306a6) · [보관 폴더](archive/0.1.0/)
