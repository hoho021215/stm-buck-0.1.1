# 벅 컨버터 시뮬레이션 실행 가이드 · 0.1.1

PC에서 **같은 회로와 같은 입력전압 변화에 대해 피드포워드를 켰을 때와 껐을 때**의 출력전압을 비교합니다. 처음에는 예시값으로 끝까지 실행한 뒤, 인덕터와 출력 커패시터 등의 실측값을 넣어 한 번 더 실행해 보세요. CubeMX 설정, CubeIDE 빌드, 보드 연결 없이 진행할 수 있습니다. [실행 코드 폴더](simulation/)

## 1. 최초 한 번 준비하기

Windows에서는 WSL2의 Ubuntu 터미널을 사용합니다. Ubuntu가 설치되어 있으면 바로 다음 단계로 넘어가세요. 처음 설치할 때는 Windows 관리자 PowerShell에서 아래 명령을 실행하고, Windows가 요청하면 재시작한 뒤 Ubuntu를 여세요.

```powershell
wsl --install -d Ubuntu
```

**아래부터는 Ubuntu 터미널에 입력합니다.** x86-64 Linux에서도 같습니다. 홈 폴더에서 저장소 전체를 내려받으면 실행에 필요한 `Core`, `tests`, `simulation`이 함께 준비됩니다.

```bash
sudo apt update
sudo apt install -y git build-essential python3 python3-matplotlib fonts-noto-cjk
cd ~
git clone https://github.com/hoho021215/stm-buck-0.1.1.git
cd stm-buck-0.1.1
```

이미 복제한 저장소라면 그 폴더에서 `git pull --ff-only`로 업데이트하세요. 아래 명령은 모두 `README.md`가 보이는 저장소 최상위 폴더를 기준으로 합니다.

```bash
python3 tests/run_host_tests.py
bash simulation/build_bridge.sh
```

첫 명령은 인터넷에서 지정 버전의 STM32 HAL·CMSIS를 `tests/.deps/`에 준비하고 PC용 테스트를 실행합니다. 완료 후에는 같은 의존성으로 `python3 tests/run_host_tests.py --offline`을 실행할 수 있습니다. 두 번째 명령은 현재 제어 C 소스와 테스트용 ADC·타이머 인터페이스를 묶어 `simulation/libbuck_sim.so`를 만듭니다. Python이 이 파일을 불러오므로 이 시뮬레이션에는 `tests/` 폴더가 필요합니다.

## 2. 예시 조건 실행하고 그래프 열기

```bash
python3 simulation/run.py simulation/out
python3 simulation/plot.py simulation/out
```

첫 명령이 오류 없이 끝나고 `7`, `6` 두 조건의 통계가 출력되면 두 번째 명령을 실행하세요. 창이 뜨지 않아도 정상입니다. 그래프는 파일로 저장됩니다. WSL에서는 아래 명령으로 Windows 탐색기를 열고 **buck_line_step.png**를 더블클릭하면 됩니다.

```bash
(cd simulation/out && explorer.exe .)
```

| 결과 파일 | 확인할 내용 |
|---|---|
| `parameters.json` | 이 실행에 사용한 소자값, 부하, 스텝 전후 입력전압, PWM 주기와 샘플 시각, 빌드 버전과 기동 시 실제 채택한 여섯 값(`active_tuning`), 마스크별 저전압 경고 이력(`uv_warning_by_mask`) |
| `waveforms.csv` | 기능 마스크, 스텝 기준 시각(µs), 입력전압(V), 회로 모델 출력전압(V)·인덕터 전류(A), 해당 주기에 적용된 듀티(0~1). Excel에서도 열 수 있습니다. |
| `summary.json` | 펌웨어가 모의 ADC 값에서 계산한 50 ms 통계. `vout_max_v`는 창 안의 최대 출력전압, `mae_v`는 목표 15 V와의 평균 절대 오차입니다. `before_step_mean_model_v`는 스텝 직전 5 ms 동안의 회로 모델 평균입니다. |
| `buck_line_step.png`, `buck_line_step.pdf` | 출력전압 전체 응답과 스텝 직후 2 ms 확대. 그래프의 소자값 표시는 같은 결과 폴더의 `parameters.json`을 읽습니다. |

## 3. 내 회로의 실측값 넣기

수정할 곳은 **simulation/run.py 상단의 `PLANT_PARAMETERS`, `VIN_BEFORE_V`, `VIN_AFTER_V`**입니다. Ubuntu에서 `nano simulation/run.py`로 열 수 있습니다. 편집 후 Ctrl+O, Enter로 저장하고 Ctrl+X로 닫으세요. 소자값과 부하는 `PLANT_PARAMETERS`에, 전원 스텝은 바로 아래 두 상수에 넣습니다. [실측값 입력 위치](simulation/run.py)

| 항목 | 기본 예시 | 실제 값의 입력 방법 |
|---|---|---|
| `L` 인덕턴스 | `470e-6` H | 330 µH를 측정했다면 `330e-6` |
| `C` 출력 유효 정전용량 | `220e-6` F | 출력에 병렬로 연결된 커패시터 전체의 유효 용량. 180 µF라면 `180e-6` |
| `dcr` 인덕터 직류저항 | `0.10` Ω | 권선 저항이 85 mΩ이면 `0.085` |
| `esr` 출력 커패시터 등가 직렬저항 | `0.04` Ω | 병렬 조합의 등가 ESR이 28 mΩ이면 `0.028` |
| `load` 저항 부하 | `18.75` Ω | 예시는 15 V에서 0.8 A에 해당합니다. 15 V에서 1 A인 **저항 부하**를 모델링할 때는 `15.0` |
| `VIN_BEFORE_V`, `VIN_AFTER_V` | `29.0`, `30.0` V | 실제 입력전압을 바꾸기 전후의 값. 두 비교 조건 모두 같은 입력을 사용합니다. |

예를 들어 **인덕턴스만 330 µH로 바꿔** 기존 파형과 비교하려면 `L`만 `330e-6`으로 수정해 저장하고 아래 두 줄을 실행하세요. 파이썬 소자값만 바꿨을 때는 C를 다시 빌드할 필요가 없습니다. 다른 측정값도 같은 방식으로 입력합니다.

```bash
python3 simulation/run.py simulation/out_330uH
python3 simulation/plot.py simulation/out_330uH
```

두 폴더의 그래프와 통계를 나란히 비교할 수 있습니다.

### Kp·Ki 바꾸기

같은 모델에서 Kp만 바꿀 때는 아래처럼 실행합니다. 첫 시험은 컴파일된 기본값으로, 두 번째는 `kp=0.004`로 실행합니다. Ki도 바꾸려면 `--ki 0.20`을 붙이세요. 두 선택지는 각각 생략할 수 있습니다.

```bash
python3 simulation/run.py simulation/out_pi_default
python3 simulation/run.py simulation/out_kp004 --kp 0.004
python3 simulation/plot.py simulation/out_kp004
```

CLI 값은 이번 모의실험의 정지 상태에서 펌웨어 요청 변수에 쓰이며 보드나 C 소스의 기본값을 바꾸지 않습니다. `parameters.json`의 `active_tuning`으로 실제 비교한 값을 확인하세요. C 소스의 영구 기본값이나 펌웨어 헤더를 바꾼 경우에는 먼저 `bash simulation/build_bridge.sh`를 다시 실행합니다. 조건마다 새 결과 폴더를 쓰면 실행 실패 후 남은 이전 파일과 혼동하지 않습니다. [보드에서 같은 값 적용하기](DESIGN_TUNING_KO.md#measured-pi)

## 4. 피드포워드 결과 해석하기

두 조건은 독립적으로 기동한 후, 시뮬레이션 시작 3초 시점에 입력을 29 V에서 30 V로 바꿉니다. 마스크 7은 측정 입력전압으로 기본 듀티를 보상합니다. 마스크 6은 `nominal_vin_v`(기본 30 V)를 사용합니다. 적분 보정과 듀티 상승률 제한은 양쪽에서 동일하게 켜져 있습니다. 아래는 이전 0.1.0에서 같은 기본 조건으로 얻은 비교 기준입니다. 이득이나 회로값을 바꿨을 때는 새 결과 폴더의 값을 사용하세요. [기능 비트 설명](FEATURE_SWITCHES_KO.md)

| 기본 소자값, 입력 스텝 후 50 ms | ON · 마스크 7 | OFF · 마스크 6 |
|---|---:|---:|
| 최대 출력전압 | 15.0511 V | 15.7860 V |
| 15 V 기준 평균 절대 오차 | 0.00697 V | 0.37350 V |
| 펌웨어 통계 샘플 수 | 2,500 | 2,500 |

출력전압 곡선은 PWM 주기 20 µs마다 주기 시작 5 µs 뒤에 읽은 회로 모델 값입니다. 펌웨어는 모의 12비트 ADC로 양자화한 전압·전류를 받고, 계산한 듀티는 다음 주기에 적용됩니다. 따라서 CSV 파형의 최대값과 펌웨어 통계의 최대값 사이에 작은 차이가 생깁니다. 주기당 한 점이므로 스위칭 리플의 순간 최대·최소를 판독하는 용도로는 부족합니다.

이 모델은 이상적인 스위치와 전원, 저항 부하, 일정한 L·C·DCR·ESR을 가정합니다. 전압 분압비는 19, ACS71240의 영점은 1.65 V, 감도는 0.132 V/A입니다. MOSFET 손실·데드타임, 실제 센서 대역폭과 잡음, ISR 실행시간 등에 따른 영향은 실제 보드 파형과 비교해야 합니다. 실측 소자값을 입력하면 그 값으로 **예측한 응답**이 생성됩니다. 기본 결과표의 수치는 예시 조건을 재현할 때 사용하세요.

## 5. 파일 역할과 오류 확인

| 파일 | 역할 |
|---|---|
| [simulation/run.py](simulation/run.py) | 실측 소자값을 읽고 7과 6을 각각 실행해 파형·통계·실행 조건을 기록합니다. |
| [simulation/plant.py](simulation/plant.py) | 각 PWM 주기의 스위치 상태, 인덕터 전류와 커패시터 전압을 계산합니다. |
| [simulation/bridge.c](simulation/bridge.c) · [build_bridge.sh](simulation/build_bridge.sh) | 회로 샘플을 PC에서 실행하는 제어 코드의 모의 ADC 입력에 전달하고, 계산된 PWM 듀티와 게이트 상태를 읽습니다. |
| [simulation/plot.py](simulation/plot.py) | 기록한 CSV로 PNG·PDF를 만듭니다. 이전 ZIP의 CSV만 있을 때도 파형을 그리며, 기록이 없는 소자값 표시는 생략합니다. |
| [simulation/check_plant.py](simulation/check_plant.py) | 기본 회로 계산을 별도의 수치 적분 결과 및 정상상태 전압·전하 평형과 대조합니다. 고정된 기준 회로를 검사합니다. |

- **공유 라이브러리 또는 HAL 헤더를 찾을 수 없음** — 저장소 최상위에서 1단계의 테스트 명령이 성공했는지 확인하고 브리지를 빌드하세요. 이미 세 헤더 저장소를 따로 관리한다면 테스트 안내의 `BUCK_HAL_DIR`, `BUCK_DEVICE_DIR`, `BUCK_CMSIS_DIR`를 같은 터미널에서 지정한 뒤 두 명령을 실행할 수 있습니다. [테스트 스크립트의 의존성 설정](tests/run_host_tests.py#L10-L19)
- **실측값 적용 후 `fault` 또는 `run qualification failed`** — 회로값의 단위와 부하, 입력전압을 확인하세요. `fault`는 해당 모델 조건에서 펌웨어 보호가 동작했다는 뜻입니다. `run qualification failed`는 스텝 직전에 정상 운전 감시와 요청 기능 선택을 확인하지 못했다는 뜻입니다. 지속 저전압 경고가 있어도 실행이 끝날 수 있으므로 `parameters.json`의 `uv_warning_by_mask`와 `summary.json`의 `final_faults`를 함께 읽습니다. 실패한 실행에서는 기존 폴더의 결과가 남을 수 있으므로 원인을 확인하고 새 폴더에서 다시 실행하세요.
- **회로 계산만 별도로 검사하고 싶을 때** — 아래 명령을 사용하세요. SciPy는 이 선택 검사에 필요합니다.

```bash
sudo apt install -y python3-scipy
python3 simulation/check_plant.py
```
