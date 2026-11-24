# CubeIDE에 0.1.1 연결하기

0.1.1의 기능 선택과 측정은 기존 ADC1·DMA·TIM1을 사용합니다. 현재 `buck_converter` 프로젝트의 `.ioc`와 CubeMX가 생성한 프로젝트 파일을 사용하세요. 기능별 ON/OFF 값과 측정 절차는 [0.1.1 사용법](README.md)에 있습니다.

## 1. Core에 복사할 파일

| 현재 프로젝트 | `Core/Src`에 넣을 파일 | `Core/Inc`에 넣을 파일 |
|---|---|---|
| 0.1.0 사용 중 | `buck_app.c` 교체 | `buck_app.h` 교체, `buck_tuning.h` 추가 |
| 새 프로젝트 또는 0.0.2에서 업그레이드 | `buck_app.c`, `buck_hw.c`, `buck_supervisor.c`, `buck_measurement.c` (4개) | `buck_app.h`, `buck_tuning.h`, `buck_board.h`, `buck_hw.h`, `buck_platform.h`, `buck_supervisor.h`, `buck_measurement.h` (7개) |

현재 프로젝트의 CubeMX 생성 파일과 보드에서 검증한 설정을 보관하고 위 사용자 모듈을 적용합니다. 이미 사용자 모듈을 수정했다면 차이를 대조해 반영하세요. 기동·폴링·정지 연결이 있는 프로젝트에서는 아래의 호출이 한 번씩만 있는지 확인합니다. 튜닝을 위해 CubeMX의 ADC·DMA·TIM 설정을 바꿀 필요가 없습니다. [펌웨어 소스](Core/Src/) · [헤더](Core/Inc/) · [새 튜닝 변수와 적용 순서](DESIGN_TUNING_KO.md#measured-pi)

## 2. Core/Src/main.c에 직접 넣을 위치

아래 줄 번호는 GitHub 0.0.2 통합 참고본(커밋 `c8b6320`)의 번호입니다. CubeMX가 사용자 PC에서 생성한 파일은 옵션에 따라 줄 번호가 달라집니다. CubeIDE에서 파일을 열어 **Ctrl+F**로 표의 `USER CODE BEGIN` 표식을 찾고 그 안에 넣으세요. 같은 참고 파일에서 번호로 이동할 때는 **Ctrl+L**을 씁니다.

| 참고본 줄 | 찾을 표식 | 넣을 내용 |
|---|---|---|
| [22–23줄](https://github.com/hoho021215/stm-buck-0.0.2/blob/c8b6320a3db2230ddb80b11abbcf6bd11fab6271/Core/Src/main.c#L22-L23) | `USER CODE BEGIN Includes` | `#include "buck_app.h"` |
| [117–121줄](https://github.com/hoho021215/stm-buck-0.0.2/blob/c8b6320a3db2230ddb80b11abbcf6bd11fab6271/Core/Src/main.c#L117-L121) | `USER CODE BEGIN 2` | GPIO·DMA·TIM1·ADC 등 초기화가 모두 끝난 뒤 `Buck_Init();` 한 번 |
| [130–133줄](https://github.com/hoho021215/stm-buck-0.0.2/blob/c8b6320a3db2230ddb80b11abbcf6bd11fab6271/Core/Src/main.c#L130-L133) | `USER CODE BEGIN 3` | `while (1)` 안에 `Buck_Poll();` 한 줄. ADC 갱신 감시를 위해 반복문에서 긴 대기를 피합니다. |
| [577–591줄](https://github.com/hoho021215/stm-buck-0.0.2/blob/c8b6320a3db2230ddb80b11abbcf6bd11fab6271/Core/Src/main.c#L577-L591) | `USER CODE BEGIN 4` | 아래 PC13 정지 콜백. 이미 같은 콜백이 있으면 그 안에 PC13 분기를 합칩니다. |
| [599–606줄](https://github.com/hoho021215/stm-buck-0.0.2/blob/c8b6320a3db2230ddb80b11abbcf6bd11fab6271/Core/Src/main.c#L599-L606) | `USER CODE BEGIN Error_Handler_Debug` | 기존 `__disable_irq();` 다음, 정지 루프 `while (1)` 직전에 `Buck_EmergencyOff();` 추가 |

`USER CODE BEGIN 4` 안에 넣을 PC13 버튼 처리 예시입니다.

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)
    {
        /* [직접 추가] 정지 명령과 게이트 차단 사이에 ADC 인터럽트가 끼어들지 않게 합니다. */
        uint32_t irq_state = __get_PRIMASK();
        __disable_irq();
        buck_arm_request = 2u;
        Buck_EmergencyOff();
        __set_PRIMASK(irq_state);
    }
}
```

`buck_app.c`가 ADC half/full/error 콜백을 구현하며, `Buck_Init()`이 ADC·DMA 및 타이머 시작 순서를 관리합니다. 기존 실습 코드에 ADC 콜백이나 PWM 자동 시작 호출이 있다면 중복을 제거하세요.

## 3. Core/Src/stm32f3xx_it.c에 직접 넣을 위치

먼저 `USER CODE BEGIN Includes` 안에 `#include "buck_app.h"`를 추가합니다. [참고본 24–25줄](https://github.com/hoho021215/stm-buck-0.0.2/blob/c8b6320a3db2230ddb80b11abbcf6bd11fab6271/Core/Src/stm32f3xx_it.c#L24-L25)

다음 표의 각 첫 사용자 구역에 같은 두 줄을 넣습니다. 예외 발생 시 인터럽트 재진입을 막고 게이트를 차단합니다.

```c
__disable_irq();
Buck_EmergencyOff();
```

| 함수 | 두 줄을 넣을 표식 | 참고본 위치 |
|---|---|---|
| `NMI_Handler` | `USER CODE BEGIN NonMaskableInt_IRQn 0` | [78–87줄](https://github.com/hoho021215/stm-buck-0.0.2/blob/c8b6320a3db2230ddb80b11abbcf6bd11fab6271/Core/Src/stm32f3xx_it.c#L78-L87) |
| `HardFault_Handler` | `USER CODE BEGIN HardFault_IRQn 0` | [95–104줄](https://github.com/hoho021215/stm-buck-0.0.2/blob/c8b6320a3db2230ddb80b11abbcf6bd11fab6271/Core/Src/stm32f3xx_it.c#L95-L104) |
| `MemManage_Handler` | `USER CODE BEGIN MemoryManagement_IRQn 0` | [112–121줄](https://github.com/hoho021215/stm-buck-0.0.2/blob/c8b6320a3db2230ddb80b11abbcf6bd11fab6271/Core/Src/stm32f3xx_it.c#L112-L121) |
| `BusFault_Handler` | `USER CODE BEGIN BusFault_IRQn 0` | [129–138줄](https://github.com/hoho021215/stm-buck-0.0.2/blob/c8b6320a3db2230ddb80b11abbcf6bd11fab6271/Core/Src/stm32f3xx_it.c#L129-L138) |
| `UsageFault_Handler` | `USER CODE BEGIN UsageFault_IRQn 0` | [146–155줄](https://github.com/hoho021215/stm-buck-0.0.2/blob/c8b6320a3db2230ddb80b11abbcf6bd11fab6271/Core/Src/stm32f3xx_it.c#L146-L155) |

위 다섯 핸들러는 차단 후 정지 루프에 머무릅니다. CubeMX가 만든 정지 루프는 그대로 사용하세요. `NMI_Handler`에 루프가 없으면 `USER CODE BEGIN NonMaskableInt_IRQn 1` 안에 `while (1) { }`을 넣습니다.

## 4. 빌드와 연결 확인

1. Project Explorer에서 `buck_converter`를 선택하고 **F5**로 새 파일을 표시합니다. **Project → Properties → C/C++ Build → Settings → MCU GCC Compiler → Optimization**에서 실험할 구성(Debug 또는 Release)을 **-O2**로 맞추고 fast-math를 끕니다. 최종 빌드 플래그를 Console에서 확인합니다.
2. **프로젝트 우클릭 → Build Project**를 누릅니다. 새로 복사했다면 Console에서 `buck_measurement.c`가 컴파일되는지, 링크 후 선택한 구성 폴더에 `buck_converter.elf`가 생기는지 확인하세요. A/B 시험은 같은 빌드의 디버거 변수로 전환합니다.
3. 현재 펌웨어가 검사하는 ADC Rank 1·2·3은 IN1·IN2·IN3이고 세 샘플 시간은 19.5 cycles입니다. 설정 검사에서 `BUCK_FAULT_SETUP`(512)이 뜨면 생성된 `main.c`의 ADC 상승 에지, 비동기 ADC 클럭 PLL /1, TIM1 deadtime 22 ticks, PC6 초기 Low부터 대조하세요. DMA의 HAL 핸들 연결 및 인터럽트 전달은 다음 두 호출에서 확인할 수 있습니다. [MSP의 __HAL_LINKDMA](https://github.com/hoho021215/stm-buck-0.0.2/blob/c8b6320a3db2230ddb80b11abbcf6bd11fab6271/Core/Src/stm32f3xx_hal_msp.c#L126) · [DMA IRQ의 HAL_DMA_IRQHandler](https://github.com/hoho021215/stm-buck-0.0.2/blob/c8b6320a3db2230ddb80b11abbcf6bd11fab6271/Core/Src/stm32f3xx_it.c#L225)
4. 첫 빌드는 `BUCK_BOARD_VERIFIED=0`으로 진행합니다. 1은 전용 EN 배선, 게이트 비중첩·센싱·보호 등을 보드에서 검증한 뒤에 사용하는 기동 허용값입니다. [보드 검증 플래그](Core/Inc/buck_board.h#L11)
