/* [직접 작성] STM32F303ZE 임시 목표 설정의 읽기 전용 검사입니다. */
#ifndef BUCK_HW_H
#define BUCK_HW_H

#include <stdbool.h>
#include "stm32f3xx_hal.h"
#include "buck_board.h"

/* MX_*_Init() 완료 뒤, ADC 보정/DMA 시작 전에 호출합니다.
 * 레지스터를 변경하지 않습니다. 실제 배선, 외부 EN 풀다운, 게이트 파형,
 * L/C 및 독립 하드웨어 보호가 정상인지 보증하는 함수는 아닙니다.
 * 현재 프로필: PC0/PA7 PWM, PA0/1/2 ADC, 전용 PC6 EN. */
bool Buck_HardwareConfigValid(ADC_HandleTypeDef *adc, TIM_HandleTypeDef *tim);

#endif
