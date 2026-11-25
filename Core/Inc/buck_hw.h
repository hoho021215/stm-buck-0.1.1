/* [직접 작성] STM32F303ZE 임시 목표 설정의 읽기 전용 검사입니다. */
#ifndef BUCK_HW_H
#define BUCK_HW_H

#include <stdbool.h>
#include "stm32f3xx_hal.h"
#include "buck_board.h"

/* 주변장치 초기화 후 Buck_Init()이 이 함수를 호출해 ADC·DMA 시작 여부를 결정합니다.
 * true이면 핸들과 검사 대상 레지스터가 기대값과 일치합니다. false이면 Buck_Init()이 SETUP(512)을 기록합니다.
 * PC0/PA7의 PWM, PA0~2의 ADC, PC6의 전용 EN 설정을 읽습니다.
 * 실제 EN 풀다운, 게이트 비중첩, 센서 배선 및 전력단 동작은 보드에서 확인해야 합니다. */
bool Buck_HardwareConfigValid(ADC_HandleTypeDef *adc, TIM_HandleTypeDef *tim);

#endif
