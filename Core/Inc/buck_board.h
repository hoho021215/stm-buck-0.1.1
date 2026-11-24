/* [직접 작성/수정하는 파일]
 * 0.0.2에서 이어받은 NUCLEO-F303ZE 클럭, 핀, ADC와 TIM 목표값입니다.
 * CubeMX로 재생성한 결과와 실제 배선을 확인한 뒤 확정합니다. */
#ifndef BUCK_BOARD_H
#define BUCK_BOARD_H

#include "stm32f3xx_hal.h"

/* 배선/센싱/데드타임/보호 검증 전에는 0을 유지합니다.
 * 0이어도 ADC/TIM이 불완전하면 초기화는 F_SETUP으로 종료됩니다. */
#define BUCK_BOARD_VERIFIED       0u

/* 공통 클럭: HSE 8MHz(ST-LINK MCO 경로), PLL x9, HCLK/TIM1 72MHz.
 * 기존 USB 48MHz(72/1.5)를 유지하기 위해 64MHz로 바꾸지 않습니다. */
#define BUCK_CORE_CLOCK_HZ        72000000u
#define BUCK_TIMER_CLOCK_HZ       72000000u

/* TIM1 Up / PSC0 / ARR1439에서 50 kHz 기준으로 동작합니다. */
#define BUCK_PWM_HZ               50000u
#define BUCK_PERIOD_TICKS         (BUCK_TIMER_CLOCK_HZ / BUCK_PWM_HZ)
#define BUCK_TRIGGER_TICKS        (BUCK_TIMER_CLOCK_HZ / 200000u)  /* 5us */
#define BUCK_MARGIN_TICKS         (BUCK_TIMER_CLOCK_HZ / 1000000u) /* 1us */
#define BUCK_DEADTIME_TICKS       22u /* CKD DIV1일 때 약 305.6ns. 튜닝 전 초깃값 */
#define BUCK_FRAME_JITTER_TICKS   (BUCK_CORE_CLOCK_HZ / 400000u) /* ±2.5us, 실제 WCET 확인 필요 */
#define BUCK_ADC_STALE_MS         1u /* HAL tick 단위. >1이면 차단(양자화 포함) */

/* PWM 핀: PC0 / AF2 = TIM1_CH1, PA7 / AF6 = TIM1_CH1N.
 * PWM 핀/AF는 CubeMX에서 생성하며 이 헤더가 GPIO를 설정하지는 않습니다.
 * ADC 정규 변환 순서: PA0=IL, PA1=Vout, PA2=Vin. */
#define BUCK_ADC_VDDA_V           3.300f /* PCC.Vdd와 다름. 실제 VDDA를 측정해 보정 */
#define BUCK_CURRENT_POLARITY     1.0f   /* 정상 IP+ -> IP- 전류에서 양수인지 확인 */

/* [배선 확인 대기] 새 프로필은 PC6를 전용 EN LOW로 설정합니다.
 * 실물에서 전용 EN 배선과 외부 풀다운을 반드시 구성·확인합니다.
 * 센서 open-drain FAULT와 공유된 선을 push-pull HIGH로 구동하지 마세요. */
#define BUCK_EN_GPIO              GPIOC
#define BUCK_EN_PIN               GPIO_PIN_6
#define BUCK_EN_CLOCK_ENABLED()   __HAL_RCC_GPIOC_IS_CLK_ENABLED()

#if BUCK_CORE_CLOCK_HZ != BUCK_TIMER_CLOCK_HZ
#error "DWT timing checks require HCLK equal to TIM1 clock"
#endif
#if BUCK_PWM_HZ != 50000u
#error "Controller coefficients currently assume 50 kHz sampling"
#endif

#endif
