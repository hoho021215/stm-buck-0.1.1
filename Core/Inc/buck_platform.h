/* 내부 사용: Cortex-M IRQ 저장/복구. 호스트 시험에서만 별도 모의 함수를 씁니다. */
#ifndef BUCK_PLATFORM_H
#define BUCK_PLATFORM_H
#include <stdint.h>
#include "stm32f3xx_hal.h"

#if defined(BUCK_HOST_TEST)
#if defined(__arm__) || defined(__thumb__)
#error "BUCK_HOST_TEST must never be enabled in STM32 firmware"
#endif
uint32_t Buck_Test_IrqSave(void);
void Buck_Test_IrqRestore(uint32_t mask);
static inline uint32_t Buck_IrqSave(void) { return Buck_Test_IrqSave(); }
static inline void Buck_IrqRestore(uint32_t mask) { Buck_Test_IrqRestore(mask); }
#else
static inline uint32_t Buck_IrqSave(void)
{
    uint32_t mask=__get_PRIMASK();  /* 예외 처리 중 호출될 수도 있으므로 기존 인터럽트 마스크를 보관합니다. */
    __disable_irq();
    return mask;
}
static inline void Buck_IrqRestore(uint32_t mask) { __set_PRIMASK(mask); }  /* 호출 전 마스크를 복원하므로 이미 차단돼 있던 IRQ는 계속 차단됩니다. */
#endif
#endif
