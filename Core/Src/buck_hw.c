#include "buck_hw.h"
#include <stddef.h>

static bool pin_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode)
{
    return ((port->MODER >> (2u * pin)) & 3u) == mode;
}

static bool pin_af(GPIO_TypeDef *port, uint32_t pin, uint32_t af)
{
    return ((port->AFR[pin / 8u] >> (4u * (pin % 8u))) & 15u) == af;
}

static bool clocks_valid(void)
{
    uint32_t timer_hz;
    const uint32_t ahb_required = RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN |
                                 RCC_AHBENR_DMA1EN | RCC_AHBENR_ADC12EN;
    if ((RCC->AHBENR & ahb_required) != ahb_required ||
        !(RCC->APB2ENR & RCC_APB2ENR_TIM1EN)) return false;
    if (HAL_RCC_GetHCLKFreq() != BUCK_CORE_CLOCK_HZ ||
        (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL ||
        !(RCC->CR & RCC_CR_PLLRDY)) return false;

    /* F303xE의 TIM1SW=1은 PLL VCO(PLLCLK x2)입니다. HAL의 TIM1
     * 주파수 getter가 이 x2를 반영한다고 가정하지 않습니다. 현재 프로필은
     * APB2 경로만 허용하며 APB 분주 시 타이머의 x2를 직접 반영합니다. */
    if (RCC->CFGR3 & RCC_CFGR3_TIM1SW) return false;
    timer_hz = HAL_RCC_GetPCLK2Freq();
    if (RCC->CFGR & RCC_CFGR_PPRE2) timer_hz *= 2u;
    if (timer_hz != BUCK_TIMER_CLOCK_HZ) return false;

    /* ADC1/2 공통 커널은 비동기 PLL /1, 명목 72MHz, 독립 ADC 모드입니다.
     * SYSCLK=PLL이므로 GetSysClockFreq()로 해당 PLL 출력 주파수를 얻습니다. */
    return HAL_RCC_GetSysClockFreq() == BUCK_CORE_CLOCK_HZ &&
           (RCC->CFGR2 & RCC_CFGR2_ADCPRE12) == RCC_ADC12PLLCLK_DIV1 &&
           (ADC12_COMMON->CCR & (ADC12_CCR_CKMODE | ADC12_CCR_MULTI)) == 0u;
}

static bool pins_valid(void)
{
    uint32_t adc_pulls = (3u << 0u) | (3u << 2u) | (3u << 4u);
    /* 이 검사로 센서/드라이버가 실제 해당 핀에 연결됐는지는 알 수 없습니다. */
    return BUCK_EN_GPIO == GPIOC && BUCK_EN_PIN == GPIO_PIN_6 &&
           pin_mode(GPIOA, 0u, 3u) && pin_mode(GPIOA, 1u, 3u) &&
           pin_mode(GPIOA, 2u, 3u) && !(GPIOA->PUPDR & adc_pulls) &&
           pin_mode(GPIOC, 0u, 2u) && pin_af(GPIOC, 0u, 2u) &&
           !(GPIOC->OTYPER & GPIO_PIN_0) &&
           pin_mode(GPIOA, 7u, 2u) && pin_af(GPIOA, 7u, 6u) &&
           !(GPIOA->OTYPER & GPIO_PIN_7) &&
           pin_mode(GPIOC, 6u, 1u) && !(GPIOC->OTYPER & GPIO_PIN_6) &&
           !(GPIOC->ODR & GPIO_PIN_6);
}

static bool adc_valid(const ADC_HandleTypeDef *adc)
{
    const uint32_t sequence_mask = ADC_SQR1_L | ADC_SQR1_SQ1 |
                                   ADC_SQR1_SQ2 | ADC_SQR1_SQ3;
    const uint32_t sequence = 2u | (1u << ADC_SQR1_SQ1_Pos) |
                              (2u << ADC_SQR1_SQ2_Pos) |
                              (3u << ADC_SQR1_SQ3_Pos);
    const uint32_t sample_mask = ADC_SMPR1_SMP1 | ADC_SMPR1_SMP2 |
                                 ADC_SMPR1_SMP3;
    const uint32_t sample_19cycles5 = ADC_SMPR1_SMP1_2 |
                                     ADC_SMPR1_SMP2_2 | ADC_SMPR1_SMP3_2;
    const uint32_t cfgr_mask = ADC_CFGR_RES | ADC_CFGR_ALIGN | ADC_CFGR_CONT |
                              ADC_CFGR_DISCEN | ADC_CFGR_AUTDLY |
                              ADC_CFGR_JAUTO | ADC_CFGR_OVRMOD |
                              ADC_CFGR_DMACFG | ADC_CFGR_EXTSEL | ADC_CFGR_EXTEN;
    const uint32_t cfgr_expected = ADC_CFGR_DMACFG |
                                  ADC_EXTERNALTRIGCONV_T1_TRGO |
                                  ADC_EXTERNALTRIGCONVEDGE_RISING;
    return adc->Init.ClockPrescaler == ADC_CLOCK_ASYNC_DIV1 &&
           adc->Init.NbrOfConversion == 3u &&
           adc->Init.Resolution == ADC_RESOLUTION_12B &&
           adc->Init.DataAlign == ADC_DATAALIGN_RIGHT &&
           adc->Init.ScanConvMode == ADC_SCAN_ENABLE &&
           adc->Init.EOCSelection == ADC_EOC_SEQ_CONV &&
           adc->Init.LowPowerAutoWait == DISABLE &&
           adc->Init.ContinuousConvMode == DISABLE &&
           adc->Init.DiscontinuousConvMode == DISABLE &&
           adc->Init.DMAContinuousRequests == ENABLE &&
           adc->Init.Overrun == ADC_OVR_DATA_PRESERVED &&
           adc->Init.ExternalTrigConv == ADC_EXTERNALTRIGCONV_T1_TRGO &&
           adc->Init.ExternalTrigConvEdge == ADC_EXTERNALTRIGCONVEDGE_RISING &&
           (ADC1->SQR1 & sequence_mask) == sequence &&
           !(ADC1->DIFSEL & ((1u << 1u) | (1u << 2u) | (1u << 3u))) &&
           (ADC1->SMPR1 & sample_mask) == sample_19cycles5 &&
           (ADC1->CFGR & cfgr_mask) == cfgr_expected &&
           !(ADC1->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART)) &&
           !(ADC1->JSQR & ADC_JSQR_JEXTEN) &&
           !(ADC1->OFR1 & ADC_OFR1_OFFSET1_EN) &&
           !(ADC1->OFR2 & ADC_OFR2_OFFSET2_EN) &&
           !(ADC1->OFR3 & ADC_OFR3_OFFSET3_EN) &&
           !(ADC1->OFR4 & ADC_OFR4_OFFSET4_EN);
}

static bool dma_valid(const DMA_HandleTypeDef *dma)
{
    const uint32_t mask = DMA_CCR_DIR | DMA_CCR_CIRC | DMA_CCR_PINC |
                          DMA_CCR_MINC | DMA_CCR_PSIZE | DMA_CCR_MSIZE |
                          DMA_CCR_MEM2MEM | DMA_CCR_EN;
    const uint32_t expected = DMA_CCR_CIRC | DMA_CCR_MINC |
                              DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0;
    return dma->Instance == DMA1_Channel1 &&
           dma->Init.Mode == DMA_CIRCULAR &&
           dma->Init.Direction == DMA_PERIPH_TO_MEMORY &&
           dma->Init.PeriphInc == DMA_PINC_DISABLE &&
           dma->Init.MemInc == DMA_MINC_ENABLE &&
           dma->Init.PeriphDataAlignment == DMA_PDATAALIGN_HALFWORD &&
           dma->Init.MemDataAlignment == DMA_MDATAALIGN_HALFWORD &&
           (DMA1_Channel1->CCR & mask) == expected;
}

static bool timer_valid(void)
{
    const uint32_t cr1_mask = TIM_CR1_CKD | TIM_CR1_CMS | TIM_CR1_DIR |
                             TIM_CR1_UDIS | TIM_CR1_OPM | TIM_CR1_ARPE;
    return TIM1->PSC == 0u && TIM1->ARR == BUCK_PERIOD_TICKS - 1u &&
           TIM1->RCR == 0u && TIM1->CCR4 == BUCK_TRIGGER_TICKS &&
           (TIM1->CR1 & cr1_mask) == TIM_CR1_ARPE &&
           !(TIM1->SMCR & (TIM_SMCR_SMS | TIM_SMCR_ECE)) &&
           (TIM1->CCMR1 & (TIM_CCMR1_CC1S | TIM_CCMR1_OC1M |
                            TIM_CCMR1_OC1PE)) ==
               (TIM_OCMODE_PWM1 | TIM_CCMR1_OC1PE) &&
           (TIM1->CCMR2 & (TIM_CCMR2_CC4S | TIM_CCMR2_OC4M |
                            TIM_CCMR2_OC4PE)) ==
               ((TIM_OCMODE_PWM2 << 8u) | TIM_CCMR2_OC4PE) &&
           !(TIM1->CCER & (TIM_CCER_CC1P | TIM_CCER_CC1NP)) &&
           !(TIM1->CR2 & (TIM_CR2_OIS1 | TIM_CR2_OIS1N | TIM_CR2_CCPC)) &&
           (TIM1->CR2 & TIM_CR2_MMS) == TIM_TRGO_OC4REF &&
           (TIM1->BDTR & (TIM_BDTR_DTG | TIM_BDTR_OSSI | TIM_BDTR_OSSR |
                          TIM_BDTR_AOE | TIM_BDTR_MOE)) ==
               (BUCK_DEADTIME_TICKS | TIM_BDTR_OSSI | TIM_BDTR_OSSR);
}

bool Buck_HardwareConfigValid(ADC_HandleTypeDef *adc, TIM_HandleTypeDef *tim)
{
    if (adc == NULL || tim == NULL || adc->Instance != ADC1 ||
        tim->Instance != TIM1 || adc->DMA_Handle == NULL) return false;
    /* __HAL_LINKDMA는 양방향 연결을 만듭니다. HAL의 DMA 완료/오류
     * 콜백은 DMA.Parent를 ADC 핸들로 역참조하므로 한쪽만 연결하면 안 됩니다. */
    if (adc->DMA_Handle->Parent != adc) return false;
    /* 먼저 클럭을 확인하여 꺼진 주변장치의 레지스터 접근을 피합니다. */
    return clocks_valid() && pins_valid() && adc_valid(adc) &&
           dma_valid(adc->DMA_Handle) && timer_valid();
}
