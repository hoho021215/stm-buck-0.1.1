/* Native Linux test using the official STM32F303xE register definitions.
 * RAM mappings model configuration registers; no peripheral behavior is
 * simulated. This verifies the read-only preflight, not converter stability. */
#define _GNU_SOURCE
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "buck_hw.h"

static ADC_HandleTypeDef adc;
static TIM_HandleTypeDef timer;
static DMA_HandleTypeDef dma;
static uint32_t hclk_hz, pclk2_hz, sysclk_hz;
static unsigned cases;

uint32_t HAL_RCC_GetHCLKFreq(void) { return hclk_hz; }
uint32_t HAL_RCC_GetPCLK2Freq(void) { return pclk2_hz; }
uint32_t HAL_RCC_GetSysClockFreq(void) { return sysclk_hz; }

enum { PERIPHERAL_BYTES=0x30000, GPIO_BYTES=0x10000, ADC_BYTES=0x10000 };
static unsigned char before_peripheral[PERIPHERAL_BYTES];
static unsigned char before_gpio[GPIO_BYTES];
static unsigned char before_adc[ADC_BYTES];

static void map_region(uintptr_t address, size_t size)
{
    void *mapped=mmap((void *)address,size,PROT_READ|PROT_WRITE,
                     MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED_NOREPLACE,-1,0);
    if (mapped==MAP_FAILED) { perror("mmap STM32 registers"); assert(0); }
}

static void fixture(void)
{
    memset((void *)0x40000000u,0,PERIPHERAL_BYTES);
    memset((void *)0x48000000u,0,GPIO_BYTES);
    memset((void *)0x50000000u,0,ADC_BYTES);
    memset(&adc,0,sizeof adc); memset(&timer,0,sizeof timer);
    memset(&dma,0,sizeof dma);
    hclk_hz=72000000u; pclk2_hz=72000000u; sysclk_hz=72000000u;
    RCC->CR=RCC_CR_HSION|RCC_CR_HSIRDY|RCC_CR_HSEON|RCC_CR_HSERDY|
            RCC_CR_PLLON|RCC_CR_PLLRDY;
    RCC->CFGR=RCC_CFGR_SW_PLL|RCC_CFGR_SWS_PLL|
              RCC_CFGR_PLLSRC_HSE_PREDIV|RCC_CFGR_PLLMUL9;
    RCC->CFGR2=RCC_ADC12PLLCLK_DIV1;
    RCC->CFGR3=RCC_TIM1CLK_HCLK;
    RCC->AHBENR=RCC_AHBENR_GPIOAEN|RCC_AHBENR_GPIOCEN|
                RCC_AHBENR_DMA1EN|RCC_AHBENR_ADC12EN;
    RCC->APB2ENR=RCC_APB2ENR_TIM1EN;

    adc.Instance=ADC1; adc.DMA_Handle=&dma;
    adc.Init.ClockPrescaler=ADC_CLOCK_ASYNC_DIV1;
    adc.Init.Resolution=ADC_RESOLUTION_12B;
    adc.Init.DataAlign=ADC_DATAALIGN_RIGHT;
    adc.Init.ScanConvMode=ADC_SCAN_ENABLE;
    adc.Init.EOCSelection=ADC_EOC_SEQ_CONV;
    adc.Init.LowPowerAutoWait=DISABLE;
    adc.Init.ContinuousConvMode=DISABLE;
    adc.Init.NbrOfConversion=3u;
    adc.Init.DiscontinuousConvMode=DISABLE;
    adc.Init.ExternalTrigConv=ADC_EXTERNALTRIGCONV_T1_TRGO;
    adc.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_RISING;
    adc.Init.DMAContinuousRequests=ENABLE;
    adc.Init.Overrun=ADC_OVR_DATA_PRESERVED;
    ADC1->CFGR=ADC_EXTERNALTRIGCONV_T1_TRGO|
               ADC_EXTERNALTRIGCONVEDGE_RISING|ADC_CFGR_DMACFG;
    ADC1->SQR1=2u|(1u<<ADC_SQR1_SQ1_Pos)|
                  (2u<<ADC_SQR1_SQ2_Pos)|(3u<<ADC_SQR1_SQ3_Pos);
    ADC1->SMPR1=(ADC_SAMPLETIME_19CYCLES_5<<ADC_SMPR1_SMP1_Pos)|
                 (ADC_SAMPLETIME_19CYCLES_5<<ADC_SMPR1_SMP2_Pos)|
                 (ADC_SAMPLETIME_19CYCLES_5<<ADC_SMPR1_SMP3_Pos);
    ADC1_2_COMMON->CCR=0u;
    dma.Instance=DMA1_Channel1;
    dma.Parent=&adc;
    dma.Init.Direction=DMA_PERIPH_TO_MEMORY;
    dma.Init.PeriphInc=DMA_PINC_DISABLE;
    dma.Init.MemInc=DMA_MINC_ENABLE;
    dma.Init.PeriphDataAlignment=DMA_PDATAALIGN_HALFWORD;
    dma.Init.MemDataAlignment=DMA_MDATAALIGN_HALFWORD;
    dma.Init.Mode=DMA_CIRCULAR;
    dma.Init.Priority=DMA_PRIORITY_VERY_HIGH;
    DMA1_Channel1->CCR=DMA_CCR_CIRC|DMA_CCR_MINC|
                       DMA_CCR_PSIZE_0|DMA_CCR_MSIZE_0|DMA_CCR_PL;

    timer.Instance=TIM1;
    timer.Init.Prescaler=0u; timer.Init.CounterMode=TIM_COUNTERMODE_UP;
    timer.Init.Period=1439u; timer.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    timer.Init.RepetitionCounter=0u;
    timer.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_ENABLE;
    TIM1->CR1=TIM_CR1_ARPE;
    TIM1->PSC=0u; TIM1->ARR=1439u; TIM1->RCR=0u;
    TIM1->CCR1=0u; TIM1->CCR4=360u;
    TIM1->CCMR1=TIM_OCMODE_PWM1|TIM_CCMR1_OC1PE;
    TIM1->CCMR2=(TIM_OCMODE_PWM2<<8u)|TIM_CCMR2_OC4PE;
    TIM1->CR2=TIM_TRGO_OC4REF;
    TIM1->BDTR=22u|TIM_BDTR_OSSI|TIM_BDTR_OSSR;

    /* PA0..2 analog; PA7 AF6=CH1N; PC0 AF2=CH1; PC6 GPIO EN low. */
    GPIOA->MODER=(3u<<0u)|(3u<<2u)|(3u<<4u)|(2u<<14u);
    GPIOA->AFR[0]=6u<<28u;
    GPIOC->MODER=(2u<<0u)|(1u<<12u);
    GPIOC->AFR[0]=2u;
    GPIOC->ODR=0u; GPIOC->OTYPER=0u;
}

static void expect(const char *name, bool expected,
                   ADC_HandleTypeDef *a, TIM_HandleTypeDef *t)
{
    ADC_HandleTypeDef adc_before=adc;
    TIM_HandleTypeDef timer_before=timer;
    DMA_HandleTypeDef dma_before=dma;
    memcpy(before_peripheral,(void *)0x40000000u,PERIPHERAL_BYTES);
    memcpy(before_gpio,(void *)0x48000000u,GPIO_BYTES);
    memcpy(before_adc,(void *)0x50000000u,ADC_BYTES);
    bool result=Buck_HardwareConfigValid(a,t);
    if (result!=expected) {
        fprintf(stderr,"FAIL %s: expected %d, got %d\n",name,expected,result);
        assert(result==expected);
    }
    assert(memcmp(before_peripheral,(void *)0x40000000u,PERIPHERAL_BYTES)==0);
    assert(memcmp(before_gpio,(void *)0x48000000u,GPIO_BYTES)==0);
    assert(memcmp(before_adc,(void *)0x50000000u,ADC_BYTES)==0);
    assert(memcmp(&adc_before,&adc,sizeof adc)==0);
    assert(memcmp(&timer_before,&timer,sizeof timer)==0);
    assert(memcmp(&dma_before,&dma,sizeof dma)==0);
    ++cases;
}

#define REJECT(name,mutation) do { fixture(); mutation; \
    expect(name,false,&adc,&timer); } while (0)

int main(void)
{
    map_region(0x40000000u,PERIPHERAL_BYTES);
    map_region(0x48000000u,GPIO_BYTES);
    map_region(0x50000000u,ADC_BYTES);
    fixture(); expect("valid target",true,&adc,&timer);
    fixture(); pclk2_hz=36000000u; RCC->CFGR|=RCC_CFGR_PPRE2_DIV2;
    expect("APB2 divider with timer x2",true,&adc,&timer);
    fixture();
    expect("null ADC",false,NULL,&timer);
    expect("null timer",false,&adc,NULL);
    REJECT("original incomplete profile",adc.DMA_Handle=NULL;
        adc.Init.NbrOfConversion=1u; TIM1->ARR=3599u;
        TIM1->CR1=TIM_COUNTERMODE_CENTERALIGNED1);
    REJECT("null DMA",adc.DMA_Handle=NULL);
    REJECT("null DMA parent",dma.Parent=NULL);
    REJECT("wrong DMA parent",dma.Parent=&timer);
    REJECT("swapped IL/Vin ranks",ADC1->SQR1=2u|
        (3u<<ADC_SQR1_SQ1_Pos)|(2u<<ADC_SQR1_SQ2_Pos)|
        (1u<<ADC_SQR1_SQ3_Pos));
    REJECT("repeated channel",ADC1->SQR1=(ADC1->SQR1&~ADC_SQR1_SQ3)|
        (2u<<ADC_SQR1_SQ3_Pos));
    REJECT("wrong sequence length",ADC1->SQR1&=~ADC_SQR1_L);
    REJECT("slow ADC handle clock",adc.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV4);
    REJECT("slow ADC hardware clock",ADC1_2_COMMON->CCR=ADC12_CCR_CKMODE);
    REJECT("ADC PLL divider",RCC->CFGR2=RCC_ADC12PLLCLK_DIV2);
    REJECT("PLL unavailable",RCC->CR&=~RCC_CR_PLLRDY);
    REJECT("wrong sample time",ADC1->SMPR1&=~ADC_SMPR1_SMP2);
    REJECT("differential input",ADC1->DIFSEL=1u<<1u);
    REJECT("ADC continuous conversion",ADC1->CFGR|=ADC_CFGR_CONT);
    REJECT("wrong trigger edge",ADC1->CFGR&=~ADC_CFGR_EXTEN);
    REJECT("ADC multimode",ADC1_2_COMMON->CCR=1u);
    REJECT("normal DMA",dma.Init.Mode=DMA_NORMAL; DMA1_Channel1->CCR&=~DMA_CCR_CIRC);
    REJECT("DMA word alignment",dma.Init.MemDataAlignment=DMA_MDATAALIGN_WORD);
    REJECT("DMA actual mode mismatch",DMA1_Channel1->CCR&=~DMA_CCR_CIRC);
    REJECT("TIM1 PLL source",RCC->CFGR3|=RCC_CFGR3_TIM1SW_PLL);
    REJECT("wrong core clock",hclk_hz=64000000u);
    REJECT("wrong timer bus clock",pclk2_hz=36000000u);
    REJECT("repetition counter",TIM1->RCR=1u);
    REJECT("updates disabled",TIM1->CR1|=TIM_CR1_UDIS);
    REJECT("center aligned",TIM1->CR1|=TIM_COUNTERMODE_CENTERALIGNED1);
    REJECT("no off-state run",TIM1->BDTR&=~TIM_BDTR_OSSR);
    REJECT("no off-state idle",TIM1->BDTR&=~TIM_BDTR_OSSI);
    REJECT("automatic gate enable",TIM1->BDTR|=TIM_BDTR_AOE);
    REJECT("wrong deadtime",TIM1->BDTR=(TIM1->BDTR&~TIM_BDTR_DTG)|21u);
    REJECT("wrong high-side pin mode",GPIOC->MODER&=~(3u<<0u));
    REJECT("wrong low-side AF",GPIOA->AFR[0]&=~(15u<<28u));
    REJECT("current pin not analog",GPIOA->MODER&=~(3u<<0u));
    REJECT("EN not output",GPIOC->MODER&=~(3u<<12u));
    REJECT("EN open drain",GPIOC->OTYPER|=GPIO_PIN_6);
    REJECT("EN initially high",GPIOC->ODR|=GPIO_PIN_6);
    REJECT("timer clock disabled",RCC->APB2ENR&=~RCC_APB2ENR_TIM1EN);
    printf("hardware configuration: %u read-only MMIO cases passed\n",cases);
    return 0;
}
