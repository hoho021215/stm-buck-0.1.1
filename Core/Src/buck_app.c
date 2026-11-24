/* [직접 작성하는 파일] Core/Src/buck_app.c
 * CubeMX 자동 생성 파일이 아니므로 이 파일 전체를 프로젝트에 추가합니다.
 * main.c에는 README.md가 안내하는 기존 USER CODE 구역에
 * #include "buck_app.h", Buck_Init(), Buck_Poll()을 추가합니다.
 * HAL ADC 콜백은 이 파일에 있으므로 main.c에 중복 작성하지 않습니다.
 * 클럭/핀/ADC/PWM의 MX_*_Init() 설정은 CubeMX에서 생성합니다. */
#include "stm32f3xx_hal.h"
#include "buck_app.h"
#include "buck_board.h"
#include "buck_hw.h"
#include "buck_measurement.h"
#include "buck_platform.h"
#include "buck_supervisor.h"
#include <math.h>
#include <string.h>

/* 보드 의존 값은 buck_board.h에서 관리합니다. 기동 전에 실제 레지스터를 검사합니다. */
#define VDDA BUCK_ADC_VDDA_V
#define CURRENT_POLARITY BUCK_CURRENT_POLARITY
#define PERIOD BUCK_PERIOD_TICKS
#define TRIGGER BUCK_TRIGGER_TICKS
#define MARGIN BUCK_MARGIN_TICKS
#define VOLTAGE_LOOP_DIVIDER 5u
#define VOLTAGE_LOOP_PERIOD_S ((float)VOLTAGE_LOOP_DIVIDER / (float)BUCK_PWM_HZ)
#define DUTY_CEILING 0.75f
#define VIN_MIN_V 23.0f
#define VIN_MAX_V 36.0f
#define STARTUP_SLEW_PER_S 1.0f
/* CubeMX가 정의한 핸들을 참조합니다. 여기에서 다시 정의하지 않습니다.
 * 초기화 함수가 main.c에 있거나 adc.c/tim.c로 나뉘어 있어도 동일합니다. */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
enum { DISARMED, PRECHARGE, SOFTSTART, RUN, FAULT };
enum { F_ADC=1, F_ZERO=2, F_TIMING=4, F_VIN=8, F_OVP=16,
       F_OC=32, F_REVERSE=64, F_PREBIAS=128, F_STARTUP=256, F_SETUP=512 };
/* 디버거 명령: 1=시작, 2=정지, 3=고장 해제(재시작하지 않음) */
volatile uint32_t buck_arm_request;
volatile uint32_t buck_feature_request = BUCK_FEATURE_ALL;
/* 자주 조정하는 값: 전원을 다시 켜도 쓸 기본값을 여기서 바꾸고 빌드합니다.
 * 실험 중에는 같은 이름의 RAM 변수를 DISARMED에서 바꾼 뒤 기동을 요청합니다. */
volatile BuckTuning buck_tuning_request = {
    .kp = 0.005f,
    .ki = 0.25f,
    .nominal_vin_v = 30.0f,
    .integral_limit = 0.25f,
    .aw_gain_per_s = 50.0f,
    .run_slew_per_s = 1.0f
};
volatile BuckDebug buck_debug;
volatile BuckFaultSnapshot buck_fault_snapshot;
/* CubeMX: DMA Circular/Halfword, ADC Rank 순서 IL,Vout,Vin.
 * 3개씩 두 프레임을 받아 Half/Full 완료 콜백으로 처리합니다.
 * DMA가 접근하지 못하는 CCM RAM에 이 버퍼를 배치하면 안 됩니다. */
static volatile uint16_t adc_frame[6];
static struct {
    uint32_t state, faults, count, phase, boot, checked;
    uint32_t active_features;
    uint32_t zero_sum, zero_n, calibrated, previous, previous_valid, expect;
    uint32_t sampling_started, watchdog_seen, watchdog_fresh_ms, precharge_enabled;
    uint32_t sample_valid, raw_adc[3];
    float sample_vin, sample_vout, sample_il;
    BuckSupervisor supervisor;
    BuckTuning tuning;
    float run_slew_step;
    uint16_t zero_min, zero_max;
    float zero, ref, integ, prop, aw, cap, duty;
} b;

static float clamp(float x, float lo, float hi)
{ return x < lo ? lo : (x > hi ? hi : x); }

void Buck_EmergencyOff(void)
{
    /* Error_Handler는 주변장치 초기화 전에도 호출될 수 있습니다.
     * 아직 클럭이 꺼져 있는 주변장치에는 접근하지 않습니다.
     * 리셋/초기화 전 OFF 상태는 PC6의 외부 EN 풀다운이 보장해야 합니다. */
    if (BUCK_EN_CLOCK_ENABLED()) {
        BUCK_EN_GPIO->BSRR = (uint32_t)BUCK_EN_PIN << 16u;
    }
    if (__HAL_RCC_TIM1_IS_CLK_ENABLED()) {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
    }
    BuckMeasurement_Abort();
    /* CCR1=0만으로 정지하면 상보 저측 MOSFET은 계속 켜질 수 있습니다. */
}

/* 최초 고장만 기록합니다. 이후 ADC/타이밍 오류가 추가돼도 덮어쓰지 않습니다.
 * valid는 마지막에 설정합니다. 디버거에서 sample_valid도 함께 확인합니다. */
static void trip(uint32_t fault)
{
    Buck_EmergencyOff();
    if (!buck_fault_snapshot.valid) {
        buck_fault_snapshot.first_fault=fault;
        buck_fault_snapshot.state_at_fault=b.state;
        buck_fault_snapshot.adc_frame=buck_debug.frames;
        buck_fault_snapshot.dwt_cycles=DWT->CYCCNT;
        buck_fault_snapshot.timer_count=__HAL_RCC_TIM1_IS_CLK_ENABLED() ? TIM1->CNT : 0u;
        buck_fault_snapshot.sample_valid=b.sample_valid;
        for (unsigned i=0u; i<3u; ++i) buck_fault_snapshot.raw_adc[i]=b.raw_adc[i];
        buck_fault_snapshot.vin=b.sample_vin;
        buck_fault_snapshot.vout=b.sample_vout;
        buck_fault_snapshot.il=b.sample_il;
        buck_fault_snapshot.reference=b.ref;
        buck_fault_snapshot.duty=b.duty;
        buck_fault_snapshot.integral=b.integ;
        buck_fault_snapshot.duty_cap=b.cap;
        buck_fault_snapshot.valid=1u;
    }
    b.faults|=fault; b.state=FAULT; b.boot=0u; b.duty=0.0f;
    b.supervisor.pgood=false;
    buck_debug.state=FAULT; buck_debug.faults=b.faults;
    buck_debug.duty=0.0f; buck_debug.pgood=0u; buck_debug.monitoring=0u;
}

static void reset_control(void)
{
    b.count=0u; b.phase=0u; b.boot=0u; b.checked=0u;
    b.precharge_enabled=0u;
    b.ref=0.0f; b.integ=0.0f; b.prop=0.0f; b.aw=0.0f;
    b.cap=DUTY_CEILING; b.duty=0.0f;
    BuckSupervisor_Init(&b.supervisor);
}

/* 1 V 오차에서 한 번의 P/I 보정 크기와 한 번의 추적 보정량을 제한합니다.
 * 모든 상·하한 비교에 NaN과 무한대는 통과할 수 없습니다. 이 범위는 안정성 보증 범위가 아닙니다. */
static bool tuning_valid(const BuckTuning *t)
{
    return t->kp >= 0.0f && t->kp <= DUTY_CEILING &&
           t->ki >= 0.0f && t->ki <= DUTY_CEILING / VOLTAGE_LOOP_PERIOD_S &&
           t->nominal_vin_v >= VIN_MIN_V && t->nominal_vin_v <= VIN_MAX_V &&
           t->integral_limit >= 0.0f && t->integral_limit <= DUTY_CEILING &&
           t->aw_gain_per_s >= 0.0f && t->aw_gain_per_s <= 1.0f / VOLTAGE_LOOP_PERIOD_S &&
           t->run_slew_per_s > 0.0f && t->run_slew_per_s <= DUTY_CEILING * BUCK_PWM_HZ;
}

static void request_start(float vin, float vout)
{
    /* 요청 변수는 한 번만 읽습니다. 검증한 복사본만 제어 경로에서 사용합니다. */
    const BuckTuning candidate = buck_tuning_request;
    const uint32_t features = buck_feature_request;
    if (!BUCK_BOARD_VERIFIED) return;
    if (features & ~BUCK_FEATURE_ALL) { trip(F_SETUP); return; }
    if (!tuning_valid(&candidate)) { trip(BUCK_FAULT_TUNING); return; }
    if (vin < VIN_MIN_V || vin > VIN_MAX_V) { trip(F_VIN); return; }
    if (vout > 0.5f) { trip(F_PREBIAS); return; }

    reset_control();
    TIM1->CCR1=0u;
    b.tuning = candidate;
    b.run_slew_step = candidate.run_slew_per_s / (float)BUCK_PWM_HZ;
    b.active_features = features;
    buck_debug.active_features = features;
    buck_debug.active_tuning = candidate;
    buck_debug.uv_warning_latched = 0u;
    b.state=PRECHARGE; b.boot=52u;
}

static bool deadline_ok(uint32_t entered, uint32_t counter)
{
    return counter>=TRIGGER && counter<PERIOD-MARGIN &&
           DWT->CYCCNT-entered < PERIOD-counter-MARGIN;
}

/* ADC DMA 인터럽트에서 20us마다 실행합니다. printf/HAL_Delay는 금지합니다.
 * 원시 전류는 표본별 소프트웨어 차단에 사용하고, PGOOD/UV는 별도 저속 조건을 확인합니다. */
static void frame(uint32_t half)
{
    uint32_t entered=DWT->CYCCNT, counter=TIM1->CNT, offset=half*3u;
    uint32_t request=buck_arm_request, frame_faults=0u;
    uint16_t ri=adc_frame[offset], rv=adc_frame[offset+1u], rin=adc_frame[offset+2u];
    float vi=(float)ri*(VDDA/4095.0f);
    float vin=(float)rin*(VDDA*19.0f/4095.0f);
    float vout=(float)rv*(VDDA*19.0f/4095.0f);
    float il=CURRENT_POLARITY*(vi-b.zero)/0.132f;
    if (request != 0u) buck_arm_request=0u; /* 읽은 명령만 소비합니다. */
    ++buck_debug.frames;
    b.sample_valid=1u;
    b.raw_adc[0]=ri; b.raw_adc[1]=rv; b.raw_adc[2]=rin;
    b.sample_vin=vin; b.sample_vout=vout; b.sample_il=il;

    if (b.previous_valid) {
        uint32_t interval=entered-b.previous;
        if (interval<PERIOD-BUCK_FRAME_JITTER_TICKS ||
            interval>PERIOD+BUCK_FRAME_JITTER_TICKS) frame_faults|=F_TIMING;
    }
    b.previous=entered; b.previous_valid=1u;
    if (!deadline_ok(entered,counter)) frame_faults|=F_TIMING;
    if (half!=b.expect) frame_faults|=F_ADC;
    b.expect=half ? 0u : 1u;
    if (ri<16u || ri>4079u || rv>4079u || rin>4079u) frame_faults|=F_ADC;
    if (frame_faults) trip(frame_faults);

    if (request==2u) {
        Buck_EmergencyOff(); reset_control();
        if (b.state!=FAULT) b.state=DISARMED;
    } else if (request==3u && (b.state==FAULT || b.state==DISARMED) &&
               !frame_faults && vin>=VIN_MIN_V && vin<=VIN_MAX_V && vout<17.0f &&
               (!b.calibrated || (il<1.8f && il>-0.4f))) {
        /* 고장 해제는 새 ADC/타이밍 오류가 없는 프레임에서만 허용합니다.
         * 고장 기록은 이 명시적 명령으로만 해제하며 자동 기동하지 않습니다. */
        Buck_EmergencyOff(); reset_control(); b.faults=0u; b.state=DISARMED;
        memset((void *)&buck_fault_snapshot,0,sizeof buck_fault_snapshot);
    }

    /* 양쪽 게이트 OFF, 외부 전류도 0인 조건에서 2048표본으로 영점을 보정합니다. */
    if (!b.calibrated && b.state==DISARMED) {
        Buck_EmergencyOff(); b.zero_sum+=ri;
        if (ri<b.zero_min) b.zero_min=ri;
        if (ri>b.zero_max) b.zero_max=ri;
        if (++b.zero_n==2048u) {
            b.zero=(float)b.zero_sum*(VDDA/(4095.0f*2048.0f));
            if (b.zero<1.45f || b.zero>1.85f ||
                (float)(b.zero_max-b.zero_min)*(VDDA/4095.0f)>0.15f) {
                b.zero_n=0u; b.zero_sum=0u; b.zero_min=4095u; b.zero_max=0u;
                trip(F_ZERO);
            } else b.calibrated=1u;
            il=CURRENT_POLARITY*(vi-b.zero)/0.132f;
            b.sample_il=il;
        }
    }
    if (request==1u && b.state==DISARMED && b.calibrated) {
        request_start(vin, vout);
    }
    if (b.state==PRECHARGE || b.state==SOFTSTART || b.state==RUN) {
        if (!isfinite(vin) || !isfinite(vout) || !isfinite(il)) trip(F_ADC);
        else if (vin<VIN_MIN_V || vin>VIN_MAX_V) trip(F_VIN);
        else if (vout>=17.0f) trip(F_OVP);
        else if (il>=1.8f) trip(F_OC);
        else if (il<=-0.4f) trip(F_REVERSE);
    }

    /* 기동 검증 후 50 ms 지속된 출력 저전압은 진단에 남기고 계속 제어합니다.
     * 입력/센서 이상과 과전압·과전류 등 즉시 차단 경로는 별도로 유지합니다. */
    bool monitor_ready=(b.state==RUN && b.checked!=0u);
    if (BuckSupervisor_Step(&b.supervisor,15.0f,vout,monitor_ready))
        buck_debug.uv_warning_latched = 1u;
    if (monitor_ready && !b.supervisor.input_valid) trip(F_ADC);

    if (b.state==PRECHARGE) {
        if (vout>0.5f) trip(F_PREBIAS);
        else if (b.precharge_enabled && !(TIM1->BDTR&TIM_BDTR_MOE)) trip(F_SETUP);
        else if (!deadline_ok(entered,counter)) trip(F_TIMING);
        else {
            /* 2주기 대기 + 저측만 약 1ms ON: bootstrap 충전. */
            if (b.boot<=50u && !b.precharge_enabled) {
                TIM1->BDTR|=TIM_BDTR_MOE;
                BUCK_EN_GPIO->BSRR=BUCK_EN_PIN;
                b.precharge_enabled=1u;
            }
            if (--b.boot==0u) b.state=SOFTSTART;
        }
    } else if (b.state==SOFTSTART || b.state==RUN) {
        if (!(TIM1->BDTR&TIM_BDTR_MOE)) trip(F_SETUP);
        if (b.state!=FAULT) {
            if (b.count<50000u) ++b.count;
            b.ref=b.count<15000u ? 15.0f*((float)b.count/15000.0f) : 15.0f;
            if (b.count>=15000u) b.state=RUN;
            if (!b.checked && b.count>=50000u) {
                if (vout<13.5f) trip(F_STARTUP);
                else b.checked=1u;
            }
            /* 전류에 따른 듀티 상한은 모든 조합에서 적용합니다. */
            if (il>1.15f) b.cap-=20.0f/50000.0f;
            else if (il<1.05f) b.cap+=0.5f/50000.0f;
            b.cap=clamp(b.cap,0.0f,DUTY_CEILING);
            if (b.phase==0u) {
                float error=b.ref-vout;
                b.integ=clamp(b.integ + b.aw + b.tuning.ki * error * VOLTAGE_LOOP_PERIOD_S,
                              -b.tuning.integral_limit, b.tuning.integral_limit);
                b.aw=0.0f; b.prop=b.tuning.kp * error;
            }
            if (++b.phase==VOLTAGE_LOOP_DIVIDER) b.phase=0u;
            /* 피드포워드 OFF는 이번 기동에 수락된 공칭 Vin을 사용합니다. */
            float feedforward_vin = (b.active_features & BUCK_FEATURE_VIN_FF) ? vin : b.tuning.nominal_vin_v;
            float requested=b.ref/feedforward_vin+b.prop+b.integ;
            float applied=clamp(requested,0.0f,b.cap);
            /* 기동 확인 전에는 항상 상승률을 제한하고, 확인 후에는 RUN_SLEW 선택을 따릅니다. */
            if (!b.checked || (b.active_features & BUCK_FEATURE_RUN_SLEW)) {
                float step = b.checked ? b.run_slew_step : STARTUP_SLEW_PER_S / (float)BUCK_PWM_HZ;
                applied=clamp(applied,0.0f,b.duty+step);
            }
            /* ON이면 이번 듀티 제한량을 다음 적분 갱신에 반영합니다. OFF여도 적분기 상하한은 적용됩니다. */
            if (b.active_features & BUCK_FEATURE_TRACKING_AW) {
                b.aw+=b.tuning.aw_gain_per_s * (applied-requested) / (float)BUCK_PWM_HZ;
            }
            if (b.state!=FAULT) {
                /* 늦은 계산 결과를 CCR에 쓰기 전에 차단합니다. */
                if (!deadline_ok(entered,counter)) trip(F_TIMING);
                else {
                    b.duty=applied;
                    TIM1->CCR1=(uint32_t)(applied*(float)PERIOD+0.5f);
                }
            }
        }
    }
    if (b.state==FAULT || b.state==DISARMED) {
        Buck_EmergencyOff(); TIM1->CCR1=0u; b.duty=0.0f;
    }
    buck_debug.vin=vin; buck_debug.vout=vout; buck_debug.il=il;
    buck_debug.zero=b.zero; buck_debug.duty=b.duty;
    buck_debug.state=b.state; buck_debug.faults=b.faults;
    buck_debug.calibrated=b.calibrated; buck_debug.verified=BUCK_BOARD_VERIFIED;
    buck_debug.pgood=b.supervisor.pgood ? 1u : 0u;
    buck_debug.monitoring=(b.state==RUN && b.checked) ? 1u : 0u;
    buck_debug.uv_active=b.supervisor.undervoltage_active ? 1u : 0u;
    buck_debug.uv_samples=b.supervisor.undervoltage_samples;
    if (!deadline_ok(entered,counter)) trip(F_TIMING);
    BuckMeasurement_Step(b.state==RUN && b.checked && !b.faults,
                         b.active_features, vout, il, b.duty);
    uint32_t elapsed=DWT->CYCCNT-entered;
    if (elapsed>buck_debug.max_cycles) buck_debug.max_cycles=elapsed;
    if (!deadline_ok(entered,counter)) trip(F_TIMING);
}

/* [직접 작성하는 HAL 콜백] HAL의 weak 기본 함수를 재정의합니다.
 * main.c의 USER CODE 4 또는 다른 .c에 같은 함수를 중복 정의하지 마세요.
 * stm32f3xx_it.c의 자동 생성 HAL_DMA_IRQHandler/HAL_ADC_IRQHandler는
 * 그대로 둡니다. 해당 HAL 처리 후 아래 콜백이 호출됩니다. */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *adc)
{ if (adc->Instance==ADC1) frame(0u); }
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc)
{ if (adc->Instance==ADC1) frame(1u); }
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *adc)
{ if (adc->Instance==ADC1) trip(F_ADC); }

void Buck_Init(void)
{
    /* main.c의 USER CODE 2에서 모든 MX_*_Init() 뒤 한 번만 호출합니다. */
    Buck_EmergencyOff(); memset(&b,0,sizeof b); reset_control();
    BuckMeasurement_Init();
    b.active_features = BUCK_FEATURE_ALL;
    buck_feature_request = BUCK_FEATURE_ALL;
    b.zero=1.65f; b.zero_min=4095u;
    memset((void *)&buck_debug,0,sizeof buck_debug);
    buck_debug.active_features = b.active_features;
    memset((void *)&buck_fault_snapshot,0,sizeof buck_fault_snapshot);
    buck_debug.verified=BUCK_BOARD_VERIFIED;
    buck_arm_request=0u;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0u; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    /* .ioc 재생성 결과가 불완전해도 NULL DMA 핸들로 HAL을 호출하지 않습니다.
     * ADC/TIM을 임의 설정하지 않고, F_SETUP(512)으로 대기합니다. */
    if (!isfinite(VDDA) || VDDA<=0.0f ||
        (CURRENT_POLARITY!=1.0f && CURRENT_POLARITY!=-1.0f) ||
        !Buck_HardwareConfigValid(&hadc1,&htim1)) {
        trip(F_SETUP); return;
    }
    TIM1->CR1 &= ~TIM_CR1_CEN; TIM1->CCR1=0u;
    if (HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED)!=HAL_OK ||
        HAL_ADC_Start_DMA(&hadc1,(uint32_t *)(void *)adc_frame,6u)!=HAL_OK) {
        trip(F_ADC); return;
    }
    /* main.c에서 HAL_TIM_PWM_Start/HAL_TIMEx_PWMN_Start를 별도로 호출하면
     * MOE가 조기에 켜질 수 있습니다. 타이머 시작 순서는 이 함수가 관리합니다. */
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE;
    TIM1->EGR=TIM_EGR_UG; TIM1->SR=0u; TIM1->CNT=0u;
    Buck_EmergencyOff();
    /* 처음 ADC 프레임 전에도 유효한 기준 시간을 사용합니다. */
    b.watchdog_seen=buck_debug.frames;
    b.watchdog_fresh_ms=HAL_GetTick();
    b.sampling_started=1u;
    TIM1->CR1|=TIM_CR1_CEN;
}

void Buck_Poll(void)
{
    if (!b.sampling_started) return;
    uint32_t now=HAL_GetTick(), frames=buck_debug.frames;
    if (frames!=b.watchdog_seen) {
        b.watchdog_seen=frames;
        b.watchdog_fresh_ms=now;
    } else if ((uint32_t)(now-b.watchdog_fresh_ms)>BUCK_ADC_STALE_MS) {
        uint32_t mask=Buck_IrqSave();
        trip(F_TIMING);
        Buck_IrqRestore(mask);
    }
}
