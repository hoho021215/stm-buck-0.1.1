/* Host-only: actual application translation unit, mocked HAL and MMIO RAM.
 * State-machine/input traces are tested; this is not an LC plant simulation. */
#define _GNU_SOURCE
#include <assert.h>
#include <float.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "stm32f3xx_hal.h"
#include "buck_board.h"
/* Only this HOST test exercises the armed path. Product header remains zero. */
#if !defined(BUCK_TEST_KEEP_BOARD_OFF)
#undef BUCK_BOARD_VERIFIED
#define BUCK_BOARD_VERIFIED 1u
#endif
#include "../Core/Src/buck_app.c"

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
static uint32_t tick_ms, irq_mask;
static unsigned calibration_calls, dma_calls, half_next, trace_samples, cases;
static bool setup_valid;
static HAL_StatusTypeDef calibration_result, dma_result;
uint32_t HAL_GetTick(void) { return tick_ms; }
uint32_t Buck_Test_IrqSave(void) { uint32_t old=irq_mask; irq_mask=1u; return old; }
void Buck_Test_IrqRestore(uint32_t mask) { irq_mask=mask; }
bool Buck_HardwareConfigValid(ADC_HandleTypeDef *a, TIM_HandleTypeDef *t)
{ return setup_valid && a==&hadc1 && t==&htim1; }
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *a,uint32_t mode)
{ assert(a==&hadc1 && mode==ADC_SINGLE_ENDED); ++calibration_calls; return calibration_result; }
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *a,uint32_t *data,uint32_t n)
{ assert(a==&hadc1 && data==(uint32_t *)(void *)adc_frame && n==6u); ++dma_calls; return dma_result; }
static void map_region(uintptr_t address,size_t bytes)
{
    void *p=mmap((void *)address,bytes,PROT_READ|PROT_WRITE,
                 MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED_NOREPLACE,-1,0);
    if (p==MAP_FAILED) { perror("mmap STM32 registers"); assert(0); }
}
static void fixture(uint32_t initial_tick)
{
    memset((void *)0x40000000u,0,0x30000u);
    memset((void *)0x48000000u,0,0x10000u);
    memset((void *)0xE0000000u,0,0x10000u);
    memset(&hadc1,0,sizeof hadc1); memset(&htim1,0,sizeof htim1);
    hadc1.Instance=ADC1; htim1.Instance=TIM1;
    RCC->AHBENR=RCC_AHBENR_GPIOCEN;
    RCC->APB2ENR=RCC_APB2ENR_TIM1EN;
    setup_valid=true; calibration_result=HAL_OK; dma_result=HAL_OK;
    calibration_calls=0u; dma_calls=0u; half_next=0u; trace_samples=0u;
    tick_ms=initial_tick; irq_mask=0u;
    Buck_Init();
}
static uint16_t raw(float volts)
{ return (uint16_t)(volts*4095.0f/BUCK_ADC_VDDA_V+0.5f); }
static void feed_cycles(float vin,float vout,float il,uint32_t cycles)
{
    unsigned offset=3u*half_next;
    adc_frame[offset]=raw(1.65f+0.132f*il);
    adc_frame[offset+1u]=raw(vout/19.0f);
    adc_frame[offset+2u]=raw(vin/19.0f);
    DWT->CYCCNT+=cycles; TIM1->CNT=500u;
    if (++trace_samples%50u==0u) ++tick_ms;
    if (!half_next) HAL_ADC_ConvHalfCpltCallback(&hadc1);
    else HAL_ADC_ConvCpltCallback(&hadc1);
    half_next^=1u;
}
static void feed(float vin,float vout,float il)
{ feed_cycles(vin,vout,il,BUCK_PERIOD_TICKS); Buck_Poll(); }
static void calibrate(void)
{
    for (unsigned i=0;i<2048u;++i) feed(30.0f,0.0f,0.0f);
    assert(buck_debug.calibrated==1u && buck_debug.state==DISARMED);
}
static void arm_to_qualified_run(uint32_t features)
{
    buck_feature_request = features;
    buck_arm_request = 1u;
    feed(30.0f,0.0f,0.0f);
    assert(buck_debug.active_features == features);
    for (unsigned i=0;i<60u;++i) feed(30.0f,0.0f,0.0f);
    for (unsigned i=0;i<50200u;++i) {
        float output=i<25000u ? 15.0f*(float)i/25000.0f : 15.0f;
        feed(30.0f,output,0.2f);
    }
    assert(buck_debug.state==RUN && b.checked && buck_debug.faults==0u);
    for (unsigned i=0;i<250u;++i) feed(30.0f,15.0f,0.2f);
    assert(buck_debug.pgood==1u && buck_debug.monitoring==1u);
}
static void run_ready_with_mask(uint32_t features)
{
    fixture(100u);
    calibrate();
    arm_to_qualified_run(features);
}
static void run_ready(void)
{
    run_ready_with_mask(BUCK_FEATURE_ALL);
}
static void test_initial_poll_and_stale(void)
{
    fixture(100u); Buck_Poll();
    assert(buck_debug.state==DISARMED && buck_debug.faults==0u);
    ++tick_ms; Buck_Poll(); assert(buck_debug.faults==0u);
    ++tick_ms; Buck_Poll();
    assert(buck_debug.state==FAULT && buck_debug.faults==F_TIMING);
    assert(buck_fault_snapshot.valid && !buck_fault_snapshot.sample_valid);
    assert(irq_mask==0u && !(TIM1->BDTR&TIM_BDTR_MOE)); ++cases;
    fixture(UINT32_MAX); Buck_Poll(); tick_ms=0u; Buck_Poll();
    assert(buck_debug.faults==0u); tick_ms=1u; Buck_Poll();
    assert(buck_debug.faults==F_TIMING); ++cases;
    fixture(200u); feed(30.0f,0.0f,0.0f); Buck_Poll();
    assert(buck_debug.faults==0u); ++cases;
}
static void test_setup_and_hal_errors(void)
{
    fixture(0u); setup_valid=false; calibration_calls=0u; dma_calls=0u; Buck_Init();
    assert(buck_debug.faults==F_SETUP && calibration_calls==0u && dma_calls==0u);
    tick_ms=500u; Buck_Poll(); assert(buck_debug.faults==F_SETUP); ++cases;
    fixture(0u); calibration_result=HAL_ERROR; dma_calls=0u; Buck_Init();
    assert(buck_debug.faults==F_ADC && dma_calls==0u); ++cases;
    fixture(0u); dma_result=HAL_ERROR; Buck_Init();
    assert(buck_debug.faults==F_ADC && !b.sampling_started); ++cases;
}
static void test_boot_and_prebias(void)
{
    fixture(10u); calibrate(); buck_arm_request=1u;
    feed(30.0f,1.0f,0.0f); assert(buck_debug.faults==F_PREBIAS);
    assert(!(TIM1->BDTR&TIM_BDTR_MOE)); ++cases;
    fixture(10u); calibrate(); buck_arm_request=1u;
    feed(30.0f,0.0f,0.0f); assert(b.state==PRECHARGE && !(TIM1->BDTR&TIM_BDTR_MOE));
    feed(30.0f,0.0f,0.0f); assert(!(TIM1->BDTR&TIM_BDTR_MOE));
    feed(30.0f,0.0f,0.0f); assert((TIM1->BDTR&TIM_BDTR_MOE) && TIM1->CCR1==0u);
    for (unsigned i=0;i<49u;++i) feed(30.0f,0.0f,0.0f);
    assert(b.state==SOFTSTART); ++cases;
    fixture(10u); calibrate(); buck_arm_request=1u;
    for (unsigned i=0;i<3u;++i) feed(30.0f,0.0f,0.0f);
    assert(b.state==PRECHARGE && b.precharge_enabled);
    TIM1->BDTR&=~TIM_BDTR_MOE;
    feed(30.0f,0.0f,0.0f);
    assert(b.state==FAULT && b.faults==F_SETUP);
    for (unsigned i=0;i<5u;++i) feed(30.0f,0.0f,0.0f);
    assert(!(TIM1->BDTR&TIM_BDTR_MOE)); ++cases;
}
static void test_uv_warning_then_hard_fault_and_clear(void)
{
    run_ready();
    for (unsigned i = 0; i < BUCK_SUPERVISOR_UV_BAD_SAMPLES - 1u; ++i)
        feed(30.0f, 11.0f, 0.2f);
    assert(buck_debug.uv_active && !buck_debug.uv_warning_latched && !buck_debug.pgood);
    feed(30.0f, 11.0f, 0.2f);
    assert(buck_debug.state == BUCK_STATE_RUN && !buck_debug.faults);
    assert(buck_debug.uv_samples == BUCK_SUPERVISOR_UV_BAD_SAMPLES);
    assert(buck_debug.uv_warning_latched && !buck_fault_snapshot.valid);
    assert((TIM1->BDTR & TIM_BDTR_MOE) && BUCK_EN_GPIO->BSRR == BUCK_EN_PIN);
    feed(30.0f, 15.0f, 0.2f);
    assert(!buck_debug.uv_active && !buck_debug.uv_samples && buck_debug.uv_warning_latched);

    feed(30.0f, 15.0f, 2.0f);
    assert(buck_debug.state == BUCK_STATE_FAULT && buck_debug.faults == BUCK_FAULT_OC);
    assert(!(TIM1->BDTR & TIM_BDTR_MOE) && b.duty == 0.0f);
    assert(buck_fault_snapshot.first_fault == BUCK_FAULT_OC && buck_debug.uv_warning_latched);
    assert(buck_fault_snapshot.state_at_fault == BUCK_STATE_RUN && buck_fault_snapshot.sample_valid);
    assert(buck_fault_snapshot.raw_adc[1] == raw(15.0f / 19.0f) && buck_fault_snapshot.il >= 1.8f);
    BuckFaultSnapshot first;
    memcpy(&first, (const void *)&buck_fault_snapshot, sizeof first);
    feed_cycles(30.0f, 1.0f, 0.0f, BUCK_PERIOD_TICKS / 2u);
    assert((b.faults & BUCK_FAULT_TIMING) != 0u);
    assert(memcmp(&first, (const void *)&buck_fault_snapshot, sizeof first) == 0);

    buck_arm_request = 2u; feed(30.0f, 0.0f, 0.0f);
    assert(b.state == FAULT && buck_fault_snapshot.valid);
    buck_arm_request = 3u; feed(0.0f, 0.0f, 0.0f);
    assert(b.state == FAULT && buck_fault_snapshot.valid);
    buck_arm_request = 3u; feed(30.0f, 0.0f, 0.0f);
    assert(b.state == DISARMED && !b.faults && !buck_fault_snapshot.valid);
    assert(buck_debug.uv_warning_latched && !(TIM1->BDTR & TIM_BDTR_MOE));
    buck_arm_request = 1u; feed(30.0f, 0.0f, 0.0f);
    assert(b.state == PRECHARGE && !buck_debug.uv_warning_latched);
    ++cases;
}
static void test_transient_and_timing(void)
{
    run_ready();
    for (unsigned i=0;i<100u;++i) feed(30.0f,0.0f,0.2f);
    feed(30.0f,15.0f,0.2f);
    assert(b.state==RUN && !b.supervisor.undervoltage_active); ++cases;
    feed_cycles(30.0f,15.0f,0.2f,BUCK_PERIOD_TICKS/2u);
    assert(b.state==FAULT && (b.faults&F_TIMING)); ++cases;
    fixture(0u); feed(30.0f,0.0f,0.0f);
    TIM1->CNT=BUCK_PERIOD_TICKS-1u; DWT->CYCCNT+=BUCK_PERIOD_TICKS;
    frame(b.expect); assert(b.state==FAULT && (b.faults&F_TIMING)); ++cases;
}
static void test_absolute_trips_and_startup(void)
{
    const float bad[][3]={{37.0f,15.0f,0.2f},{30.0f,18.0f,0.2f},
                         {30.0f,15.0f,2.0f},{30.0f,15.0f,-0.5f}};
    const uint32_t expected[]={F_VIN,F_OVP,F_OC,F_REVERSE};
    for (unsigned i=0;i<4u;++i) {
        run_ready(); feed(bad[i][0],bad[i][1],bad[i][2]);
        assert(b.state==FAULT && b.faults==expected[i]);
        assert(!(TIM1->BDTR&TIM_BDTR_MOE)); ++cases;
    }
    fixture(0u);
    for (unsigned i=0;i<2048u;++i) feed(30.0f,0.0f,-3.0f);
    assert(b.state==FAULT && b.faults==F_ZERO && !b.calibrated); ++cases;
    fixture(0u); calibrate(); buck_arm_request=1u;
    for (unsigned i=0;i<50100u;++i) feed(30.0f,0.0f,0.0f);
    assert(b.state==FAULT && b.faults==F_STARTUP); ++cases;
    run_ready(); buck_arm_request=2u; feed(30.0f,15.0f,0.2f);
    assert(b.state==DISARMED && !b.faults && !buck_debug.pgood);
    assert(!(TIM1->BDTR&TIM_BDTR_MOE)); ++cases;
}

static void test_profile_arm_stop_and_clear(void)
{
    fixture(0u);
    assert(buck_feature_request == BUCK_FEATURE_ALL);
    calibrate();
    buck_feature_request = BUCK_FEATURE_ALL | 0x80u;
    buck_arm_request = 1u;
    feed(30.0f, 0.0f, 0.0f);
    assert(buck_debug.state == BUCK_STATE_FAULT && buck_debug.faults == BUCK_FAULT_SETUP);
    assert(!(TIM1->BDTR & TIM_BDTR_MOE));

    buck_feature_request = 0u;
    buck_arm_request = 3u;
    feed(30.0f, 0.0f, 0.0f);
    assert(buck_debug.state == BUCK_STATE_DISARMED && !(TIM1->BDTR & TIM_BDTR_MOE));
    buck_arm_request = 1u;
    feed(30.0f, 0.0f, 0.0f);
    assert(buck_debug.state == BUCK_STATE_PRECHARGE && buck_debug.active_features == 0u);
    buck_feature_request = BUCK_FEATURE_ALL;
    for (unsigned i = 0; i < 100; ++i) feed(30.0f, 0.0f, 0.0f);
    assert(buck_debug.state == BUCK_STATE_SOFTSTART && buck_debug.active_features == 0u);

    buck_arm_request = 2u;
    feed(30.0f, 0.0f, 0.0f);
    assert(buck_debug.state == BUCK_STATE_DISARMED && !(TIM1->BDTR & TIM_BDTR_MOE));
    buck_arm_request = 1u;
    feed(30.0f, 0.0f, 0.0f);
    assert(buck_debug.state == BUCK_STATE_PRECHARGE && buck_debug.active_features == BUCK_FEATURE_ALL);
    ++cases;
}

static void test_line_feedforward(void)
{
    /* Same output trace and 30 -> 34 V line step. Downward duty is not slew limited. */
    const uint32_t profiles[] = {0u, BUCK_FEATURE_VIN_FF};
    for (unsigned i = 0; i < 2; ++i) {
        run_ready_with_mask(profiles[i]);
        float start_duty = buck_debug.duty;
        float start_vin = buck_debug.vin;
        buck_feature_request = BUCK_FEATURE_ALL; /* Edit while running must wait for next arm. */
        feed(34.0f, 15.0f, 0.2f);
        assert(buck_debug.active_features == profiles[i]);
        float expected_change = (i == 0) ? 0.0f : 15.0f/buck_debug.vin - 15.0f/start_vin;
        assert(fabsf((buck_debug.duty - start_duty) - expected_change) < 0.0001f);
        assert(fabsf((float)TIM1->CCR1/(float)BUCK_PERIOD_TICKS - buck_debug.duty) < 0.001f);
        ++cases;
    }
}

static void test_current_cap_and_tracking(void)
{
    const uint32_t profiles[] = {0u, BUCK_FEATURE_TRACKING_AW};
    for (unsigned i = 0; i < 2; ++i) {
        run_ready_with_mask(profiles[i]);
        float initial_integral = b.integ;
        /* 1.35 A is above the duty-cap threshold, below the independent overcurrent trip. */
        for (unsigned sample = 0; sample < 1500u; ++sample) feed(30.0f, 15.0f, 1.35f);
        assert(buck_debug.state == BUCK_STATE_RUN && b.cap < 0.16f);
        assert(buck_debug.duty <= b.cap && fabsf(b.integ) <= 0.25f);
        if (i == 0) {
            assert(fabsf(b.integ - initial_integral) < 0.001f);
        } else {
            assert(b.integ - initial_integral < -0.02f);
        }
        feed(30.0f, 15.0f, 2.0f);
        assert(buck_debug.faults == BUCK_FAULT_OC && !(TIM1->BDTR & TIM_BDTR_MOE));
        ++cases;
    }
}

static void test_slew_qualification(void)
{
    const float max_step = 1.0f / 50000.0f;
    /* Before the 1-second startup check even RUN must limit duty growth with the bit clear. */
    fixture(0u);
    calibrate();
    buck_feature_request = BUCK_FEATURE_VIN_FF;
    buck_arm_request = 1u;
    for (unsigned i = 0; i < 60u; ++i) feed(34.0f, 0.0f, 0.0f);
    for (unsigned i = 0; i < 30000u; ++i)
        feed(34.0f, i < 25000u ? 15.0f * (float)i / 25000.0f : 15.0f, 0.2f);
    assert(buck_debug.state == BUCK_STATE_RUN && !buck_debug.monitoring);
    float start_duty = buck_debug.duty;
    feed(24.0f, 15.0f, 0.2f);
    assert(fabsf((buck_debug.duty - start_duty) - max_step) < 0.000001f);
    ++cases;

    const uint32_t profiles[] = {BUCK_FEATURE_VIN_FF, BUCK_FEATURE_VIN_FF | BUCK_FEATURE_RUN_SLEW};
    for (unsigned i = 0; i < 2; ++i) {
        run_ready_with_mask(profiles[i]);
        /* Start at high line to leave room below the mandatory duty cap. */
        feed(34.0f, 15.0f, 0.2f);
        start_duty = buck_debug.duty;
        float start_vin = buck_debug.vin;
        feed(30.0f, 15.0f, 0.2f);
        float delta = buck_debug.duty - start_duty;
        float expected = fminf(b.cap - start_duty, 15.0f/buck_debug.vin - 15.0f/start_vin);
        assert(expected > 0.02f);
        if (i == 0) assert(fabsf(delta - expected) < 0.0001f);
        else assert(fabsf(delta - max_step) < 0.000001f);
        assert(buck_debug.state == BUCK_STATE_RUN);
        ++cases;
    }
}

/* Start each pulse exactly on the 10 kHz PI update and observe applied duty. */
static void check_pi_response(float kp, float ki)
{
    while (b.phase != 0u) feed(30.0f, 15.0f, 0.2f);
    const float previous_integral = b.integ;
    feed(30.0f, 15.75f, 0.2f);
    const float error_v = 15.0f - buck_debug.vout;
    const float expected_integral = previous_integral + ki * error_v * 0.0001f;
    const float expected_duty = 15.0f / 30.0f + kp * error_v + expected_integral;
    assert(fabsf(b.integ - expected_integral) < 0.0000001f);
    assert(fabsf(buck_debug.duty - expected_duty) < 0.000001f);
    assert(TIM1->CCR1 == (uint32_t)(expected_duty * BUCK_PERIOD_TICKS + 0.5f));
}

static void test_tuning_at_start(void)
{
    fixture(0u);
    const BuckTuning saved = buck_tuning_request;
    calibrate();
    buck_tuning_request.kp = 0.005f;
    buck_tuning_request.ki = 0.25f;
    buck_tuning_request.nominal_vin_v = 30.0f;
    buck_tuning_request.integral_limit = 0.25f;
    arm_to_qualified_run(0u);
    check_pi_response(0.005f, 0.25f);

    buck_tuning_request.kp = 0.004f;
    buck_tuning_request.ki = 0.20f;
    check_pi_response(0.005f, 0.25f);
    assert(buck_debug.active_tuning.kp == 0.005f && buck_debug.active_tuning.ki == 0.25f);
    buck_arm_request = 2u;
    feed(30.0f, 0.0f, 0.0f);
    assert(buck_debug.state == BUCK_STATE_DISARMED);
    arm_to_qualified_run(0u);
    assert(buck_debug.active_tuning.kp == 0.004f && buck_debug.active_tuning.ki == 0.20f);
    check_pi_response(0.004f, 0.20f);
    buck_tuning_request = saved;
    ++cases;
}

static void test_nominal_feedforward_and_run_slew_tuning(void)
{
    fixture(0u);
    const BuckTuning saved = buck_tuning_request;
    calibrate();
    buck_tuning_request.kp = 0.01f;
    buck_tuning_request.ki = 0.25f;
    buck_tuning_request.nominal_vin_v = 32.0f;
    buck_tuning_request.integral_limit = 0.0f;
    buck_tuning_request.run_slew_per_s = 2.0f;
    arm_to_qualified_run(BUCK_FEATURE_RUN_SLEW | BUCK_FEATURE_TRACKING_AW);
    while (b.phase != 0u) feed(30.0f, 15.0f, 0.2f);
    const float initial_duty = buck_debug.duty;
    const float unclamped_duty = 15.0f / 32.0f + 0.01f * (15.0f - buck_debug.vout);
    assert(b.integ == 0.0f && fabsf(initial_duty - unclamped_duty) < 0.000001f);
    feed(30.0f, 13.8f, 0.2f);
    assert(buck_debug.state == BUCK_STATE_RUN && b.integ == 0.0f);
    assert(fabsf((buck_debug.duty - initial_duty) - 2.0f / BUCK_PWM_HZ) < 0.000001f);
    buck_tuning_request = saved;
    ++cases;
}

static void test_invalid_tuning_before_gate(void)
{
    fixture(0u);
    calibrate();
    const BuckTuning saved = buck_tuning_request;
    BuckTuning bad[7];
    for (unsigned i = 0; i < sizeof bad / sizeof bad[0]; ++i) bad[i] = saved;
    bad[0].kp = NAN;
    bad[1].kp = FLT_MAX;
    bad[2].ki = -1.0f;
    bad[3].nominal_vin_v = 0.0f;
    bad[4].integral_limit = INFINITY;
    bad[5].aw_gain_per_s = -1.0f;
    bad[6].run_slew_per_s = 0.0f;
    for (unsigned i = 0; i < sizeof bad / sizeof bad[0]; ++i) {
        buck_tuning_request = bad[i];
        buck_arm_request = 1u;
        feed(30.0f, 0.0f, 0.0f);
        assert(buck_debug.state == BUCK_STATE_FAULT && buck_debug.faults == BUCK_FAULT_TUNING);
        assert(buck_fault_snapshot.first_fault == BUCK_FAULT_TUNING);
        assert(!(TIM1->BDTR & TIM_BDTR_MOE) && BUCK_EN_GPIO->BSRR == ((uint32_t)BUCK_EN_PIN << 16));
        assert(!buck_debug.active_tuning.kp && !buck_debug.active_tuning.ki);
        buck_arm_request = 3u;
        feed(30.0f, 0.0f, 0.0f);
        assert(buck_debug.state == BUCK_STATE_DISARMED);
    }
    buck_tuning_request = saved;
    buck_arm_request = 1u;
    feed(30.0f, 0.0f, 0.0f);
    assert(buck_debug.state == BUCK_STATE_PRECHARGE && !buck_debug.faults);
    ++cases;
}

static void test_measurement_abort_paths(void)
{
    run_ready_with_mask(BUCK_FEATURE_VIN_FF);
    buck_measure_request = 1u;
    feed(30.0f, 15.0f, 0.2f);
    assert(buck_measurement.status == BUCK_MEASURE_COLLECTING);
    assert(buck_measurement.samples == 1u && buck_measurement.active_features == BUCK_FEATURE_VIN_FF);
    buck_arm_request = 2u;
    feed(30.0f, 15.0f, 0.2f);
    assert(buck_debug.state == BUCK_STATE_DISARMED && buck_measurement.status == BUCK_MEASURE_ABORTED);
    ++cases;

    run_ready_with_mask(0u);
    buck_measure_request = 1u;
    feed(30.0f, 15.0f, 0.2f);
    HAL_ADC_ErrorCallback(&hadc1); /* No later DMA frame should be needed to invalidate the window. */
    assert(buck_debug.state == BUCK_STATE_FAULT && buck_measurement.status == BUCK_MEASURE_ABORTED);
    assert(buck_measurement.samples == 1u && !(TIM1->BDTR & TIM_BDTR_MOE));
    ++cases;
}

int main(void)
{
    map_region(0x40000000u,0x30000u); map_region(0x48000000u,0x10000u);
    map_region(0xE0000000u,0x10000u);
    if (BUCK_BOARD_VERIFIED==0u) {
        fixture(100u); calibrate(); buck_arm_request=1u;
        for (unsigned i=0;i<100u;++i) feed(30.0f,0.0f,0.0f);
        assert(b.state==DISARMED && !b.faults && !(TIM1->BDTR&TIM_BDTR_MOE));
        puts("default-off profile: calibrated start request cannot enable gates");
        return 0;
    }
    test_initial_poll_and_stale(); test_setup_and_hal_errors();
    test_boot_and_prebias(); test_uv_warning_then_hard_fault_and_clear(); test_transient_and_timing();
    test_absolute_trips_and_startup();
    test_profile_arm_stop_and_clear(); test_line_feedforward();
    test_tuning_at_start(); test_invalid_tuning_before_gate();
    test_nominal_feedforward_and_run_slew_tuning();
    test_current_cap_and_tracking(); test_slew_qualification(); test_measurement_abort_paths();
    printf("buck_app: %u host scenarios passed; production board flag remains 0\n",cases);
    return 0;
}
