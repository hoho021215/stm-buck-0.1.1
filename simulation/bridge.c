/* Host process only. The test fixture supplies the ADC/DMA callbacks and
 * mocked MMIO/HAL. The controller itself is included from the repository source
 * by test_app.c; nothing here should be compiled for a microcontroller. */
#if !defined(BUCK_HOST_TEST) || !defined(__linux__) || defined(__arm__) || defined(__thumb__)
#error "This adapter is for a Linux host controller/circuit co-simulation."
#endif
#define main unused_app_fixture_main
#include "test_app.c"
#undef main

void sim_init(uint32_t mask)
{
    static int mapped;
    if (!mapped) {
        map_region(0x40000000u, 0x30000u);
        map_region(0x48000000u, 0x10000u);
        map_region(0xE0000000u, 0x10000u);
        mapped = 1;
    }
    fixture(0u);
    buck_feature_request = mask;
}

/* Use the same ACS71240 zeros/sensitivity and voltage divider as the fixture.
 * Keep real out-of-range analog inputs clipped to the ADC input rails. */
static float model_rail(float value, float lo, float hi)
{
    return fminf(fmaxf(value, lo), hi);
}

uint32_t sim_feed(float vin, float vout, float il, uint32_t arm, uint32_t measure)
{
    assert(isfinite(vin) && isfinite(vout) && isfinite(il));
    if (arm) buck_arm_request = arm;
    if (measure) buck_measure_request = measure;
    const float vmax = BUCK_ADC_VDDA_V * 19.0f;
    feed(model_rail(vin, 0.0f, vmax), model_rail(vout, 0.0f, vmax),
         model_rail(il, -1.65f/0.132f, (BUCK_ADC_VDDA_V-1.65f)/0.132f));
    return TIM1->CCR1;
}

uint32_t sim_gate(void)
{
    /* In this fixture's plain RAM model BSRR retains the most recent EN write. */
    return !!(TIM1->BDTR & TIM_BDTR_MOE) && BUCK_EN_GPIO->BSRR == BUCK_EN_PIN;
}
uint32_t sim_state(void) { return buck_debug.state; }
uint32_t sim_fault(void) { return buck_debug.faults; }
uint32_t sim_monitoring(void) { return buck_debug.monitoring; }
uint32_t sim_frames(void) { return buck_debug.frames; }
uint32_t sim_active(void) { return buck_debug.active_features; }
uint32_t sim_calibrated(void) { return buck_debug.calibrated; }
uint32_t sim_pgood(void) { return buck_debug.pgood; }

/* The 0.1.1 Python host mirrors this compact debugger struct with ctypes. */
_Static_assert(sizeof(BuckTuning) == 6 * sizeof(float), "Update the simulation tuning binding");

const volatile BuckTuning *sim_active_tuning(void) { return &buck_debug.active_tuning; }
uint32_t sim_uv_warning(void) { return buck_debug.uv_warning_latched; }
const char *sim_firmware_version(void) { return BUCK_VERSION; }
