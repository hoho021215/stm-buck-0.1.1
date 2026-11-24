#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "buck_measurement.h"

static void sample(void)
{
    BuckMeasurement_Step(true, 7u, 14.5f, 0.5f, 0.625f);
}

static void test_window_and_repeat(void)
{
    BuckMeasurement_Init();
    sample();
    assert(buck_measurement.status == BUCK_MEASURE_IDLE);
    buck_measure_request = 1u;
    for (unsigned i = 0u; i < BUCK_MEASURE_WINDOW_SAMPLES; ++i) {
        if (i == 100u) buck_measure_request = 1u;
        if (i % 2u == 0u) sample();
        else BuckMeasurement_Step(true, 7u, 16.0f, -1.25f, 0.25f);
    }
    assert(buck_measurement.status == BUCK_MEASURE_COLLECTING);
    assert(buck_measurement.samples == 2500u && buck_measurement.rejected_requests == 1u);
    BuckMeasurement_Step(true, 7u, 20.0f, 2.0f, 0.8f);
    assert(buck_measurement.status == BUCK_MEASURE_COMPLETE);
    assert(buck_measurement.active_features == 7u);
    assert(buck_measurement.vout_min_v == 14.5f && buck_measurement.vout_max_v == 16.0f);
    assert(buck_measurement.mae_v == 0.75f && buck_measurement.peak_abs_il_a == 1.25f);
    assert(buck_measurement.duty_min == 0.25f && buck_measurement.duty_max == 0.625f);

    BuckMeasurement_Abort();
    buck_measure_request = 1u;
    BuckMeasurement_Step(false, 0u, 0.0f, 0.0f, 0.0f);
    assert(buck_measurement.status == BUCK_MEASURE_COMPLETE && buck_measurement.mae_v == 0.75f);
    assert(buck_measurement.rejected_requests == 2u);

    buck_measure_request = 1u;
    for (unsigned i = 0; i <= BUCK_MEASURE_WINDOW_SAMPLES; ++i)
        BuckMeasurement_Step(true, 0u, 15.0f, -0.125f, 0.5f);
    assert(buck_measurement.status == BUCK_MEASURE_COMPLETE && buck_measurement.active_features == 0u);
    assert(buck_measurement.mae_v == 0.0f && buck_measurement.peak_abs_il_a == 0.125f);
    assert(buck_measurement.vout_min_v == 15.0f && buck_measurement.duty_max == 0.5f);
}

static void test_abort_paths(void)
{
    BuckMeasurement_Init();
    buck_measure_request = 9u;
    sample();
    assert(buck_measure_request == 0u && buck_measurement.rejected_requests == 1u);
    assert(buck_measurement.status == BUCK_MEASURE_IDLE);

    buck_measure_request = 1u;
    sample();
    BuckMeasurement_Step(false, 7u, 15.0f, 0.0f, 0.5f);
    assert(buck_measurement.status == BUCK_MEASURE_ABORTED && buck_measurement.samples == 1u);

    buck_measure_request = 1u;
    sample();
    BuckMeasurement_Step(true, 3u, 15.0f, 0.0f, 0.5f);
    assert(buck_measurement.status == BUCK_MEASURE_ABORTED && buck_measurement.active_features == 7u);

    buck_measure_request = 1u;
    BuckMeasurement_Step(true, 7u, NAN, 0.0f, 0.5f);
    assert(buck_measurement.status == BUCK_MEASURE_ABORTED && buck_measurement.samples == 0u);

    buck_measure_request = 1u;
    for (unsigned i = 0u; i < BUCK_MEASURE_WINDOW_SAMPLES; ++i) sample();
    BuckMeasurement_Abort();  /* 마지막 프레임의 시간 초과에 해당 */
    sample();
    assert(buck_measurement.status == BUCK_MEASURE_ABORTED);
    assert(buck_measurement.samples == BUCK_MEASURE_WINDOW_SAMPLES);
    buck_measure_request = 1u;
    BuckMeasurement_Abort();
    assert(buck_measure_request == 0u && buck_measurement.rejected_requests == 2u);
}

int main(void)
{
    test_window_and_repeat();
    test_abort_paths();
    puts("measurement window, repeat, extrema, request and abort tests passed");
    return 0;
}
