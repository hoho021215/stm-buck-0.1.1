#include "buck_supervisor.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>

static void repeat(BuckSupervisor *s, unsigned n, float reference,
                   float output, bool ready, bool expected_trip)
{
    unsigned i;
    for (i = 0u; i < n; ++i) {
        assert(BuckSupervisor_Step(s, reference, output, ready) == expected_trip);
    }
}

static void test_not_ready_and_startup_exclusion(void)
{
    BuckSupervisor s;
    BuckSupervisor_Init(&s);
    assert(!s.pgood && !s.input_valid);
    repeat(&s, 10000u, 15.0f, 0.0f, false, false);
    assert(s.input_valid && !s.pgood && s.undervoltage_samples == 0u);
    repeat(&s, BUCK_SUPERVISOR_PGOOD_SAMPLES, 15.0f, 15.0f, false, false);
    assert(!s.pgood && s.good_samples == 0u);
    /* UV must arm without waiting for PGOOD ever to have become true. */
    repeat(&s, BUCK_SUPERVISOR_UV_BAD_SAMPLES - 1u, 15.0f, 0.0f, true, false);
    assert(BuckSupervisor_Step(&s, 15.0f, 0.0f, true));
    assert(!s.pgood);
    assert(!BuckSupervisor_Step(&s, 15.0f, 0.0f, false));
    assert(s.undervoltage_samples == 0u);
}

static void test_pgood_qualification_and_hysteresis(void)
{
    BuckSupervisor s;
    BuckSupervisor_Init(&s);
    repeat(&s, BUCK_SUPERVISOR_PGOOD_SAMPLES - 1u, 15.0f, 14.25f, true, false);
    assert(!s.pgood);
    assert(!BuckSupervisor_Step(&s, 15.0f, 14.25f, true));
    assert(s.pgood);
    repeat(&s, 100u, 15.0f, 13.5f, true, false); /* Outer lower bound: hold. */
    assert(s.pgood);
    repeat(&s, 100u, 15.0f, 16.5f, true, false); /* Outer upper bound: hold. */
    assert(s.pgood);
    assert(!BuckSupervisor_Step(&s, 15.0f, 16.51f, true));
    assert(!s.pgood && s.good_samples == 0u);
    repeat(&s, 249u, 15.0f, 15.75f, true, false);
    assert(!s.pgood);
    assert(!BuckSupervisor_Step(&s, 15.0f, 15.76f, true)); /* Break consecutive count. */
    assert(s.good_samples == 0u);
    repeat(&s, 250u, 15.0f, 15.75f, true, false);
    assert(s.pgood);
    assert(!BuckSupervisor_Step(&s, 15.0f, 13.49f, true));
    assert(!s.pgood);
}

static void test_uv_continuous_hysteretic_dwell(void)
{
    BuckSupervisor s;
    BuckSupervisor_Init(&s);
    repeat(&s, 10000u, 15.0f, 12.0f, true, false); /* Equality alone does not enter. */
    repeat(&s, 10000u, 15.0f, 12.5f, true, false); /* Initial band is inactive. */
    assert(!s.undervoltage_active && s.undervoltage_samples == 0u);
    repeat(&s, 1000u, 15.0f, 11.99f, true, false);
    assert(s.undervoltage_active);
    repeat(&s, 1499u, 15.0f, 12.0f, true, false); /* Active state counts in band. */
    assert(s.undervoltage_samples == 2499u);
    assert(BuckSupervisor_Step(&s, 15.0f, 12.75f, true)); /* Equality retains active. */
    assert(s.undervoltage_samples == 2500u);
    repeat(&s, 10000u, 15.0f, 0.0f, true, true); /* Saturation, no wrap. */
    assert(s.undervoltage_samples == 2500u);
    assert(!BuckSupervisor_Step(&s, 15.0f, 12.76f, true));
    assert(!s.undervoltage_active && s.undervoltage_samples == 0u);
    /* Caller retains its own fault latch; this pure supervisor reports level. */
    repeat(&s, 50u, 15.0f, 0.0f, true, false);
    assert(!BuckSupervisor_Step(&s, 15.0f, 15.0f, true));
    assert(s.undervoltage_samples == 0u);
    repeat(&s, 2499u, 15.0f, 0.0f, true, false);
}

static void test_reset_invalid_input_and_scaled_reference(void)
{
    BuckSupervisor s;
    const float invalid_targets[] = {0.0f, -1.0f, NAN, INFINITY, -INFINITY};
    const float invalid_outputs[] = {-0.1f, NAN, INFINITY, -INFINITY};
    unsigned i;
    BuckSupervisor_Init(&s);
    repeat(&s, 250u, 10.0f, 10.0f, true, false);
    assert(s.pgood);
    BuckSupervisor_Init(&s);
    assert(!s.pgood && s.good_samples == 0u && s.undervoltage_samples == 0u);
    for (i = 0u; i < sizeof invalid_targets / sizeof invalid_targets[0]; ++i) {
        assert(!BuckSupervisor_Step(&s, invalid_targets[i], 15.0f, true));
        assert(!s.input_valid && !s.pgood && s.undervoltage_samples == 0u);
    }
    for (i = 0u; i < sizeof invalid_outputs / sizeof invalid_outputs[0]; ++i) {
        assert(!BuckSupervisor_Step(&s, 15.0f, invalid_outputs[i], true));
        assert(!s.input_valid && !s.pgood);
    }
    assert(!BuckSupervisor_Step(&s, FLT_MIN, FLT_MAX, true));
    assert(!s.input_valid); /* Finite inputs with overflowing normalized ratio. */
    repeat(&s, 250u, 10.0f, 10.0f, true, false);
    assert(s.pgood && s.input_valid);
    repeat(&s, 2499u, 10.0f, 7.9f, true, false);
    assert(BuckSupervisor_Step(&s, 10.0f, 7.9f, true));
}

int main(void)
{
    test_not_ready_and_startup_exclusion();
    test_pgood_qualification_and_hysteresis();
    test_uv_continuous_hysteretic_dwell();
    test_reset_invalid_input_and_scaled_reference();
    puts("buck_supervisor: all tests passed");
    return 0;
}
