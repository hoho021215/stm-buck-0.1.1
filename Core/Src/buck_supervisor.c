#include "buck_supervisor.h"

#include <math.h>

void BuckSupervisor_Init(BuckSupervisor *supervisor)
{
    supervisor->good_samples = 0u;
    supervisor->undervoltage_samples = 0u;
    supervisor->undervoltage_active = false;
    supervisor->pgood = false;
    supervisor->input_valid = false;
}

bool BuckSupervisor_Step(BuckSupervisor *supervisor, float reference_v,
                         float output_v, bool monitor_ready)
{
    /* 측정 출력전압을 목표전압에 대한 비율로 환산해 각 경계와 비교합니다.
     * 큰 목표전압에 경계 비율을 곱할 때 생길 수 있는 오버플로를 피합니다. */
    float ratio;
    supervisor->input_valid = isfinite(reference_v) && reference_v > 0.0f &&
                              isfinite(output_v) && output_v >= 0.0f;
    if (!supervisor->input_valid || !monitor_ready) {
        const bool valid = supervisor->input_valid;
        BuckSupervisor_Init(supervisor);
        supervisor->input_valid = valid;
        return false;
    }
    ratio = output_v / reference_v;
    if (!isfinite(ratio)) {
        BuckSupervisor_Init(supervisor);
        return false;
    }
    /* PGOOD는 정상 범위에서 충분히 머문 뒤 켜고, 더 넓은 유지 범위를 벗어나면 끕니다. */
    if (supervisor->pgood) {
        if (ratio < BUCK_SUPERVISOR_PGOOD_EXIT_LOW ||
            ratio > BUCK_SUPERVISOR_PGOOD_EXIT_HIGH) {
            supervisor->pgood = false;
            supervisor->good_samples = 0u;
        }
    } else if (ratio >= BUCK_SUPERVISOR_PGOOD_ENTER_LOW &&
               ratio <= BUCK_SUPERVISOR_PGOOD_ENTER_HIGH) {
        if (supervisor->good_samples < BUCK_SUPERVISOR_PGOOD_SAMPLES) {
            ++supervisor->good_samples;
        }
        supervisor->pgood = supervisor->good_samples >=
                            BUCK_SUPERVISOR_PGOOD_SAMPLES;
    } else {
        supervisor->good_samples = 0u;
    }
    /* UV 경계 사이에서는 직전 상태를 유지합니다. 복귀하면 지속시간 카운터를 지웁니다. */
    if (ratio < BUCK_SUPERVISOR_UV_BAD_RATIO) {
        supervisor->undervoltage_active = true;
    } else if (ratio > BUCK_SUPERVISOR_UV_RECOVER_RATIO) {
        supervisor->undervoltage_active = false;
        supervisor->undervoltage_samples = 0u;
    }
    if (supervisor->undervoltage_active) {
        if (supervisor->undervoltage_samples < BUCK_SUPERVISOR_UV_BAD_SAMPLES) {
            ++supervisor->undervoltage_samples;
        }
    }
    return supervisor->undervoltage_samples >= BUCK_SUPERVISOR_UV_BAD_SAMPLES;
}
