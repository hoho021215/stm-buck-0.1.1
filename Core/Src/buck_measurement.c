#include "buck_measurement.h"
#include <math.h>

volatile uint32_t buck_measure_request;
volatile BuckMeasurement buck_measurement;
static float sum_abs_error_v;  /* 측정 중 누적한 |Vout − 15 V|의 합. 완료 시 표본 수로 나눕니다. */

void BuckMeasurement_Init(void)
{
    buck_measure_request = 0u;
    buck_measurement = (BuckMeasurement){0};
    sum_abs_error_v = 0.0f;
}
/* 게이트 차단 시 수집 중이던 측정은 중단으로 표시하고 대기 중인 요청은 버립니다. */
void BuckMeasurement_Abort(void)
{
    if (buck_measurement.status == BUCK_MEASURE_COLLECTING)
        buck_measurement.status = BUCK_MEASURE_ABORTED;
    if (buck_measure_request != 0u) {
        buck_measure_request = 0u;
        ++buck_measurement.rejected_requests;
    }
}
/* ADC 프레임마다 호출합니다. 기동 확인을 마친 정상 운전(eligible)에서 요청을 수락합니다. */
void BuckMeasurement_Step(bool eligible, uint32_t active_features,
                          float vout, float il, float duty)
{
    uint32_t request = buck_measure_request;
    if (request != 0u) {
        buck_measure_request = 0u;
        if (request == 1u && eligible &&
            buck_measurement.status != BUCK_MEASURE_COLLECTING) {
            buck_measurement.status = BUCK_MEASURE_COLLECTING;
            buck_measurement.samples = 0u;
            buck_measurement.active_features = active_features;
            buck_measurement.mae_v = 0.0f;
            buck_measurement.vout_min_v = vout;
            buck_measurement.vout_max_v = vout;
            buck_measurement.peak_abs_il_a = 0.0f;
            buck_measurement.duty_min = duty;
            buck_measurement.duty_max = duty;
            sum_abs_error_v = 0.0f;
        } else {
            ++buck_measurement.rejected_requests;
        }
    }
    if (buck_measurement.status != BUCK_MEASURE_COLLECTING)
        return;
    if (!eligible || active_features != buck_measurement.active_features) {
        BuckMeasurement_Abort();
        return;
    }

    /* 2,500번째 표본의 시간 검사까지 통과한 뒤 다음 적격 프레임에서 완료 상태를 기록합니다. */
    if (buck_measurement.samples == BUCK_MEASURE_WINDOW_SAMPLES) {
        if (!isfinite(sum_abs_error_v)) {
            BuckMeasurement_Abort();
            return;
        }
        buck_measurement.mae_v = sum_abs_error_v / (float)BUCK_MEASURE_WINDOW_SAMPLES;
        buck_measurement.status = BUCK_MEASURE_COMPLETE;
        return;
    }
    if (!isfinite(vout) || !isfinite(il) || !isfinite(duty)) {
        BuckMeasurement_Abort();
        return;
    }
    /* 전압 오차는 15 V 기준으로 누적하고 전류는 방향을 포함한 표본의 절댓값으로 비교합니다. */
    sum_abs_error_v += fabsf(vout - 15.0f);
    if (vout < buck_measurement.vout_min_v) buck_measurement.vout_min_v = vout;
    if (vout > buck_measurement.vout_max_v) buck_measurement.vout_max_v = vout;
    float abs_il = fabsf(il);
    if (abs_il > buck_measurement.peak_abs_il_a) buck_measurement.peak_abs_il_a = abs_il;
    if (duty < buck_measurement.duty_min) buck_measurement.duty_min = duty;
    if (duty > buck_measurement.duty_max) buck_measurement.duty_max = duty;
    ++buck_measurement.samples;
}
