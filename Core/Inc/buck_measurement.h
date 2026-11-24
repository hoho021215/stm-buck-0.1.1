#ifndef BUCK_MEASUREMENT_H
#define BUCK_MEASUREMENT_H

#include <stdbool.h>
#include <stdint.h>

#define BUCK_MEASURE_WINDOW_SAMPLES 2500u /* 50 kHz에서 50 ms */

enum {
    BUCK_MEASURE_IDLE = 0,
    BUCK_MEASURE_COLLECTING = 1,
    BUCK_MEASURE_COMPLETE = 2,
    BUCK_MEASURE_ABORTED = 3
};

typedef struct {
    uint32_t status;
    uint32_t samples;
    uint32_t active_features;
    uint32_t rejected_requests;
    float vout_min_v, vout_max_v;
    float mae_v;          /* 15 V 기준 출력전압 평균 절대 오차 */
    float peak_abs_il_a;  /* ADC가 표본화한 전류의 절댓값 최댓값 */
    float duty_min, duty_max; /* 명령 듀티 0..1 */
} BuckMeasurement;

/* 디버거에서 정상 운전 중 1을 한 번 씁니다. 완료 결과는 다음 유효 요청까지 유지됩니다. */
extern volatile uint32_t buck_measure_request;
extern volatile BuckMeasurement buck_measurement;

void BuckMeasurement_Init(void);
void BuckMeasurement_Step(bool eligible, uint32_t active_features,
                          float vout, float il, float duty);
/* Step과 동일 IRQ 우선순위 또는 IRQ 보호 구간에서 호출합니다. */
void BuckMeasurement_Abort(void);

#endif
