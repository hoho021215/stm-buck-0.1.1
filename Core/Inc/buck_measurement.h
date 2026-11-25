#ifndef BUCK_MEASUREMENT_H
#define BUCK_MEASUREMENT_H

#include <stdbool.h>
#include <stdint.h>

#define BUCK_MEASURE_WINDOW_SAMPLES 2500u /* 50 kHz에서 50 ms */
/* 펌웨어가 갱신하는 측정 상태입니다. Live Expressions에서 2(완료)일 때 결과를 읽으세요. */
enum {
    BUCK_MEASURE_IDLE = 0,
    BUCK_MEASURE_COLLECTING = 1,
    BUCK_MEASURE_COMPLETE = 2,
    BUCK_MEASURE_ABORTED = 3
};

typedef struct {
    uint32_t status;
    uint32_t samples;  /* 현재 측정에서 수집한 표본 수 */
    uint32_t active_features;  /* 측정 시작 때 실제 적용 중이던 기능 비트 조합 */
    uint32_t rejected_requests;  /* 초기화 후 거부된 요청의 누적 횟수 */
    float vout_min_v, vout_max_v;  /* 측정 구간에서 ADC로 읽은 출력전압 최솟값·최댓값(V) */
    float mae_v;          /* 15 V 기준 출력전압 평균 절대 오차 */
    float peak_abs_il_a;  /* ADC가 표본화한 전류의 절댓값 최댓값 */
    float duty_min, duty_max; /* 명령 듀티 0..1 */
} BuckMeasurement;

/* 정상 운전 중 디버거에서 요청 변수에 1을 쓰면 펌웨어가 한 번 읽고 0으로 되돌립니다. */
extern volatile uint32_t buck_measure_request;
extern volatile BuckMeasurement buck_measurement;  /* 펌웨어가 기록하는 통계. 완료값은 다음 요청 수락 또는 초기화까지 유지됩니다. */

void BuckMeasurement_Init(void);
void BuckMeasurement_Step(bool eligible, uint32_t active_features,
                          float vout, float il, float duty);
/* Step과 동일 IRQ 우선순위 또는 IRQ 보호 구간에서 호출합니다. */
void BuckMeasurement_Abort(void);

#endif
