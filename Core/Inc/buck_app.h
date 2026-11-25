/* [직접 작성하는 파일] Core/Inc/buck_app.h
 * 생성된 main.h를 덮어쓰지 말고 이 헤더를 별도 파일로 추가합니다.
 * main.c/stm32f3xx_it.c의 기존 USER CODE Includes 안에서 include합니다. */
#ifndef BUCK_APP_H
#define BUCK_APP_H

#include <stdint.h>
#include "buck_tuning.h"

#define BUCK_VERSION "0.1.1"

/* 기동 전에 켤 기능의 비트값을 더합니다. 7은 모두 켜고 6은 피드포워드만 끕니다. */
#define BUCK_FEATURE_VIN_FF      (1u << 0) /* 실측 Vin 피드포워드. OFF 기준은 nominal_vin_v */
#define BUCK_FEATURE_TRACKING_AW (1u << 1) /* 적용 듀티와 요청 듀티 차이의 적분기 보정 */
#define BUCK_FEATURE_RUN_SLEW    (1u << 2) /* 기동 검증을 통과한 뒤의 듀티 상승률 제한 */
#define BUCK_FEATURE_ALL (BUCK_FEATURE_VIN_FF | BUCK_FEATURE_TRACKING_AW | BUCK_FEATURE_RUN_SLEW)
/* buck_debug.state의 순서: 0 대기 → 1 부트스트랩 충전 → 2 소프트스타트 → 3 운전. 고장은 4입니다. */
enum {
    BUCK_STATE_DISARMED, BUCK_STATE_PRECHARGE, BUCK_STATE_SOFTSTART,
    BUCK_STATE_RUN, BUCK_STATE_FAULT
};
enum { /* 고장 원인의 비트값입니다. 같은 프레임에서 여러 비트가 함께 켜질 수 있습니다. */
    BUCK_FAULT_ADC=1u, BUCK_FAULT_ZERO=2u, BUCK_FAULT_TIMING=4u,
    BUCK_FAULT_VIN=8u, BUCK_FAULT_OVP=16u, BUCK_FAULT_OC=32u,
    BUCK_FAULT_REVERSE=64u, BUCK_FAULT_PREBIAS=128u,
    BUCK_FAULT_STARTUP=256u, BUCK_FAULT_SETUP=512u,
    BUCK_FAULT_REGULATION=1024u, /* 0.1.0 기록 해석용 예약 비트 */
    BUCK_FAULT_TUNING=2048u /* 기동 전에 요청한 튜닝값이 허용 범위를 벗어남 */
};

typedef struct {
    float vin, vout, il, zero, duty; /* 입력·출력 전압(V), 인덕터 전류(A), 전류센서 영점(V), 명령 듀티(0~1) */
    uint32_t state, faults, calibrated, frames, max_cycles, verified; /* 상태, 누적 고장 비트, 영점 보정 여부, ADC 프레임 수, 최대 처리 사이클 수, 보드 검증 설정값 */
    uint32_t pgood, monitoring, uv_active, uv_samples; /* 출력 정상 판정, 기동 확인 후 감시 중, 현재 저전압 상태, 연속 저전압 상태 표본 수 */
    uint32_t active_features; /* 마지막으로 수락한 기동의 설정, 초기값 BUCK_FEATURE_ALL */
    /* 기동 확인 후 지속 저전압을 한 번이라도 감지하면 1. 다음 기동 수락/초기화까지 유지. */
    uint32_t uv_warning_latched;
    BuckTuning active_tuning; /* 마지막 수락된 설정을 읽는 용도. 첫 기동 전에는 전부 0. */
} BuckDebug;

/* trip()이 첫 차단 원인을 RAM에 남깁니다. valid=1이면 기록이 완성됐습니다. 고장 해제 수락 또는 초기화 때 지웁니다.
 * sample_valid=0이면 저장할 ADC 표본이 없었습니다. 오류 발생 시점에 따라 마지막으로 받은 표본이 남을 수 있습니다.
 * duty는 펌웨어가 직전에 내린 듀티 명령입니다. 실제 게이트 파형은 오실로스코프로 확인합니다. */
typedef struct {
    uint32_t valid, first_fault, state_at_fault, adc_frame; /* 기록 완료 여부, 첫 고장 비트, 고장 전 상태, 프레임 번호 */
    uint32_t dwt_cycles, timer_count, sample_valid, raw_adc[3]; /* CPU·TIM1 카운터값, 표본 유무, ADC 원시값 순서 IL/Vout/Vin */
    float vin, vout, il, reference, duty, integral, duty_cap; /* 차단 시 입력·출력(V), 전류(A), 목표(V), 명령 듀티·적분항·듀티 상한 */
} BuckFaultSnapshot;

extern volatile BuckDebug buck_debug; /* Live Expressions에서 운전 상태를 읽습니다. */
extern volatile BuckFaultSnapshot buck_fault_snapshot; /* 정지 원인을 확인할 때 valid와 first_fault를 먼저 읽습니다. */

#ifdef __cplusplus
extern "C" {
#endif

/* main.c의 USER CODE 2에서 주변장치 초기화 뒤 한 번 호출합니다. 게이트를 끄고 설정을 검사한 뒤 샘플링을 시작합니다. */
void Buck_Init(void);
/* main.c의 while (1), USER CODE 3에서 반복 호출해 ADC 프레임이 끊겼는지 확인합니다. */
void Buck_Poll(void);
/* 전용 EN LOW + TIM1 MOE 해제. 오류 처리 구역의 정지 루프 전에 호출합니다.
 * Error_Handler/HardFault는 호출 후 기존 정지 루프에 머물러야 합니다.
 * 정상 실행 중 정지할 때는 buck_arm_request=2를 사용합니다. */
void Buck_EmergencyOff(void);

/* 디버거에서 한 번 쓰는 명령입니다. 1=시작, 2=정지, 3=고장 해제. 펌웨어가 읽으면 0으로 되돌립니다.
 * 시작 조건: buck_board.h의 BUCK_BOARD_VERIFIED=1, 영점 보정 완료,
 * 유효한 센서 값과 ADC/TIM/DMA 설정, 실측으로 확인한 배선과 게이트 파형. */
extern volatile uint32_t buck_arm_request;
/* 다음 기동에 요청할 값(0~7)입니다. 현재 적용값은 buck_debug.active_features에서 읽습니다.
 * 알 수 없는 비트가 켜진 상태로 기동을 요청하면 F_SETUP으로 차단됩니다. */
extern volatile uint32_t buck_feature_request;

#ifdef __cplusplus
}
#endif
#endif
