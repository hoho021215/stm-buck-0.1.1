/* [직접 작성하는 파일] Core/Inc/buck_app.h
 * 생성된 main.h를 덮어쓰지 말고 이 헤더를 별도 파일로 추가합니다.
 * main.c/stm32f3xx_it.c의 기존 USER CODE Includes 안에서 include합니다. */
#ifndef BUCK_APP_H
#define BUCK_APP_H

#include <stdint.h>

#define BUCK_VERSION "0.1.0"

/* Live Expressions에서 buck_feature_request에 켤 기능의 값을 더해 씁니다. 7은 모두 ON, 6/5/3은 각각 한 기능만 OFF입니다. */
#define BUCK_FEATURE_VIN_FF      (1u << 0) /* 실측 Vin 피드포워드. OFF 기준은 정격 30 V */
#define BUCK_FEATURE_TRACKING_AW (1u << 1) /* 적용 듀티와 요청 듀티 차이의 적분기 보정 */
#define BUCK_FEATURE_RUN_SLEW    (1u << 2) /* 기동 검증을 통과한 뒤의 듀티 상승률 제한 */
#define BUCK_FEATURE_ALL (BUCK_FEATURE_VIN_FF | BUCK_FEATURE_TRACKING_AW | BUCK_FEATURE_RUN_SLEW)

enum {
    BUCK_STATE_DISARMED, BUCK_STATE_PRECHARGE, BUCK_STATE_SOFTSTART,
    BUCK_STATE_RUN, BUCK_STATE_FAULT
};
enum {
    BUCK_FAULT_ADC=1u, BUCK_FAULT_ZERO=2u, BUCK_FAULT_TIMING=4u,
    BUCK_FAULT_VIN=8u, BUCK_FAULT_OVP=16u, BUCK_FAULT_OC=32u,
    BUCK_FAULT_REVERSE=64u, BUCK_FAULT_PREBIAS=128u,
    BUCK_FAULT_STARTUP=256u, BUCK_FAULT_SETUP=512u,
    BUCK_FAULT_REGULATION=1024u
};

typedef struct {
    float vin, vout, il, zero, duty;
    uint32_t state, faults, calibrated, frames, max_cycles, verified;
    uint32_t pgood, monitoring, uv_active, uv_samples;
    uint32_t active_features; /* 마지막으로 수락한 기동의 설정, 초기값 BUCK_FEATURE_ALL */
} BuckDebug;

/* 최초 고장만 보존합니다. valid=1일 때 읽고, 명시적 고장 해제 후 초기화됩니다.
 * ADC 오류 등에서는 직전 표본이며 sample_valid=0이면 표본이 아직 없습니다.
 * duty는 명령 듀티이며 실제 게이트의 유효 듀티를 측정한 값이 아닙니다. */
typedef struct {
    uint32_t valid, first_fault, state_at_fault, adc_frame;
    uint32_t dwt_cycles, timer_count, sample_valid, raw_adc[3];
    float vin, vout, il, reference, duty, integral, duty_cap;
} BuckFaultSnapshot;

extern volatile BuckDebug buck_debug;
extern volatile BuckFaultSnapshot buck_fault_snapshot;

#ifdef __cplusplus
extern "C" {
#endif

/* main.c: USER CODE 2. GPIO/DMA/ADC1/TIM1 초기화 뒤 한 번 호출합니다. */
void Buck_Init(void);
/* main.c: USER CODE 3. 메인 반복문 안에서 계속 호출합니다. */
void Buck_Poll(void);
/* 전용 EN LOW + TIM1 MOE 해제. 오류 처리 구역의 정지 루프 전에 호출합니다.
 * Error_Handler/HardFault는 호출 후 기존 정지 루프에 머물러야 합니다.
 * 정상 실행 중 정지할 때는 buck_arm_request=2를 사용합니다. */
void Buck_EmergencyOff(void);

/* 디버거 명령: 1=시작, 2=정지, 3=고장 해제(자동 재시작하지 않음).
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
