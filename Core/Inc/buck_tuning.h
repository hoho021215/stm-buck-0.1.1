#ifndef BUCK_TUNING_H
#define BUCK_TUNING_H

/* 듀티는 0..1, 시간은 초입니다. 영구 기본값은 buck_app.c의
 * buck_tuning_request 초기값 한 곳에 둡니다. */
typedef struct {
    float kp;              /* duty/V. 목표보다 1 V 낮을 때 비례항이 요청 듀티에 더하는 값 */
    float ki;              /* duty/(V·s). 오차(V)에 이 값과 PI 주기(100 µs)를 곱해 누적합니다. */
    float nominal_vin_v;   /* V. 피드포워드 OFF일 때 목표전압을 이 값으로 나누어 기본 듀티를 구합니다. */
    float integral_limit;  /* duty. 적분항을 −integral_limit부터 +integral_limit까지 제한합니다. */
    float aw_gain_per_s;   /* 1/s. 추적 보정이 켜지면 제한 후 듀티와 요청 듀티의 차이를 적분기로 되먹이는 이득 */
    float run_slew_per_s;  /* duty/s. 기동 확인 후 RUN_SLEW가 켜진 동안 1초당 허용하는 듀티 증가량 */
} BuckTuning;

/* Live Expressions에서 DISARMED 상태로 모든 필드를 편집한 뒤 기동을 요청합니다.
 * 여러 필드를 쓰는 중에는 기동 명령을 보내지 않습니다. MCU 리셋은 빌드 기본값을 복원합니다.
 * 수락된 복사본은 buck_debug.active_tuning으로 확인하며, 운전 중 요청값 수정은 다음 기동에 적용됩니다. */
extern volatile BuckTuning buck_tuning_request;

#endif
