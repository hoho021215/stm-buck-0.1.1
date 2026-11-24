#ifndef BUCK_TUNING_H
#define BUCK_TUNING_H

/* 듀티는 0..1, 시간은 초입니다. 영구 기본값은 buck_app.c의
 * buck_tuning_request 초기값 한 곳에 둡니다. */
typedef struct {
    float kp;              /* duty/V, 전압 비례 이득 */
    float ki;              /* duty/(V·s), 연속시간 적분 이득. Ts는 코드가 곱합니다. */
    float nominal_vin_v;   /* V, 입력전압 피드포워드 OFF일 때의 기준 */
    float integral_limit;  /* duty, 적분항의 대칭 상한/하한 크기 */
    float aw_gain_per_s;   /* 1/s, 제한 후 듀티를 추종하는 anti-windup 이득 */
    float run_slew_per_s;  /* duty/s, 기동 확인 후 RUN_SLEW가 켜졌을 때의 상승률 */
} BuckTuning;

/* Live Expressions에서 DISARMED 상태로 모든 필드를 편집한 뒤 기동을 요청합니다.
 * 여러 필드를 쓰는 중에는 기동 명령을 보내지 않습니다. MCU 리셋은 빌드 기본값을 복원합니다.
 * 수락된 복사본은 buck_debug.active_tuning으로 확인하며, 운전 중 요청값 수정은 다음 기동에 적용됩니다. */
extern volatile BuckTuning buck_tuning_request;

#endif
