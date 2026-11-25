/* [직접 작성/설정하는 파일] HAL과 독립적인 출력 상태 감시입니다.
 * 아래 비율/시간은 검증용 초깃값이며 실제 부하/보호 요구에 맞춰 확인합니다.
 * 50 kHz Step 호출 기준: 250표본=5 ms, 2500표본=50 ms. */
#ifndef BUCK_SUPERVISOR_H
#define BUCK_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

#define BUCK_SUPERVISOR_PGOOD_ENTER_LOW     0.95f
#define BUCK_SUPERVISOR_PGOOD_ENTER_HIGH    1.05f
#define BUCK_SUPERVISOR_PGOOD_EXIT_LOW      0.90f
#define BUCK_SUPERVISOR_PGOOD_EXIT_HIGH     1.10f
#define BUCK_SUPERVISOR_PGOOD_SAMPLES       250u
#define BUCK_SUPERVISOR_UV_BAD_RATIO        0.80f
#define BUCK_SUPERVISOR_UV_RECOVER_RATIO    0.85f
#define BUCK_SUPERVISOR_UV_BAD_SAMPLES      2500u

typedef struct {
    uint32_t good_samples;  /* PGOOD 진입 범위 안에서 연속으로 관측한 표본 수 */
    uint32_t undervoltage_samples;  /* 현재 저전압 상태의 연속 표본 수. 경고 기준에 도달하면 증가를 멈춥니다. */
    bool undervoltage_active;
    bool pgood;
    bool input_valid;  /* 기준전압과 측정전압을 사용해 판정할 수 있으면 참 */
} BuckSupervisor;
/* 새 감시를 시작할 때 PGOOD와 저전압 상태 및 표본 카운터를 초기화합니다. */
void BuckSupervisor_Init(BuckSupervisor *supervisor);
/* ADC 프레임마다 50 kHz로 호출합니다. 기동 확인을 마치면 monitor_ready가 참이 됩니다.
 * 그때부터 PGOOD(출력 정상 표시)와 저전압(UV) 지속시간을 각각 판정합니다.
 * PGOOD는 출력이 목표의 95~105% 범위에 연속 250표본 머물 때 켜집니다.
 * 켜진 뒤에는 90% 미만 또는 110% 초과에서 꺼집니다.
 * 저전압 상태는 목표의 80% 미만에서 시작하고 85% 초과에서 해제됩니다.
 * 두 경곗값 및 그 사잇값에서는 직전 상태를 유지합니다.
 * 저전압 상태가 활성인 동안은 이 구간에 머문 표본도 카운트에 포함합니다.
 * 연속 2,500표본(50 ms)에 도달하면 복귀할 때까지 참을 반환합니다.
 * buck_app.c가 이 반환값으로 이번 운전의 경고 이력(uv_warning_latched)을 기록합니다.
 * monitor_ready가 거짓이면 카운터와 PGOOD를 초기화합니다.
 * 입력이 유효하지 않으면 초기화 후 거짓을 반환하고 input_valid에 오류를 표시합니다.
 * 기준전압은 유한한 양수, 측정전압은 0 이상의 유한한 값이어야 합니다.
 * 목표전압을 변경할 때는 monitor_ready를 먼저 해제한 뒤 새 목표로 감시를 시작합니다. */
bool BuckSupervisor_Step(BuckSupervisor *supervisor, float reference_v,
                         float output_v, bool monitor_ready);

#endif
