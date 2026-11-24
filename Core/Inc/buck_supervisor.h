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
    uint32_t good_samples;
    uint32_t undervoltage_samples;
    bool undervoltage_active;
    bool pgood;
    bool input_valid;
} BuckSupervisor;

void BuckSupervisor_Init(BuckSupervisor *supervisor);
/* Call at 50 kHz. monitor_ready becomes true only AFTER successful startup;
 * UV monitoring does not wait for PGOOD qualification.
 * PGOOD enters after consecutive samples inside the inclusive inner window;
 * it exits strictly outside the outer window.
 * UV condition enters strictly below 0.80*reference and clears strictly above
 * 0.85*reference. Between/equal thresholds retain the condition. Count EVERY
 * sample while active, including time in the band; 2500 continuous active
 * samples qualify a sustained UV event. Starting in the band does not activate UV.
 * Returns the sustained UV level. buck_app.c latches the diagnostic warning for the run.
 * !monitor_ready resets counts/PGOOD. Invalid/nonfinite reference/output also
 * resets and returns false, with input_valid=false; caller handles sensor fault.
 * Valid reference must be positive; valid measured output may be zero.
 * Changing the target should first set monitor_ready=false to requalify. */
bool BuckSupervisor_Step(BuckSupervisor *supervisor, float reference_v,
                         float output_v, bool monitor_ready);

#endif
