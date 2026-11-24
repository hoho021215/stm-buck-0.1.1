#!/usr/bin/env bash
set -eu
work=$(cd "$(dirname "$0")" && pwd)
project="${BUCK_SOURCE_DIR:-$work/..}"
hal="${BUCK_HAL_DIR:-$project/tests/.deps/hal-664a3bd898dc}"
device="${BUCK_DEVICE_DIR:-$project/tests/.deps/device-402c039a0a6d}"
cmsis="${BUCK_CMSIS_DIR:-$project/tests/.deps/cmsis-55b19837f570}"
if [[ ! -f "$hal/Inc/stm32f3xx_hal.h" || ! -f "$device/Include/stm32f303xe.h" || ! -f "$cmsis/CMSIS/Core/Include/core_cm4.h" ]]; then
    echo "Run python3 tests/run_host_tests.py from the repository root to prepare HAL and CMSIS." >&2
    exit 1
fi
gcc -shared -fPIC -std=c11 -O2 -Wall -Wextra -Werror -Wpedantic \
    -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast \
    -DSTM32F303xE -DUSE_HAL_DRIVER -DBUCK_HOST_TEST \
    -I"$project/tests" -I"$project/Core/Inc" -I"$project/tests/hal_config" \
    -isystem "$hal/Inc" -isystem "$device/Include" -isystem "$cmsis/CMSIS/Core/Include" \
    "$work/bridge.c" "$project/Core/Src/buck_measurement.c" \
    "$project/Core/Src/buck_supervisor.c" -lm -o "$work/libbuck_sim.so"
