#!/usr/bin/env python3
"""Linux/WSL host checks; no STM32 board or desktop tools are accessed."""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
DEPS = (
    ('hal', 'https://github.com/STMicroelectronics/stm32f3xx-hal-driver.git',
     '664a3bd898dc001d253a47c3e058d4f053cc4da5', 'BUCK_HAL_DIR'),
    ('device', 'https://github.com/STMicroelectronics/cmsis-device-f3.git',
     '402c039a0a6d73c31303a05e6a1d6b8b59474fa1', 'BUCK_DEVICE_DIR'),
    ('cmsis', 'https://github.com/ARM-software/CMSIS_5.git',
     '55b19837f5703e418ca37894d5745b1dc05e4c91', 'BUCK_CMSIS_DIR'),
)

def run(args, **kwargs):
    print('+ ' + shlex.join(map(str, args)), flush=True)
    subprocess.run(list(map(str, args)), check=True, **kwargs)

def dependency(name, url, sha, env, offline):
    supplied = os.environ.get(env)
    path = Path(supplied).resolve() if supplied else ROOT / 'tests/.deps' / (name + '-' + sha[:12])
    if not (path / '.git').exists():
        if supplied or offline:
            raise RuntimeError(f'{env}: pinned dependency unavailable at {path}')
        path.mkdir(parents=True, exist_ok=True)
        run(['git', 'init', '--quiet', path])
        run(['git', '-C', path, 'remote', 'add', 'origin', url])
        run(['git', '-C', path, 'fetch', '--depth=1', 'origin', sha])
        run(['git', '-C', path, 'checkout', '--quiet', '--detach', 'FETCH_HEAD'])
    head = subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()
    dirty = subprocess.check_output(['git', '-C', str(path), 'status', '--porcelain', '--untracked-files=no'], text=True)
    if head != sha or dirty:
        raise RuntimeError(f'{env}: require clean official checkout at {sha}; got {head}')
    return path

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--offline', action='store_true', help='never download dependencies')
    parser.add_argument('--sanitize', action='store_true', help='also run UBSan (and pure supervisor ASan)')
    parser.add_argument('--arm', action='store_true', help='also compile production ARM objects; requires ARM_CC/toolchain')
    args = parser.parse_args()
    build = ROOT / 'tests/build'
    build.mkdir(parents=True, exist_ok=True)
    cc = shlex.split(os.environ.get('CC', 'gcc'))
    strict = ['-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-Wpedantic']
    core = ROOT / 'Core'
    supervisor = core / 'Src/buck_supervisor.c'
    measurement = core / 'Src/buck_measurement.c'
    pure_test = ROOT / 'tests/test_supervisor.c'
    measure_test = ROOT / 'tests/test_measurement.c'
    pure = cc + strict + ['-I' + str(core / 'Inc')]
    run(pure + [supervisor, pure_test, '-lm', '-o', build / 'test_supervisor'])
    run([build / 'test_supervisor'])
    run(pure + [measurement, measure_test, '-lm', '-o', build / 'test_measurement'])
    run([build / 'test_measurement'])
    dep_paths = [dependency(*spec, args.offline) for spec in DEPS]
    includes = ['-I' + str(core / 'Inc'), '-I' + str(ROOT / 'tests/hal_config')]
    for path in (dep_paths[0] / 'Inc', dep_paths[1] / 'Include', dep_paths[2] / 'CMSIS/Core/Include'):
        includes += ['-isystem', str(path)]
    target = cc + strict + ['-Wno-int-to-pointer-cast', '-Wno-pointer-to-int-cast',
                           '-DSTM32F303xE', '-DUSE_HAL_DRIVER'] + includes
    hardware = core / 'Src/buck_hw.c'
    app_test = ROOT / 'tests/test_app.c'
    hardware_test = ROOT / 'tests/test_hw_config.c'
    run(target + [hardware, hardware_test, '-o', build / 'test_hw_config'])
    run([build / 'test_hw_config'])
    run(target + ['-DBUCK_HOST_TEST', app_test, supervisor, measurement, '-lm', '-o', build / 'test_app'])
    run([build / 'test_app'])
    run(target + ['-DBUCK_HOST_TEST', '-DBUCK_TEST_KEEP_BOARD_OFF', app_test, supervisor, measurement,
                  '-lm', '-o', build / 'test_default_off'])
    run([build / 'test_default_off'])
    run(target + ['-fsyntax-only'] + sorted((core / 'Src').glob('*.c')))
    if args.sanitize:
        run(pure + ['-O1', '-g', '-fsanitize=address,undefined', supervisor, pure_test,
                    '-lm', '-o', build / 'test_supervisor_sanitize'])
        run([build / 'test_supervisor_sanitize'])
        run(pure + ['-O1', '-g', '-fsanitize=address,undefined', measurement, measure_test,
                    '-lm', '-o', build / 'test_measurement_sanitize'])
        run([build / 'test_measurement_sanitize'])
        # Fixed STM32 MMIO addresses overlap ASan's shadow mapping. Use UBSan
        # only for the MMIO tests; do not claim ASan coverage for those tests.
        for name, extra, sources in (
            ('test_hw_ubsan', [], [hardware, hardware_test]),
            ('test_app_ubsan', ['-DBUCK_HOST_TEST'], [app_test, supervisor, measurement]),
        ):
            run(target + ['-O1', '-g', '-fsanitize=undefined', '-fno-sanitize-recover=all'] +
                extra + sources + ['-lm', '-o', build / name])
            run([build / name])
    if args.arm:
        arm_cc = shlex.split(os.environ.get('ARM_CC', 'arm-none-eabi-gcc'))
        run(arm_cc + ['--version'])
        arm_build = build / 'arm'
        arm_build.mkdir(exist_ok=True)
        arm_includes = includes.copy()
        # Optional headers for a user-owned, extracted toolchain. Normal Arm GNU
        # installations find newlib automatically and do not need this variable.
        if os.environ.get('BUCK_ARM_INCLUDE'):
            arm_includes += ['-isystem', os.environ['BUCK_ARM_INCLUDE']]
        arm = arm_cc + strict + ['-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16',
                                '-mfloat-abi=hard', '-ffunction-sections', '-fdata-sections',
                                '-DSTM32F303xE', '-DUSE_HAL_DRIVER'] + arm_includes
        sources = sorted((core / 'Src').glob('*.c'))
        for source in sources:
            run(arm + ['-c', source, '-o', arm_build / (source.stem + '.o')])
        print(f'PASS: {len(sources)} Cortex-M4F production objects compiled (not linked)')
    else:
        print('NOT TESTED: ARM object compilation (add --arm with an Arm GNU toolchain)')
    print('PASS: host suites, control and acquisition modules, default-off profile and production source syntax')
    print('NOT TESTED: ARM link, real CubeMX regeneration, ISR WCET, power-stage behavior')

if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError, AssertionError) as error:
        print(f'FAIL: {error}', file=sys.stderr)
        sys.exit(1)
