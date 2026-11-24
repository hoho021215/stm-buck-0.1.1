# 0.1.0에서 분리한 코드

0.1.1에서 정상 운전 중 지속 저전압 판정을 경고로 바꾸기 전의 펌웨어 원문입니다. 두 파일은 0.1.0 커밋 `876db1b4186849ae9688c47f8ed3d5e323f306a6`과 바이트 단위로 같습니다. [원본 커밋](https://github.com/hoho021215/stm-buck-0.1.0/tree/876db1b4186849ae9688c47f8ed3d5e323f306a6)

| 보관 파일 | 확인할 내용 |
|---|---|
| [buck_app.c.txt](0.1.0/buck_app.c.txt#L189-L194) | 189~194행의 정상 운전 저전압 차단, 71~95행의 고장 기록 및 게이트 차단, 219~247행의 기존 PI·제한 계산과 수치 |
| [buck_app.h.txt](0.1.0/buck_app.h.txt#L17-L46) | 당시 고장 코드 1024(`BUCK_FAULT_REGULATION`), 운전·디버그·고장 스냅샷 구조 |

이전에는 정상 운전 감시가 시작된 뒤 출력이 기준의 80% 미만이면 저전압 상태에 진입하고 85% 초과에서 벗어났습니다. 활성 상태가 2,500표본에 도달하면 아래 호출이 게이트를 끄고 고장을 유지했습니다.

```c
if (BuckSupervisor_Step(&b.supervisor,15.0f,vout,monitor_ready)) trip(F_REGULATION);
```

0.1.1도 같은 조건을 검출합니다. 그 시점에 `buck_debug.uv_warning_latched`를 기록하며, 다른 차단 조건이 없으면 출력을 계속 제어합니다. 원문 파일은 `.txt` 확장자로 보관하므로 CubeIDE 빌드에는 현재의 `Core` 파일만 추가하세요. 기존 시스템처럼 지속 저전압 때 반드시 출력을 끊어야 하는 용도에는 [0.1.0 소스](https://github.com/hoho021215/stm-buck-0.1.0/tree/876db1b4186849ae9688c47f8ed3d5e323f306a6)를 참조하세요.
