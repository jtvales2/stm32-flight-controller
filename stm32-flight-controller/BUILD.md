# Build

## Project and recorded tools

Open `firmware/stm32/MDK-ARM/fly1.0.uvprojx` and select target **`fly1.0`**. Keep the contents of `Core`, `Drivers`, `FlightController`, `MDK-ARM/RTE` and the startup assembly in their current relative locations.

| Item | Recorded configuration |
| --- | --- |
| MCU | STM32F407ZGTx, Cortex-M4 with FPU |
| Keil used for this preparation rebuild | uVision 5.24a, installed at `E:\keil5\UV4\UV4.exe` |
| Compiler selected by the project and used for the rebuild | ARM Compiler 5.06 update 5, build 528 (`ARMCC`, `uAC6=0`) |
| Device pack | `Keil.STM32F4xx_DFP.2.17.0` |
| RTE metadata | ARM CMSIS package 4.5.0, CORE component 4.3.0 |
| C settings | `USE_HAL_DRIVER`, `STM32F407xx`; C99; `--c99 --no_strict --gnu` |
| Output directory | `firmware/stm32/MDK-ARM/fly1.0/` |
| Outputs | `fly1.0` (extensionless AXF-format executable), `fly1.hex`, and other Keil build products |

The bundled CMSIS core headers identify CMSIS Core(M) 5.6. The RTE package metadata is retained as found; it is not a declaration that all bundled headers came from CMSIS 4.5.0.

`firmware/stm32/fly1.0.ioc` records CubeMX 6.14.0, STM32Cube FW_F4 1.28.3 and target toolchain MDK-ARM V5.32. These are regeneration metadata, distinct from the tools used for the successful rebuild. Building the checked-in Keil project does not require regenerating sources in CubeMX. Review regeneration changes separately because custom firmware extends the generated application.

## Full rebuild

1. Install Keil with the recorded ARMCC 5 compiler and STM32F4 device pack available.
2. Open the project above and select target `fly1.0`.
3. Check **Project > Options for Target > C/C++ > Include Paths**. The unused MAVLink paths have been removed; retain the other source, HAL, CMSIS and flight-controller paths.
4. Run **Project > Rebuild all target files**. A complete rebuild is required after the source and configuration changes.
5. Inspect the build log, record warnings and verify that linking and HEX generation finish. Build output remains excluded from Git.

The equivalent PowerShell invocation used locally is shown below. Run from the repository root, adjust the Keil installation path if necessary, and write the log to the verification directory.

```powershell
$projectPath = (Resolve-Path -LiteralPath '.\firmware\stm32\MDK-ARM\fly1.0.uvprojx').Path
$logPath = Join-Path (Get-Location).Path 'docs\verification\local-keil-rebuild.log'
New-Item -ItemType Directory -Path (Split-Path -Parent $logPath) -Force | Out-Null
$arguments = '-r "{0}" -t "fly1.0" -o "{1}"' -f $projectPath, $logPath
$process = Start-Process -FilePath 'E:\keil5\UV4\UV4.exe' -ArgumentList $arguments -WindowStyle Hidden -Wait -PassThru
$process.ExitCode
Get-Content -LiteralPath $logPath
```

The recorded invocation returned exit code **1**, with **0 errors and 4 warnings** in the Keil log. For this run the warning exit code did not indicate a failed build; assess the log and generated outputs together.

## Recorded rebuild result

On **2026-10-08 (Asia/Shanghai)**, the preparation copy completed a full rebuild in approximately 14 seconds using the tools above. Keil reported:

```text
0 Error(s), 4 Warning(s)
Code=55436 RO-data=8180 RW-data=448 ZI-data=9688
HEX generated
```

The log is [docs/verification/keil-rebuild.log](docs/verification/keil-rebuild.log). Three warnings concern `fc_motor_tool_write_all`, `fc_motor_tool_write_one` and `fc_motor_tool_combo`, which are unused with the tool features disabled. The remaining warning is the existing unused function `vofa_send_att_csv` in `main.c` at line 344. Generated outputs are cleaned from the preparation repository after verification; no firmware binary is supplied as a validated release.

## Defaults changed from the historical project

In `firmware/stm32/FlightController/fc_cfg/fc_cfg.h`:

| Macro | Historical value | Preparation default |
| --- | --- | --- |
| `FC_MOTOR_TEST_ENABLE` | `1` | `0` |
| `FC_ESC_CAL_ENABLE` | `1` | `0` |

These changes disable the two special tool features at compile time. They do not constitute safety validation of the controller. Other motor output behavior, controller gains and IMU timing were preserved.

## Interrupt-state changes and required checks

`sbus_read_latest()` in `Core/Src/sbus.c` and `fc_core_baro_feed()` in `FlightController/fc_core/fc_core.c` save `__get_PRIMASK()`, enter their existing critical sections with `__disable_irq()`, and restore the saved value with `__set_PRIMASK()`. The no-frame and invalid-input branches share the same restoration point as the successful branches. Both files explicitly include the STM32/CMSIS declarations used by these operations.

Review [002-preserve-primask.patch](docs/changes/002-preserve-primask.patch) separately from the default changes. The Keil rebuild checks target compilation of the intrinsics; it does not verify runtime preservation of interrupt state. Host checks passed for incoming PRIMASK **0 and 1**, SBUS parser/read and frame-boundary race paths, and the exact barometer feed's valid/invalid paths and retained state. A source check also confirms that barometer payload and stale-pending state precede valid-last publication. These checks use mocked interrupt primitives; target checks for both masks and all applicable return paths remain required. See [validation status](docs/validation-status.md).

To repeat the host checks from the repository root with a host GCC installation:

```powershell
python docs/verification/test_critical_sections.py --cc gcc
```

The script accepts `--repo` and `--cc` paths when needed. The test source and recorded result are [test_critical_sections.py](docs/verification/test_critical_sections.py) and [host-critical-sections.log](docs/verification/host-critical-sections.log). Host mocks do not verify target assembly, interrupt latency or hardware operation; the independent Keil rebuild above verifies the complete target project compiles and links.

Before any hardware check, remove all propellers. Target checks, IMU timing, receiver behavior, arming/fault behavior, motor output and MS5611/altitude hold require their own recorded evidence before a hardware or flight validation claim is made.

## Repeat the preparation checks

The host check requires Python 3 and a GCC-compatible host C compiler. From the repository root:

```powershell
python .\docs\verification\test_critical_sections.py --cc gcc
python .\docs\verification\check_release.py
```

Use `--cc` with your compiler's full path if it is not on PATH. The first command checks C behavior with IRQ mocks. The second checks this preparation snapshot against the original manifest, the expected five-file cumulative change set and the two-file follow-up delta, retained licenses and Keil paths. Run the snapshot check after removing regenerated build outputs. It is a baseline audit for this preparation stage; future intentional source changes require a new baseline. Optional `--original` and `--source` directories additionally verify the preserved historical copies. The sealed first-stage audit is [release-audit.json](docs/verification/release-audit.json); the current audit is [phase2-release-audit.json](docs/verification/phase2-release-audit.json).

## Follow-up PRIMASK remediation

The first four preparation stages are sealed. The follow-up changes only `fc_core_service_async()` and `imu_set_gyro_bias_dps()`, saving/restoring the incoming PRIMASK around their existing critical sections. `imu_bringup.c` now explicitly includes the device header. PID gains, bias values, IMU timing and motor behavior were retained. See [004-preserve-async-and-bias-primask.patch](docs/changes/004-preserve-async-and-bias-primask.patch) and the [call-context review](docs/concurrency-review.md).

The actual follow-up source completed a full ARMCC5 rebuild on 2026-10-08: **0 errors, 4 warnings**, 14 seconds, `Code=55448 RO-data=8184 RW-data=448 ZI-data=9688`. The same four unused-function warnings remain. The sealed prior log stays unchanged; new evidence is [phase2-keil-rebuild.log](docs/verification/phase2-keil-rebuild.log) and [phase2-build-result.json](docs/verification/phase2-build-result.json).

```powershell
python docs/verification/test_followup_irq.py --cc gcc
```

The new suite passed incoming PRIMASK 0/1, stale/active/switch combinations, event conditions, null returns and normal/aliased bias updates. The prior SBUS/barometer suite also passed again. Recorded logs are [phase2-host-followup.log](docs/verification/phase2-host-followup.log) and [phase2-host-regression.log](docs/verification/phase2-host-regression.log). These are host behavior checks, not board runtime validation.

`SystemClock_Config()` uses the internal **16 MHz HSI** oscillator as the PLL source and selects PLLCLK for SYSCLK.

```text
PLLM = 8
PLLN = 168
PLLP = 2

Nominal SYSCLK = (16 / 8) × (168 / 2) = 168 MHz.
```

`HSE_VALUE` is defined as 25 MHz, but HSE is not selected as the PLL source in this firmware. Actual runtime clock and peripheral timing remain subject to on-target verification. This documentation correction does not change the clock initialization code.
