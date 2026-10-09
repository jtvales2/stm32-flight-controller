# Follow-up interrupt-state review

This review covers the two additional application functions identified after the
initial `sbus_read_latest()` and `fc_core_baro_feed()` remediation. It checks how
existing critical sections restore PRIMASK. It does not establish firmware-wide
concurrency correctness or hardware/flight safety.

## Changes

| Function | Problem before this follow-up | Change and preserved behavior |
| --- | --- | --- |
| `fc_core_service_async()` in `FlightController/fc_core/fc_core.c` | The short section consuming `s.baro.stale_pending` unconditionally enabled interrupts on exit. A caller entering with PRIMASK 1 would unexpectedly leave with PRIMASK 0. | Save PRIMASK before masking, then restore it immediately after consuming the flag. The no-stale return, altitude cleanup, original section extent, and exit-event conditions stay unchanged. |
| `imu_set_gyro_bias_dps()` in `Core/Src/imu_bringup.c` | The section calling `IMU_BMI088_FE_SetBias()` also unconditionally enabled interrupts on exit. | Save and restore PRIMASK around the existing setter call. The null guard, local acceleration/gyro bias copies, return values, and bias values stay unchanged. |

Paths in this document are relative to `firmware/stm32/`. The changes do not alter
PID gains, motor outputs, IMU sampling/transfer scheduling, calibration thresholds,
or sensor pairing.

## Caller and shared-state evidence

`fc_core_service_async()` is called by the main loop, including the periodic service
inside the gyro-drain loop. `HAL_TIM_PeriodElapsedCallback()` calls
`fc_core_watchdog_1khz()` for TIM2; its barometer age check may set
`s.baro.stale_pending`. The existing critical section reads and clears that flag.
When the captured flag is zero, the function returns. When it is nonzero, the
function invalidates the barometer state and clears the altitude-control state.
It emits `FC_EVT_ALT_EXIT` with reason `FC_ALT_EXIT_BARO_STALE` only if altitude
control was active before cleanup. The cleanup remains after restoration of the
caller's interrupt state; this follow-up does not broaden the critical section.

The only application call to `imu_set_gyro_bias_dps()` is in
`fc_prearm_cal_service()` in `FlightController/fc_safety/fc_arm.c`. It reads the
current gyro bias through `imu_get_gyro_bias_dps()`, adds the measured mean, and
sets the replacement. The path is reached through
`fc_core_step()` -> `fc_arm_ahrs_reset_service()` in main control context. Its
existing conditions require the pre-arm calibration state and static/RC checks.
This review preserves those conditions and does not validate the calibration
algorithm or its thresholds.

`IMU_BMI088_FE_SetBias()` in `Core/Src/imu_bmi088_frontend.c` has a null guard
and copies acceleration and gyro bias arrays into `fe->cfg`. The SPI DMA completion
path invokes `imu_pipeline_dma_on_spi_done()` through
`HAL_SPI_TxRxCpltCallback()`. Its `IMU_BMI088_FE_ProcessGyrRaw()` and
`IMU_BMI088_FE_ProcessAccRaw()` calls consume those bias arrays. Keeping the setter
call masked prevents an ordinary maskable interrupt consumer from observing the
middle of those copies. PRIMASK does not mask NMI or HardFault.

The other application caller of the frontend setter is
`maybe_static_calibrate_gz()` in `Core/Src/autocal.c`. It already brackets the
setter with a PRIMASK-saving helper and `__set_PRIMASK()`. Its caller in
`Core/Src/sync_pair.c` is serviced by the main gyro-processing path. The existing
bias getter is unchanged: the inspected application callers/writers use main
context, while the identified DMA path reads bias. This is call-graph evidence,
not a proof covering arbitrary future callers or frontend reinitialization.

`imu_bringup.c` now explicitly includes `stm32f4xx.h`; it also receives CMSIS declarations through `spi.h` -> `main.h`
-> `stm32f4xx_hal.h`. `fc_core.c` explicitly includes `stm32f4xx.h` after the
first-stage remediation.

## Classification of remaining primitive calls

Application `.c` files under `Core/` and `FlightController/` were searched for
`__get_PRIMASK`, `__set_PRIMASK`, `__disable_irq`, and `__enable_irq`. The table
classifies the observed patterns; it does not claim that every interrupt-related
data access or shared-state protocol has been audited.

| Location | Observed pattern | Disposition |
| --- | --- | --- |
| `Core/Src/sbus.c`, `sbus_read_latest()` | Saves PRIMASK; checks/publishes frame availability inside the existing masked section; restores PRIMASK. | Initial remediation retained. |
| `FlightController/fc_core/fc_core.c`, `fc_core_baro_feed()` | Saves PRIMASK; restores on the common exit after valid/invalid updates. | Initial remediation retained. |
| `FlightController/fc_core/fc_core.c`, `fc_core_service_async()` | Original unconditional enable replaced with restoration of saved PRIMASK. | This follow-up. |
| `Core/Src/imu_bringup.c`, `imu_set_gyro_bias_dps()` | Original unconditional enable replaced with restoration of saved PRIMASK. | This follow-up. |
| `Core/Src/ringbuf_spsc.c`, `rb_irq_save()` / `rb_irq_restore()` | Saves PRIMASK and restores with `__set_PRIMASK()`, including early-return callers. | Existing pattern preserved. |
| `Core/Src/autocal.c`, `irq_save()` / `irq_restore()` | Saves PRIMASK and restores with `__set_PRIMASK()`. | Existing pattern preserved. |
| `Core/Src/pipeline.c`, `_irq_save()` / `_irq_restore()` | Saves PRIMASK, disables interrupts, and enables only if the saved state was unmasked. | Conditional restoration already preserves entry state; retained. |
| `Core/Src/usart.c`, `sbus_irq_save()` / `sbus_irq_restore()` | Saves PRIMASK and enables only when the saved value is zero. | Conditional restoration already preserves entry state; retained. |
| `FlightController/fc_log/fc_evt.c`, `irq_save()` / `irq_restore()` | Saves PRIMASK and enables only if the saved state was unmasked. | Conditional restoration already preserves entry state; retained. |
| `Core/Src/main.c`, `imu_stats_snapshot()` and the ring reset in `baseline_align()` | Save PRIMASK and conditionally enable after the protected copy/reset. | Existing entry-state preservation retained. |
| `Core/Src/main.c`, `Error_Handler()` | Disables interrupts, then enters an endless loop. | Terminal error path; intentionally has no returning restoration. |

The conditional helpers above preserve the two valid entry states because their
inspected sections leave interrupts masked until the helper exit. A saved 0 is
re-enabled and a saved 1 stays masked. The mere presence of `__enable_irq()` is
therefore not evidence of the unconditional-restoration defect.

The source search also found third-party HAL/LL implementation patterns in
`Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_eth.c`,
`stm32f4xx_hal_lptim.c`, and `stm32f4xx_ll_lptim.c`. These save PRIMASK, use
`__set_PRIMASK(1)` during protected work, and restore the saved value. They are
vendored source observations rather than a claim that those modules are selected
by this Keil target. CMSIS headers contain the primitive definitions themselves;
they are dependencies and have not been rewritten.

## Reproducible host verification

From the repository root, run:

```sh
python docs/verification/test_critical_sections.py --cc gcc
python docs/verification/test_followup_irq.py --cc gcc
```

Both scripts accept `--repo PATH`, `--cc PATH_TO_GCC`, and `--temp-parent PATH`
for a different repository, compiler, or writable temporary-build directory.
They require Python and a GCC-compatible host C compiler; generated executables
and harness files live in automatically cleaned temporary directories.

The follow-up script extracts exact source definitions instead of maintaining a
separate rewritten implementation. It verifies:

- `fc_core_service_async()`: PRIMASK 0/1, stale flag 0/1, altitude active 0/1,
  RC altitude switch 0/1, retained fields, original small-section extent,
  stale-exit event arguments/conditions, and duplicate calls after consumption.
- `imu_set_gyro_bias_dps()`: null frontend/input paths return without changing
  interrupt state or invoking the setter; normal and aliased inputs preserve
  acceleration bias, forward the exact gyro values, and invoke the setter once
  while masked for PRIMASK 0/1.
- The extracted real `imu_get_gyro_bias_dps()` and frontend
  `IMU_BMI088_FE_SetBias()` provide readback/copy behavior in the bias harness.
- Each normal critical path saves, masks, and restores once; it does not call an
  unconditional enable primitive.

The existing first-stage host checks remain separate and cover SBUS and barometer
feed behavior. Host mocks cannot verify the generated target CMSIS assembly,
complete firmware linkage, interrupt latency, actual DMA/interrupt scheduling,
frontend initialization lifetime, or hardware/flight behavior. Record the full
Keil rebuild separately and perform hardware checks with propellers removed.

## Encoding and change records

The two edited source files were not valid UTF-8 when inspected and decoded with
GB18030-compatible historical encoding. `imu_bringup.c` used CRLF;
`fc_core.c` contained mixed CRLF/LF. Only the ASCII interrupt-state edits are
required here. Preserve all other source bytes, comments, line endings,
third-party notices, and control behavior. Track this follow-up separately from
the initial release cleanup and from any later encoding or PID changes.
