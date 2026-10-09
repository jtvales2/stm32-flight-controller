# Diagram source notes

The SVGs are code-derived diagrams, not measured data or test evidence. Labels describe the retained implementation and its configured policies. The flow is intentionally grouped for reading; it does not claim that every box executes in a single context or at a fixed measured rate.

## System architecture

[system-architecture.svg](../media/system-architecture.svg) and [Mermaid source](../media/system-architecture.mmd) connect these implementation boundaries:

- [main.c](../firmware/stm32/Core/Src/main.c): peripheral initialization, EXTI/SPI callbacks, DMA/SBUS/barometer service, gyro queue budgets, lag handling, and TIM2 watchdog callback.
- [pipeline.c](../firmware/stm32/Core/Src/pipeline.c), [ringbuf_spsc.c](../firmware/stm32/Core/Src/ringbuf_spsc.c), [sync_pair.c](../firmware/stm32/Core/Src/sync_pair.c): asynchronous acquisition, timestamped buffers and fusion input selection.
- [fc_core.c](../firmware/stm32/FlightController/fc_core/fc_core.c): time, IMU, RC, attitude, arming and tilt gates before controller execution.
- [fc_control.c](../firmware/stm32/FlightController/fc_control/fc_control.c), [fc_mixer_out.c](../firmware/stm32/FlightController/fc_output/fc_mixer_out.c), [motors.c](../firmware/stm32/Core/Src/motors.c): angle/rate/yaw commands, bounded mixing and four TIM3 outputs. [fc_throttle.c](../firmware/stm32/FlightController/fc_control/fc_throttle.c) integrates altitude-hold throttle behavior before this control path.

Hardware interface labels come from this source and [fly1.0.ioc](../firmware/stm32/fly1.0.ioc), not a verified wiring diagram. The barometer driver is retained; its installed physical sensor and operation still require confirmation.

## IMU pipeline

[imu-pipeline.svg](../media/imu-pipeline.svg) and [Mermaid source](../media/imu-pipeline.mmd) show:

- DRDY pending events retain the latest timestamp and a count. A transfer consumes the pending count; counts above one contribute to merge-drop statistics. The diagram does not imply that every IRQ produces a separate stored sensor sample.
- `GYRO_STREAK_MAX` is 3. When both sensors are pending, accel is selected after that gyro streak; the streak is committed on successful DMA completion.
- Raw data passes through [imu_bmi088_frontend.c](../firmware/stm32/Core/Src/imu_bmi088_frontend.c) into timestamped rings. Ring overflow drops oldest data; [main.c](../firmware/stm32/Core/Src/main.c) can also invoke backlog sweeping.
- `imu_sync_pair_step_one()` consumes the oldest gyro sample and retains the latest accel timestamp at or before it. Gyro-derived `dt` feeds [fusion_mahony.c](../firmware/stm32/Core/Src/fusion_mahony.c). Invalid timestamp order is rejected, and accel values are zeroed for fusion when their age exceeds the pairing limit.
- Attitude fusion runs for consumed gyro samples. Under sample lag, main-loop catch-up can defer controller calls and merge elapsed `dt` before a call using the latest gyro sample; fusion and control are therefore not presented as an unconditional one-to-one mapping.

## Fault handling

[fault-handling.svg](../media/fault-handling.svg) and [Mermaid source](../media/fault-handling.mmd) distinguish three paths:

- **Acquisition restart:** `imu_pipeline_dma_on_spi_error()` releases chip select, stops DMA and clears in-flight state. `imu_pipeline_dma_poll()` detects a transfer duration strictly greater than 10 ms and additionally resets the SPI ready state. Both request a future kick. More than three DMA timeouts in the one-second counting window while armed invokes an IMU emergency stop.
- **Latched stop:** fault checks in [fc_rc.c](../firmware/stm32/FlightController/fc_input/fc_rc.c), [fc_time.c](../firmware/stm32/FlightController/fc_core/fc_time.c), [fc_core.c](../firmware/stm32/FlightController/fc_core/fc_core.c), and [fc_fs.c](../firmware/stm32/FlightController/fc_safety/fc_fs.c) have their own validity, armed, duration, throttle or attitude gates. `fc_emergency_stop()` latches a reason, disarms, resets control and invalidates barometer state. The normal latch-clear path in [fc_arm.c](../firmware/stm32/FlightController/fc_safety/fc_arm.c) requires ARM off held over 300 ms, low throttle, level attitude and stable RC. Startup disarm-first handling is a separate branch. Clearing does not arm the motors; re-arming requires a new ARM rising edge, no latch, completed AHRS-reset prerequisites and the arming gates. Some faults set pending AHRS reset explicitly; the diagram does not claim every fault performs an automatic reset.
- **Altitude-hold exit:** the 1 kHz watchdog sets a stale pending flag. `fc_core_service_async()` processes it in the main loop, invalidates barometer state and resets altitude hold. [fc_alt_hold.c](../firmware/stm32/FlightController/fc_control/fc_alt_hold.c) requires valid data and entry gates, with OFF→ON for re-entry after the blocked/stale state. A barometer stale event is not illustrated as a global motor-stop latch.

## Editing

Run `python media/generate_diagrams.py` from any directory to regenerate the deterministic SVG artwork; it uses only the Python standard library. The `.mmd` files preserve the semantic graph for Mermaid editing. Keep the two representations consistent when changing the diagrams. Diagrams use a 1600-pixel viewBox and text/vector elements for GitHub display; no remote image service is involved.
