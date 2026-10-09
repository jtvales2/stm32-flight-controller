# STM32 Flight Controller

A modular quadrotor flight controller for **STM32F407ZGTx**, integrating asynchronous BMI088 acquisition, timestamped sensor pairing, Mahony attitude estimation, angle/rate/yaw control, SBUS input, and MS5611 altitude-hold code.

[中文](README.zh-CN.md) · [Build](BUILD.md) · [Validation](docs/validation-status.md) · [Change records](docs/changes/README.md) · [Evidence guide](docs/evidence-guide.md)

The project documents the complete path from sensor interrupts to motor commands. Its most useful entry points are the acquisition pipeline, concurrency boundaries, controller scheduling, and explicit arming/fault state transitions. The preserved `fly1.0` firmware is organized for source review and reproducible Keil builds.

**Status:** release preparation. A recorded Keil rebuild passed with **0 errors / 4 warnings**; host interrupt regression checks passed with mocks. Target runtime, hardware, flight, and altitude-hold validation remain pending. See the dated [validation record](docs/validation-status.md) for the current scope. Project licensing and broader authorship confirmation must be completed before public release.

## System architecture

![Code-derived system architecture](media/system-architecture.svg)

The main loop services IMU DMA, SBUS reception, barometer conversions, and deferred work. It consumes gyro samples to run fusion and control; a separate TIM2 interrupt calls the 1 kHz watchdog. Angle/rate mode selection, altitude-hold requests, pre-arm calibration, and output limits are implemented in separate modules.

| Interface | Implementation in this source tree |
| --- | --- |
| MCU | STM32F407ZGTx, Cortex-M4F; internal 16 MHz HSI, nominal 168 MHz SYSCLK; runtime timing verification pending |
| IMU | BMI088 accel/gyro, DRDY timestamps, SPI1 DMA |
| Receiver | SBUS on USART6 with circular receive DMA |
| Attitude | Mahony 6-DoF quaternion filter; Euler attitude for control |
| Barometer | MS5611 scheduled SPI conversions; relative altitude/vertical-speed estimation |
| Actuation | Four TIM3 PWM channels, mixing and bounded motor commands |
| Diagnostics | UART logging, pipeline counters, timestamp/lag statistics, controller events |

The CH6 three-position request selects ACRO/rate, ANGLE, or ANGLE with altitude hold. Actual mode changes and altitude-hold entry follow their code gates; selecting a switch position does not establish that a mode has been hardware validated. The 6-DoF configuration does not provide magnetometer-referenced absolute heading.

## Asynchronous acquisition and timing

![Code-derived IMU acquisition and processing pipeline](media/imu-pipeline.svg)

DRDY callbacks publish pending timestamps rather than performing blocking sensor reads. DMA arbitration gives gyro priority while serving pending accel after three completed gyro transfers. Multiple pending events coalesce; merge/drop counters expose the skipped events. Timestamped rings connect the interrupt producer to the main-loop consumer.

Pairing consumes the oldest gyro sample and the latest accel sample whose timestamp is no later than the gyro timestamp. It derives `dt` from gyro timestamps, rejects nonmonotonic timestamps, and disables accel correction when the paired sample exceeds its age limit. Queue depth and sample lag guide processing budgets and catch-up behavior. These are implementation policies; measured throughput and latency require real logs.

## Fault handling and re-entry

![Code-derived peripheral recovery, latched stop and altitude-hold exit flows](media/fault-handling.svg)

SPI/DMA recovery can restart acquisition. Flight faults latch a reason, disarm, reset controller state, and write stop pulses. Returning sensor or receiver data does not automatically re-arm the aircraft: latch clearing and a fresh ARM transition have explicit prerequisites. Stale barometer data exits altitude hold through deferred main-loop service, with guarded switch-based re-entry.

## Read the implementation

All paths below are relative to `firmware/stm32`.

| Area | Entry points |
| --- | --- |
| Scheduling and integration | [main.c](firmware/stm32/Core/Src/main.c), [fc_core.c](firmware/stm32/FlightController/fc_core/fc_core.c) |
| IMU DMA and buffering | [pipeline.c](firmware/stm32/Core/Src/pipeline.c), [ringbuf_spsc.c](firmware/stm32/Core/Src/ringbuf_spsc.c) |
| Pairing and attitude | [sync_pair.c](firmware/stm32/Core/Src/sync_pair.c), [fusion_mahony.c](firmware/stm32/Core/Src/fusion_mahony.c) |
| Angle / rate / yaw | [fc_angle.c](firmware/stm32/FlightController/fc_control/fc_angle.c), [fc_rate.c](firmware/stm32/FlightController/fc_control/fc_rate.c), [fc_yaw.c](firmware/stm32/FlightController/fc_control/fc_yaw.c) |
| Barometer / altitude hold | [ms5611.c](firmware/stm32/Core/Src/ms5611.c), [fc_baro.c](firmware/stm32/FlightController/fc_estimator/fc_baro.c), [fc_alt_hold.c](firmware/stm32/FlightController/fc_control/fc_alt_hold.c) |
| Arming / fault protection | [fc_arm.c](firmware/stm32/FlightController/fc_safety/fc_arm.c), [fc_fs.c](firmware/stm32/FlightController/fc_safety/fc_fs.c), [fc_time.c](firmware/stm32/FlightController/fc_core/fc_time.c) |
| Configuration / output | [fc_cfg.h](firmware/stm32/FlightController/fc_cfg/fc_cfg.h), [fc_mixer_out.c](firmware/stm32/FlightController/fc_output/fc_mixer_out.c), [motors.c](firmware/stm32/Core/Src/motors.c) |

## Build and verification

Open [fly1.0.uvprojx](firmware/stm32/MDK-ARM/fly1.0.uvprojx) in Keil and rebuild target `fly1.0`. The recorded toolchain is **ARM Compiler 5.06 update 5 build 528**, with `Keil.STM32F4xx_DFP 2.17.0`. Full instructions, changed defaults, warnings, and host-check commands are in [BUILD.md](BUILD.md).

| Evidence | Recorded result / limit |
| --- | --- |
| Full target build | Compilation, linking and HEX generation passed; 0 errors, 4 warnings in the current follow-up rebuild |
| Host regression | SBUS, barometer feed, deferred stale handling and gyro-bias update passed with mocked IRQ primitives, including incoming PRIMASK 0 and 1 |
| On-target concurrency / timing | Pending |
| Bench / flight / altitude hold | Pending; no validated flight binary or flight-ready claim |

`FC_MOTOR_TEST_ENABLE` and `FC_ESC_CAL_ENABLE` default to `0` in the release preparation copy. Preparation changes preserve control gains and IMU timing. Remove propellers before any hardware checks. The diagrams explain source behavior and are not test results; photographs, waveforms and video will be added only with their original evidence and conditions.

## Repository and licensing

```text
firmware/stm32/       Firmware, HAL/CMSIS dependencies, Keil project and CubeMX metadata
docs/                Change records, verification logs and validation documentation
media/               Source-derived SVG diagrams and editable Mermaid sources
BUILD.md             Toolchain and rebuild procedure
THIRD_PARTY_NOTICES.md  Dependency notices and provenance records
```

The author has confirmed independent authorship of `ms5611.c/.h` and `fusion_mahony.c/.h`. The project-owned license and remaining custom-source ownership are being finalized. Third-party Bosch, ST and CMSIS files retain their existing notices and applicable licenses; a project license must not replace them. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) and the [release checklist](docs/release-checklist.md).

The diagrams are generated locally from vector primitives with [generate_diagrams.py](media/generate_diagrams.py), and their logical flows are editable as `.mmd` files. [Diagram notes](docs/diagram-notes.md) link the illustrated behavior to the source.
