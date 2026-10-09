# STM32 Flight Controller

**A source-available, STM32F407-based quadrotor flight-control firmware project**, focused on asynchronous sensor acquisition, time-aware estimation, closed-loop control, and fault handling.

[简体中文](README.zh-CN.md) · [Firmware](firmware/stm32/) · [Build guide](BUILD.md) · [Third-party notices](THIRD_PARTY_NOTICES.md)

> **Project scope:** This repository contains an author-maintained flight-controller implementation and selected historical bench captures. The architecture diagrams describe code paths, not measured performance. A recorded Keil build exists, but this cleaned publication copy is **not a flight-qualified firmware release**. The project-owned license and remaining authorship boundaries must be finalized before treating the entire codebase as permissively licensed open source.

## System at a glance

| Layer | Implementation |
| --- | --- |
| Microcontroller | STM32F407ZGTx (Cortex-M4F); HSI/PLL nominal SYSCLK 168 MHz |
| Inertial sensors | BMI088 accelerometer + gyroscope, independent DRDY and SPI1 DMA |
| Estimation | Timestamp-aligned samples and an author-written Mahony-based 6-DoF quaternion estimator |
| Control | Roll/pitch angle outer P loops and angular-rate PD loops; separate yaw control |
| Height-related code | Author-written MS5611 driver, barometric altitude/vertical-speed processing and altitude-hold logic |
| Input / output | SBUS over USART6 DMA; four-channel motor PWM and mixer |
| Fault paths | IMU/DMA timeout handling, sample freshness checks, arming guards and latched stop behavior |

## Architecture

![Flight controller software architecture, derived from source](media/system-architecture.svg)

The firmware connects sensor interrupts and DMA completions to timestamped data buffers. A main-loop consumer then performs pairing, attitude estimation and control before motor mixing. Receiver handling and safety checks are integrated into that execution path. This figure is a **source-derived overview**, not a timing measurement.

## 1. Timestamped IMU acquisition

![BMI088 acquisition and sensor-data pipeline](media/imu-pipeline.svg)

- Independent BMI088 ACC/GYR data-ready interrupts record pending events and timestamps instead of blocking on SPI inside the interrupt.
- SPI DMA transfers are arbitrated, prioritizing gyro while periodically servicing pending accelerometer work.
- Timestamped ring buffers bridge acquisition and processing. Overflow and backlog can discard older samples; this is **not** advertised as an entirely lock-free pipeline.
- Gyro-driven pairing picks an appropriate earlier accelerometer sample, checks sample age, and computes estimator `dt` from timestamps. The main loop includes catch-up policies when processing falls behind.

**Read the code:** [DMA pipeline](firmware/stm32/Core/Src/pipeline.c) · [Ring buffer](firmware/stm32/Core/Src/ringbuf_spsc.c) · [Timestamp pairing](firmware/stm32/Core/Src/sync_pair.c) · [Main scheduling](firmware/stm32/Core/Src/main.c)

## 2. Attitude and control

Roll and pitch use a cascaded structure (angle command → angular-rate command → rate controller). Yaw has a separate control path. The motor mixer constrains and distributes control commands to the outputs. The Mahony method is a published algorithm; **this project's C implementation is author-written**, not a claim of inventing the algorithm.

**Read the code:** [Mahony estimator](firmware/stm32/Core/Src/fusion_mahony.c) · [Angle loop](firmware/stm32/FlightController/fc_control/fc_angle.c) · [Rate loop](firmware/stm32/FlightController/fc_control/fc_rate.c) · [Yaw](firmware/stm32/FlightController/fc_control/fc_yaw.c) · [Mixing](firmware/stm32/FlightController/fc_output/fc_mixer_out.c)

Barometric control modules are included ([MS5611 driver](firmware/stm32/Core/Src/ms5611.c), [altitude hold](firmware/stm32/FlightController/fc_control/fc_alt_hold.c)); this source presence **does not by itself establish validated altitude-hold flight performance**.

## 3. Fault handling and safe re-entry

![Acquisition recovery, fault handling and guarded re-entry](media/fault-handling.svg)

The code contains SPI/DMA recovery attempts and flight-safety paths that can latch a stop, disarm and require a guarded re-arm. Barometer data freshness can also govern exit from altitude-hold logic. These are **implemented behaviors subject to on-target verification**, not a claim that all failure cases have been flight-tested.

**Read the code:** [Safety conditions](firmware/stm32/FlightController/fc_safety/fc_fs.c) · [Arming](firmware/stm32/FlightController/fc_safety/fc_arm.c) · [Integration](firmware/stm32/FlightController/fc_core/fc_core.c)

## Historical bench captures

The following original VOFA screenshots were recorded during controller bench tuning in **December 2025**. The old `I0–I5` channel map, physical units, firmware revision and test conditions have not been recovered, so **no PID gains, rise-time figures, stability margins or before/after performance improvements are inferred from these images**. They are retained as genuine process records, not numerical validation of the current release copy.

| 2025-12-08 — oscillatory transient | 2025-12-08 — decaying response |
| --- | --- |
| ![Historical VOFA bench trace showing a transient with oscillations](media/bench/vofa-2025-12-08-221113.png) | ![Historical VOFA bench trace showing a transient that decays toward zero](media/bench/vofa-2025-12-08-220512.png) |

**Another historical capture (2025-12-09):** [Two visible traces in VOFA](media/bench/vofa-2025-12-09-212009.png) (channel identities unverified).

Real aircraft photos and flight-video links can be added to `media/photos/` and `media/flight/` after their dates and test conditions are confirmed. The repository does not currently include such evidence.

## Build and verification scope

- **Build entry:** [Keil µVision project](firmware/stm32/MDK-ARM/fly1.0.uvprojx) — target `fly1.0`, ARM Compiler 5.06 update 5 build 528, STM32F4xx DFP 2.17.0. See [BUILD.md](BUILD.md).
- **Recorded local rebuild:** 0 errors, 4 unused-function warnings, with linking and HEX generation. The historical build log was retained separately by the author, not bundled into this minimal presentation repository. This archive has **not been independently rebuilt**.
- **Host-level regression:** interrupt-state preservation was previously checked with mocked interrupt primitives; this is not hardware-level concurrency validation.
- **Not established for this repository snapshot:** on-board runtime timing, full failsafe behavior, altitude-hold flight performance or suitability for operational flight.
- **Special tool defaults:** `FC_MOTOR_TEST_ENABLE=0` and `FC_ESC_CAL_ENABLE=0` in the published preparation copy. **Remove all propellers before bench or motor checks.**

## Directory layout

```text
firmware/stm32/        STM32 source, HAL/CMSIS, CubeMX metadata, Keil project
media/                Source-derived architecture SVGs, editable .mmd diagrams, authentic bench captures
README.zh-CN.md       Chinese project overview
BUILD.md              Minimal build instructions
THIRD_PARTY_NOTICES.md  Dependency licenses and attribution boundaries
```

### Attribution and release status

The repository author confirmed personal authorship of `ms5611.c/.h` and `fusion_mahony.c/.h`; ownership of additional custom components must be completed before a project-level license is applied. HAL/CMSIS/ST/Bosch sources keep their own notices. **No root LICENSE is included yet**; publication does not automatically grant permission to reuse otherwise-unlicensed original code. See [third-party notices](THIRD_PARTY_NOTICES.md).
