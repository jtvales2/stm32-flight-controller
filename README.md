# STM32 Flight Controller

**My STM32F407-based quadrotor flight-control firmware, built around real-time sensor acquisition, timestamp-aware estimation, closed-loop control, and explicit fault-handling paths.**

`STM32F407` · `BMI088` · `MS5611` · `SPI DMA` · `Ring Buffer` · `Mahony` · `Cascaded Control` · `Failsafe`

[简体中文](README.zh-CN.md) · [Source code](firmware/stm32/) · [Build guide](BUILD.md) · [Third-party notices](THIRD_PARTY_NOTICES.md)

## Why I built it

For me, building a flight controller was about more than getting a PID loop to run. I wanted to work through the entire embedded control path: **when sensor data becomes available, how it is acquired without blocking interrupts, how samples are matched in time, what happens when processing falls behind, and how the system responds to faults before driving the motors.**

I organized the firmware around an STM32F407, BMI088 inertial sensors, an MS5611 barometer, SBUS input, and four motor outputs. My main focus was on making acquisition, estimation, control, and safety responsibilities clear in the source code.

## System architecture

![Architecture of my STM32 flight-controller firmware](media/system-architecture.svg)

I connect BMI088 data-ready interrupts and asynchronous SPI DMA transfers to timestamped buffers, then process sensor pairing, attitude estimation, and control in the main loop before motor mixing. SBUS reception, barometer processing, and safety-state management are integrated alongside that path.

| Area | My implementation |
| --- | --- |
| MCU | STM32F407ZGTx, Cortex-M4F; nominal 168 MHz HSI/PLL configuration |
| IMU | Independent BMI088 accelerometer/gyroscope DRDY events, SPI1 DMA, timestamps, and buffering |
| Estimation | My C implementation of a Mahony-based six-axis quaternion estimator with timestamp-aware sensor inputs |
| Control | Roll/pitch outer angle P and inner rate PD loops; separate yaw control and four-motor mixing |
| Altitude-related modules | My MS5611 driver; barometric altitude/vertical-speed processing and altitude-hold code |
| RC and actuation | SBUS over USART6 DMA; four PWM motor outputs |
| Safety and diagnostics | DMA timeout recovery, sample freshness checks, arming gates, latched stop behavior, and diagnostic counters |

## My main design focus: time-aware sensor data

![BMI088 asynchronous acquisition and timestamp pipeline](media/imu-pipeline.svg)

### 1. Keep interrupt work short and acquire data asynchronously

I use independent BMI088 accelerometer and gyroscope data-ready signals to record pending acquisition events and their timestamps. SPI DMA then handles transfers outside the data-ready interrupt. Because the two sensors share acquisition resources, the pipeline prioritizes gyroscope work while periodically servicing pending accelerometer requests.

**Code:** [pipeline.c](firmware/stm32/Core/Src/pipeline.c) · [main.c](firmware/stm32/Core/Src/main.c)

### 2. Separate acquisition from consumption and check sample age

I placed timestamped ring buffers between acquisition and main-loop processing. The estimator is driven by gyroscope timestamps: it selects an accelerometer sample no newer than the gyro sample, checks its age, and derives integration `dt` from the gyro timeline. This allows stale accelerometer correction to be limited rather than silently treating old samples as current data.

**Code:** [ringbuf_spsc.c](firmware/stm32/Core/Src/ringbuf_spsc.c) · [sync_pair.c](firmware/stm32/Core/Src/sync_pair.c) · [fusion_mahony.c](firmware/stm32/Core/Src/fusion_mahony.c)

### 3. Define behavior for backlog and DMA faults

I did not assume the main loop would always keep up. The firmware tracks queue depth and sample lag, adjusts processing work to catch up, and can discard older events or samples under backlog conditions. SPI DMA timeout recovery and fault escalation are separate from the normal sampling path.

These are concrete design choices for how an embedded control system behaves under load or faults; measured latency, throughput, and jitter still require on-target profiling.

**Code:** [pipeline.c](firmware/stm32/Core/Src/pipeline.c) · [main.c](firmware/stm32/Core/Src/main.c)

## Attitude estimation and control

I wrote `fusion_mahony.c/h` as a C implementation based on the published Mahony estimation method. In the controller, roll and pitch follow a cascaded angle-to-rate structure, yaw has a separate path, and the mixer distributes bounded commands to the four outputs.

I also wrote the `ms5611.c/h` barometer driver and integrated it with altitude and vertical-speed estimation and altitude-hold code. **Having the code is not the same as demonstrating altitude-hold flight performance**; I do not present uncorrelated historical material as validation of this cleaned source snapshot.

**Code:** [Mahony](firmware/stm32/Core/Src/fusion_mahony.c) · [Angle loop](firmware/stm32/FlightController/fc_control/fc_angle.c) · [Rate loop](firmware/stm32/FlightController/fc_control/fc_rate.c) · [Yaw](firmware/stm32/FlightController/fc_control/fc_yaw.c) · [Mixer](firmware/stm32/FlightController/fc_output/fc_mixer_out.c) · [MS5611](firmware/stm32/Core/Src/ms5611.c) · [Altitude hold](firmware/stm32/FlightController/fc_control/fc_alt_hold.c)

## Fault handling and re-arming

![DMA recovery, fault latching, and guarded re-entry](media/fault-handling.svg)

I separated recovery from permission to run the motors. The firmware includes SPI/DMA recovery attempts, sensor freshness checks, receiver and arming gates, motor-stop handling, and guarded re-arming after a latched fault. Recovering sensor data alone should not automatically re-arm the vehicle.

**Code:** [fc_fs.c](firmware/stm32/FlightController/fc_safety/fc_fs.c) · [fc_arm.c](firmware/stm32/FlightController/fc_safety/fc_arm.c) · [fc_core.c](firmware/stm32/FlightController/fc_core/fc_core.c)

## My bench-tuning records

I used VOFA to inspect transient responses during bench tuning. I kept both strongly oscillatory and decaying traces because they document the actual debugging process—not just its most presentable moments.

| Dec 8, 2025 · Oscillatory transient | Dec 8, 2025 · Decaying response |
| --- | --- |
| ![Original VOFA bench capture with a marked oscillatory transient](media/bench/vofa-2025-12-08-221113.png) | ![Original VOFA bench capture showing a decaying response](media/bench/vofa-2025-12-08-220512.png) |

[Another dual-trace bench capture from Dec 9, 2025](media/bench/vofa-2025-12-09-212009.png)

These are my original historical captures. The old `I0–I5` channel mapping, physical units, exact excitation conditions, and corresponding firmware revision have not been fully recovered. I therefore do not infer PID gains, percentage improvements, or quantified stability metrics from the screenshots. I plan to add aircraft photographs, bench clips, and flight footage with their original context.

## Source, build, and current verification scope

The repository contains the STM32 firmware source, Keil project, CubeMX metadata, and required HAL/CMSIS dependencies. Start with [fly1.0.uvprojx](firmware/stm32/MDK-ARM/fly1.0.uvprojx) and see [BUILD.md](BUILD.md) for toolchain details.

My earlier Keil rebuild record reported **0 errors / 4 warnings**, with successful linking and HEX generation. I have not independently rebuilt this stripped-down publication snapshot. Some interrupt-state paths were host-tested using mocked primitives; full on-target timing, fault behavior, altitude hold, and flight safety are not established by those tests. **This repository is a source and engineering portfolio, not a flight-qualified firmware release.**

In the publication copy, `FC_MOTOR_TEST_ENABLE=0` and `FC_ESC_CAL_ENABLE=0`. Remove all propellers before motor or bench checks.

**Authorship and licensing:** I wrote the specific implementations in `ms5611.c/h` and `fusion_mahony.c/h`; I do not claim invention of the Mahony method or ownership of sensor protocols. Third-party ST, Arm CMSIS, and Bosch components retain their notices. A root license covering the remaining project-owned source has not yet been finalized; public visibility alone does not grant reuse rights. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
