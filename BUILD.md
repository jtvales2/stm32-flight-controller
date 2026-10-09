# Build instructions

## Requirements

- Keil µVision 5 with **ARM Compiler 5.06 update 5 (build 528)** available
- **Keil.STM32F4xx_DFP 2.17.0** device pack (the version used for the recorded local rebuild)
- Target: **STM32F407ZGTx**; project target name: `fly1.0`

This project retains the required HAL, CMSIS headers, the Keil project, startup assembly, and CubeMX `.ioc` metadata. Rebuilding the checked-in Keil project does not require regenerating source code with CubeMX.

## Rebuild

1. Open `firmware/stm32/MDK-ARM/fly1.0.uvprojx` in Keil µVision.
2. Select target `fly1.0`. Confirm device, ARM Compiler 5 and include paths if your environment differs.
3. Choose **Project → Rebuild all target files**.
4. Inspect the entire Build Output, including warnings, link results and HEX generation.
5. Do not commit Keil-generated products or user-specific IDE configuration.

**Existing recorded result (before this documentation-only restructuring):** Keil rebuild completed linking and HEX generation with **0 errors / 4 warnings**. Warnings concerned unused `vofa_send_att_csv`, `fc_motor_tool_write_all`, `fc_motor_tool_write_one`, and `fc_motor_tool_combo`. The detailed historical build logs and verification scripts are kept in the author's **separate offline audit archive**, not in this minimal GitHub portfolio. **The rebuilt archive supplied here has not been independently compiled on the target toolchain.**

## Clock configuration

`SystemClock_Config()` configures the internal **HSI 16 MHz** oscillator as PLL source (`PLLM=8`, `PLLN=168`, `PLLP=2`), yielding a *nominal* **168 MHz SYSCLK**. `HSE_VALUE=25 MHz` may appear in definitions, but HSE is not the selected PLL source in this firmware. Actual clock/peripheral timing remains to be measured on hardware.

## Differences from the historical firmware

- `FC_MOTOR_TEST_ENABLE`: original `1`, preparation copy `0`
- `FC_ESC_CAL_ENABLE`: original `1`, preparation copy `0`
- PRIMASK preservation changes in short interrupt-protected sections of `sbus.c`, `fc_core.c`, and `imu_bringup.c`
- Removed unused MAVLink include search paths from the `.uvprojx`

The controller gains, motor mixing, and IMU timing policies were not intentionally redesigned as part of this release preparation. See the C source to confirm the exact revision in use.

## Safety and validation limits

**Remove all propellers before any board, motor, or receiver bench test.** Disabling the two tool features does not make the firmware flight-qualified. Before actual flight, independently verify sensor identities, motor directions, IMU timing, RC loss handling, arming and failsafe states, and the complete hardware configuration. A successful compilation is not evidence of safe flight behavior.
