# Release preparation checklist

This checklist covers the review of stages one through four and records remaining work. A passed rebuild does not complete hardware, flight or licensing validation.

## Completed preparation

- [x] Preserve the historical project in an independent `Original/fly1.0` directory.
- [x] Copy its contents directly into the independent `OpenSource/stm32-flight-controller/firmware/stm32` directory.
- [x] Record the historical file inventory and hashes in [original-manifest.csv](original-manifest.csv).
- [x] Audit Keil source/include references before removing unused dependencies.
- [x] Remove old build products, user/session files, unused CMSIS trees and empty MAVLink files; record omissions in [removed-files.txt](removed-files.txt).
- [x] Retain the project, regeneration files, startup assembly, HAL, required CMSIS files, RTE and existing licenses.
- [x] Change only `FC_MOTOR_TEST_ENABLE` and `FC_ESC_CAL_ENABLE` from `1` to `0` as safety defaults.
- [x] Record the two-function PRIMASK remediation separately from configuration changes.
- [x] Remove the four unused MAVLink include paths without changing the other paths.
- [x] Complete a full Keil rebuild and retain its log: **0 errors, 4 warnings**.
- [x] Complete host checks for both PRIMASK values, applicable SBUS/barometer paths and valid-last ordering using mocked IRQ primitives.
- [x] Verify final source preservation: all 1,773 historical files unchanged; final firmware has 495 byte-identical files and 5 recorded edits.
- [x] Recheck all 64 project file references and 13 retained include directories after cleanup.
- [x] Review and record all four unused-function warnings in [BUILD.md](../BUILD.md); retain the existing helpers without expanding this source change.
- [x] Add conservative Git ignore rules for Keil products and user configuration.

## Verification still required

- [ ] Verify both affected functions on the STM32 target with incoming PRIMASK `0` and `1`.
- [x] Fix `fc_core_service_async()` and `imu_set_gyro_bias_dps()` in a separate follow-up; review call contexts and record passing host checks and a full rebuild.
- [ ] Perform board checks with all propellers removed, recording board/sensor identities and the tested firmware revision.
- [ ] Verify IMU acquisition/timestamps, receiver operation, arming/fault responses and motor output on hardware.
- [ ] Confirm actual MS5611 operation before presenting barometer or altitude-hold validation.
- [ ] Record any later flight validation with its setup, conditions and limitations.

## Stage five and public-release decisions

- [ ] Identify each original C/H file's encoding before any UTF-8 conversion; preserve an auditable byte baseline.
- [ ] Recover comments only where their original meaning can be established; avoid bulk reformatting.
- [x] Record owner confirmation that `ms5611.c/h` and `fusion_mahony.c/h` were independently written.
- [ ] Finalize their chosen license and confirm the remaining custom-code ownership scope.
- [ ] Confirm ownership of project additions and choose their license; document the exact covered scope.
- [ ] Resolve missing package-license provenance and supplemental third-party license texts while retaining existing headers/notices.
- [x] Prepare the English/Chinese homepage, three source-derived diagrams, and evidence instructions.
- [ ] Add real photos, waveforms and flight materials; the owner will handle these.
- [ ] Place historical waveforms in validation documentation only with established channel meanings and provenance.
- [x] Prepare repository-only packaging with backup/output exclusions and archive CRC/byte checks.
- [ ] Measure runtime clock and peripheral timing on target; the nominal HSI/PLL SYSCLK configuration is 168 MHz.

Publishing is a later decision. This preparation stage does not add an assumed project license, claim flight readiness or publish the repository.
