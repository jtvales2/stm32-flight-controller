# Preparation change records

This directory retains the sealed first four preparation stages and records the separate follow-up below. The historical project is preserved outside this repository; its contents were copied directly into `firmware/stm32` to preserve Keil relative paths. No PID tuning, IMU timing change, broad formatting or bulk encoding conversion is part of this stage.

| Record | Purpose |
| --- | --- |
| [../original-manifest.csv](../original-manifest.csv) | Historical input inventory and file hashes used to compare the preserved copy |
| [../removed-files.txt](../removed-files.txt) | Files omitted from the preparation copy during cleanup |
| [001-safe-tool-defaults.patch](001-safe-tool-defaults.patch) | `FC_MOTOR_TEST_ENABLE` and `FC_ESC_CAL_ENABLE`: `1` to `0` |
| [002-preserve-primask.patch](002-preserve-primask.patch) | Interrupt-state preservation in `sbus_read_latest()` and `fc_core_baro_feed()`, with explicit STM32/CMSIS declarations |
| [003-remove-unused-mavlink-includes.patch](003-remove-unused-mavlink-includes.patch) | Removal of the four unused MAVLink include paths from the Keil project |
| [../verification/keil-rebuild.log](../verification/keil-rebuild.log) | Full Keil rebuild evidence for the preparation copy |
| [../verification/test_critical_sections.py](../verification/test_critical_sections.py) and [host-critical-sections.log](../verification/host-critical-sections.log) | Repeatable host checks using actual SBUS source and the exact barometer feed definition with mocked interrupt primitives |
| [../verification/release-audit.json](../verification/release-audit.json) | Final source preservation, remaining dependency checks and firmware hashes |

The patches are UTF-8 review records. Historical C/H files retain their existing bytes and mixed encodings except for the explicitly recorded source edits. Do not apply these review patches blindly to the original GBK/GB18030 or other historical files: verify the file bytes, context and encoding first. Use the preserved input and manifest when investigating any unexpected difference.

## Cleanup boundaries

Cleanup removes the old `MDK-ARM/fly1.0/` output, `MDK-ARM/DebugConfig/`, Keil user/session files and generated compilation products. The empty `mavlink2` directory and its four include paths are removed.

The sealed first-stage preservation audit found all **1,773** desktop source files and preserved-original files unchanged by hash. Cleaned firmware contains **500** files: **496** remain byte-identical and **4** have the recorded edits; **1,273** historical files are omitted. All **64** project file references and **13** retained include directories resolve. Historical high-byte comment sequences in the three edited C/H files remain unchanged. Fresh rebuild products were removed after verification logs were preserved.

An audit of the Keil source list, include paths, application sources and retained dependencies found no current dependency on `Drivers/CMSIS/DSP`, `NN`, `Documentation`, `DAP`, `Core_A`, `RTOS` or `RTOS2`; these unused trees are removed. `Drivers/CMSIS/Core` is retained conservatively, together with `Include`, `Device` and the license file. STM32 HAL is retained, as are `Core`, `FlightController`, `MDK-ARM/RTE`, `.uvprojx`, `.ioc`, `.mxproject` and `startup_stm32f407xx.s`.

The generated scatter file inside the old build-output directory is an output artifact. The ignore rules do not exclude custom `.sct` files elsewhere.

## Source boundaries

The tool-default patch changes only the two feature defaults. The interrupt patch preserves incoming PRIMASK on the affected functions' return paths, while retaining the existing shared-data operations and frame validity behavior. These records are separate so the configuration and concurrency changes can be reviewed independently.

The first-stage snapshot left `fc_core_service_async()` unchanged. Follow-up patch 004 now fixes that function and `imu_set_gyro_bias_dps()`; the sealed initial audit and patches remain intact. This work does not claim firmware-wide concurrency validation.

## Deferred work

Stage five remains pending: per-file encoding identification and careful UTF-8 conversion, comment recovery, implementation provenance, project-owned license selection, the full English/Chinese homepage and documented historical waveform interpretation. Existing third-party headers remain intact. No channel meanings, hardware results or flight results have been inferred from absent evidence.

## Separate follow-up

- [004-preserve-async-and-bias-primask.patch](004-preserve-async-and-bias-primask.patch): exactly two additional source files changed relative to the sealed baseline.
- [Concurrency review](../concurrency-review.md): caller/reader evidence and classification of existing returning and terminal IRQ patterns.
- [phase2-source-changes.json](../verification/phase2-source-changes.json) and [phase2-release-audit.json](../verification/phase2-release-audit.json): the before/after hashes and final preservation checks.
- [Source provenance](../source-provenance.md): the owner confirmed independent authorship of the MS5611 and Mahony C/H files. Remaining custom ownership and the chosen license are pending.
- English/Chinese homepages and source-derived diagrams are present. Real photos, waveforms and flight materials will be added by the owner.

No dependency cleanup was repeated. Only fresh files generated by the follow-up rebuild were removed after their logs and hashes were recorded. Bulk source encoding conversion remains deferred.
