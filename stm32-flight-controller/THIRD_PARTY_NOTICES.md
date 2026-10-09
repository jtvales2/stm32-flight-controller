# Third-party notices

This is an inventory of notices present in the source tree during preparation, not a new license for the project. Existing copyright headers and license files are retained. No project-wide license has been selected or inferred for code of unresolved ownership.

| Component | Included locations | Notice found |
| --- | --- | --- |
| Arm CMSIS core headers | `firmware/stm32/Drivers/CMSIS/Include/` and retained `Drivers/CMSIS/Core/` | Arm copyright headers and SPDX `Apache-2.0`; full license at `Drivers/CMSIS/LICENSE.txt` |
| ST STM32F4 device support | `firmware/stm32/Drivers/CMSIS/Device/ST/STM32F4xx/` | ST copyright headers; component `LICENSE.txt` references package terms, with Apache-2.0 fallback when received without applicable package terms |
| ST STM32F4 HAL | `firmware/stm32/Drivers/STM32F4xx_HAL_Driver/` | ST copyright headers; component `LICENSE.txt` references package terms, with BSD-3-Clause fallback when received without applicable package terms |
| Bosch BMI08x/BMI088 Sensor API | `firmware/stm32/Core/Src/bmi08a.c`, `bmi08g.c`, `bmi088.c`; `Core/Inc/bmi08x.h`, `bmi08x_defs.h`, `bmi088.h` | Copyright (c) 2020 Bosch Sensortec GmbH; BSD-3-Clause text in file headers; headers identify version 1.4.4 |
| ST-generated application/device templates | ST-header-bearing files in `firmware/stm32/Core/` and `MDK-ARM/startup_stm32f407xx.s` | ST copyright headers reference a component root `LICENSE` file and state an AS-IS fallback if no such file accompanies the software |

Paths abbreviated in the notice column are relative to `firmware/stm32`. The retained license files are:

- [CMSIS Apache-2.0](firmware/stm32/Drivers/CMSIS/LICENSE.txt).
- [ST device-support notice](firmware/stm32/Drivers/CMSIS/Device/ST/STM32F4xx/LICENSE.txt).
- [ST HAL notice](firmware/stm32/Drivers/STM32F4xx_HAL_Driver/LICENSE.txt).

No `Package_license` file was found in the historical input. The ST component files above contain pointers and fallback statements, rather than full copies of all referenced license terms. Preserve those files and headers, and resolve any required supplemental license texts and package provenance before distribution. The root licensing decision must not replace licenses that apply to third-party components.

The project owner confirmed on 2026-10-08 that `Core/Src/ms5611.c`, `Core/Inc/ms5611.h`, `Core/Src/fusion_mahony.c` and `Core/Inc/fusion_mahony.h` were independently written. Their implementation ownership is recorded in [source-provenance.md](docs/source-provenance.md). Algorithm references are separate from implementation derivation and licensing. The preferred project-owned license and remaining custom-source ownership are pending; no root license is inferred.

ST-generated portions of `Core` and the startup file must not be covered by an assertion that every file in `Core` is project-original. Project-owned additions also require a confirmed ownership and licensing scope before a root `LICENSE` is added. See the [release checklist](docs/release-checklist.md).
