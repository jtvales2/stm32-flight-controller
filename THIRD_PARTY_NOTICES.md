# Third-party notices and code ownership

The firmware contains the author's application code together with software maintained by STMicroelectronics, Arm, and Bosch Sensortec. Existing third-party headers and bundled license files are retained inside `firmware/stm32`. **The project's original code has not yet been assigned a repository-wide software license.** The notices here do not replace third-party license terms.

| Component | Location | Provenance / notice |
| --- | --- | --- |
| STM32 HAL | `firmware/stm32/Drivers/STM32F4xx_HAL_Driver/` | ST headers; see component `LICENSE.txt` |
| STM32F4 device support | `firmware/stm32/Drivers/CMSIS/Device/ST/STM32F4xx/` | ST headers; see device-support `LICENSE.txt` |
| CMSIS core | `firmware/stm32/Drivers/CMSIS/` | Arm notices and included CMSIS `LICENSE.txt` (Apache-2.0 for the relevant Core portions) |
| BMI08x/BMI088 sensor API | `firmware/stm32/Core/Src/bmi08a.c`, `bmi08g.c`, `bmi088.c` and associated Bosch headers | Copyright Bosch Sensortec; BSD-3-Clause notices in source headers |
| STM32-generated application and startup code | ST-header-bearing `Core/` files and `MDK-ARM/startup_stm32f407xx.s` | ST copyright notices apply to the provided portions |

**ST package licensing caveat:** The historical source snapshot includes ST component license pointers/fallback language, but the referenced `Package_license` file was not found during preparation. Confirm applicable STM32Cube redistribution terms and include any missing required license texts before declaring the public release complete.

## Author-written implementations

The repository owner explicitly confirmed that the **source implementations** in these four files were personally written:

- `firmware/stm32/Core/Src/ms5611.c`
- `firmware/stm32/Core/Inc/ms5611.h`
- `firmware/stm32/Core/Src/fusion_mahony.c`
- `firmware/stm32/Core/Inc/fusion_mahony.h`

Mahony estimation is an existing published method; this authorship statement concerns the particular C implementation, **not** original invention of the algorithm. MS5611 sensor communication/calibration principles likewise originate in manufacturer specifications. Any incorporated third-party fragments would still require appropriate attribution.

Other custom application files have not received an exhaustive file-by-file ownership confirmation. CubeMX-generated files can contain both ST template code and the author's additions. Do not apply a new copyright statement across entire directories without examining their contents.

## Status before public open-source licensing

- Confirm ownership of the remaining custom files and applicable third-party redistribution rights.
- Choose a license (for example MIT or BSD-3-Clause) for **only the code contributions you have the right to license** and add the appropriate copyright holder information.
- Preserve all ST/Arm/Bosch license files and per-file notices.

**Until the above decisions are completed, the repository has no root `LICENSE`; public visibility alone is not a grant of permission to reuse otherwise-unlicensed original code.**
