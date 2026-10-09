# Source Provenance and Ownership Inventory

**Project:** STM32 Flight Controller  
**Original audit date:** 2026-10-08  
**Authorship reconfirmation:** 2026-10-10  
**Status:** Source ownership review in progress

This document records the provenance, authorship, and third-party licensing boundaries of the STM32 flight controller firmware.

The accompanying [source-ownership.csv](source-ownership.csv) contains the inspected source paths, ownership classifications, and SHA-256 hashes recorded during the original source audit.

## 1. Author-Written Sensor and Estimation Components

The project owner has confirmed that the following four files were written independently by the repository author:

| Source file | Description | Authorship status |
| --- | --- | --- |
| `firmware/stm32/Core/Src/ms5611.c` | MS5611/MS5607 barometric sensor driver implementation | Author-confirmed |
| `firmware/stm32/Core/Inc/ms5611.h` | Sensor definitions, data structures and public interfaces | Author-confirmed |
| `firmware/stm32/Core/Src/fusion_mahony.c` | Mahony-based attitude estimation implementation | Author-confirmed |
| `firmware/stm32/Core/Inc/fusion_mahony.h` | Attitude estimator configuration, state structures and interfaces | Author-confirmed |

**Authorship clarification:**

The authorship statement applies to the actual C/C++ source code implementation and does not claim ownership or original invention of the underlying mathematical algorithms, published methods, hardware communication protocols or manufacturer specifications.

These four files are identified as author-written implementations based on the project owner's explicit confirmation. This declaration does not automatically determine the ownership of other firmware files.

## 2. MS5611 Barometric Sensor Driver

**Files:**
- `Core/Src/ms5611.c`
- `Core/Inc/ms5611.h`

**Implementation authorship:** Repository author.

The inspected driver implements functionality including:

- STM32 HAL-based SPI sensor communication
- ADC conversion and data acquisition
- Calibration and pressure compensation
- MS5611/MS5607-related handling
- CRC4 processing
- Pressure-derived altitude interfaces
- Sensor polling and conversion scheduling

The implementation follows the publicly documented operating principles and mathematical formulas of the sensor family.

Reference material:

[TE Connectivity Pressure Sensor Documentation](https://www.te.com/content/dam/te-com/documents/sensors/global/te-sensor-solutions-catalog-pressure-sensors.pdf)

The exact datasheet and application-note editions used during the original development have not been independently established.

**Source code authorship and sensor specification ownership are separate matters.**

## 3. Mahony Attitude Estimation

**Files:**
- `Core/Src/fusion_mahony.c`
- `Core/Inc/fusion_mahony.h`

**Implementation authorship:** Repository author.

The inspected implementation contains functionality including:

- Quaternion-based attitude representation
- Gyroscope-based attitude propagation
- Accelerometer-based correction
- Configurable correction gains
- Gyroscope bias-related state handling
- Sensor normalization and validity handling
- Time-step processing
- Estimator configuration and state interfaces

The implementation is based on established Mahony attitude estimation principles.

**Algorithm reference:**

Mahony, R., Hamel, T., and Pflimlin, J.-M. (2008).

*Nonlinear Complementary Filters on the Special Orthogonal Group.*

IEEE Transactions on Automatic Control, 53(5), 1203–1218.

DOI: 10.1109/TAC.2008.923738

The repository author claims authorship of the specific implementation, not invention of the Mahony algorithm.

The existence of similar mathematical formulas in other implementations does not, by itself, establish source-code copying. Conversely, authorship confirmation does not replace a review for any externally copied code fragments.

## 4. Other Application Firmware

The original source inventory covered 93 C/H files under `Core` and `FlightController`, together with startup assembly and third-party dependency scopes.

| Source category | Count | Ownership status |
| --- | ---: | --- |
| MS5611 and Mahony implementation files | 4 | Author-confirmed |
| Other custom-code candidates | 66 | Further confirmation required |
| Bosch BMI08x/BMI088 Sensor API | 6 | Third-party Bosch implementation |
| ST-header-bearing application and template files | 17 | ST portions identified; custom additions require review |
| STM32 startup assembly | 1 | ST implementation |
| HAL, device support and CMSIS dependency scopes | 3 scope entries | Third-party components |

These counts reflect the existing source inventory and are not additional files.

The lack of a copyright header does not prove original authorship.

Some application files may combine project-specific code with STM32CubeMX-generated initialization code. Their ownership boundaries must be determined before applying a project-owned license.

Refer to [source-ownership.csv](source-ownership.csv) for the detailed file-level inventory.

## 5. Third-Party Components

The firmware includes software components originating from:

- **STMicroelectronics:** STM32 HAL, device support, startup code and generated templates
- **Arm:** CMSIS components
- **Bosch Sensortec:** BMI08x/BMI088 Sensor API

Their existing copyright statements, license files and attribution notices must be retained.

Third-party source code is not claimed as independently written by the repository author.

See [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md).

## 6. Historical Source Encoding

The original firmware contains source files with legacy text encodings.

During the source audit, the four author-confirmed files were readable using GB18030-compatible decoding, but their exact original editor encoding settings were not independently verified.

Encoding compatibility is not evidence of authorship.

Any future conversion to UTF-8 should preserve source behavior and be recorded separately.

## 7. Licensing Status

The repository author's implementation ownership has been confirmed for the four MS5611 and Mahony files listed above.

The following items remain unresolved:

- Ownership verification of other custom application source files
- Review of project-specific modifications to generated or third-party code
- Selection of a license for independently owned source code
- Selection of the copyright holder display name
- Completion of any required third-party attribution or supplemental license files

No project-wide license is implied by this document.

See [license-scope.md](license-scope.md) for the licensing decision record.

## 8. 中文说明

本项目为基于 STM32F407 的自研四旋翼飞控系统。

项目作者明确确认以下四个文件由本人编写：

- `ms5611.c`
- `ms5611.h`
- `fusion_mahony.c`
- `fusion_mahony.h`

其中：

**MS5611 驱动：** 作者依据传感器公开技术资料，实现 SPI 通信、数据采集、补偿计算及高度相关接口。

**Mahony 姿态估计：** 作者依据已有的 Mahony 姿态估计理论，自行编写具体的软件实现。

上述声明仅针对具体源代码实现的作者身份，不声称原创发明 Mahony 算法或拥有传感器通信协议的知识产权。

ST、Arm CMSIS、Bosch 等第三方代码仍遵循各自的版权与许可证要求。

其余应用层源码的归属及最终开源许可证仍需进一步确认。

---

**Document status:** Authorship recorded for the four identified files. Broader source ownership and licensing review remain in progress.