# Project-Owned Licensing Scope

**Project:** STM32 Flight Controller  
**Status:** Licensing decision pending  
**Last updated:** 2026-10-10

## 1. Purpose

This document defines the current ownership and licensing boundaries for the STM32 flight controller source tree.

The repository contains independently written firmware components, third-party software dependencies, and STM32-generated application code.

These categories must not be treated as having identical ownership or licensing terms.

## 2. Confirmed Author-Written Components

The repository owner has confirmed authorship of the following source files:

| File | Ownership status |
| --- | --- |
| `firmware/stm32/Core/Src/ms5611.c` | Author-confirmed |
| `firmware/stm32/Core/Inc/ms5611.h` | Author-confirmed |
| `firmware/stm32/Core/Src/fusion_mahony.c` | Author-confirmed |
| `firmware/stm32/Core/Inc/fusion_mahony.h` | Author-confirmed |

These are author-written software implementations.

The MS5611 sensor specifications and Mahony algorithm principles are not claimed as inventions of the repository author.

The copyright and license applicable to any incorporated third-party material remain separate from the ownership of the author's original implementation.

## 3. Additional Custom Firmware

Other application-level firmware components are currently classified as custom-code candidates pending full ownership confirmation.

These include project-specific areas such as:

- IMU data acquisition and processing
- Flight control and scheduling
- Attitude and angular-rate control
- Sensor data buffering and synchronization
- Receiver handling
- Safety and failsafe logic
- Motor output and related interfaces

Listing these components does not constitute a blanket originality claim.

The file-level ownership record is maintained in [source-ownership.csv](source-ownership.csv).

## 4. Third-Party Software

The following dependencies retain their own copyright notices and licensing terms:

| Component | Rights holder |
| --- | --- |
| STM32 HAL and device support | STMicroelectronics |
| STM32-generated templates and startup code | STMicroelectronics |
| CMSIS | Arm and applicable contributors |
| BMI08x/BMI088 Sensor API | Bosch Sensortec |

All original third-party notices must be preserved.

A future project-owned license must not override these notices or incorrectly claim ownership of third-party implementations.

For further information, see [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md).

## 5. Mixed-Ownership Files

Some STM32CubeMX-generated files may contain both manufacturer-provided template code and project-specific additions.

Copyright ownership of newly written additions does not automatically transfer ownership of the original template.

These files must be reviewed with their existing notices preserved.

The applicable permissions for redistribution must be verified before public release.

## 6. Project License Selection

The repository owner has not yet finalized a project-owned software license.

Possible permissive licenses under consideration include:

- [MIT License](https://opensource.org/license/mit)
- [BSD 3-Clause License](https://opensource.org/license/bsd-3-clause)

Other licenses may be considered.

Before adding the repository's root `LICENSE`, the following must be confirmed:

1. Which source files and original contributions are covered.
2. Which third-party files retain separate licensing terms.
3. Which copyright display name will be used.
4. Whether all included third-party components may be redistributed under their applicable licenses.

No project-wide license is automatically assigned by this document.

## 7. Current Release Boundary

The firmware may be prepared and reviewed in a private repository while ownership and redistribution requirements are being completed.

A private repository does not resolve third-party licensing or ownership questions.

Public distribution should proceed only after the relevant copyright and license requirements have been reviewed.

This document is an ownership and licensing status record, not a substitute for a software license.

## 8. 中文说明

本项目包含三类代码：

**第一类：作者本人编写的代码**

目前已由作者明确确认：

- MS5611 气压计驱动实现
- Mahony 姿态估计实现

上述声明针对具体源代码，而非公开算法或硬件协议本身。

**第二类：其他待完成归属确认的应用层代码**

包括飞控控制、传感器数据处理、实时调度及安全机制等模块。后续需要逐文件确认来源和授权范围。

**第三类：第三方依赖及生成代码**

包括 STMicroelectronics、Arm CMSIS、Bosch Sensortec 的相关代码。这些部分必须遵循原有版权和许可证要求。

项目尚未最终选择整体适用的自有代码开源许可证。

在完成版权归属和第三方授权核查之前，不将整个源码树统一声明为 MIT、BSD-3-Clause 或其他许可证。

---

**Licensing status:** Author confirmation recorded for four files. Final project-owned license and broader ownership scope remain pending.