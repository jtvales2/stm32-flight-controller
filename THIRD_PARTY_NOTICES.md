# Third-party components / 第三方组件

本项目使用以下现有软件组件，我负责在其基础上开发和集成飞控应用代码：

- **STMicroelectronics** — STM32 HAL、STM32F4 器件支持、CubeMX 生成模板及启动代码。
- **Arm** — CMSIS 核心头文件与支持组件。
- **Bosch Sensortec** — BMI08x/BMI088 Sensor API。

这些组件的版权归各自权利方所有，继续遵循源文件中的声明及相应许可证。随工程保留的 ST/CMSIS 许可证位于各自 `Drivers/` 目录；Bosch 的许可声明见其源码文件头。

本项目中由我编写且拥有授权权利的飞控应用代码采用 **BSD-3-Clause License**。

STM32 HAL、CMSIS、Bosch Sensor API 和 ST 提供的生成模板等第三方代码，继续遵循各自适用的许可证及版权声明，不因本仓库的 BSD-3-Clause 授权而改变。

第三方组件的原始版权头及许可证文件予以保留。
