# Third-party components / 第三方组件

本项目使用以下现有软件组件，我负责在其基础上开发和集成飞控应用代码：

- **STMicroelectronics** — STM32 HAL、STM32F4 器件支持、CubeMX 生成模板及启动代码。
- **Arm** — CMSIS 核心头文件与支持组件。
- **Bosch Sensortec** — BMI08x/BMI088 Sensor API。

这些组件的版权归各自权利方所有，继续遵循源文件中的声明及相应许可证。随工程保留的 ST/CMSIS 许可证位于各自 `Drivers/` 目录；Bosch 的许可声明见其源码文件头。

本项目自有代码尚未统一选择开源许可证。在正式授权再分发前，仍需核实适用的 ST 包级许可条款；公开仓库本身不自动授予未许可代码的使用权。
