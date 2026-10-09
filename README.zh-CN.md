# STM32 飞行控制器

基于 **STM32F407ZGTx** 的模块化四旋翼飞控工程，包含 BMI088 异步采集、带时间戳的传感器配对、Mahony 姿态估计、角度/角速度/Yaw 控制、SBUS 遥控输入，以及 MS5611 定高相关代码。

[English](README.md) · [构建说明](BUILD.md) · [验证状态](docs/validation-status.md) · [修改记录](docs/changes/README.md) · [实测证据整理](docs/evidence-guide.md)

工程展示从传感器中断到电机指令的完整路径，重点包括异步采集、并发边界、控制调度以及解锁和故障状态切换。保留原有 `fly1.0` 实现，整理目录、构建入口和修改记录，方便代码审阅与技术交流。

**当前状态：开源准备。** 已记录的 Keil 全量构建为 **0 错误 / 4 警告**；使用中断模拟原语的主机回归检查已通过。目标板运行、实物、飞行及定高验证仍待完成，具体范围以[带日期的验证记录](docs/validation-status.md)为准。公开发布前还需完成原创代码授权和其余源码归属确认。

## 系统结构

![根据源码绘制的系统架构](media/system-architecture.svg)

主循环服务 IMU DMA、SBUS 接收、气压计转换和异步待处理事项，消费陀螺仪样本并驱动融合与控制。独立的 TIM2 中断调用 1 kHz 看门狗。模式选择、定高请求、解锁前校准和电机限幅分别由独立模块处理。

| 部分 | 当前源码实现 |
| --- | --- |
| 主控 | STM32F407ZGTx、Cortex-M4F；内部 16 MHz HSI，标称 SYSCLK 为 168 MHz；实际运行时序待上板验证 |
| IMU | BMI088 加速度计/陀螺仪，DRDY 时间戳与 SPI1 DMA |
| 遥控 | USART6 SBUS，循环接收 DMA |
| 姿态 | Mahony 六轴四元数滤波，输出欧拉角用于控制 |
| 气压计 | MS5611 调度式 SPI 转换，相对高度与垂直速度估计 |
| 电机 | TIM3 四路 PWM，混控与输出限幅 |
| 诊断 | UART 日志、采集计数器、时间戳/延迟统计及控制事件 |

CH6 三段开关分别请求 ACRO 角速度模式、ANGLE 自稳模式、ANGLE 加定高。实际模式切换与定高进入仍受代码中的条件约束，开关位置不代表功能已经通过实物验证。当前六轴配置不提供磁力计参考的绝对航向。

## 异步采集与时间处理

![根据源码绘制的 IMU 数据处理链路](media/imu-pipeline.svg)

DRDY 回调发布待处理时间戳，DMA 状态机负责传感器读取。仲裁优先服务陀螺仪；连续完成三次陀螺仪读取后，若加速度计待处理则插入一次读取。积压事件会合并，并通过 merge/drop 计数记录跳过事件。带时间戳的数据环连接中断生产者和主循环消费者。

配对模块消费最旧的陀螺仪样本，并选择时间戳不晚于它的最新加速度计样本。融合 `dt` 来自陀螺仪时间戳；非单调时间戳被拒绝，过旧的加速度计样本不参与姿态修正。主循环根据队列水位和样本延迟调整处理预算及追赶策略。图中描述的是实现策略，实际吞吐率和延迟需要原始日志支持。

## 故障处置与重新进入

![根据源码绘制的采集恢复、故障锁存和定高退出流程](media/fault-handling.svg)

SPI/DMA 恢复可以重新启动采集。飞行故障锁存原因、取消解锁、重置控制状态，并写入电机停止脉宽。传感器或遥控数据恢复不会自动重新解锁，清除锁存和新的 ARM 上升沿都需要满足相应条件。气压计数据过期由主循环处理并退出定高，重新进入需要有效数据、进入条件以及定高开关 OFF→ON。

## 代码入口

以下路径均位于 `firmware/stm32`。

| 模块 | 入口文件 |
| --- | --- |
| 主调度与集成 | [main.c](firmware/stm32/Core/Src/main.c)、[fc_core.c](firmware/stm32/FlightController/fc_core/fc_core.c) |
| IMU DMA 与缓冲 | [pipeline.c](firmware/stm32/Core/Src/pipeline.c)、[ringbuf_spsc.c](firmware/stm32/Core/Src/ringbuf_spsc.c) |
| 时间配对与姿态 | [sync_pair.c](firmware/stm32/Core/Src/sync_pair.c)、[fusion_mahony.c](firmware/stm32/Core/Src/fusion_mahony.c) |
| 角度/角速度/Yaw 控制 | [fc_angle.c](firmware/stm32/FlightController/fc_control/fc_angle.c)、[fc_rate.c](firmware/stm32/FlightController/fc_control/fc_rate.c)、[fc_yaw.c](firmware/stm32/FlightController/fc_control/fc_yaw.c) |
| 气压计与定高 | [ms5611.c](firmware/stm32/Core/Src/ms5611.c)、[fc_baro.c](firmware/stm32/FlightController/fc_estimator/fc_baro.c)、[fc_alt_hold.c](firmware/stm32/FlightController/fc_control/fc_alt_hold.c) |
| 解锁与故障保护 | [fc_arm.c](firmware/stm32/FlightController/fc_safety/fc_arm.c)、[fc_fs.c](firmware/stm32/FlightController/fc_safety/fc_fs.c)、[fc_time.c](firmware/stm32/FlightController/fc_core/fc_time.c) |
| 配置与电机输出 | [fc_cfg.h](firmware/stm32/FlightController/fc_cfg/fc_cfg.h)、[fc_mixer_out.c](firmware/stm32/FlightController/fc_output/fc_mixer_out.c)、[motors.c](firmware/stm32/Core/Src/motors.c) |

## 构建与验证

在 Keil 中打开 [fly1.0.uvprojx](firmware/stm32/MDK-ARM/fly1.0.uvprojx)，全量重新构建目标 `fly1.0`。已记录工具链为 **ARM Compiler 5.06 update 5 build 528**，器件包为 `Keil.STM32F4xx_DFP 2.17.0`。构建步骤、变更默认值、警告及主机检查命令见 [BUILD.md](BUILD.md)。

| 验证层级 | 已记录结果及边界 |
| --- | --- |
| 目标工程全量构建 | 编译、链接、HEX 生成通过；本轮构建为 0 错误、4 警告 |
| 主机回归 | SBUS、气压计发布、异步过期处理及陀螺仪偏置更新使用中断模拟原语检查通过，覆盖进入时 PRIMASK 为 0 和 1 |
| 目标板并发与时序 | 待验证 |
| 台架、飞行与定高 | 待验证；未提供经过验证的飞行固件 |

开源准备副本中的 `FC_MOTOR_TEST_ENABLE` 与 `FC_ESC_CAL_ENABLE` 默认均为 `0`。整理过程保留控制增益及 IMU 时序。任何实物检查前先拆除桨叶。架构图说明源码行为，不作为测试结果；实物照片、波形和视频由作者后续补充，并附原始证据和条件。

## 目录与授权

```text
firmware/stm32/       固件、HAL/CMSIS 依赖、Keil 工程与 CubeMX 元数据
docs/                修改记录、验证日志与验证说明
media/               基于源码绘制的 SVG 与可编辑 Mermaid 源文件
BUILD.md             工具链和重新构建步骤
THIRD_PARTY_NOTICES.md  第三方声明与来源记录
```

作者已确认 `ms5611.c/.h` 与 `fusion_mahony.c/.h` 为独立编写。原创代码许可证和其余自定义源码归属仍在确认。Bosch、ST 和 CMSIS 文件保留已有版权及适用许可证，项目许可证不能替代第三方许可。详见[第三方声明](THIRD_PARTY_NOTICES.md)及[发布检查表](docs/release-checklist.md)。

图示通过 [generate_diagrams.py](media/generate_diagrams.py) 使用矢量元素确定性生成，逻辑流程同时保留 `.mmd` 源文件。[图示说明](docs/diagram-notes.md)列出对应源码和解释边界。
