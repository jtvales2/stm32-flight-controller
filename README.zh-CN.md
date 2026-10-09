# STM32 自研飞控系统

**基于 STM32F407 的四旋翼飞控固件项目。** 重点展示异步 IMU 采集、带时间戳的数据处理、姿态估计、闭环控制，以及异常恢复与安全状态管理。

[English](README.md) · [固件源码](firmware/stm32/) · [编译说明](BUILD.md) · [第三方版权说明](THIRD_PARTY_NOTICES.md)

> **说明：** 这里公开的是经过目录整理的源码及部分历史台架波形。架构图根据源码绘制，不代表实测性能。已有 Keil 编译记录，但这份整理版本并非经过完整实机/飞行安全验证的固件。项目自有代码的最终授权范围仍需确认。

## 核心硬件与功能

| 层次 | 对应实现 |
| --- | --- |
| MCU | STM32F407ZGTx，Cortex-M4F，HSI/PLL 标称 SYSCLK 168 MHz |
| IMU | BMI088 加速度计、陀螺仪，独立 DRDY 与 SPI1 DMA |
| 姿态估计 | 时间戳配对、Mahony 六轴四元数姿态估计（具体代码由作者编写） |
| 控制 | Roll/Pitch 角度外环 P + 角速度内环 PD；Yaw 独立控制 |
| 气压计 | 作者编写的 MS5611 驱动、气压高度/垂直速度处理和定高控制逻辑 |
| 遥控与输出 | USART6 DMA SBUS 接收、四路电机 PWM 与混控 |
| 异常处理 | DMA 超时恢复、数据新鲜度、解锁约束、故障锁存与停止输出 |

## 系统架构

![基于源码绘制的飞控系统架构](media/system-architecture.svg)

系统由 IMU 中断/DMA 采集、带时间戳的缓冲区、主循环中的融合与控制，以及电机输出构成。图示是源码逻辑概览，**不是实时性实测结果**。

## 1. 异步 IMU 数据链

![BMI088 异步采集和数据处理链路](media/imu-pipeline.svg)

- BMI088 ACC/GYR DRDY 中断记录待处理事件及时间戳，不在中断内进行阻塞式 SPI 读取。
- SPI DMA 仲裁优先处理陀螺仪，并按策略穿插加速度计请求。
- 时间戳环形缓冲区连接采集与主循环，积压或溢出时可丢弃较旧样本；**不将其宣传为全程无锁**。
- 按陀螺仪时间戳配对加速度样本、检查新鲜度，并计算估计器的积分时间。主循环包含积压追赶策略。

源码：[DMA 管线](firmware/stm32/Core/Src/pipeline.c) · [Ring Buffer](firmware/stm32/Core/Src/ringbuf_spsc.c) · [时间配对](firmware/stm32/Core/Src/sync_pair.c) · [主循环](firmware/stm32/Core/Src/main.c)

## 2. 姿态估计与控制

Roll/Pitch 采用角度外环和角速度内环的级联控制结构，Yaw 采用独立控制路径，四电机混控对输出指令进行分配和约束。Mahony 方法属于已有公开算法，**本项目中的具体 C 源码由作者编写**，不宣称发明该算法。

源码：[Mahony](firmware/stm32/Core/Src/fusion_mahony.c) · [角度环](firmware/stm32/FlightController/fc_control/fc_angle.c) · [角速度环](firmware/stm32/FlightController/fc_control/fc_rate.c) · [Yaw](firmware/stm32/FlightController/fc_control/fc_yaw.c) · [混控输出](firmware/stm32/FlightController/fc_output/fc_mixer_out.c)

另外保留了 [MS5611 驱动](firmware/stm32/Core/Src/ms5611.c) 和 [定高逻辑](firmware/stm32/FlightController/fc_control/fc_alt_hold.c)，**不能仅凭代码存在就宣称定高实飞性能已经验证**。

## 3. 故障处理与重新解锁

![外设恢复、安全锁存与重新解锁流程](media/fault-handling.svg)

工程具有 SPI/DMA 异常恢复路径、解锁/故障条件检查、停止输出与重新解锁条件。气压计样本过期也可触发退出定高的处理。图中是**实现路径，不是完整故障注入测试证明**。

源码：[安全处理](firmware/stm32/FlightController/fc_safety/fc_fs.c) · [解锁状态](firmware/stm32/FlightController/fc_safety/fc_arm.c) · [核心集成](firmware/stm32/FlightController/fc_core/fc_core.c)

## 历史台架波形

以下是作者在 **2025 年 12 月** 保存的 VOFA 台架调试截图。由于当时 `I0–I5` 的通道映射、物理单位、固件版本和具体测试条件尚未恢复，**不根据图片推断具体 PID 参数、性能提升百分比或定量稳定性指标**。这些图片用于展示真实开发过程，不代表当前发布副本的性能验证。

| 2025-12-08：明显振荡的瞬态 | 2025-12-08：逐渐衰减的响应 |
| --- | --- |
| ![VOFA 历史台架波形：振荡瞬态](media/bench/vofa-2025-12-08-221113.png) | ![VOFA 历史台架波形：响应逐渐衰减](media/bench/vofa-2025-12-08-220512.png) |

另有 [2025-12-09 的双轨迹历史截图](media/bench/vofa-2025-12-09-212009.png)，通道含义尚未确认。

后续可将确认日期和测试条件的飞机照片、飞行视频链接分别添加到 `media/photos/` 和 `media/flight/`。当前仓库尚未包含这些材料。

## 构建与验证边界

- **构建入口：** [Keil 工程](firmware/stm32/MDK-ARM/fly1.0.uvprojx)，目标 `fly1.0`。已记录 ARM Compiler 5.06 update 5 build 528、STM32F4xx DFP 2.17.0；操作见 [BUILD.md](BUILD.md)。
- **历史本地编译记录：** 0 Error / 4 Warning，曾完成链接和 HEX 生成；原始日志由作者另行存档，本精简仓库不附带完整审计日志。本次重构**未重新执行 Keil 编译**。
- **历史主机回归：** 使用模拟中断原语验证了部分临界区状态恢复路径，不代表真实硬件并发验证。
- **当前尚未证明：** 当前副本的实机时序、全部故障保护、定高飞行结果及实飞安全性。
- **安全默认设置：** `FC_MOTOR_TEST_ENABLE=0`、`FC_ESC_CAL_ENABLE=0`。任何台架/电机检查前应拆除全部桨叶。

## 目录与版权

```text
firmware/stm32/      完整固件源码、HAL/CMSIS、CubeMX 和 Keil 工程
media/               系统架构图、可编辑 Mermaid 源图和历史台架波形
README.md            英文项目主页
BUILD.md             编译说明
THIRD_PARTY_NOTICES.md   第三方依赖、版权边界
```

项目作者确认 `ms5611.c/.h` 与 `fusion_mahony.c/.h` 由本人编写；其余自定义源码仍需完成归属确认。ST、Bosch、CMSIS 等第三方源文件保留原有声明。**当前尚未添加项目根目录 LICENSE**，公开源码并不自动意味着这些未明确授权的自有代码可以自由再使用。详情见 [版权说明](THIRD_PARTY_NOTICES.md)。
