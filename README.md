# Gimbal --- 步兵云台电控系统

基于 STM32F405RGT6 的 RoboMaster 步兵机器人云台电控方案，采用 FreeRTOS 实时多任务架构，通过 CAN 总线控制 DJI 官方电机（GM6020 / M3508 / M2006），实现云台 Yaw/Pitch 角度闭环、拨盘供弹与摩擦轮发射等功能。

## 硬件架构

| 组件 | 型号 | 说明 |
|------|------|------|
| MCU | STM32F405RGT6 | 168MHz, Cortex-M4F |
| 云台 Yaw 电机 | GM6020 | 大扭矩无刷电机, 支持角度/速度回传, CAN2 |
| 云台 Pitch 电机 | GM6020 | 同上, CAN1 |
| 摩擦轮 x2 | M3508 | 高速无刷电机, CAN1, ID 1/2 |
| 拨盘电机 | M2006 | 拨弹盘驱动, CAN1, ID 3 |
| 遥控器 | DR16 接收机 | DBUS 协议, USART2 + DMA |
| 上位机/调试 | --- | USART3 (printf 重定向), 100Hz 日志输出 |
| 姿态传感器(预留) | MPU6050 | I2C1, DMP 驱动未启用 |

CAN 总线拓扑：

- **CAN1**: 摩擦轮左(M3508 ID1) + 摩擦轮右(M3508 ID2) + 拨盘(M2006 ID3) + Pitch(GM6020 ID2)
- **CAN2**: Yaw(GM6020 ID4)

## 软件架构

### RTOS 任务

基于 FreeRTOS (CMSIS-RTOS2) 的多任务设计，共 12 个线程 + 3 个消息队列：

| 任务 | 优先级 | 周期 | 职责 |
|------|--------|------|------|
| InitTask | Realtime7 | 一次性 | 硬件初始化(CAN/DBUS/PID)，运行后自销毁 |
| DR16DataTask | Realtime2 | 队列阻塞 | 解析 DBUS 遥控器数据 (S.Bus) |
| RemoteTask | Realtime1 | 3ms | 遥控器超时保护与电机状态检查 |
| AngleCalculateT | Realtime | 1ms | Yaw/Pitch/拨盘角度累积计算 |
| YawTask | AboveNormal7 | 1ms | Yaw 级联 PID 控制 |
| PitchTask | AboveNormal6 | 1ms | Pitch 级联 PID 控制(上/下方向独立参数) |
| FrictionTask | AboveNormal5 | 1ms | 摩擦轮速度 PID 控制 |
| DriverTask | High4 | 1ms | 拨盘角度 PID 控制 |
| CANTask | High1 | 3ms | CAN 报文统一发送 |
| CAN1ProcessTask | High2 | 队列阻塞 | CAN1 接收数据解析 |
| CAN2ProcessTask | High3 | 队列阻塞 | CAN2 接收数据解析 |
| PrintTask | Low | 10ms | printf 调试输出 (USART3) |

### 控制算法

#### 级联 PID（位置环 + 速度环）

各轴详细参数：

| 轴 | 角度环 Kp/Ki/Kd | 速度环 Kp/Ki/Kd | 前馈 | 输出限幅 |
|---|---|---|---|---|
| Yaw | 1.633 / 0.030 / 2.690 | 1320 / 0 / 200 | 前馈 | +/-25V |
| Pitch(上) | 1.711 / 0.021 / 0.154 | 500 / 0 / 70 | 前馈 | +/-25V |
| Pitch(下) | 0.800 / 0.060 / 0.000 | 380 / 0 / 20 | 前馈 | +/-25V |
| 摩擦轮左 | --- | 4.2 / 0.001 / 0.45 | --- | +/-16384(电流) |
| 摩擦轮右 | --- | 4.2 / 0.001 / 0.45 | --- | +/-16384(电流) |

#### 关键设计点

- **前馈控制**: 目标角度变化时计算前馈分量叠加到速度环输出，提高动态响应
- **积分分离**: 积分输出限制在 +/-5，防止大幅超调
- **边缘触发**: 非空闲状态下单次触发角度步进，配合 200ms 消抖
- **死区**: 200 通道值的死区阈值，滤除摇杆零位漂移
- **角度累加**: 跨 360度 边界处理，实现多圈连续角度控制
- **一阶低通滤波**: 用于角度/速度平滑
- **遥控器保护**: 10 次(约30ms)无数据变化视为超时，切断电机输出

### 控制模式

- **云台**: 摇杆边缘触发, 30度/步 的角度步进
- **摩擦轮**: 摇杆 CH4 开关控制，目标转速 6000 RPM(M3508)
- **拨盘**: 摇杆 CH3 触发 1440度(4圈) 单次旋转
- **电机初始化**: 遥控器 SW2 切换至下档位时触发电机角度零位重置

## 项目结构

```
gimbal/
+--- Core/
|   +--- Inc/                 HAL 层头文件 (STM32CubeMX 生成)
|   |   +--- main.h
|   |   +--- can.h, dma.h, gpio.h, i2c.h, usart.h
|   |   +--- FreeRTOSConfig.h
|   +--- Src/                 HAL 层源文件 (STM32CubeMX 生成)
|       +--- main.c
|       +--- freertos.c       任务定义、消息队列、任务函数实现
|       +--- can.c, dma.c, gpio.c, i2c.c, usart.c
|       +--- stm32f4xx_hal_msp.c, stm32f4xx_it.c
+--- Drivers/                 CMSIS + STM32F4xx HAL 驱动库
+--- MDK-ARM/
|   +--- User/                用户应用代码
|   |   +--- control.c/h      云台/摩擦轮/拨盘控制任务
|   |   +--- pid.c/h          PID 控制器实现
|   |   +--- dbus.c/h         DR16 遥控器驱动
|   |   +--- drv_can.c/h      CAN 总线底层驱动
|   |   +--- calculate.c/h    工具函数(限幅/滤波/角度累加)
|   +--- Gimbal.uvguix.20829  Keil MDK 项目文件
+--- Middlewares/             第三方中间件(MPU6050 DMP库)
+--- Embedded/                Embedded IDE 项目配置(备选)
```

### 用户代码模块说明

| 模块 | 文件 | 职责 |
|------|------|------|
| control.c/h | 控制逻辑 | Yaw/Pitch 级联 PID、摩擦轮开关、拨盘触发、CAN 报文组包、遥控保护 |
| pid.c/h | PID 控制器 | 增量式 PID 实现，含积分限幅与输出限幅 |
| dbus.c/h | 遥控器驱动 | S.Bus 协议解析、遥控器超时检测 |
| drv_can.c/h | CAN 驱动 | CAN 初始化/发送/接收中断、电机数据统一解析 |
| calculate.c/h | 工具函数 | 16位高低字节转换、限幅、一阶低通滤波、电机多圈角度累加 |
| freertos.c | RTOS 配置 | 12 个任务函数实现、3 个消息队列、同步原语 |

## 快速开始

### 开发环境

- **IDE**: Keil MDK-ARM V5 (.uvguix 项目文件)
- **编译器**: ARM Compiler V5/V6
- **调试器**: ST-Link / J-Link
- **备选**: Embedded IDE

### 构建与烧录

1. 打开 MDK-ARM/Gimbal.uvguix.20829
2. 编译所有目标 (F7)
3. 连接 ST-Link 烧录 (F8)
4. 上电后 InitTask 自动完成 CAN/DBUS/PID 初始化并退出

### 调试

USART3 (PA2/PA3) printf 重定向，波特率 115200。每 10ms 输出：

`
yaw_target,yaw_actual,pitch_target,pitch_actual,friction_left_speed,friction_left_actual
`
