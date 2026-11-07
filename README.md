# DS_Gimbal — 2025 电赛 E 题「简易自行瞄准装置」瞄准模块

基于 **STM32F407VET6** 的双轴视觉云台固件，用于 2025 年全国大学生电子设计竞赛（TI 杯）本科组 **E 题：简易自行瞄准装置**。

题目中的装置由「MSPM0 循迹小车 + 二维云台瞄准模块」两部分组成；**本仓库只包含瞄准模块固件**（云台控制、视觉通信、激光控制），小车循迹部分不在本仓库。

## 赛题要点

- 靶纸位于 AB 边外 50 cm，A4 紫外感光纸，靶心有 2 / 4 / 6 / 8 / 10 cm 红圈
- 基本要求：2 s / 4 s 内自动瞄准并击中靶心，光斑距靶心 ≤ 2 cm
- 发挥要求：小车行驶过程中激光连续打靶；行驶 1 圈时在靶面画半径 6 cm 的圆，同步误差 < 1/4 圈

## 主要功能

- 自动扫描找靶、目标锁定、视觉伺服跟踪
- Yaw：IMU 绝对航向闭环 + 目标角速度前馈（速度模式）
- Pitch：MIT 位置/力矩混合控制，角度限幅 ±60°
- 激光打靶；6 cm 半径画圆（17 s/圈，2 s 渐入，yaw 透视补偿）
- 按键切换模式、IMU Z 轴校准、电机使能/失能
- OLED 显示当前模式，USART6 输出调试日志

## 硬件连接

| 功能 | 资源 | 说明 |
| --- | --- | --- |
| 主控 | STM32F407VET6 | 25 MHz HSE，PLL 168 MHz |
| Yaw 电机 | CAN1（PD0/PD1） | 达妙 DM4310，速度模式，电机 ID `0x01` / 反馈 `0x11` |
| Pitch 电机 | CAN2（PB5/PB6） | 达妙 DM4310，MIT 模式，电机 ID `0x01` / 反馈 `0x11` |
| IMU | 硬件 I2C3（PA8/PC9，400 kHz） | WIT 维特传感器，地址 `0x50`，读 Roll/Pitch/Yaw |
| OLED | 软件 I2C（PD12/PA10） | 0.96 寸 I2C OLED，地址 `0x78` |
| 视觉 | USB CDC（PA11/PA12） | 自定义 `0xAA` 协议，环形缓冲接收 |
| 调试串口 | USART6（PC6/PC7，115200） | `printf` 重定向 |
| 按键 | PC13 / PA6 / PB1 | 上拉输入，TIM4 50 Hz 扫描，松开沿触发 |
| 激光 | PB10 | 高电平点亮 |

两路 CAN 均工作在 **1 Mbps**。两个电机使用独立 CAN 总线，避免相同的默认 ID 在同一总线上冲突。

## 软件结构

```
DS_Gimbal/
├── Core/                 CubeMX 生成的 HAL 初始化、main、中断
├── BSP/
│   ├── DM_Motor/         达妙 DM4310 CAN 驱动（MIT / 速度模式、反馈解析）
│   ├── wit_c_sdk/        WIT IMU I2C SDK
│   ├── motor.c/h         云台控制环、PID、模式状态机、画圆
│   ├── uart_me.c/h       USB CDC 视觉协议解析
│   ├── OLED.c/h          0.96 寸 OLED 驱动（软件 I2C）
│   ├── key.c/h           按键扫描与模式切换
│   └── hal_hard_i2c.c    WIT SDK 的 HAL I2C 适配
├── Drivers/              STM32F4 HAL + CMSIS
├── Middlewares/          STM32 USB Device Library
├── USB_DEVICE/           USB CDC 应用与配置
├── MDK-ARM/              Keil MDK 工程
└── DS_Gimbal.ioc         STM32CubeMX 配置
```

## 运行时序

- **TIM3，1 kHz**：读取 IMU → 像素误差换算角度增量 → PID → CAN 下发（`BSP/motor.c` 的 `Gimbal_Control_Loop`）
- **TIM4，50 Hz**：按键扫描（`BSP/key.c`）
- **主循环**：USB CDC 字节流解析（`BSP/uart_me.c`）、按键动作、OLED 刷新
- 视觉数据超过 **200 ms** 未更新时，自动清除 `is_tracking`，退出跟踪并保持当前航向/俯仰

## 控制模式

| 模式 | 行为 |
| --- | --- |
| 0 | 待机（电机未使能 / 不运行控制环） |
| 1 | 搜索：pitch 回 0，yaw 以 2.5 rad/s 匀速旋转；检测到目标后记录当前 IMU 绝对 yaw，切入模式 2 |
| 2 | 跟踪：像素误差→角度增量；yaw 走 IMU 绝对航向闭环 + 角速度前馈；pitch 走 MIT 位置控制；锁定后点亮激光 |
| 3 | 画圆：在跟踪基础上叠加正弦偏移，`α_pitch = atan(R/D)`，`α_yaw = α_pitch · cosγ`；17 s/圈，2 s 渐入 |

按键：`Key1` 循环切换模式，`Key2` 重新校准 IMU Z 轴，`Key3` 使能/失能电机。

## 视觉通信协议

帧格式：

```
0xAA | cmd | len | payload[len]
```

| cmd | 含义 | payload |
| --- | --- | --- |
| `0x02` | 目标像素坐标 | `x` int16、`y` int16（大端） |
| `0x03` | 目标距离 | int16，单位 **mm** |
| `0x08` | 目标像素速度 | `vx` int16、`vy` int16（大端） |

> 注意：协议目前没有校验和；`rx_len` 未做 128 字节上限检查，上位机需保证帧完整。

## 关键参数

| 参数 | 值 | 位置 |
| --- | --- | --- |
| 图像中心 | 320 × 240（640×480） | `BSP/motor.h` |
| 像素→弧度 | 0.00146 rad/px（水平 FOV ≈ 53.5°） | `BSP/motor.h` |
| Pitch 限幅 | ±1.05 rad（≈ ±60°） | `BSP/motor.c` |
| 画圆半径 | 6 cm | `BSP/motor.h` |
| 画圆周期 | 17 s/圈，2 s 渐入 | `BSP/motor.h` |
| 测距自适应范围 | 500–1500 mm | `BSP/motor.h` |
| 锁定阈值 | 位置 <15 px，速度 <60 px/s | `BSP/motor.h` |
| 视觉断流超时 | 200 ms | `Core/Src/main.c` |

控制思路：

- 像素误差乘以 `PIXEL_TO_RAD_K` 得到角度误差，PID 输出作为**角度增量**累加到目标角
- Yaw 使用 IMU 绝对航向，抵消小车在方形轨迹上转弯带来的基座旋转
- 画圆时按测距计算角半径，并对 yaw 乘以 `cosγ` 做靶面透视补偿
- 激光与摄像头存在光轴偏差，使用测距三次多项式做视差补偿

## 构建与烧录

1. 使用 **Keil MDK 5** 打开 `MDK-ARM/DS_Gimbal.uvprojx`
2. 编译后通过 J-Link 下载
3. 串口调试：USART6，115200-8-N-1
4. 修改外设配置：使用 STM32CubeMX 打开 `DS_Gimbal.ioc`（STM32Cube FW_F4 V1.28.3）

## 已知限制

- 1 kHz 控制中断内通过阻塞 I2C 读取 IMU，`HAL_I2C_Mem_Read` 超时为 100 ms；传感器异常时会拉长中断
- 距离自适应增益当前实际只在 1.00→0.99 之间变化（宏注释写的 40% 与代码不一致），效果接近关闭
- 模式 3 的 2 s 渐入阶段激光关闭；视觉短暂丢帧会重置画圆计时并重新渐入
- 视觉协议无校验和，`rx_len` 无上限检查
- `BSP/motor.c` 中保留了多段被注释的历史控制环实现，当前生效的是 `Gimbal_Control_Loop`

## 第三方代码与致谢

- STM32Cube HAL / CMSIS（STMicroelectronics）
- STM32 USB Device Library（STMicroelectronics）
- WIT 维特智能 IMU SDK
- 达妙 DM4310 驱动
- OLED 驱动：江协科技，zeruns 的 HAL 库移植版

第三方代码请遵循其原始许可证。

## 免责声明

本项目仅用于学习与竞赛交流。使用激光时请注意安全，避免直射人眼和皮肤。
