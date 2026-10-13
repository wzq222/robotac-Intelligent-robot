# robotac-Intelligent-robot

STM32F103 嵌入式固件，用于 **ROBOTAC / 智能机器人**竞赛机器人的控制。

本仓库包含**两个固件变体**，分别放在各自子目录下：

| 变体 | 路径 | 说明 |
| --- | --- | --- |
| `3.27` | [`3.27/user/`](3.27/user) | 3.27 版本固件 |
| `bianmaqi` | [`bianmaqi/user/`](bianmaqi/user) | 编码器（bianmaqi）版本固件 |

---

## 硬件平台

- **MCU**：STM32F103C8（ARM Cortex-M3，72 MHz）
- **开发环境**：Keil uVision 5（ARM-MDK）
- **外设库**：STM32F10x Standard Peripheral Library（标准外设库）

## 功能模块（每个变体均包含）

位于 `Hardware/` 目录下：

- `motor.c/h` —— 直流电机驱动（PWM 调速）
- `encoder.c/h` —— 编码器测速 / 里程
- `pid.c/h` —— PID 速度 / 位置控制
- `line_track.c/h` —— 红外 / 灰度循迹
- `steering.c/h` + `Control_steering.c/h` —— 舵机转向控制
- `claw.c/h` —— 机械爪控制
- `Digital_input.c/h` —— 数字量输入（限位、按键等）
- `USART2.c/h` —— 串口通信
- `app_ircomm.c/h` —— 红外通信应用层
- `BSP.c/h` —— 板级支持包（引脚与时钟初始化）
- `sys.c/h`、`Delay.c/h` —— 系统 / 延时基础功能

## 仓库结构

```
robotac-Intelligent-robot/
├── README.md                 # 本说明
├── .gitignore
├── 3.27/user/                # 变体 1
│   ├── Hardware/             # 外设驱动与应用层
│   ├── Library/              # STM32 标准外设库
│   ├── Start/                # 启动文件 + 系统初始化
│   ├── User/                 # main.c / 中断 / 配置
│   ├── DebugConfig/          # Keil 调试配置
│   ├── project1.uvprojx      # Keil 工程
│   └── project1.uvoptx
└── bianmaqi/user/            # 变体 2（编码器版，结构同上）
```

> 编译产物（`Objects/`、`Listings/`、`.o`、`.d`、`.axf` 等）由 `.gitignore` 排除，不纳入版本控制。

## 编译与下载

1. 用 **Keil uVision 5** 打开对应变体的 `project1.uvprojx`；
2. 选择器件 **STM32F103C8**；
3. `Project → Rebuild` 编译，输出位于 `Objects/`；
4. 通过 **ST-Link** 或串口下载器烧录到板子。

## 代码规范

- 代码基于 STM32 标准外设库，不依赖 HAL；
- 启动文件 `Start/startup_stm32f10x_md.s` 对应中等容量（C8 系列）；
- 若更换芯片型号，需同步替换 `Start/` 下对应的启动文件。

## License

内部 / 竞赛项目，未指定公开许可证。如需开源，请另行添加 LICENSE 文件。
