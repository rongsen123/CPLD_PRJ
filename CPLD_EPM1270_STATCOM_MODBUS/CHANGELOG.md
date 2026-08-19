# Changelog - CPLD_EPM1270_STATCOM_MODBUS

All notable changes to the `CPLD_EPM1270_STATCOM_MODBUS` project will be documented in this file.

## [0.1.5] - 2026-08-19

### Added
- 适配协议基线 `0x0105`，配套 4号 DSP 工程与 V0105 上位机。
- 增加本地 Vdc 软件过压 3 次连续确认滤波与锁存保护逻辑。
- 增加本地温度 100 ms 脉冲计数门限监控（固定 5000 count/100 ms，等效约 50 kHz 脉冲门限）。
- 分离并支持独立上报 CPLD UART 错误、CRC 错误与不完整帧错误计数。

### Changed
- 调整 Modbus 寄存器映射，支持读写 `1000` Vdc 原始码软件过压保护阈值。
- 重构工程目录至多工程架构规范子文件夹 `CPLD_EPM1270_STATCOM_MODBUS/`。

### Security
- 四路桥臂 PWM 输出恒定保持 0，`pwm_hold_o` 恒定输出 1，维持绝对功率硬封锁。
