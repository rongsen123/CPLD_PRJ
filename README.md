# CPLD_PRJ

本仓库用于统一管理 SST 项目相关的 Altera / Intel CPLD (MAX II) 工程。每个子目录对应一个可独立打开、编译和配置的 Quartus 工程。

## 工程目录

详见 [PROJECT_INDEX.md](PROJECT_INDEX.md)。

| 工程目录 | 目标器件 | 功能说明 | 配套 DSP |
| --- | --- | --- | --- |
| [CPLD_EPM1270_STATCOM_MODBUS](CPLD_EPM1270_STATCOM_MODBUS/) | EPM1270T144C5 | Modbus RTU 从站通信、ADS7818 采样与保护执行（功率输出硬封锁） | 4_STATCOM_CURRENT_LOOP |
| [CPLD_EPM1270_BOARD_TEST](CPLD_EPM1270_BOARD_TEST/) | EPM1270T144C5 | Modbus RTU 从站通信、ADS7818 原始码采样、互补 PWM 测试与故障模拟注入 | 独立板级测试（上位机见 host_app） |

## 开发环境

- 开发工具：Altera Quartus II 13.1 (64-bit)
- 目标器件：MAX II EPM1270T144C5
- 系统主频：30 MHz
- 通信接口：光纤 UART (115200 bit/s, 8N1)

## 安全边界

- `CPLD_EPM1270_STATCOM_MODBUS`：严格维持功率输出硬封锁，四路桥臂输出为 0，`pwm_hold_o=1`，继电器与风机保持安全状态。
- `CPLD_EPM1270_BOARD_TEST`：专用测试镜像，仅在上位机下发 START 且无有效故障时方可输出测试互补 PWM；硬件过压、驱动故障或链路超时将立即硬件封锁。
