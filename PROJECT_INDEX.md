# CPLD_PRJ 工程索引

本仓库用于管理 SST 项目相关的 Altera / Intel CPLD (MAX II) 工程。每个子目录对应一个可独立打开、编译和烧录的工程。

## 工程列表

| 工程目录 | 目标器件 | 用途说明 | 配套 DSP 工程 | 协议版本 | Quartus 版本 | 关键状态 / 安全边界 |
| --- | --- | --- | --- | --- | --- | --- |
| [CPLD_EPM1270_STATCOM_MODBUS](CPLD_EPM1270_STATCOM_MODBUS/) | EPM1270T144C5 | Modbus RTU 从站通信、ADS7818 母线采样与本地保护 | `4_STATCOM_CURRENT_LOOP` | `0x0105` | 13.1 | 4路桥臂恒为0，`pwm_hold_o=1`（功率输出硬封锁） |
| [CPLD_EPM1270_BOARD_TEST](CPLD_EPM1270_BOARD_TEST/) | EPM1270T144C5 | Modbus RTU 从站通信、ADS7818 原始码采样、互补 PWM 测试与故障模拟注入 | 独立板级测试（上位机见 host_app） | `0x0106` | 13.1 | 仅测试状态允许使能 PWM，故障与链路失效时硬件封锁 |
