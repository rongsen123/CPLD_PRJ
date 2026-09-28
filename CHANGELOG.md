# Changelog - CPLD_PRJ

All notable changes to the CPLD repository will be documented in this file.

## [0.1.6] - 2026-09-28

### Added
- 收录 EPM1270 板级测试专用工程 `CPLD_EPM1270_BOARD_TEST/`。
- 支持 Modbus RTU (0x0106)、ADS7818 原始码采样、温度频率计数、互补测试 PWM 生成与故障模拟/屏蔽注入。
- 配套上位机独立归档至 `host_app` 仓库的 `CPLD_BOARD_TEST_HOST_V0106/` 目录。

## [0.1.5] - 2026-08-19

### Changed
- 重构仓库目录为“一工程一文件夹”规范结构，将单相 STATCOM 工程移至 `CPLD_EPM1270_STATCOM_MODBUS/`。
- 升级通信与保护逻辑至基线 `0x0105`，配套 4号 DSP 工程。
- 增加仓库根 `PROJECT_INDEX.md` 与总文件追踪列表。
