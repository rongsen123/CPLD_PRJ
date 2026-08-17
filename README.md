# CPLD_EPM1270_STATCOM_MODBUS

本工程以`F:\Quartus_project\SST_prj\CPLD_1270_prj`为硬件母版，面向
`F:\DSP\SST_PRJ\3_STATCOM_PI_PR`的SCI-A Modbus RTU通信调试。

## 硬件与通信

- 器件：MAX II `EPM1270T144C5`，系统时钟30 MHz。
- 保留母版完整引脚：光纤RX为PIN42、TX为PIN43。
- 保留母版实测电气关系：PIN42物理空闲低，顶层取反后进入标准UART。
- UART：115200 bit/s、8N1；CPLD为Modbus从站地址1。
- DSP请求固定为8字节，CPLD收到第8字节立即做CRC校验并回复。
- 支持FC04、FC03、FC06以及异常响应01/02/03。

## 安全边界

START/STOP只更新寄存器回显。四路桥臂始终为0，`pwm_hold_o`始终为1，
风机和旁路继电器保持不动作。写入`0101=A55A`仅在STOP状态产生一次
故障清除脉冲，仍存在的硬件故障不会被掩盖。

## 无示波器调试LED

- LED1：每256个UART有效字节翻转一次，用于确认PIN42物理接收链路。
- LED2：收到有效请求且链路在线时点亮。
- LED3：每16个有效Modbus请求翻转一次。
- LED4：每16个协议/UART错误翻转一次。

详细寄存器和测试命令见`docs/MODBUS_RTU_REGISTER_MAP.md`。
