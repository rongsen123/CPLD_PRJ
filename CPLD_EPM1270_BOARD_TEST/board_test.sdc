# 30 MHz board oscillator: 33.333 ns nominal period.
create_clock -name sys_clk_i -period 33.333 [get_ports {sys_clk_i}]
derive_clock_uncertainty
