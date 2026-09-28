onerror {quit -code 1 -f}
vlib work
vlog -lint -work work ../rtl/ads7818_acquisition_controller.v ../rtl/temperature_frequency_monitor.v ../rtl/fiber_uart_byte_receiver.v ../rtl/fiber_uart_byte_transmitter.v ../rtl/system_fault_monitor.v ../rtl/board_test_modbus.v ../rtl/top.v ../tb/tb_pwm_complementary.v
vsim -c work.tb_pwm_complementary
run -all
quit -f
