onerror {quit -code 1 -f}
vlib work
vlog -lint -work work ../rtl/board_test_modbus.v ../tb/tb_board_test_modbus.v
vsim -c work.tb_board_test_modbus
run -all
quit -f
