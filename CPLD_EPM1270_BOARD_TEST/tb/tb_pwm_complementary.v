`timescale 1ns/1ps
module tb_pwm_complementary;
    reg clk=0;
    always #5 clk=~clk;
    reg hw_ov_n=1, drive1_n=1, drive2_n=1;
    wire hold, a1, b1, a2, b2;
    integer high_cycles=0, low_cycles=0;
    integer n;
    top dut (
        .sys_clk_i(clk), .adc_serial_data_i(1'b0),
        .adc_serial_clk_o(), .adc_convert_o(),
        .temperature_freq_i(1'b0), .bypass_status_i(1'b0),
        .peer_module_fault_i(1'b0), .spare_digital_input_i(1'b0),
        .fan_relay_n_o(), .bypass_relay_n_o(), .fault_output_n_o(),
        .fiber_rx_i(1'b0), .fiber_tx_o(),
        .pwm_bridge_1_a_o(a1), .pwm_bridge_1_b_o(b1),
        .pwm_bridge_2_a_o(a2), .pwm_bridge_2_b_o(b2),
        .pwm_hold_o(hold),
        .dc_overvoltage_fault_i(hw_ov_n),
        .drive_fault_1_i(drive1_n), .drive_fault_2_i(drive2_n),
        .fault_led_n_o(), .rx_fault_led_n_o(), .tx_fault_led_n_o(),
        .LED1(), .LED2(), .LED3(), .LED4()
    );
    initial begin
        repeat(30) @(negedge clk);
        if (!hold || a1 || b1 || a2 || b2)
            $fatal(1,"RESET_OUTPUTS_UNSAFE");
        force dut.u_modbus.command_echo_o = 16'd1;
        force dut.u_modbus.test_control_o = 3'b100;
        force dut.u_modbus.link_seen_o = 1'b1;
        force dut.u_modbus.link_fault_o = 1'b0;
        force dut.u_modbus.pwm_period_o = 16'd600;
        force dut.u_modbus.pwm_duty_o = 16'd300;
        repeat(2) @(negedge clk);
        for(n=0;n<1200;n=n+1) begin
            @(negedge clk);
            if (hold || (a1 ^ b1) !== 1'b1 ||
                a1 !== a2 || b1 !== b2)
                $fatal(1,"PWM_NOT_COMPLEMENTARY cycle=%0d a=%b b=%b hold=%b",
                       n,a1,b1,hold);
            if(a1) high_cycles=high_cycles+1;
            if(b1) low_cycles=low_cycles+1;
        end
        if(high_cycles < 500 || low_cycles < 500)
            $fatal(1,"PWM_DUTY_NOT_OBSERVED");
        hw_ov_n=0;
        #1;
        if(!hold || a1 || b1 || a2 || b2)
            $fatal(1,"RAW_HW_FAULT_DID_NOT_BLOCK_PWM");
        hw_ov_n=1;
        force dut.u_modbus.command_echo_o = 16'd0;
        #1;
        if(!hold || a1 || b1 || a2 || b2)
            $fatal(1,"STOP_DID_NOT_BLOCK_PWM");
        $display("PWM_COMPLEMENTARY_PASS");
        $finish;
    end
endmodule
