`timescale 1ns/1ps
module tb_board_test_modbus;
    reg clk=0;
    always #5 clk=~clk;
    reg reset_n=0;
    reg [7:0] rx_byte=0;
    reg rx_valid=0;
    reg rx_error=0;
    wire [7:0] tx_byte;
    wire tx_start;
    reg tx_busy=0, tx_done=0;
    reg [11:0] adc=12'h456, avg=12'h400;
    reg [15:0] temp=16'd123;
    reg [31:0] faults=0;
    reg fault_any=0;
    reg [15:0] digital=16'hA55A;
    reg [11:0] effective=0;
    reg pwm_active=0;
    wire [15:0] command, period, duty;
    wire [11:0] limit, mask, sim;
    wire [2:0] control;
    wire clear_fault, soft_reset, link_seen, link_fault, error_pulse;
    wire [15:0] valid_count, error_count, uart_error_count;
    wire [15:0] crc_error_count, incomplete_count;
    reg [7:0] response [0:63];
    integer response_count=0;
    integer failures=0;
    integer timeout;
    reg [15:0] crc;
    integer j;

    board_test_modbus dut (
        .clk_i(clk), .reset_n_i(reset_n),
        .rx_byte_i(rx_byte), .rx_byte_valid_i(rx_valid),
        .rx_frame_error_i(rx_error),
        .tx_byte_o(tx_byte), .tx_start_o(tx_start),
        .tx_busy_i(tx_busy), .tx_done_i(tx_done),
        .vdc_raw_i(adc), .vdc_average_i(avg), .temperature_count_i(temp),
        .adc_valid_i(1'b1), .temperature_valid_i(1'b1),
        .fault_flags_i(faults), .fault_any_i(fault_any),
        .command_echo_o(command), .vdc_over_limit_o(limit),
        .clear_fault_o(clear_fault), .soft_reset_o(soft_reset),
        .link_seen_o(link_seen), .link_fault_o(link_fault),
        .protocol_error_pulse_o(error_pulse),
        .valid_frame_count_o(valid_count), .error_frame_count_o(error_count),
        .uart_error_count_o(uart_error_count),
        .crc_error_count_o(crc_error_count),
        .incomplete_frame_count_o(incomplete_count),
        .digital_inputs_i(digital), .effective_faults_i(effective),
        .pwm_active_i(pwm_active), .fault_mask_o(mask),
        .fault_sim_o(sim), .test_control_o(control),
        .pwm_period_o(period), .pwm_duty_o(duty)
    );

    always @(posedge clk) begin
        tx_done <= 0;
        if (tx_start) begin
            response[response_count] <= tx_byte;
            response_count <= response_count+1;
            tx_busy <= 1;
        end else if (tx_busy) begin
            tx_busy <= 0;
            tx_done <= 1;
        end
    end
    function [15:0] crc_update;
        input [15:0] current;
        input [7:0] data;
        reg [15:0] c;
        integer k;
        begin
            c=current ^ data;
            for(k=0;k<8;k=k+1)
                if(c[0]) c=(c>>1)^16'hA001;
                else c=c>>1;
            crc_update=c;
        end
    endfunction
    task send_byte;
        input [7:0] b;
        begin
            @(negedge clk); rx_byte=b; rx_valid=1;
            @(negedge clk); rx_valid=0;
        end
    endtask
    task request;
        input [7:0] fc;
        input [15:0] addr;
        input [15:0] value;
        input integer expected_len;
        reg [15:0] c;
        integer start_count;
        begin
            start_count=response_count;
            c=16'hFFFF;
            c=crc_update(c,8'h01); send_byte(8'h01);
            c=crc_update(c,fc); send_byte(fc);
            c=crc_update(c,addr[15:8]); send_byte(addr[15:8]);
            c=crc_update(c,addr[7:0]); send_byte(addr[7:0]);
            c=crc_update(c,value[15:8]); send_byte(value[15:8]);
            c=crc_update(c,value[7:0]); send_byte(value[7:0]);
            send_byte(c[7:0]); send_byte(c[15:8]);
            timeout=0;
            while(response_count < start_count+expected_len && timeout < 300) begin
                @(negedge clk); timeout=timeout+1;
            end
            if(response_count != start_count+expected_len) begin
                $display("FAIL response length fc=%h addr=%h got=%0d expected=%0d",
                    fc,addr,response_count-start_count,expected_len);
                failures=failures+1;
            end
            repeat(4) @(negedge clk);
        end
    endtask
    task check;
        input condition;
        input [255:0] label;
        begin
            if(!condition) begin $display("FAIL %s",label); failures=failures+1; end
        end
    endtask
    initial begin
        repeat(5) @(negedge clk);
        reset_n=1;
        repeat(2) @(negedge clk);
        check(mask==12'h003 && control==3'b100,
              "power-on board-test defaults");
        request(8'h04,16'h0000,16'd1,7);
        check(response[3]==8'h01 && response[4]==8'h06,
              "firmware build identifier 0106");
        request(8'h04,16'h000E,16'd2,9);
        check(response[response_count-9]==8'h01 &&
              response[response_count-8]==8'h04 &&
              response[response_count-6]==8'hA5 &&
              response[response_count-5]==8'h5A,
              "digital input register");
        request(8'h06,16'h1100,16'h0070,5);
        check(response[response_count-3]==8'h03 && mask==12'h003,
              "hardware fault mask rejected");
        request(8'h06,16'h1100,16'h0008,8);
        check(mask==12'h008,"software fault mask accepted");
        request(8'h06,16'h1101,16'h0010,8);
        check(sim==12'h010,"fault simulation accepted");
        request(8'h06,16'h1102,16'h0007,8);
        check(control==3'b111,"test outputs configured");
        request(8'h06,16'h1103,16'd2000,8);
        request(8'h06,16'h1104,16'd800,8);
        check(period==2000 && duty==800,"PWM period and duty configured");
        fault_any=1;
        request(8'h06,16'h0100,16'd1,5);
        check(command==0 && response[response_count-3]==8'h03,
              "START blocked by fault");
        fault_any=0;
        request(8'h06,16'h0100,16'd1,8);
        check(command==1,"START accepted when clear");
        request(8'h03,16'h1103,16'd1,7);
        check(response[response_count-4]==8'h07 &&
              response[response_count-3]==8'hD0,
              "PWM period readback");
        request(8'h06,16'h1102,16'd0,5);
        check(control==7,"configuration locked during START");
        request(8'h06,16'h0100,16'd0,8);
        check(command==0,"STOP accepted");
        if(failures==0) $display("BOARD_TEST_MODBUS_PASS");
        else $fatal(1,"BOARD_TEST_MODBUS_FAIL count=%0d",failures);
        $finish;
    end
endmodule
