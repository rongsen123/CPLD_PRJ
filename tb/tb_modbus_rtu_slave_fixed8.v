`timescale 1ns/1ps
module tb_modbus_rtu_slave_fixed8;
    reg clk = 0;
    reg reset_n = 0;
    reg [7:0] rx_byte = 0;
    reg rx_valid = 0;
    wire [7:0] tx_byte;
    wire tx_start;
    reg tx_busy = 0;
    reg tx_done = 0;
    wire [15:0] command_echo;
    wire clear_fault;
    wire soft_reset;
    wire link_seen, link_fault, protocol_error;
    wire [15:0] valid_count, error_count;
    wire [15:0] uart_error_count, crc_error_count, incomplete_frame_count;
    reg [7:0] captured [0:63];
    integer captured_count = 0;
    integer errors = 0;
    integer i;
    reg [15:0] crc;
    reg clear_seen = 0;
    reg reset_seen = 0;

    always #16.666 clk = ~clk;

    modbus_rtu_slave_fixed8 dut (
        .clk_i(clk), .reset_n_i(reset_n),
        .rx_byte_i(rx_byte), .rx_byte_valid_i(rx_valid),
        .rx_frame_error_i(1'b0),
        .tx_byte_o(tx_byte), .tx_start_o(tx_start),
        .tx_busy_i(tx_busy), .tx_done_i(tx_done),
        .vdc_raw_i(12'h123), .vdc_average_i(12'h234),
        .temperature_count_i(16'h3456),
        .adc_valid_i(1'b1), .temperature_valid_i(1'b1),
        .fault_flags_i(32'h00000ABC), .fault_any_i(1'b1),
        .command_echo_o(command_echo), .clear_fault_o(clear_fault),
        .soft_reset_o(soft_reset),
        .link_seen_o(link_seen), .link_fault_o(link_fault),
        .protocol_error_pulse_o(protocol_error),
        .valid_frame_count_o(valid_count), .error_frame_count_o(error_count),
        .uart_error_count_o(uart_error_count),
        .crc_error_count_o(crc_error_count),
        .incomplete_frame_count_o(incomplete_frame_count)
    );

    function [15:0] crc_update;
        input [15:0] crc_in;
        input [7:0] data_in;
        integer b;
        reg [15:0] c;
        begin
            c = crc_in ^ data_in;
            for (b = 0; b < 8; b = b + 1)
                if (c[0]) c = (c >> 1) ^ 16'hA001;
                else c = c >> 1;
            crc_update = c;
        end
    endfunction

    always @(posedge clk) begin
        if (clear_fault) clear_seen <= 1'b1;
        if (soft_reset) reset_seen <= 1'b1;
        tx_done <= 1'b0;
        if (tx_start && !tx_busy) begin
            captured[captured_count] <= tx_byte;
            captured_count <= captured_count + 1;
            tx_busy <= 1'b1;
        end else if (tx_busy) begin
            tx_busy <= 1'b0;
            tx_done <= 1'b1;
        end
    end

    task send_byte;
        input [7:0] value;
        begin
            @(posedge clk); rx_byte <= value; rx_valid <= 1'b1;
            @(posedge clk); rx_valid <= 1'b0;
        end
    endtask

    task send_request;
        input [7:0] address;
        input [7:0] function_code;
        input [15:0] register_address;
        input [15:0] register_value;
        input corrupt_crc;
        begin
            captured_count = 0;
            crc = 16'hFFFF;
            crc = crc_update(crc, address);
            crc = crc_update(crc, function_code);
            crc = crc_update(crc, register_address[15:8]);
            crc = crc_update(crc, register_address[7:0]);
            crc = crc_update(crc, register_value[15:8]);
            crc = crc_update(crc, register_value[7:0]);
            send_byte(address); send_byte(function_code);
            send_byte(register_address[15:8]); send_byte(register_address[7:0]);
            send_byte(register_value[15:8]); send_byte(register_value[7:0]);
            send_byte(corrupt_crc ? (crc[7:0] ^ 8'h01) : crc[7:0]);
            send_byte(crc[15:8]);
            repeat (200) @(posedge clk);
        end
    endtask

    task check_crc;
        input integer length;
        reg [15:0] response_crc;
        begin
            response_crc = 16'hFFFF;
            for (i = 0; i < length - 2; i = i + 1)
                response_crc = crc_update(response_crc, captured[i]);
            if ((captured[length-2] !== response_crc[7:0]) ||
                (captured[length-1] !== response_crc[15:8])) begin
                $display("FAIL response CRC");
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        repeat (8) @(posedge clk);
        reset_n = 1'b1;

        send_request(8'h01, 8'h04, 16'h0000, 16'd14, 1'b0);
        if (captured_count != 33 || captured[0] != 8'h01 ||
            captured[1] != 8'h04 || captured[2] != 8'd28 ||
            captured[3] != 8'h01 || captured[4] != 8'h03 ||
            captured[7] != 8'h01 || captured[8] != 8'h23) begin
            $display("FAIL FC04 full map/count=%0d", captured_count);
            errors = errors + 1;
        end else check_crc(33);

        send_request(8'h01, 8'h06, 16'h0100, 16'd1, 1'b0);
        if (captured_count != 8 || command_echo != 1 || captured[1] != 8'h06)
            errors = errors + 1;
        else check_crc(8);

        send_request(8'h01, 8'h03, 16'h0100, 16'd1, 1'b0);
        if (captured_count != 7 || captured[1] != 8'h03 ||
            captured[3] != 0 || captured[4] != 1)
            errors = errors + 1;
        else check_crc(7);

        clear_seen = 1'b0;
        send_request(8'h01, 8'h06, 16'h0101, 16'hA55A, 1'b0);
        if (captured_count != 5 || captured[1] != 8'h86 ||
            captured[2] != 8'h03 || clear_seen)
            errors = errors + 1;
        else check_crc(5);

        send_request(8'h01, 8'h06, 16'h0100, 16'd0, 1'b0);
        if (captured_count != 8 || command_echo != 0)
            errors = errors + 1;
        else check_crc(8);

        clear_seen = 1'b0;
        send_request(8'h01, 8'h06, 16'h0101, 16'hA55A, 1'b0);
        if (captured_count != 8 || !clear_seen)
            errors = errors + 1;
        else check_crc(8);

        reset_seen = 1'b0;
        send_request(8'h01, 8'h06, 16'h0102, 16'hC33C, 1'b0);
        if (captured_count != 8 || !reset_seen || captured[1] != 8'h06)
            errors = errors + 1;
        else check_crc(8);

        send_request(8'h01, 8'h04, 16'h0000, 16'd1, 1'b1);
        if (captured_count != 0 || error_count == 0 || crc_error_count == 0)
            errors = errors + 1;

        if (errors == 0) $display("PASS tb_modbus_rtu_slave_fixed8");
        else $display("FAIL tb_modbus_rtu_slave_fixed8 errors=%0d", errors);
        $finish;
    end
endmodule
