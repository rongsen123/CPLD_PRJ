/*
 * Board-test Modbus RTU slave address 1 for the optical PC link.
 * 115200 bit/s, 8N1. FC03/FC04/FC06 requests are exactly eight bytes and are
 * validated immediately when byte 8 arrives; an incomplete request is dropped
 * after the standard fixed 1.75 ms RTU gap.
 */
module board_test_modbus (
    input  wire        clk_i,                   // 30 MHz system clock
    input  wire        reset_n_i,               // Active-low reset
    input  wire [7:0]  rx_byte_i,               // Byte from UART receiver
    input  wire        rx_byte_valid_i,         // One-clock RX byte pulse
    input  wire        rx_frame_error_i,        // UART stop-bit error latch
    output reg  [7:0]  tx_byte_o,               // Byte to UART transmitter
    output reg         tx_start_o,              // One-clock TX byte request
    input  wire        tx_busy_i,               // UART transmitter busy
    input  wire        tx_done_i,               // One-clock TX byte-done pulse
    input  wire [11:0] vdc_raw_i,               // Latest ADS7818 raw code
    input  wire [11:0] vdc_average_i,           // 256-sample block average
    input  wire [15:0] temperature_count_i,     // Pulses per 100 ms
    input  wire        adc_valid_i,             // Sticky ADC-valid state
    input  wire        temperature_valid_i,     // Sticky temperature-valid state
    input  wire [31:0] fault_flags_i,           // CPLD latched fault bitmap
    input  wire        fault_any_i,             // Any CPLD fault
    output reg  [15:0] command_echo_o,           // 0=STOP, 1=START echo only
    output reg  [11:0] vdc_over_limit_o,         // Local software OV threshold, raw ADC counts
    output reg         clear_fault_o,            // One-clock clear pulse
    output reg         soft_reset_o,             // Pulse after reset-command reply is sent
    output reg         link_seen_o,              // Valid request received once
    output reg         link_fault_o,             // 500 ms without valid request
    output reg         protocol_error_pulse_o,   // One-clock rejected-frame pulse
    output reg  [15:0] valid_frame_count_o,      // Valid requests for address 1
    output reg  [15:0] error_frame_count_o,      // Combined CRC/UART/incomplete requests
    output reg  [15:0] uart_error_count_o,       // UART stop-bit error events
    output reg  [15:0] crc_error_count_o,        // Eight-byte requests with bad CRC
    output reg  [15:0] incomplete_frame_count_o, // Partial requests expired by t3.5
    input  wire [15:0] digital_inputs_i,
    input  wire [11:0] effective_faults_i,
    input  wire        pwm_active_i,
    output reg  [11:0] fault_mask_o,
    output reg  [11:0] fault_sim_o,
    output reg  [2:0]  test_control_o,
    output reg  [15:0] pwm_period_o,
    output reg  [15:0] pwm_duty_o
);
    localparam [15:0] FRAME_GAP_CLKS = 16'd52500;       // 1.75 ms at 30 MHz
    localparam [23:0] LINK_TIMEOUT_CLKS = 24'd15000000; // 500 ms at 30 MHz
    localparam [1:0] STATE_RX = 2'd0;
    localparam [1:0] STATE_SEND = 2'd1;
    localparam [1:0] STATE_WAIT = 2'd2;

    reg [1:0] state_r;                          // Receive/transmit state
    reg [2:0] rx_index_r;                       // Next request-byte index 0..7
    reg [7:0] request_r [0:7];                  // Fixed eight-byte request buffer
    reg [15:0] request_crc_r;                   // CRC accumulated over bytes 0..5
    reg [15:0] gap_count_r;                     // Incomplete-request silence timer
    reg [23:0] link_count_r;                    // Time since last valid request
    reg rx_frame_error_d_r;                     // UART error edge detector
    reg reset_pending_r;                        // Delay reset until FC06 echo completes

    reg [7:0] response_function_r;              // FC03/FC04/FC06 response function
    reg [7:0] exception_code_r;                 // 0 or Modbus exception 01/02/03
    reg [5:0] payload_length_r;                 // Response bytes excluding CRC
    reg [5:0] tx_index_r;                       // Current response byte index
    reg [15:0] tx_crc_r;                        // Response CRC accumulator


    wire [15:0] request_address_w = {request_r[2], request_r[3]};
    wire [15:0] request_value_w = {request_r[4], request_r[5]};
    wire [15:0] received_crc_w = {rx_byte_i, request_r[6]};
    wire uart_error_rise_w = rx_frame_error_i & ~rx_frame_error_d_r;

    function [15:0] crc16_update;
        input [15:0] crc_in;
        input [7:0] data_in;
        integer bit_index;
        reg [15:0] crc;
        begin
            crc = crc_in ^ data_in;
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
                if (crc[0]) crc = (crc >> 1) ^ 16'hA001;
                else crc = crc >> 1;
            crc16_update = crc;
        end
    endfunction

    function [15:0] input_register;
        input [3:0] register_number;
        begin
            case (register_number)
                4'h0: input_register = 16'h0106;
                4'h1: input_register = valid_frame_count_o;
                4'h2: input_register = {4'd0, vdc_raw_i};
                4'h3: input_register = {4'd0, vdc_average_i};
                4'h4: input_register = temperature_count_i;
                4'h5: input_register = {11'd0, 1'b0, fault_any_i, temperature_valid_i, adc_valid_i, 1'b1};
                4'h6: input_register = fault_flags_i[31:16];
                4'h7: input_register = fault_flags_i[15:0];
                4'h8: input_register = command_echo_o;
                4'h9: input_register = valid_frame_count_o;
                4'hA: input_register = error_frame_count_o;
                4'hB: input_register = uart_error_count_o;
                4'hC: input_register = crc_error_count_o;
                4'hD: input_register = incomplete_frame_count_o;
                4'hE: input_register = digital_inputs_i;
                4'hF: input_register = {3'd0, pwm_active_i, effective_faults_i};
                default: input_register = 16'd0;
            endcase
        end
    endfunction

    function [15:0] holding_register;
        input [15:0] register_address;
        begin
            case (register_address)
                16'h0100: holding_register = command_echo_o;
                16'h1000: holding_register = {4'd0, vdc_over_limit_o};
                16'h1100: holding_register = {4'd0, fault_mask_o};
                16'h1101: holding_register = {4'd0, fault_sim_o};
                16'h1102: holding_register = {13'd0, test_control_o};
                16'h1103: holding_register = pwm_period_o;
                16'h1104: holding_register = pwm_duty_o;
                default: holding_register = 16'd0;
            endcase
        end
    endfunction

    function [7:0] response_byte;
        input [5:0] byte_index;
        reg [15:0] value;
        reg [5:0] data_offset;
        reg [3:0] register_number;
        begin
            value = 16'd0;
            data_offset = 6'd0;
            register_number = 4'd0;
            if (exception_code_r != 0) begin
                case (byte_index)
                    0: response_byte = 8'h01;
                    1: response_byte = response_function_r | 8'h80;
                    default: response_byte = exception_code_r;
                endcase
            end else if (response_function_r == 8'h06) begin
                case (byte_index)
                    0: response_byte = 8'h01;
                    1: response_byte = 8'h06;
                    2: response_byte = request_r[2];
                    3: response_byte = request_r[3];
                    4: response_byte = request_r[4];
                    default: response_byte = request_r[5];
                endcase
            end else if (response_function_r == 8'h03) begin
                value = holding_register(request_address_w);
                case (byte_index)
                    0: response_byte = 8'h01;
                    1: response_byte = 8'h03;
                    2: response_byte = 8'd2;
                    3: response_byte = value[15:8];
                    default: response_byte = value[7:0];
                endcase
            end else begin
                if (byte_index == 0)
                    response_byte = 8'h01;
                else if (byte_index == 1)
                    response_byte = 8'h04;
                else if (byte_index == 2)
                    response_byte = {request_value_w[6:0], 1'b0};
                else begin
                    data_offset = byte_index - 6'd3;
                    register_number = request_r[3][3:0] + data_offset[4:1];
                    value = input_register(register_number);
                    if (!data_offset[0]) response_byte = value[15:8];
                    else response_byte = value[7:0];
                end
            end
        end
    endfunction

    task start_response;
        input [7:0] function_code;
        input [7:0] exception_code;
        input [5:0] payload_length;
        begin
            response_function_r <= function_code;
            exception_code_r <= exception_code;
            payload_length_r <= payload_length;
            tx_index_r <= 6'd0;
            tx_crc_r <= 16'hFFFF;
            state_r <= STATE_SEND;
        end
    endtask

    always @(posedge clk_i or negedge reset_n_i) begin
        if (!reset_n_i) begin
            state_r <= STATE_RX;
            rx_index_r <= 3'd0;
            request_crc_r <= 16'hFFFF;
            gap_count_r <= 16'd0;
            link_count_r <= 24'd0;
            rx_frame_error_d_r <= 1'b0;
            response_function_r <= 8'd0;
            exception_code_r <= 8'd0;
            payload_length_r <= 6'd0;
            tx_index_r <= 6'd0;
            tx_crc_r <= 16'hFFFF;
            tx_byte_o <= 8'd0;
            tx_start_o <= 1'b0;
            command_echo_o <= 16'd0;
            vdc_over_limit_o <= 12'd3410;
            fault_mask_o <= 12'h003;
            fault_sim_o <= 12'd0;
            test_control_o <= 3'b100;
            pwm_period_o <= 16'd1500;
            pwm_duty_o <= 16'd750;
            clear_fault_o <= 1'b0;
            soft_reset_o <= 1'b0;
            reset_pending_r <= 1'b0;
            link_seen_o <= 1'b0;
            link_fault_o <= 1'b0;
            protocol_error_pulse_o <= 1'b0;
            valid_frame_count_o <= 16'd0;
            error_frame_count_o <= 16'd0;
            uart_error_count_o <= 16'd0;
            crc_error_count_o <= 16'd0;
            incomplete_frame_count_o <= 16'd0;
        end else begin
            tx_start_o <= 1'b0;
            clear_fault_o <= 1'b0;
            soft_reset_o <= 1'b0;
            protocol_error_pulse_o <= 1'b0;
            rx_frame_error_d_r <= rx_frame_error_i;

            if (link_seen_o) begin
                if (link_count_r < LINK_TIMEOUT_CLKS)
                    link_count_r <= link_count_r + 1'b1;
                else
                    link_fault_o <= 1'b1;
            end

            if (uart_error_rise_w) begin
                error_frame_count_o <= {8'd0, (error_frame_count_o[7:0] + 8'd1)};
                uart_error_count_o <= {8'd0, (uart_error_count_o[7:0] + 8'd1)};
                protocol_error_pulse_o <= 1'b1;
                rx_index_r <= 3'd0;
                request_crc_r <= 16'hFFFF;
                gap_count_r <= 16'd0;
            end

            case (state_r)
                STATE_RX: begin
                    if (rx_byte_valid_i) begin
                        gap_count_r <= 16'd0;
                        request_r[rx_index_r] <= rx_byte_i;
                        if (rx_index_r == 3'd0)
                            request_crc_r <= crc16_update(16'hFFFF, rx_byte_i);
                        else if (rx_index_r < 3'd6)
                            request_crc_r <= crc16_update(request_crc_r, rx_byte_i);

                        if (rx_index_r == 3'd7) begin
                            rx_index_r <= 3'd0;
                            request_crc_r <= 16'hFFFF;
                            if (request_crc_r != received_crc_w) begin
                                error_frame_count_o <= {8'd0, (error_frame_count_o[7:0] + 8'd1)};
                                crc_error_count_o <= {8'd0, (crc_error_count_o[7:0] + 8'd1)};
                                protocol_error_pulse_o <= 1'b1;
                            end else if (request_r[0] == 8'h01) begin
                                valid_frame_count_o <= {8'd0, (valid_frame_count_o[7:0] + 8'd1)};
                                link_seen_o <= 1'b1;
                                link_fault_o <= 1'b0;
                                link_count_r <= 24'd0;

                                if (request_r[1] == 8'h04) begin
                                    if ((request_r[2] == 0) &&
                                        (request_value_w != 0) &&
                                        (request_value_w <= 16) &&
                                        ({8'd0, request_r[3]} + request_value_w <= 16))
                                        start_response(8'h04, 8'h00,
                                                       3 + request_value_w[4:0] * 2);
                                    else
                                        start_response(8'h04, 8'h02, 6'd3);
                                end else if (request_r[1] == 8'h03) begin
                                    if (((request_address_w == 16'h0100) ||
                                         (request_address_w == 16'h1000) ||
                                         ((request_address_w >= 16'h1100) &&
                                          (request_address_w <= 16'h1104))) &&
                                        (request_value_w == 16'd1))
                                        start_response(8'h03, 8'h00, 6'd5);
                                    else
                                        start_response(8'h03, 8'h02, 6'd3);
                                end else if (request_r[1] == 8'h06) begin
                                    if (request_address_w == 16'h0100) begin
                                        if ((request_value_w > 1) ||
                                            ((request_value_w == 1) && fault_any_i))
                                            start_response(8'h06, 8'h03, 6'd3);
                                        else begin
                                            command_echo_o <= request_value_w;
                                            start_response(8'h06, 8'h00, 6'd6);
                                        end
                                    end else if (request_address_w == 16'h0101) begin
                                        if ((request_value_w != 16'hA55A) ||
                                            (command_echo_o != 16'd0))
                                            start_response(8'h06, 8'h03, 6'd3);
                                        else begin
                                            clear_fault_o <= 1'b1;
                                            start_response(8'h06, 8'h00, 6'd6);
                                        end
                                    end else if (request_address_w == 16'h0102) begin
                                        if ((request_value_w != 16'hC33C) ||
                                            (command_echo_o != 16'd0))
                                            start_response(8'h06, 8'h03, 6'd3);
                                        else begin
                                            reset_pending_r <= 1'b1;
                                            start_response(8'h06, 8'h00, 6'd6);
                                        end
                                    end else if (request_address_w == 16'h1000) begin
                                        if ((command_echo_o != 16'd0) ||
                                            (request_value_w == 16'd0) ||
                                            (request_value_w > 16'd3410))
                                            start_response(8'h06, 8'h03, 6'd3);
                                        else begin
                                            vdc_over_limit_o <= request_value_w[11:0];
                                            start_response(8'h06, 8'h00, 6'd6);
                                        end
                                    end else if (request_address_w == 16'h1100) begin
                                        if ((command_echo_o != 0) ||
                                            ((request_value_w & 16'hF270) != 0))
                                            start_response(8'h06, 8'h03, 6'd3);
                                        else begin
                                            fault_mask_o <= request_value_w[11:0];
                                            start_response(8'h06, 8'h00, 6'd6);
                                        end
                                    end else if (request_address_w == 16'h1101) begin
                                        if ((command_echo_o != 0) ||
                                            ((request_value_w & 16'hF200) != 0))
                                            start_response(8'h06, 8'h03, 6'd3);
                                        else begin
                                            fault_sim_o <= request_value_w[11:0];
                                            start_response(8'h06, 8'h00, 6'd6);
                                        end
                                    end else if (request_address_w == 16'h1102) begin
                                        if ((command_echo_o != 0) || (request_value_w > 7))
                                            start_response(8'h06, 8'h03, 6'd3);
                                        else begin
                                            test_control_o <= request_value_w[2:0];
                                            start_response(8'h06, 8'h00, 6'd6);
                                        end
                                    end else if (request_address_w == 16'h1103) begin
                                        if ((command_echo_o != 0) ||
                                            (request_value_w < 600) ||
                                            (request_value_w > 60000) ||
                                            (request_value_w <= pwm_duty_o))
                                            start_response(8'h06, 8'h03, 6'd3);
                                        else begin
                                            pwm_period_o <= request_value_w;
                                            start_response(8'h06, 8'h00, 6'd6);
                                        end
                                    end else if (request_address_w == 16'h1104) begin
                                        if ((command_echo_o != 0) ||
                                            (request_value_w == 0) ||
                                            (request_value_w >= pwm_period_o))
                                            start_response(8'h06, 8'h03, 6'd3);
                                        else begin
                                            pwm_duty_o <= request_value_w;
                                            start_response(8'h06, 8'h00, 6'd6);
                                        end
                                    end else
                                        start_response(8'h06, 8'h02, 6'd3);
                                end else
                                    start_response(request_r[1], 8'h01, 6'd3);
                            end
                        end else begin
                            rx_index_r <= rx_index_r + 1'b1;
                        end
                    end else if (rx_index_r != 0) begin
                        if (gap_count_r == FRAME_GAP_CLKS - 1'b1) begin
                            gap_count_r <= 16'd0;
                            rx_index_r <= 3'd0;
                            request_crc_r <= 16'hFFFF;
                            error_frame_count_o <= {8'd0, (error_frame_count_o[7:0] + 8'd1)};
                            incomplete_frame_count_o <= {8'd0, (incomplete_frame_count_o[7:0] + 8'd1)};
                            protocol_error_pulse_o <= 1'b1;
                        end else
                            gap_count_r <= gap_count_r + 1'b1;
                    end
                end

                STATE_SEND: begin
                    if (!tx_busy_i) begin
                        if (tx_index_r < payload_length_r) begin
                            tx_byte_o <= response_byte(tx_index_r);
                            tx_crc_r <= crc16_update(tx_crc_r,
                                                   response_byte(tx_index_r));
                        end else if (tx_index_r == payload_length_r)
                            tx_byte_o <= tx_crc_r[7:0];
                        else
                            tx_byte_o <= tx_crc_r[15:8];
                        tx_start_o <= 1'b1;
                        state_r <= STATE_WAIT;
                    end
                end

                default: begin
                    if (tx_done_i) begin
                        if (tx_index_r == payload_length_r + 1'b1) begin
                            state_r <= STATE_RX;
                            tx_index_r <= 6'd0;
                            if (reset_pending_r) begin
                                reset_pending_r <= 1'b0;
                                soft_reset_o <= 1'b1;
                            end
                        end else begin
                            tx_index_r <= tx_index_r + 1'b1;
                            state_r <= STATE_SEND;
                        end
                    end
                end
            endcase
        end
    end
endmodule
