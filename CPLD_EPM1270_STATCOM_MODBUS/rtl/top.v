/*
 * STATCOM Modbus communication/safety top derived from CPLD_1270_prj.
 * The verified board pinout and physical RX inversion are preserved.
 * START is diagnostic echo only; every bridge and relay output is hard-safe.
 */
module top (
    input  wire sys_clk_i,                   // PIN18: 30 MHz board clock
    input  wire adc_serial_data_i,           // PIN71: ADS7818 serial data
    output wire adc_serial_clk_o,            // PIN72: ADS7818 serial clock
    output wire adc_convert_o,               // PIN70: ADS7818 conversion control
    input  wire temperature_freq_i,          // PIN27: temperature frequency input
    input  wire bypass_status_i,             // PIN77: bypass feedback, active high
    input  wire peer_module_fault_i,         // PIN76: peer fault, active high
    input  wire spare_digital_input_i,       // PIN75: reserved board input
    output wire fan_relay_n_o,               // PIN114: fan relay, active low
    output wire bypass_relay_n_o,            // PIN107: bypass relay, active low
    output wire fault_output_n_o,            // PIN108: total fault, active low
    input  wire fiber_rx_i,                  // PIN42: optical RX, physical idle low
    output wire fiber_tx_o,                  // PIN43: standard UART TX, idle high
    output wire pwm_bridge_1_a_o,            // PIN141: hard-safe low
    output wire pwm_bridge_1_b_o,            // PIN142: hard-safe low
    output wire pwm_bridge_2_a_o,            // PIN143: hard-safe low
    output wire pwm_bridge_2_b_o,            // PIN144: hard-safe low
    output wire pwm_hold_o,                  // PIN140: hard-safe high
    input  wire dc_overvoltage_fault_i,      // PIN49: hardware fault, active low
    input  wire drive_fault_1_i,             // PIN30: drive fault, active low
    input  wire drive_fault_2_i,             // PIN29: drive fault, active low
    output wire fault_led_n_o,               // PIN39: total fault LED, active low
    output wire rx_fault_led_n_o,            // PIN40: RX fault LED, active low
    output wire tx_fault_led_n_o,            // PIN41: link fault LED, active low
    output wire LED1,                        // PIN38: raw UART-byte activity
    output wire LED2,                        // PIN37: link-online indicator
    output wire LED3,                        // PIN32: valid-Modbus-frame activity
    output wire LED4                         // PIN31: protocol-error activity
);
    // Fixed raw-code temperature protection.  The threshold and direction
    // remain local to the CPLD and are intentionally not writable by Modbus.
    // TODO: replace 5000 only after the sensor count/temperature curve and
    // frequency direction have been verified on hardware.
    localparam [15:0] TEMPERATURE_OVER_LIMIT_COUNT = 16'd5000;
    localparam        TEMPERATURE_HIGH_FREQ_IS_HOT = 1'b1;

    reg [4:0] por_count_r;                    // 16-clock power-on reset counter
    reg [5:0] soft_reset_count_r;             // Internal reset stretch counter
    wire board_reset_n_w = por_count_r[4];    // Power-on reset release
    wire logic_reset_n_w = board_reset_n_w & (soft_reset_count_r == 0);
    wire fiber_uart_rx_w = ~fiber_rx_i;       // Restore standard UART idle high

    wire [11:0] adc_raw_w;                    // Latest ADS7818 sample
    wire adc_valid_w;                         // One-clock ADC sample pulse
    wire [11:0] adc_average_w;                // 256-sample block average
    wire adc_average_valid_w;                 // One-clock average update pulse
    reg adc_seen_r;                           // Sticky ADC-valid status
    wire [15:0] temperature_count_w;          // Pulses per 100 ms
    wire temperature_valid_w;                 // One-clock temperature update
    wire temperature_over_w;                  // Temperature threshold fault
    wire temperature_sensor_fault_w;          // Missing sensor pulses
    reg temperature_seen_r;                   // Sticky temperature-valid status

    wire [7:0] uart_rx_byte_w;                // Received UART byte
    wire uart_rx_valid_w;                     // One-clock RX byte pulse
    wire uart_rx_frame_error_w;               // UART stop-bit error latch
    reg [15:0] uart_rx_byte_count_r;           // Raw valid UART-byte counter
    wire [7:0] uart_tx_byte_w;                // UART transmit byte
    wire uart_tx_start_w;                     // One-clock TX start pulse
    wire uart_tx_busy_w;                      // UART transmitter busy
    wire uart_tx_done_w;                      // One-clock byte-done pulse

    wire [15:0] command_echo_w;                // 0=STOP, 1=START echo only
    wire [11:0] vdc_over_limit_w;              // Runtime local software OV threshold
    wire clear_fault_w;                       // Accepted clear-fault pulse
    wire soft_reset_request_w;                // Asserted after FC06 reset echo completes
    wire link_seen_w;                         // A valid addressed request was seen
    wire link_fault_w;                        // 500 ms request timeout
    wire protocol_error_pulse_w;              // CRC/UART/incomplete-frame pulse
    wire [15:0] valid_frame_count_w;           // Valid addressed requests
    wire [15:0] error_frame_count_w;           // Rejected frames
    wire [15:0] uart_error_count_w;            // UART stop-bit error events
    wire [15:0] crc_error_count_w;             // Modbus CRC mismatch events
    wire [15:0] incomplete_frame_count_w;      // t3.5-expired partial requests
    wire [11:0] fault_flags_w;                 // Base-project latched faults
    wire [15:0] fault_code_unused_w;           // Packed base-project fault code
    wire fault_any_w;                          // Any latched fault
    wire unused_input_w;                       // Preserve reserved input in analysis

    initial por_count_r = 5'd0;
    initial soft_reset_count_r = 6'd0;
    always @(posedge sys_clk_i) begin
        if (!board_reset_n_w)
            por_count_r <= por_count_r + 1'b1;
    end

    always @(posedge sys_clk_i or negedge board_reset_n_w) begin
        if (!board_reset_n_w)
            soft_reset_count_r <= 6'd0;
        else if (soft_reset_request_w)
            soft_reset_count_r <= 6'd31;
        else if (soft_reset_count_r != 0)
            soft_reset_count_r <= soft_reset_count_r - 1'b1;
    end

    always @(posedge sys_clk_i or negedge logic_reset_n_w) begin
        if (!logic_reset_n_w) begin
            adc_seen_r <= 1'b0;
            temperature_seen_r <= 1'b0;
            uart_rx_byte_count_r <= 16'd0;
        end else begin
            if (adc_valid_w) adc_seen_r <= 1'b1;
            if (temperature_valid_w) temperature_seen_r <= 1'b1;
            if (uart_rx_valid_w) uart_rx_byte_count_r <= uart_rx_byte_count_r + 1'b1;
        end
    end

    ads7818_acquisition_controller u_adc (
        .clk_i(sys_clk_i), .reset_n_i(logic_reset_n_w),
        .serial_data_i(adc_serial_data_i),
        .serial_clk_o(adc_serial_clk_o), .convert_o(adc_convert_o),
        .raw_data_o(adc_raw_w), .raw_data_valid_o(adc_valid_w)
    );

    adc_block_average u_adc_average (
        .clk_i(sys_clk_i), .reset_n_i(logic_reset_n_w),
        .sample_i(adc_raw_w), .sample_valid_i(adc_valid_w),
        .average_o(adc_average_w), .average_valid_o(adc_average_valid_w)
    );

    temperature_frequency_monitor #(
        .OVER_TEMP_COUNT(TEMPERATURE_OVER_LIMIT_COUNT),
        .HIGH_FREQ_IS_HOT(TEMPERATURE_HIGH_FREQ_IS_HOT)
    ) u_temperature (
        .clk_i(sys_clk_i), .reset_n_i(logic_reset_n_w),
        .temperature_freq_i(temperature_freq_i),
        .temperature_count_o(temperature_count_w),
        .temperature_valid_o(temperature_valid_w),
        .temperature_over_fault_o(temperature_over_w),
        .temperature_sensor_fault_o(temperature_sensor_fault_w)
    );

    fiber_uart_byte_receiver u_uart_rx (
        .clk_i(sys_clk_i), .reset_n_i(logic_reset_n_w),
        .serial_rx_i(fiber_uart_rx_w),
        .rx_byte_o(uart_rx_byte_w), .rx_byte_valid_o(uart_rx_valid_w),
        .rx_frame_error_o(uart_rx_frame_error_w)
    );

    fiber_uart_byte_transmitter u_uart_tx (
        .clk_i(sys_clk_i), .reset_n_i(logic_reset_n_w),
        .tx_byte_i(uart_tx_byte_w), .tx_start_i(uart_tx_start_w),
        .serial_tx_o(fiber_tx_o), .tx_busy_o(uart_tx_busy_w),
        .tx_done_o(uart_tx_done_w)
    );

    modbus_rtu_slave_fixed8 u_modbus (
        .clk_i(sys_clk_i), .reset_n_i(logic_reset_n_w),
        .rx_byte_i(uart_rx_byte_w), .rx_byte_valid_i(uart_rx_valid_w),
        .rx_frame_error_i(uart_rx_frame_error_w),
        .tx_byte_o(uart_tx_byte_w), .tx_start_o(uart_tx_start_w),
        .tx_busy_i(uart_tx_busy_w), .tx_done_i(uart_tx_done_w),
        .vdc_raw_i(adc_raw_w), .vdc_average_i(adc_average_w),
        .temperature_count_i(temperature_count_w),
        .adc_valid_i(adc_seen_r), .temperature_valid_i(temperature_seen_r),
        .fault_flags_i({20'd0, fault_flags_w}), .fault_any_i(fault_any_w),
        .command_echo_o(command_echo_w),
        .vdc_over_limit_o(vdc_over_limit_w), .clear_fault_o(clear_fault_w),
        .soft_reset_o(soft_reset_request_w),
        .link_seen_o(link_seen_w), .link_fault_o(link_fault_w),
        .protocol_error_pulse_o(protocol_error_pulse_w),
        .valid_frame_count_o(valid_frame_count_w),
        .error_frame_count_o(error_frame_count_w),
        .uart_error_count_o(uart_error_count_w),
        .crc_error_count_o(crc_error_count_w),
        .incomplete_frame_count_o(incomplete_frame_count_w)
    );

    system_fault_monitor u_fault_monitor (
        .clk_i(sys_clk_i), .reset_n_i(logic_reset_n_w),
        .fault_reset_i(clear_fault_w),
        .bypass_closed_i(bypass_status_i),
        .peer_fault_i(peer_module_fault_i),
        .dc_overvoltage_fault_i(~dc_overvoltage_fault_i),
        .drive_fault_1_i(~drive_fault_1_i),
        .drive_fault_2_i(~drive_fault_2_i),
        .uplink_fault_i(1'b0), .downlink_fault_i(link_fault_w),
        .rx_frame_fault_i(protocol_error_pulse_w),
        .temperature_over_fault_i(temperature_over_w),
        .temperature_sensor_fault_i(temperature_sensor_fault_w),
        .adc_raw_data_i(adc_raw_w), .adc_raw_data_valid_i(adc_valid_w),
        .adc_over_limit_i(vdc_over_limit_w),
        .fault_flags_o(fault_flags_w), .fault_code_o(fault_code_unused_w),
        .fault_any_o(fault_any_w), .fault_led_n_o(fault_led_n_o),
        .rx_fault_led_n_o(rx_fault_led_n_o),
        .tx_fault_led_n_o(tx_fault_led_n_o)
    );

    // Communication is diagnostic only in this stage.
    assign pwm_bridge_1_a_o = 1'b0;
    assign pwm_bridge_1_b_o = 1'b0;
    assign pwm_bridge_2_a_o = 1'b0;
    assign pwm_bridge_2_b_o = 1'b0;
    assign pwm_hold_o = 1'b1;
    assign fan_relay_n_o = 1'b1;
    assign bypass_relay_n_o = 1'b1;
    assign fault_output_n_o = ~fault_any_w;

    // Board LEDs are active low and provide a no-oscilloscope receive trace.
    assign LED1 = ~uart_rx_byte_count_r[7];
    assign LED2 = ~(link_seen_w & ~link_fault_w);
    assign LED3 = ~valid_frame_count_w[4];
    assign LED4 = ~error_frame_count_w[4];
    assign unused_input_w = spare_digital_input_i ^ command_echo_w[15] ^
                            adc_average_valid_w;
endmodule
