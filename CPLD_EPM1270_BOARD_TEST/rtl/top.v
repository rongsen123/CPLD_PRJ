/*
 * EPM1270 dedicated board-test image. Optical UART is the PC/DSP Modbus link.
 * Power outputs are off at reset; raw hardware faults always inhibit them.
 */
module top #(
    parameter [21:0] TEMPERATURE_WINDOW_CLKS = 22'd3000000
) (
    input wire sys_clk_i,
    input wire adc_serial_data_i,
    output wire adc_serial_clk_o,
    output wire adc_convert_o,
    input wire temperature_freq_i,
    input wire bypass_status_i,
    input wire peer_module_fault_i,
    input wire spare_digital_input_i,
    output wire fan_relay_n_o,
    output wire bypass_relay_n_o,
    output wire fault_output_n_o,
    input wire fiber_rx_i,
    output wire fiber_tx_o,
    output wire pwm_bridge_1_a_o,
    output wire pwm_bridge_1_b_o,
    output wire pwm_bridge_2_a_o,
    output wire pwm_bridge_2_b_o,
    output wire pwm_hold_o,
    input wire dc_overvoltage_fault_i,
    input wire drive_fault_1_i,
    input wire drive_fault_2_i,
    output wire fault_led_n_o,
    output wire rx_fault_led_n_o,
    output wire tx_fault_led_n_o,
    output wire LED1,
    output wire LED2,
    output wire LED3,
    output wire LED4
);
    reg [4:0] por_r;
    wire board_reset_n_w = por_r[4];
    reg [5:0] reset_stretch_r;
    wire logic_reset_n_w = board_reset_n_w & (reset_stretch_r == 0);
    wire soft_reset_w;
    initial reset_stretch_r = 6'd0;
    always @(posedge sys_clk_i or negedge board_reset_n_w) begin
        if (!board_reset_n_w) reset_stretch_r <= 6'd0;
        else if (soft_reset_w) reset_stretch_r <= 6'd31;
        else if (reset_stretch_r != 0)
            reset_stretch_r <= reset_stretch_r - 1'b1;
    end
    initial por_r = 5'd0;
    always @(posedge sys_clk_i)
        if (!board_reset_n_w) por_r <= por_r + 1'b1;

    wire [11:0] adc_raw_w, adc_average_w;
    wire adc_valid_w;
    reg adc_seen_r;
    wire [15:0] temperature_count_w;
    wire temperature_valid_w, temperature_over_w, temperature_sensor_w;
    reg temperature_seen_r;

    ads7818_acquisition_controller u_adc (
        .clk_i(sys_clk_i), .reset_n_i(logic_reset_n_w),
        .serial_data_i(adc_serial_data_i),
        .serial_clk_o(adc_serial_clk_o), .convert_o(adc_convert_o),
        .raw_data_o(adc_raw_w), .raw_data_valid_o(adc_valid_w)
    );
    assign adc_average_w = adc_raw_w;
    temperature_frequency_monitor #(
        .WINDOW_CYCLES(TEMPERATURE_WINDOW_CLKS),
        .OVER_TEMP_COUNT(16'd5000), .HIGH_FREQ_IS_HOT(1'b1)
    ) u_temperature (
        .clk_i(sys_clk_i), .reset_n_i(logic_reset_n_w),
        .temperature_freq_i(temperature_freq_i),
        .temperature_count_o(temperature_count_w),
        .temperature_valid_o(temperature_valid_w),
        .temperature_over_fault_o(temperature_over_w),
        .temperature_sensor_fault_o(temperature_sensor_w)
    );
    always @(posedge sys_clk_i or negedge logic_reset_n_w) begin
        if (!logic_reset_n_w) begin
            adc_seen_r <= 1'b0;
            temperature_seen_r <= 1'b0;
        end else begin
            if (adc_valid_w) adc_seen_r <= 1'b1;
            if (temperature_valid_w) temperature_seen_r <= 1'b1;
        end
    end

    wire [7:0] rx_byte_w, tx_byte_w;
    wire rx_valid_w, rx_error_w, tx_start_w, tx_busy_w, tx_done_w;
    fiber_uart_byte_receiver #(.CLKS_PER_BIT(9'd260)) u_rx (
        .clk_i(sys_clk_i), .reset_n_i(logic_reset_n_w),
        .serial_rx_i(~fiber_rx_i),
        .rx_byte_o(rx_byte_w), .rx_byte_valid_o(rx_valid_w),
        .rx_frame_error_o(rx_error_w)
    );
    fiber_uart_byte_transmitter #(.CLKS_PER_BIT(9'd260)) u_tx (
        .clk_i(sys_clk_i), .reset_n_i(logic_reset_n_w),
        .tx_byte_i(tx_byte_w), .tx_start_i(tx_start_w),
        .serial_tx_o(fiber_tx_o), .tx_busy_o(tx_busy_w),
        .tx_done_o(tx_done_w)
    );

    wire [11:0] fault_flags_w, fault_mask_w, fault_sim_w;
    wire [11:0] raw_faults_w = fault_flags_w | fault_sim_w;
    wire [11:0] effective_faults_w = raw_faults_w & ~fault_mask_w;
    wire fault_any_w = |effective_faults_w;
    wire direct_hw_fault_w = ~dc_overvoltage_fault_i |
                             ~drive_fault_1_i | ~drive_fault_2_i;
    wire [15:0] fault_code_unused_w;
    wire fault_any_unused_w;
    wire clear_fault_w, link_seen_w, link_fault_w;
    wire protocol_error_w;
    wire [15:0] command_w, pwm_period_w, pwm_duty_w;
    wire [11:0] vdc_limit_w;
    wire [2:0] test_control_w;
    wire [15:0] valid_count_w, error_count_w;
    wire [15:0] uart_error_count_w, crc_error_count_w, incomplete_count_w;
    wire run_w = command_w[0] & ~fault_any_w & ~direct_hw_fault_w &
                 link_seen_w & ~link_fault_w & logic_reset_n_w;
    wire pwm_active_w = run_w & test_control_w[2];
    wire [15:0] digital_inputs_w = {
        ~bypass_relay_n_o, ~fan_relay_n_o, pwm_active_w, run_w,
        fault_any_w, link_seen_w & ~link_fault_w,
        temperature_seen_r, adc_seen_r, temperature_sensor_w,
        temperature_over_w, ~drive_fault_2_i, ~drive_fault_1_i,
        ~dc_overvoltage_fault_i, spare_digital_input_i,
        peer_module_fault_i, bypass_status_i
    };

    board_test_modbus u_modbus (
        .clk_i(sys_clk_i), .reset_n_i(logic_reset_n_w),
        .rx_byte_i(rx_byte_w), .rx_byte_valid_i(rx_valid_w),
        .rx_frame_error_i(rx_error_w),
        .tx_byte_o(tx_byte_w), .tx_start_o(tx_start_w),
        .tx_busy_i(tx_busy_w), .tx_done_i(tx_done_w),
        .vdc_raw_i(adc_raw_w), .vdc_average_i(adc_average_w),
        .temperature_count_i(temperature_count_w),
        .adc_valid_i(adc_seen_r),
        .temperature_valid_i(temperature_seen_r),
        .fault_flags_i({20'd0, raw_faults_w}),
        .fault_any_i(fault_any_w | direct_hw_fault_w),
        .command_echo_o(command_w), .vdc_over_limit_o(vdc_limit_w),
        .clear_fault_o(clear_fault_w), .soft_reset_o(soft_reset_w),
        .link_seen_o(link_seen_w), .link_fault_o(link_fault_w),
        .protocol_error_pulse_o(protocol_error_w),
        .valid_frame_count_o(valid_count_w),
        .error_frame_count_o(error_count_w),
        .uart_error_count_o(uart_error_count_w),
        .crc_error_count_o(crc_error_count_w),
        .incomplete_frame_count_o(incomplete_count_w),
        .digital_inputs_i(digital_inputs_w),
        .effective_faults_i(effective_faults_w),
        .pwm_active_i(pwm_active_w),
        .fault_mask_o(fault_mask_w), .fault_sim_o(fault_sim_w),
        .test_control_o(test_control_w),
        .pwm_period_o(pwm_period_w), .pwm_duty_o(pwm_duty_w)
    );

    system_fault_monitor u_fault (
        .clk_i(sys_clk_i), .reset_n_i(logic_reset_n_w),
        .fault_reset_i(clear_fault_w),
        .bypass_closed_i(bypass_status_i),
        .peer_fault_i(peer_module_fault_i),
        .dc_overvoltage_fault_i(~dc_overvoltage_fault_i),
        .drive_fault_1_i(~drive_fault_1_i),
        .drive_fault_2_i(~drive_fault_2_i),
        .uplink_fault_i(1'b0),
        .downlink_fault_i(link_fault_w),
        .rx_frame_fault_i(protocol_error_w),
        .temperature_over_fault_i(temperature_over_w),
        .temperature_sensor_fault_i(temperature_sensor_w),
        .adc_raw_data_i(adc_raw_w), .adc_raw_data_valid_i(adc_valid_w),
        .adc_over_limit_i(vdc_limit_w),
        .fault_flags_o(fault_flags_w), .fault_code_o(fault_code_unused_w),
        .fault_any_o(fault_any_unused_w),
        .fault_led_n_o(), .rx_fault_led_n_o(), .tx_fault_led_n_o()
    );

    // A single registered PWM state drives both complementary pin pairs.
    // No intentional dead time is inserted. Both pins are LOW under HOLD.
    reg [15:0] pwm_count_r;
    reg        pwm_state_r;
    reg        pwm_started_r;
    always @(posedge sys_clk_i or negedge logic_reset_n_w) begin
        if (!logic_reset_n_w) begin
            pwm_count_r   <= 16'd0;
            pwm_state_r   <= 1'b0;
            pwm_started_r <= 1'b0;
        end else if (!pwm_active_w) begin
            pwm_count_r   <= 16'd0;
            pwm_state_r   <= 1'b0;
            pwm_started_r <= 1'b0;
        end else if (!pwm_started_r) begin
            pwm_count_r   <= 16'd0;
            pwm_state_r   <= 1'b1;
            pwm_started_r <= 1'b1;
        end else if (pwm_count_r >= pwm_period_w - 1'b1) begin
            pwm_count_r <= 16'd0;
            pwm_state_r <= 1'b1;
        end else begin
            pwm_count_r <= pwm_count_r + 1'b1;
            if (pwm_count_r == pwm_duty_w - 1'b1)
                pwm_state_r <= 1'b0;
        end
    end
    wire pwm_permit_w = pwm_active_w & pwm_started_r & ~direct_hw_fault_w;
    assign pwm_hold_o = ~pwm_permit_w;
    assign pwm_bridge_1_a_o = pwm_permit_w & pwm_state_r;
    assign pwm_bridge_1_b_o = pwm_permit_w & ~pwm_state_r;
    assign pwm_bridge_2_a_o = pwm_permit_w & pwm_state_r;
    assign pwm_bridge_2_b_o = pwm_permit_w & ~pwm_state_r;
    assign fan_relay_n_o = ~(run_w & test_control_w[0]);
    assign bypass_relay_n_o = ~(run_w & test_control_w[1]);
    assign fault_output_n_o = ~(fault_any_w | direct_hw_fault_w);
    assign fault_led_n_o = ~(|raw_faults_w);
    assign rx_fault_led_n_o = ~(raw_faults_w[10] | raw_faults_w[3]);
    assign tx_fault_led_n_o = ~(link_seen_w & ~link_fault_w);
    assign LED1 = ~valid_count_w[4];
    assign LED2 = ~pwm_active_w;
    assign LED3 = ~adc_seen_r;
    assign LED4 = ~temperature_seen_r;
endmodule
