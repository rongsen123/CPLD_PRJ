/*
 * Collect and latch hardware, communication and software protection faults.
 * A reset pulse clears only faults whose sources are no longer active.
 */
module system_fault_monitor #(
    parameter [3:0]  MODULE_ID = 4'h1,
    parameter [11:0] ADC_OVER_LIMIT = 12'd3410
) (
    input  wire        clk_i,                       // 30 MHz system clock
    input  wire        reset_n_i,                   // Active-low reset
    input  wire        fault_reset_i,               // One-clock valid reset pulse
    input  wire        bypass_closed_i,             // HIGH means module is bypassed
    input  wire        peer_fault_i,                // HIGH means peer module fault
    input  wire        dc_overvoltage_fault_i,      // HIGH means hardware DC fault
    input  wire        drive_fault_1_i,             // HIGH means bridge 1 drive fault
    input  wire        drive_fault_2_i,             // HIGH means bridge 2 drive fault
    input  wire        uplink_fault_i,              // Remote reports uplink failure
    input  wire        downlink_fault_i,            // UART receive timeout
    input  wire        rx_frame_fault_i,            // UART or CRC frame error
    input  wire        temperature_over_fault_i,    // Temperature threshold fault
    input  wire        temperature_sensor_fault_i,  // Temperature input timeout
    input  wire [11:0] adc_raw_data_i,              // Latest raw ADC code
    input  wire        adc_raw_data_valid_i,        // ADC update pulse
    output reg  [11:0] fault_flags_o,               // Latched individual fault bits
    output reg  [15:0] fault_code_o,                // Module ID plus latched bits
    output reg         fault_any_o,                 // At least one latched fault
    output reg         fault_led_n_o,               // Active-low fault LED
    output reg         rx_fault_led_n_o,            // Active-low downlink fault LED
    output reg         tx_fault_led_n_o             // Active-low uplink fault LED
);

    reg [4:0] async_fault_meta_r;
    reg [4:0] async_fault_sync_r;
    reg       sw_over_fault_r;
    wire [4:0] async_fault_w;
    wire [11:0] active_fault_flags_w;
    wire [11:0] next_latched_fault_flags_w;
    wire        next_fault_any_w;

    assign async_fault_w = {
        drive_fault_2_i,
        drive_fault_1_i,
        dc_overvoltage_fault_i,
        peer_fault_i,
        bypass_closed_i
    };

    // [11:0] = temp sensor, RX frame, reserved, overtemp, software OV,
    // drive2, drive1, hardware VDC, downlink, uplink, peer, bypass.
    assign active_fault_flags_w = {
        temperature_sensor_fault_i,
        rx_frame_fault_i,
        1'b0,
        temperature_over_fault_i,
        sw_over_fault_r,
        async_fault_sync_r[4],
        async_fault_sync_r[3],
        async_fault_sync_r[2],
        downlink_fault_i,
        uplink_fault_i,
        async_fault_sync_r[1],
        async_fault_sync_r[0]
    };

    // Reset is not allowed to hide a fault that is still physically active.
    assign next_latched_fault_flags_w = fault_reset_i ?
        active_fault_flags_w : (fault_flags_o | active_fault_flags_w);
    assign next_fault_any_w = |next_latched_fault_flags_w;

    always @(posedge clk_i or negedge reset_n_i) begin
        if (!reset_n_i) begin
            async_fault_meta_r <= 5'd0;
            async_fault_sync_r <= 5'd0;
            sw_over_fault_r    <= 1'b0;
            fault_flags_o      <= 12'd0;
            fault_code_o       <= 16'd0;
            fault_any_o        <= 1'b0;
            fault_led_n_o      <= 1'b1;
            rx_fault_led_n_o   <= 1'b1;
            tx_fault_led_n_o   <= 1'b1;
        end else begin
            async_fault_meta_r <= async_fault_w;
            async_fault_sync_r <= async_fault_meta_r;

            if (adc_raw_data_valid_i)
                sw_over_fault_r <= (adc_raw_data_i >= ADC_OVER_LIMIT);

            fault_flags_o <= next_latched_fault_flags_w;
            fault_any_o   <= next_fault_any_w;
            fault_code_o  <= next_fault_any_w ?
                             {MODULE_ID, next_latched_fault_flags_w} : 16'h0000;
            fault_led_n_o    <= ~next_fault_any_w;
            rx_fault_led_n_o <= ~(next_latched_fault_flags_w[10] |
                                   next_latched_fault_flags_w[3]);
            tx_fault_led_n_o <= ~next_latched_fault_flags_w[2];
        end
    end

endmodule
