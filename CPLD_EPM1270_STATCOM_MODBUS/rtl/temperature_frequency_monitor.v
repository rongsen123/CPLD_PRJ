/*
 * Temperature frequency input monitor.
 * The raw output is the pulse count in a 100 ms window.
 */
module temperature_frequency_monitor #(
    parameter [21:0] WINDOW_CYCLES  = 22'd3000000,
    parameter [15:0] OVER_TEMP_COUNT = 16'd5000,
    parameter        HIGH_FREQ_IS_HOT = 1'b1
) (
    input  wire        clk_i,                      // 30 MHz system clock
    input  wire        reset_n_i,                  // Active-low reset
    input  wire        temperature_freq_i,       // Asynchronous sensor frequency
    output reg  [15:0] temperature_count_o,      // Pulse count per window
    output reg         temperature_valid_o,      // One-clock update pulse
    output reg         temperature_over_fault_o, // Temperature threshold fault
    output reg         temperature_sensor_fault_o // No pulse detected
);

    reg        temperature_meta_r;
    reg        temperature_sync_r;
    reg        temperature_sync_d_r;
    reg [21:0] window_cnt_r;
    reg [15:0] pulse_cnt_r;
    wire       temperature_rise_w;
    wire [15:0] window_pulse_count_w;

    assign temperature_rise_w = temperature_sync_r & ~temperature_sync_d_r;
    assign window_pulse_count_w =
        (pulse_cnt_r == 16'hFFFF) ? 16'hFFFF :
        pulse_cnt_r + {{15{1'b0}}, temperature_rise_w};

    always @(posedge clk_i or negedge reset_n_i) begin
        if (!reset_n_i) begin
            temperature_meta_r   <= 1'b0;
            temperature_sync_r   <= 1'b0;
            temperature_sync_d_r <= 1'b0;
            window_cnt_r         <= 22'd0;
            pulse_cnt_r          <= 16'd0;
            temperature_count_o       <= 16'd0;
            temperature_valid_o       <= 1'b0;
            temperature_over_fault_o  <= 1'b0;
            // Sensor status is unknown until the first complete window; do
            // not create a false startup fault for the fault latch.
            temperature_sensor_fault_o<= 1'b0;
        end else begin
            temperature_meta_r   <= temperature_freq_i;
            temperature_sync_r   <= temperature_meta_r;
            temperature_sync_d_r <= temperature_sync_r;
            temperature_valid_o  <= 1'b0;

            if (window_cnt_r == WINDOW_CYCLES - 1'b1) begin
                window_cnt_r        <= 22'd0;
                pulse_cnt_r         <= 16'd0;
                temperature_count_o        <= window_pulse_count_w;
                temperature_valid_o        <= 1'b1;
                temperature_sensor_fault_o <= (window_pulse_count_w == 16'd0);

                if (HIGH_FREQ_IS_HOT)
                    temperature_over_fault_o <= (window_pulse_count_w >= OVER_TEMP_COUNT);
                else
                    temperature_over_fault_o <= (window_pulse_count_w <= OVER_TEMP_COUNT);
            end else begin
                window_cnt_r <= window_cnt_r + 1'b1;
                if (temperature_rise_w && (pulse_cnt_r != 16'hFFFF))
                    pulse_cnt_r <= pulse_cnt_r + 1'b1;
            end
        end
    end

endmodule
