/*
 * ADS7818 continuous acquisition controller.
 * 30 MHz input clock, 7.5 MHz ADC clock, 468.75 kSPS.
 */
module ads7818_acquisition_controller (
    input  wire        clk_i,            // 30 MHz system clock
    input  wire        reset_n_i,        // Active-low reset
    input  wire        serial_data_i,    // ADS7818 serial data
    output wire        serial_clk_o,     // 7.5 MHz ADC clock
    output reg         convert_o,        // ADS7818 conversion control
    output reg  [11:0] raw_data_o,       // Raw conversion result
    output reg         raw_data_valid_o  // One-clock result pulse
);

    reg [1:0]  clk_div_cnt_r;
    reg [3:0]  adc_clk_cnt_r;
    reg [11:0] adc_shift_r;
    reg        conversion_started_r;

    assign serial_clk_o = clk_div_cnt_r[1];

    // Generate the 16-clock conversion frame and receive D11 to D0.
    always @(posedge clk_i or negedge reset_n_i) begin
        if (!reset_n_i) begin
            clk_div_cnt_r        <= 2'd0;
            adc_clk_cnt_r        <= 4'd15;
            adc_shift_r          <= 12'd0;
            conversion_started_r <= 1'b0;
            convert_o            <= 1'b1;
            raw_data_o           <= 12'd0;
            raw_data_valid_o     <= 1'b0;
        end else begin
            clk_div_cnt_r    <= clk_div_cnt_r + 2'd1;
            raw_data_valid_o <= 1'b0;

            // Capture serial data on ADC clock rising edges 2 to 13.
            if (clk_div_cnt_r == 2'd1) begin
                if (adc_clk_cnt_r == 4'd15)
                    adc_clk_cnt_r <= 4'd0;
                else
                    adc_clk_cnt_r <= adc_clk_cnt_r + 4'd1;

                if (conversion_started_r && (adc_clk_cnt_r <= 4'd11)) begin
                    adc_shift_r <= {adc_shift_r[10:0], serial_data_i};

                    if (adc_clk_cnt_r == 4'd11) begin
                        raw_data_o       <= {adc_shift_r[10:0], serial_data_i};
                        raw_data_valid_o <= 1'b1;
                    end
                end
            end

            // Acquisition starts after clock 12 and lasts 466.67 ns.
            if ((clk_div_cnt_r == 2'd0) && (adc_clk_cnt_r == 4'd11))
                convert_o <= 1'b1;

            // A falling CONV edge during clock 16 starts the next conversion.
            if ((clk_div_cnt_r == 2'd2) && (adc_clk_cnt_r == 4'd15)) begin
                convert_o            <= 1'b0;
                conversion_started_r <= 1'b1;
            end
        end
    end

endmodule
