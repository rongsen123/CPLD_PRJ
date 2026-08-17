/* Calculate one arithmetic mean for every 256 consecutive 12-bit samples. */
module adc_block_average (
    input  wire        clk_i,             // 30 MHz system clock
    input  wire        reset_n_i,         // Active-low reset
    input  wire [11:0] sample_i,          // Latest unsigned ADC sample
    input  wire        sample_valid_i,    // One-clock sample pulse
    output reg  [11:0] average_o,         // Completed 256-sample average
    output reg         average_valid_o    // One-clock average update pulse
);
    reg [19:0] sum_r;                     // Maximum 256*4095 = 1,048,320
    reg [7:0] sample_count_r;             // Samples accumulated in current block
    wire [19:0] sum_with_sample_w = sum_r + {8'd0, sample_i};

    always @(posedge clk_i or negedge reset_n_i) begin
        if (!reset_n_i) begin
            sum_r <= 20'd0;
            sample_count_r <= 8'd0;
            average_o <= 12'd0;
            average_valid_o <= 1'b0;
        end else begin
            average_valid_o <= 1'b0;
            if (sample_valid_i) begin
                if (sample_count_r == 8'hFF) begin
                    average_o <= sum_with_sample_w[19:8];
                    average_valid_o <= 1'b1;
                    sum_r <= 20'd0;
                    sample_count_r <= 8'd0;
                end else begin
                    sum_r <= sum_with_sample_w;
                    sample_count_r <= sample_count_r + 1'b1;
                end
            end
        end
    end
endmodule
