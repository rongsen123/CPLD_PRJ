/*
 * Transmit one standard UART byte: 1 start, 8 data, 1 stop, LSB first.
 */
module fiber_uart_byte_transmitter #(
    parameter [8:0] CLKS_PER_BIT = 9'd256
) (
    input  wire       clk_i,          // 30 MHz system clock
    input  wire       reset_n_i,      // Active-low reset
    input  wire [7:0] tx_byte_i,       // Byte to transmit
    input  wire       tx_start_i,      // One-clock transmit request
    output reg        serial_tx_o,     // Optical transmitter, idle HIGH
    output reg        tx_busy_o,       // HIGH while one byte is active
    output reg        tx_done_o        // One-clock completion pulse
);

    localparam [1:0] STATE_IDLE  = 2'd0;
    localparam [1:0] STATE_START = 2'd1;
    localparam [1:0] STATE_DATA  = 2'd2;
    localparam [1:0] STATE_STOP  = 2'd3;

    reg [1:0] state_r;       // UART transmit state
    reg [8:0] baud_cnt_r;    // Clock count within one UART bit
    reg [2:0] bit_cnt_r;     // Transmitted data-bit index
    reg [7:0] shift_data_r;  // LSB-first transmit shift register

    always @(posedge clk_i or negedge reset_n_i) begin
        if (!reset_n_i) begin
            state_r      <= STATE_IDLE;
            baud_cnt_r   <= 9'd0;
            bit_cnt_r    <= 3'd0;
            shift_data_r <= 8'd0;
            serial_tx_o  <= 1'b1;
            tx_busy_o    <= 1'b0;
            tx_done_o    <= 1'b0;
        end else begin
            tx_done_o <= 1'b0;

            case (state_r)
                STATE_IDLE: begin
                    serial_tx_o <= 1'b1;
                    tx_busy_o   <= 1'b0;
                    baud_cnt_r  <= 9'd0;
                    if (tx_start_i) begin
                        shift_data_r <= tx_byte_i;
                        serial_tx_o  <= 1'b0;
                        tx_busy_o    <= 1'b1;
                        state_r      <= STATE_START;
                    end
                end

                STATE_START: begin
                    if (baud_cnt_r == CLKS_PER_BIT - 1'b1) begin
                        baud_cnt_r  <= 9'd0;
                        bit_cnt_r   <= 3'd0;
                        serial_tx_o <= shift_data_r[0];
                        state_r     <= STATE_DATA;
                    end else begin
                        baud_cnt_r <= baud_cnt_r + 1'b1;
                    end
                end

                STATE_DATA: begin
                    if (baud_cnt_r == CLKS_PER_BIT - 1'b1) begin
                        baud_cnt_r <= 9'd0;
                        if (bit_cnt_r == 3'd7) begin
                            serial_tx_o <= 1'b1;
                            state_r     <= STATE_STOP;
                        end else begin
                            shift_data_r <= {1'b0, shift_data_r[7:1]};
                            bit_cnt_r    <= bit_cnt_r + 1'b1;
                            serial_tx_o  <= shift_data_r[1];
                        end
                    end else begin
                        baud_cnt_r <= baud_cnt_r + 1'b1;
                    end
                end

                STATE_STOP: begin
                    if (baud_cnt_r == CLKS_PER_BIT - 1'b1) begin
                        baud_cnt_r  <= 9'd0;
                        serial_tx_o <= 1'b1;
                        tx_busy_o   <= 1'b0;
                        tx_done_o   <= 1'b1;
                        state_r     <= STATE_IDLE;
                    end else begin
                        baud_cnt_r <= baud_cnt_r + 1'b1;
                    end
                end

                default: state_r <= STATE_IDLE;
            endcase
        end
    end

endmodule
