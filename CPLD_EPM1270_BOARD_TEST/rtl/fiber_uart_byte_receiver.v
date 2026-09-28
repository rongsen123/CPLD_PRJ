/*
 * Receive one standard UART byte: 1 start, 8 data, 1 stop, LSB first.
 */
module fiber_uart_byte_receiver #(
    parameter [8:0] CLKS_PER_BIT = 9'd256
) (
    input  wire       clk_i,              // 30 MHz system clock
    input  wire       reset_n_i,          // Active-low reset
    input  wire       serial_rx_i,        // Optical receiver, idle HIGH
    output reg  [7:0] rx_byte_o,           // Last valid UART byte
    output reg        rx_byte_valid_o,     // One-clock byte-valid pulse
    output reg        rx_frame_error_o     // One-clock stop-bit error pulse
);

    localparam [1:0] STATE_IDLE  = 2'd0;
    localparam [1:0] STATE_START = 2'd1;
    localparam [1:0] STATE_DATA  = 2'd2;
    localparam [1:0] STATE_STOP  = 2'd3;

    reg       rx_meta_r;        // First asynchronous-input synchronizer stage
    reg       rx_sync_r;        // Synchronized serial input
    reg [1:0] state_r;          // UART receive state
    reg [8:0] baud_cnt_r;       // Clock count within one UART bit
    reg [2:0] bit_cnt_r;        // Received data-bit index
    reg [7:0] shift_data_r;     // LSB-first receive shift register

    always @(posedge clk_i or negedge reset_n_i) begin
        if (!reset_n_i) begin
            rx_meta_r        <= 1'b1;
            rx_sync_r        <= 1'b1;
            state_r          <= STATE_IDLE;
            baud_cnt_r       <= 9'd0;
            bit_cnt_r        <= 3'd0;
            shift_data_r     <= 8'd0;
            rx_byte_o        <= 8'd0;
            rx_byte_valid_o  <= 1'b0;
            rx_frame_error_o <= 1'b0;
        end else begin
            rx_meta_r       <= serial_rx_i;
            rx_sync_r       <= rx_meta_r;
            rx_byte_valid_o <= 1'b0;
            rx_frame_error_o <= 1'b0;

            case (state_r)
                STATE_IDLE: begin
                    baud_cnt_r <= 9'd0;
                    if (!rx_sync_r)
                        state_r <= STATE_START;
                end

                STATE_START: begin
                    if (baud_cnt_r == (CLKS_PER_BIT >> 1) - 1'b1) begin
                        baud_cnt_r <= 9'd0;
                        if (!rx_sync_r) begin
                            bit_cnt_r <= 3'd0;
                            state_r   <= STATE_DATA;
                        end else begin
                            state_r <= STATE_IDLE;
                        end
                    end else begin
                        baud_cnt_r <= baud_cnt_r + 1'b1;
                    end
                end

                STATE_DATA: begin
                    if (baud_cnt_r == CLKS_PER_BIT - 1'b1) begin
                        baud_cnt_r             <= 9'd0;
                        shift_data_r[bit_cnt_r] <= rx_sync_r;
                        if (bit_cnt_r == 3'd7)
                            state_r <= STATE_STOP;
                        else
                            bit_cnt_r <= bit_cnt_r + 1'b1;
                    end else begin
                        baud_cnt_r <= baud_cnt_r + 1'b1;
                    end
                end

                STATE_STOP: begin
                    if (baud_cnt_r == CLKS_PER_BIT - 1'b1) begin
                        baud_cnt_r <= 9'd0;
                        state_r    <= STATE_IDLE;
                        if (rx_sync_r) begin
                            rx_byte_o        <= shift_data_r;
                            rx_byte_valid_o  <= 1'b1;
                        end else begin
                            rx_frame_error_o <= 1'b1;
                        end
                    end else begin
                        baud_cnt_r <= baud_cnt_r + 1'b1;
                    end
                end

                default: state_r <= STATE_IDLE;
            endcase
        end
    end

endmodule
