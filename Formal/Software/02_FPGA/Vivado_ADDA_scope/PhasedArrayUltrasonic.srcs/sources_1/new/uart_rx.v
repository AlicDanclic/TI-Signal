`timescale 1ns / 1ps

module uart_rx #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD_RATE   = 115200
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg [7:0]  data_out,
    output reg        data_valid
);

    localparam integer BAUD_DIV      = CLK_FREQ_HZ / BAUD_RATE;
    localparam integer HALF_BAUD_DIV = BAUD_DIV / 2;

    localparam [1:0] S_IDLE  = 2'd0;
    localparam [1:0] S_START = 2'd1;
    localparam [1:0] S_DATA  = 2'd2;
    localparam [1:0] S_STOP  = 2'd3;

    reg        rx_ff0;
    reg        rx_ff1;
    reg [1:0]  state;
    reg [15:0] baud_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  shift_reg;

    wire rx_sync = rx_ff1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_ff0 <= 1'b1;
            rx_ff1 <= 1'b1;
        end else begin
            rx_ff0 <= rx;
            rx_ff1 <= rx_ff0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            baud_cnt   <= 16'd0;
            bit_idx    <= 3'd0;
            shift_reg  <= 8'd0;
            data_out   <= 8'd0;
            data_valid <= 1'b0;
        end else begin
            data_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    baud_cnt <= 16'd0;
                    bit_idx  <= 3'd0;
                    if (!rx_sync) begin
                        state    <= S_START;
                        baud_cnt <= HALF_BAUD_DIV - 1;
                    end
                end

                S_START: begin
                    if (baud_cnt == 0) begin
                        if (!rx_sync) begin
                            state    <= S_DATA;
                            baud_cnt <= BAUD_DIV - 1;
                            bit_idx  <= 3'd0;
                        end else begin
                            state <= S_IDLE;
                        end
                    end else begin
                        baud_cnt <= baud_cnt - 1'b1;
                    end
                end

                S_DATA: begin
                    if (baud_cnt == 0) begin
                        shift_reg[bit_idx] <= rx_sync;
                        baud_cnt           <= BAUD_DIV - 1;

                        if (bit_idx == 3'd7) begin
                            state <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                        end
                    end else begin
                        baud_cnt <= baud_cnt - 1'b1;
                    end
                end

                S_STOP: begin
                    if (baud_cnt == 0) begin
                        state <= S_IDLE;
                        if (rx_sync) begin
                            data_out   <= shift_reg;
                            data_valid <= 1'b1;
                        end
                    end else begin
                        baud_cnt <= baud_cnt - 1'b1;
                    end
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
