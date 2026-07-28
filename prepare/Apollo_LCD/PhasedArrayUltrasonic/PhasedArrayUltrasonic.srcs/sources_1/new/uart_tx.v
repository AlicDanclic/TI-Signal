`timescale 1ns / 1ps

module uart_tx #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD_RATE   = 115200
) (
    input  wire      clk,
    input  wire      rst_n,
    input  wire [7:0] data_in,
    input  wire      data_valid,
    output reg       tx,
    output wire      ready
);

    localparam integer BAUD_DIV = CLK_FREQ_HZ / BAUD_RATE;

    reg [9:0]  shift_reg;
    reg [15:0] baud_cnt;
    reg [3:0]  bit_idx;
    reg        busy;

    assign ready = ~busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg <= 10'h3ff;
            baud_cnt  <= 16'd0;
            bit_idx   <= 4'd0;
            busy      <= 1'b0;
            tx        <= 1'b1;
        end else begin
            if (!busy) begin
                tx <= 1'b1;
                if (data_valid) begin
                    shift_reg <= {1'b1, data_in, 1'b0};
                    baud_cnt  <= BAUD_DIV - 1;
                    bit_idx   <= 4'd0;
                    busy      <= 1'b1;
                    tx        <= 1'b0;
                end
            end else if (baud_cnt == 0) begin
                baud_cnt  <= BAUD_DIV - 1;
                bit_idx   <= bit_idx + 1'b1;
                shift_reg <= {1'b1, shift_reg[9:1]};

                if (bit_idx == 4'd9) begin
                    busy <= 1'b0;
                    tx   <= 1'b1;
                end else begin
                    tx <= shift_reg[1];
                end
            end else begin
                baud_cnt <= baud_cnt - 1'b1;
            end
        end
    end

endmodule
