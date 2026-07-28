`timescale 1ns / 1ps
// ============================================================================
// ============================================================================
module hc595_driver_fixed #(
    parameter CASCADE_NUM        = 1,
    parameter TOTAL_BITS         = CASCADE_NUM * 8,
    parameter CNT_WIDTH          = (TOTAL_BITS > 1) ? $clog2(TOTAL_BITS) : 1,
    parameter SCLK_DIV           = 2,
    parameter SCLK_DIV_WIDTH     = (2*SCLK_DIV > 1) ? $clog2(2*SCLK_DIV) : 1
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire [TOTAL_BITS-1:0] data_in,
    output reg                   ser_out,
    output reg                   sclk,
    output reg                   rclk
);

    localparam S_LOAD      = 2'd0;
    localparam S_SHIFT     = 2'd1;
    localparam S_LAST_FALL = 2'd2;
    localparam S_LATCH     = 2'd3;

    reg [1:0]                 state;
    reg [SCLK_DIV_WIDTH-1:0]  div_cnt;
    reg [TOTAL_BITS-1:0]      shift_reg;
    reg [CNT_WIDTH-1:0]       bit_cnt;

    // ======================================================================
    // ======================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_LOAD;
            sclk     <= 1'b0;
            rclk     <= 1'b0;
            ser_out  <= 1'b0;
            div_cnt  <= {SCLK_DIV_WIDTH{1'b0}};
            shift_reg<= {TOTAL_BITS{1'b0}};
            bit_cnt  <= {CNT_WIDTH{1'b0}};
        end else begin
            case (state)
                S_LOAD: begin
                    sclk     <= 1'b0;
                    rclk     <= 1'b0;
                    div_cnt  <= {SCLK_DIV_WIDTH{1'b0}};
                    shift_reg<= {data_in[TOTAL_BITS-2:0], 1'b0};
                    bit_cnt  <= {CNT_WIDTH{1'b0}};
                    ser_out  <= data_in[TOTAL_BITS-1];
                    state    <= S_SHIFT;
                end

                S_SHIFT: begin
                    rclk <= 1'b0;
                    if (div_cnt == SCLK_DIV - 1) begin
                        div_cnt <= {SCLK_DIV_WIDTH{1'b0}};
                        if (sclk) begin
                            sclk <= 1'b0;
                            ser_out <= shift_reg[TOTAL_BITS-1];
                            shift_reg<= {shift_reg[TOTAL_BITS-2:0], 1'b0};
                        end else begin
                            sclk <= 1'b1;
                            if (bit_cnt == TOTAL_BITS - 1) begin
                                state <= S_LAST_FALL;
                            end else begin
                                bit_cnt <= bit_cnt + 1'b1;
                            end
                        end
                    end else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end

                S_LAST_FALL: begin
                    rclk <= 1'b0;
                    if (div_cnt == SCLK_DIV - 1) begin
                        div_cnt <= {SCLK_DIV_WIDTH{1'b0}};
                        sclk    <= 1'b0;
                        state   <= S_LATCH;
                    end else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end

                S_LATCH: begin
                    sclk <= 1'b0;
                    rclk <= 1'b1;
                    if (div_cnt == 2*SCLK_DIV - 1) begin
                        div_cnt <= {SCLK_DIV_WIDTH{1'b0}};
                        state   <= S_LOAD;
                    end else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end

                default: state <= S_LOAD;
            endcase
        end
    end

endmodule
