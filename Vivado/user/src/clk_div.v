/**
 * @file clk_div.v
 * @brief 基础时钟分频电路，输入/输出频率均可参数化。
 * @details 根据 INPUT_FREQ 与 OUTPUT_FREQ 自动计算分频系数，输出一路时钟。
 *          占空比约 50%（偶数分频精确，奇数分频近似）。
 *          当 OUTPUT_FREQ = INPUT_FREQ 时，输出直通输入时钟（无分频）。
 * @param INPUT_FREQ  输入时钟频率 (Hz)，默认 50MHz
 * @param OUTPUT_FREQ 输出时钟频率 (Hz)，默认 25MHz
 * @param clk_in      输入时钟
 * @param rst_n       异步低电平复位
 * @param clk_out     分频后的时钟输出
 * @note  分频系数 DIV = (INPUT_FREQ + OUTPUT_FREQ/2) / OUTPUT_FREQ，四舍五入取整。
 *        DIV 至少为 1，且 OUTPUT_FREQ 不应为 0。
 */
`timescale 1ns / 1ps

module clk_div #(
    parameter INPUT_FREQ  = 50_000_000,   // 输入频率 (Hz)
    parameter OUTPUT_FREQ = 25_000_000    // 输出频率 (Hz)
) (
    input  wire clk_in,
    input  wire rst_n,
    output wire clk_out
);

    // 计算分频系数（四舍五入，最小为1）
    localparam DIV = (INPUT_FREQ + OUTPUT_FREQ/2) / OUTPUT_FREQ;
    localparam DIV_CLAMPED = (DIV < 1) ? 1 : DIV;   // 安全下限

    // 分频系数为 1 时：直通输出
    generate
        if (DIV_CLAMPED == 1) begin : gen_bypass
            assign clk_out = clk_in;
        end else begin : gen_div
            // 计数器位宽自适应
            localparam CNT_W = $clog2(DIV_CLAMPED);
            reg [CNT_W-1:0] cnt;
            reg clk_reg;

            always @(posedge clk_in or negedge rst_n) begin
                if (!rst_n) begin
                    cnt     <= 0;
                    clk_reg <= 0;
                end else begin
                    if (cnt == DIV_CLAMPED - 1)
                        cnt <= 0;
                    else
                        cnt <= cnt + 1;

                    // 偶数分频精确 50% 占空比，奇数分频取近似
                    clk_reg <= (cnt < DIV_CLAMPED/2);
                end
            end
            assign clk_out = clk_reg;
        end
    endgenerate

endmodule