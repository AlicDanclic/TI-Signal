/**
 * @file uart_tx.v
 * @brief UART 发送器，支持可配置波特率，8 数据位，无校验，1 停止位 (8N1)。
 * @details 内部使用移位寄存器发送起始位 (0)、8 个数据位 (LSB 优先)、停止位 (1)。
 *          当 data_valid 脉冲且 ready 为高时，加载 data_in 并开始发送。发送期间 busy 为高，ready 为低。
 *          发送完成后自动回到空闲状态。
 * @param CLK_FREQ_HZ 系统时钟频率 (Hz)
 * @param BAUD_RATE   期望波特率 (bps)
 * @param clk         系统时钟
 * @param rst_n       异步低电平复位
 * @param data_in     待发送的字节数据
 * @param data_valid  发送请求脉冲 (高电平有效)
 * @param tx          UART 发送引脚
 * @param ready       发送器空闲标志 (高可接受新数据)
 * @note BAUD_DIV = CLK_FREQ_HZ / BAUD_RATE，需确保 BAUD_DIV 不小于 4。
 * @author AI Assistant
 * @date 2026-06-11
 * @version 1.0
 */
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

    localparam integer BAUD_DIV = CLK_FREQ_HZ / BAUD_RATE;  // 每 bit 的时钟周期数

    reg [9:0]  shift_reg;   // 移位寄存器: {停止位(1), 数据位(8), 起始位(0)}
    reg [15:0] baud_cnt;    // 波特率计数器
    reg [3:0]  bit_idx;     // 已发送的 bit 数 (0~9)
    reg        busy;        // 发送忙标志

    assign ready = ~busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg <= 10'h3ff;   // 全 1，保持空闲状态
            baud_cnt  <= 16'd0;
            bit_idx   <= 4'd0;
            busy      <= 1'b0;
            tx        <= 1'b1;      // 空闲时为高
        end else begin
            if (!busy) begin
                tx <= 1'b1;
                if (data_valid) begin
                    // 加载数据帧: {1'b1 (停止位), data_in, 1'b0 (起始位)}
                    shift_reg <= {1'b1, data_in, 1'b0};
                    baud_cnt  <= BAUD_DIV - 1;
                    bit_idx   <= 4'd0;
                    busy      <= 1'b1;
                    tx        <= 1'b0;   // 发送起始位
                end
            end else if (baud_cnt == 0) begin
                baud_cnt  <= BAUD_DIV - 1;
                bit_idx   <= bit_idx + 1'b1;
                shift_reg <= {1'b1, shift_reg[9:1]};  // 右移，高位补1

                if (bit_idx == 4'd9) begin
                    busy <= 1'b0;      // 发送完毕
                    tx   <= 1'b1;
                end else begin
                    tx <= shift_reg[1]; // 输出下一个 bit
                end
            end else begin
                baud_cnt <= baud_cnt - 1'b1;
            end
        end
    end

endmodule