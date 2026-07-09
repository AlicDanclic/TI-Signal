/**
 * @file uart_rx.v
 * @brief UART 接收器，支持可配置波特率，8 数据位，无校验，1 停止位 (8N1)。
 * @details 内部使用过采样方式，以 16 倍波特率时钟采样 (但本实现使用整数分频，通过 HALF_BAUD_DIV 采样起始位)。
 *          状态机：IDLE → START → DATA (8 bits) → STOP → 回到 IDLE。
 *          在 START 位检测下降沿，并在半周期处再次确认，随后每个位周期采样一次。
 *          数据以 LSB 优先方式存入 shift_reg，接收完成后输出 data_out 和 data_valid 脉冲。
 * @param CLK_FREQ_HZ 系统时钟频率 (Hz)
 * @param BAUD_RATE   期望波特率 (bps)
 * @param clk         系统时钟
 * @param rst_n       异步低电平复位
 * @param rx          UART 接收引脚
 * @param data_out    接收到的字节数据
 * @param data_valid  数据有效脉冲 (宽度 1 时钟)
 * @note 分频系数 BAUD_DIV = CLK_FREQ_HZ / BAUD_RATE，要求 BAUD_DIV >= 8，且不能太大导致计数器溢出。
 *       本模块未对噪声或帧错误做额外处理，仅检查停止位是否为高。
 * @author AI Assistant
 * @date 2026-06-11
 * @version 1.0
 */
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

    localparam integer BAUD_DIV      = CLK_FREQ_HZ / BAUD_RATE;   // 每 bit 的时钟周期数
    localparam integer HALF_BAUD_DIV = BAUD_DIV / 2;               // 半 bit 计数

    localparam [1:0] S_IDLE  = 2'd0;
    localparam [1:0] S_START = 2'd1;
    localparam [1:0] S_DATA  = 2'd2;
    localparam [1:0] S_STOP  = 2'd3;

    reg        rx_ff0;         // 同步寄存器 0
    reg        rx_ff1;         // 同步寄存器 1 (同步后的 rx)
    reg [1:0]  state;
    reg [15:0] baud_cnt;       // 波特率计数器
    reg [2:0]  bit_idx;        // 当前接收的 bit 索引 (0~7)
    reg [7:0]  shift_reg;      // 串行移位寄存器

    wire rx_sync = rx_ff1;     // 同步后的接收信号

    // 两级同步消除亚稳态
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_ff0 <= 1'b1;
            rx_ff1 <= 1'b1;
        end else begin
            rx_ff0 <= rx;
            rx_ff1 <= rx_ff0;
        end
    end

    // 接收状态机
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
                    // 检测起始位 (下降沿)
                    if (!rx_sync) begin
                        state    <= S_START;
                        baud_cnt <= HALF_BAUD_DIV - 1;  // 半周期后采样
                    end
                end

                S_START: begin
                    if (baud_cnt == 0) begin
                        // 在半周期处再次确认起始位仍为低
                        if (!rx_sync) begin
                            state    <= S_DATA;
                            baud_cnt <= BAUD_DIV - 1;
                            bit_idx  <= 3'd0;
                        end else begin
                            state <= S_IDLE;   // 毛刺，放弃
                        end
                    end else begin
                        baud_cnt <= baud_cnt - 1'b1;
                    end
                end

                S_DATA: begin
                    if (baud_cnt == 0) begin
                        shift_reg[bit_idx] <= rx_sync;  // 采样数据位
                        baud_cnt           <= BAUD_DIV - 1;

                        if (bit_idx == 3'd7) begin
                            state <= S_STOP;            // 所有数据位接收完毕
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
                        // 检查停止位是否为高 (可选，若不匹配也可选择不输出)
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