/**
 * @file uart_command_parser.v
 * @brief 从 UART 接收的 ASCII 命令中提取 TT, AA, BB, CC 数值
 * @details 格式：@TT|AA|BB|CC#，每个字段为两位十六进制字符
 *          输出解析后的 8 位二进制值，并伴随有效脉冲。
 *          本模块不区分命令类型，只做格式检查与提取。
 */
`timescale 1ns / 1ps

module uart_command_parser #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD_RATE   = 115200
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        uart_rx_i,

    output reg  [7:0]  TT,
    output reg  [7:0]  AA,
    output reg  [7:0]  BB,
    output reg  [7:0]  CC,
    output reg         parser_valid
);

    // 命令字符定义
    localparam [7:0] CHAR_AT   = 8'h40;   // '@'
    localparam [7:0] CHAR_HASH = 8'h23;   // '#'
    localparam [7:0] CHAR_PIPE = 8'h7C;   // '|'

    // RX 状态机状态
    localparam [3:0] RX_IDLE  = 0, RX_TT_HI = 1, RX_TT_LO = 2, RX_SEP1 = 3,
                     RX_AA_HI = 4, RX_AA_LO = 5, RX_SEP2 = 6, RX_BB_HI = 7,
                     RX_BB_LO = 8, RX_SEP3 = 9, RX_CC_HI =10, RX_CC_LO =11,
                     RX_END   =12;

    // UART 接收子模块信号
    wire [7:0] rx_data;
    wire       rx_data_valid;

    // 解析寄存器（存储 ASCII 字符）
    reg [7:0] r_tt_hi, r_tt_lo, r_aa_hi, r_aa_lo;
    reg [7:0] r_bb_hi, r_bb_lo, r_cc_hi, r_cc_lo;
    reg [3:0] rx_state;

    // ---------- 辅助函数 ----------
    // 将 ASCII 十六进制字符转换为 4 位数值
    function [3:0] hex_val;
        input [7:0] ch;
        begin
            if (ch >= 8'h30 && ch <= 8'h39)       hex_val = ch[3:0];
            else if (ch >= 8'h41 && ch <= 8'h46)  hex_val = ch[3:0] + 4'd9;
            else if (ch >= 8'h61 && ch <= 8'h66)  hex_val = ch[3:0] + 4'd9;
            else                                  hex_val = 4'd0;
        end
    endfunction

    // 判断字符是否为十六进制数字
    function is_hex;
        input [7:0] ch;
        begin
            is_hex = ((ch >= 8'h30 && ch <= 8'h39) ||
                       (ch >= 8'h41 && ch <= 8'h46) ||
                       (ch >= 8'h61 && ch <= 8'h66));
        end
    endfunction

    // ---------- UART 接收实例 ----------
    // 假设存在标准的 uart_rx 模块，接口如下：
    // input  clk, rst_n, rx
    // output data_out[7:0], data_valid
    uart_rx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) u_uart_rx (
        .clk       (clk),
        .rst_n     (rst_n),
        .rx        (uart_rx_i),
        .data_out  (rx_data),
        .data_valid(rx_data_valid)
    );

    // ---------- 解析状态机 ----------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state <= RX_IDLE;
            {r_tt_hi, r_tt_lo, r_aa_hi, r_aa_lo, r_bb_hi, r_bb_lo, r_cc_hi, r_cc_lo} <= 0;
            TT <= 0;
            AA <= 0;
            BB <= 0;
            CC <= 0;
            parser_valid <= 1'b0;
        end else begin
            parser_valid <= 1'b0;   // 默认无脉冲

            if (rx_data_valid) begin
                case (rx_state)
                    RX_IDLE: begin
                        if (rx_data == CHAR_AT)
                            rx_state <= RX_TT_HI;
                        else
                            rx_state <= RX_IDLE;
                    end

                    RX_TT_HI: begin
                        if (is_hex(rx_data)) begin
                            r_tt_hi <= rx_data;
                            rx_state <= RX_TT_LO;
                        end else
                            rx_state <= RX_IDLE;
                    end

                    RX_TT_LO: begin
                        if (is_hex(rx_data)) begin
                            r_tt_lo <= rx_data;
                            rx_state <= RX_SEP1;
                        end else
                            rx_state <= RX_IDLE;
                    end

                    RX_SEP1: begin
                        rx_state <= (rx_data == CHAR_PIPE) ? RX_AA_HI : RX_IDLE;
                    end

                    RX_AA_HI: begin
                        if (is_hex(rx_data)) begin
                            r_aa_hi <= rx_data;
                            rx_state <= RX_AA_LO;
                        end else
                            rx_state <= RX_IDLE;
                    end

                    RX_AA_LO: begin
                        if (is_hex(rx_data)) begin
                            r_aa_lo <= rx_data;
                            rx_state <= RX_SEP2;
                        end else
                            rx_state <= RX_IDLE;
                    end

                    RX_SEP2: begin
                        rx_state <= (rx_data == CHAR_PIPE) ? RX_BB_HI : RX_IDLE;
                    end

                    RX_BB_HI: begin
                        if (is_hex(rx_data)) begin
                            r_bb_hi <= rx_data;
                            rx_state <= RX_BB_LO;
                        end else
                            rx_state <= RX_IDLE;
                    end

                    RX_BB_LO: begin
                        if (is_hex(rx_data)) begin
                            r_bb_lo <= rx_data;
                            rx_state <= RX_SEP3;
                        end else
                            rx_state <= RX_IDLE;
                    end

                    RX_SEP3: begin
                        rx_state <= (rx_data == CHAR_PIPE) ? RX_CC_HI : RX_IDLE;
                    end

                    RX_CC_HI: begin
                        if (is_hex(rx_data)) begin
                            r_cc_hi <= rx_data;
                            rx_state <= RX_CC_LO;
                        end else
                            rx_state <= RX_IDLE;
                    end

                    RX_CC_LO: begin
                        if (is_hex(rx_data)) begin
                            r_cc_lo <= rx_data;
                            rx_state <= RX_END;
                        end else
                            rx_state <= RX_IDLE;
                    end

                    RX_END: begin
                        // 原模块允许额外的 '|' 并停留在 RX_END，这里保留兼容
                        if (rx_data == CHAR_PIPE) begin
                            rx_state <= RX_END;
                        end else begin
                            if (rx_data == CHAR_HASH) begin
                                // 解析完成，将 ASCII 转为数值并输出
                                TT <= {hex_val(r_tt_hi), hex_val(r_tt_lo)};
                                AA <= {hex_val(r_aa_hi), hex_val(r_aa_lo)};
                                BB <= {hex_val(r_bb_hi), hex_val(r_bb_lo)};
                                CC <= {hex_val(r_cc_hi), hex_val(r_cc_lo)};
                                parser_valid <= 1'b1;
                            end
                            rx_state <= RX_IDLE;
                        end
                    end

                    default: rx_state <= RX_IDLE;
                endcase
            end
        end
    end

endmodule