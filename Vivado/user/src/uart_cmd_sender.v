/**
 * @file uart_cmd_sender.v
 * @brief 格式化发送 @TT|AA|BB|CC|# 的控制器
 * @details 将输入的 4 个 8 位数值转换为 ASCII 字符，依次通过 uart_tx 发送。
 *          提供 start 脉冲触发发送，发送完成后 done 信号拉高一个周期。
 */
`timescale 1ns / 1ps

module uart_cmd_sender #(
    parameter CLK_FREQ_HZ = 50_000_000,
    parameter BAUD_RATE   = 115200
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  tt,
    input  wire [7:0]  aa,
    input  wire [7:0]  bb,
    input  wire [7:0]  cc,
    input  wire        start,          // 脉冲触发发送
    output wire        uart_tx,        // 直接连接上位机
    output wire        done            // 发送完成脉冲
);

    // ---------- 发送器实例 ----------
    wire       tx_ready;
    reg        tx_data_valid;
    reg  [7:0] tx_data;

    uart_tx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) u_tx (
        .clk       (clk),
        .rst_n     (rst_n),
        .data_in   (tx_data),
        .data_valid(tx_data_valid),
        .tx        (uart_tx),
        .ready     (tx_ready)
    );

    // ---------- 发送状态机 ----------
    localparam NUM_BYTES = 14;  // @ TT | AA | BB | CC | # (总共14字节)
    // ASCII 字符顺序：
    // '@', TT_hi, TT_lo, '|', AA_hi, AA_lo, '|', BB_hi, BB_lo, '|', CC_hi, CC_lo, '|', '#'

    reg [3:0]  byte_idx;
    reg [7:0]  ascii_buf [0:NUM_BYTES-1];
    reg        sending;
    reg        done_reg;
    reg        last_byte_loaded;   // 标志最后一个字节已加载

    // 将 nibble 转 ASCII 的函数
    function [7:0] nibble_to_ascii;
        input [3:0] nibble;
        begin
            if (nibble < 10) nibble_to_ascii = 8'h30 + nibble;
            else nibble_to_ascii = 8'h41 + (nibble - 10);
        end
    endfunction

    // 组装 ASCII 序列（组合逻辑）
    always @(*) begin
        ascii_buf[0]  = 8'h40;                         // '@'
        ascii_buf[1]  = nibble_to_ascii(tt[7:4]);
        ascii_buf[2]  = nibble_to_ascii(tt[3:0]);
        ascii_buf[3]  = 8'h7C;                         // '|'
        ascii_buf[4]  = nibble_to_ascii(aa[7:4]);
        ascii_buf[5]  = nibble_to_ascii(aa[3:0]);
        ascii_buf[6]  = 8'h7C;
        ascii_buf[7]  = nibble_to_ascii(bb[7:4]);
        ascii_buf[8]  = nibble_to_ascii(bb[3:0]);
        ascii_buf[9]  = 8'h7C;
        ascii_buf[10] = nibble_to_ascii(cc[7:4]);
        ascii_buf[11] = nibble_to_ascii(cc[3:0]);
        ascii_buf[12] = 8'h7C;                         // 额外的 '|'
        ascii_buf[13] = 8'h23;                         // '#'
    end

    // 状态机（时序逻辑）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sending <= 0;
            byte_idx <= 0;
            tx_data_valid <= 0;
            done_reg <= 0;
            last_byte_loaded <= 0;
        end else begin
            tx_data_valid <= 0;
            done_reg <= 0;

            if (!sending && start) begin
                sending <= 1;
                byte_idx <= 0;
                last_byte_loaded <= 0;
            end

            if (sending) begin
                // 如果没有加载最后一个字节，且发送器空闲，则加载下一个字节
                if (!last_byte_loaded && tx_ready) begin
                    tx_data <= ascii_buf[byte_idx];
                    tx_data_valid <= 1;
                    if (byte_idx == NUM_BYTES - 1) begin
                        last_byte_loaded <= 1;   // 标记已加载最后一个字节
                    end else begin
                        byte_idx <= byte_idx + 1;
                    end
                end

                // 如果已经加载了最后一个字节，且发送器空闲，则发送完成
                if (last_byte_loaded && tx_ready) begin
                    sending <= 0;
                    done_reg <= 1;
                    last_byte_loaded <= 0;
                end
            end
        end
    end

    assign done = done_reg;

endmodule