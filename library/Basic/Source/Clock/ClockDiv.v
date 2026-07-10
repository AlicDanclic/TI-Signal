`timescale 1ns / 1ns

/*
*   Date : 2024-06-27
*   Author : nitcloud
*   Module Name:   ClockAny.v - ClockAny
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description : 整数分频器。
*   Dependencies: none
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

module ClockDiv #(
        // 分频系数的位宽。
        parameter WIDTH = 8
    ) (
        // 时钟输入
        input                 clock,
        // 异步复位，高电平有效
        input                 reset,
        
        // 分频系数。
        input  [WIDTH - 1:0]  N,
        // 输出时钟信号
        output                div_clk
    );

    reg [WIDTH-1:0] cnt_p;      // 上升沿计数单元
    reg [WIDTH-1:0] cnt_n;      // 下降沿计数单元
    reg             clk_in_p;   // 上升沿时钟
    reg             clk_in_n;   // 下降沿时钟

    // 当N==1时，表示不分频。
    // N[0]用于判断分频数是奇数还是偶数；
    // 为1表示奇数分频，为0表示偶数分频。
    assign div_clk = (N == 1) ? clock : (N[0])   ? (clk_in_p | clk_in_n) : (clk_in_p);
            
    always@(posedge clock or posedge reset) begin
        if (reset) begin
            cnt_p <= 0;
        end
        else if (cnt_p == (N-1)) begin
            cnt_p <= 0;
        end
        else begin
            cnt_p <= cnt_p + 1;
        end
    end

    always@(posedge clock or posedge reset) begin
        if (reset) begin
            clk_in_p <= 1;
        end
        // N右移一位，高位补零，相当于N/2，
        // 但在计算奇数时显示出显著优势。
        else if (cnt_p < (N>>1)) begin
            clk_in_p <= 1;
        end
        else begin
            clk_in_p <= 0;    
        end
    end

    always@(posedge clock or posedge reset) begin
        if (reset) begin
            cnt_n <= 0;
        end
        else if (cnt_n == (N-1)) begin
            cnt_n <= 0;
        end
        else begin
            cnt_n <= cnt_n + 1;
        end
    end

    always@(posedge clock or posedge reset) begin
        if (reset) begin
            clk_in_n <= 1;
        end
        else if (cnt_n < (N>>1)) begin
            clk_in_n <= 1;
        end
        else begin
            clk_in_n <= 0;
        end
    end

endmodule

