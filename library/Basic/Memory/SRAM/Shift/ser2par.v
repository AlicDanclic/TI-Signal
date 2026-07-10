`timescale 1 ns / 1 ns

/*
*   Date : 2024-06-27
*   Author : nitcloud
*   Module Name:   ser2par.v - ser2par
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description : 单比特串行数据转换为并行数据。
*   Dependencies: none
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

/* @wavedrom
{signal: [
  {name: 'clock', wave: 'n..................'},
  {name: 'case1:ivalid', wave: '01.................'},
  {name: 'case2:ivalid', wave: 'n..................'},
  {name: 'idata', wave: '01010101..01..01.0.'},
  {name: 'ovalid', wave: '0.......10......10.'},
  {name: 'odata', wave: 'x.......5.......5..', data: ['11010101','10111011']}, // direct = 0
  {name: 'odata', wave: 'x.......5.......5..', data: ['10101011','11011101']}, // direct = 1
]}
*/
module ser2par #(
        // 指定转换到并行数据的长度。
        parameter LENGTH = 8
    ) (
        // 时钟输入
        input clock,
        // 异步复位，高电平有效
        input reset,

        // 指定转换方向。
        // 0 : 第一个输入是并行数据的最低位(LSB)。
        // 1 : 第一个输入是并行数据的最高位(MSB)。
        input direct,

        // @Flow 输入有效
        input ivalid,
        // @Flow 单比特串行数据输入
        input idata,

        // @Flow 输出有效
        output reg              ovalid,
        // @Flow 并行数据输出
        output reg [LENGTH-1:0] odata
    );

    reg [LENGTH-1:0]            data_buf;
    reg [$clog2(LENGTH)-1:0]    cnt;

    always @(posedge clock or posedge reset) begin
        if (reset) begin
            cnt <= 0;
            data_buf <= 0;
            odata <= 0;
            ovalid <= 0;
        end 
        else if (ivalid) begin
            data_buf[LENGTH-1] <= idata;
            data_buf[LENGTH-2:0] <= data_buf[LENGTH-1:1];
            cnt <= cnt + 1;
            if (cnt == LENGTH-1) begin
                if (direct == 0) begin
                    odata <= {idata, data_buf[LENGTH-1:1]};
                end
                else begin
                    odata <= {data_buf[LENGTH-1:1], idata};
                end
                ovalid <= 1;
            end 
            else begin
                ovalid <= 0;
            end
        end 
        else begin
            ovalid <= 0;
        end
    end
endmodule
