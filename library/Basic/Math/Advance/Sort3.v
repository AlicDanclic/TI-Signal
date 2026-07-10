`timescale 1ns/1ns

/*
*   Date : 2024-06-27
*   Author : nitcloud
*   Module Name:   Sort3.v - Sort3
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description : 3输入无符号数据并行排序模块。
*   Dependencies: none
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

/* @wavedrom
{signal: [
  {name: 'clock', wave: '10101010101010101'},
  {name: 'reset', wave: '10...............'},
  {name: 'data1', wave: 'x3.3.3.3.x.', data: ['10','20','35','50']},
  {name: 'data2', wave: 'x3.3.3.3.x.', data: ['5','25','30','50']},
  {name: 'data3', wave: 'x3.3.3.3.x.', data: ['15','30','25','50']},
  {name: 'max', wave: 'x4.4.4.4.x.', data: ['15','30','35','50']},
  {name: 'mid', wave: 'x4.4.4.4.x.', data: ['10','25','30','50']},
  {name: 'min', wave: 'x4.4.4.4.x.', data: ['5','20','25','50']},
]}
*/
module	Sort3 #(
        // 数据位宽。
        parameter WIDTH = 16
    ) (
        // 时钟输入
        input				clock,
        // 异步复位，高电平有效
        input				reset,
        
        // 无符号数据1输入端口
        input		[WIDTH-1:0]	data1, 
        // 无符号数据2输入端口
        input		[WIDTH-1:0]	data2, 
        // 无符号数据3输入端口
        input		[WIDTH-1:0]	data3,
        
        // 无符号最大值输出端口
        output	reg	[WIDTH-1:0]	max_data, 
        // 无符号中间值输出端口
        output	reg	[WIDTH-1:0]	mid_data, 
        // 无符号最小值输出端口
        output	reg	[WIDTH-1:0]	min_data
    );

    //-----------------------------------
    //3个数据的排序	
    always@(posedge clock or negedge reset) begin
        if(reset) begin
            max_data <= 0;
            mid_data <= 0;
            min_data <= 0;
        end
        else begin
            //获取最大值
            if(data1 >= data2 && data1 >= data3) begin
                max_data <= data1;
            end
            else if(data2 >= data1 && data2 >= data3) begin
                max_data <= data2;
            end
            else begin
                max_data <= data3;
            end

            //获取中间值
            if((data1 >= data2 && data1 <= data3) || (data1 >= data3 && data1 <= data2)) begin
                mid_data <= data1;
            end
            else if((data2 >= data1 && data2 <= data3) || (data2 >= data3 && data2 <= data1)) begin
                mid_data <= data2;
            end
            else begin
                mid_data <= data3;
            end
                
            //获取最小值
            if(data1 <= data2 && data1 <= data3) begin
                min_data <= data1;
            end
            else if(data2 <= data1 && data2 <= data3) begin
                min_data <= data2;
            end
            else begin
                min_data <= data3;
            end
        end
    end

endmodule
