`timescale 1 ns / 1 ns

/*
*   Date : 2024-07-02
*   Author : nitcloud
*   Module Name:   quant.v - quant
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description : 用于向量缩放的8位量化方案。
*   Dependencies: none
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

/* @wavedrom

*/
module quant (
        // 时钟输入
        input clock,
        // 异步复位，高电平有效
        input reset,

        // @Flow 有符号数据输入端口
        input signed [17:0] idata,
        // @Flow 有符号缩放系数输入端口
        input signed [15:0] scale,
        // @Flow 移位输入端口
        input  [3:0] shift,
        // @Flow 零点输入端口
        input  [7:0] zero_point,
        // @Flow 数据输出端口
        output [7:0] odata
    );

    reg [15:0] data16;
	always@(posedge clock or posedge reset) begin
        if (reset) begin
            data16 <= 0;
        end
        else begin            
            if((idata[17] == 1'b0) && ((idata[16:15] & 2'b11) != 2'b00)) begin
                data16 <= 16'b0111111111111111;
            end 
            else begin
                if((idata[17] == 1'b1) && ((idata[16:15] | 2'b00) != 2'b11)) begin
                    data16 <= 16'b1000000000000000;
                end 
                else begin
                    data16 <= idata[15:0];
                end
            end
        end
	end
    
    reg signed [15:0] val_d;
	reg signed [31:0] dout_t;
    reg signed [31:0] dout_r;
	reg signed [15:0] scale_d;
	always@(posedge clock or posedge reset) begin
        if (reset) begin
            val_d <= 0;
            dout_t <= 0;
            dout_r <= 0;
            scale_d <= 0;
        end
        else begin            
            val_d <= data16;
            scale_d <= scale;
            dout_t <= val_d * scale_d;
            dout_r <= dout_t;
        end
	end
    
	reg trunc_reg;
	reg signed [15:0] dout_tmp;

	wire [15:0] din_up = dout_r[29:14];
    wire        trunc  = din_up[shift];
    wire signed [15:0] scale_out;
	
	always@(posedge clock or posedge reset) begin
        if (reset) begin
            dout_tmp <= 0;
            trunc_reg <= 0;
        end
        else begin            
            dout_tmp <= (dout_r >>> (15+shift));
            trunc_reg <= trunc;
        end
	end
	assign scale_out = (trunc_reg==1'b1) ? (dout_tmp+1) : (dout_tmp);

    reg signed [7:0] data8;
	always@(posedge clock or posedge reset) begin
        if (reset) begin
            data8 <= 0;
        end
        else begin            
            if((scale_out[15] == 1'b0) && ((scale_out[14:7] & 8'hFF) != 8'h00)) begin
                data8 <= 8'b01111111;
            end else begin
                if((scale_out[15]==1'b1) && ((scale_out[14:7] | 8'h00) != 8'hFF)) begin
                    data8 <= 8'b10000000;
                end else begin
                    data8 <= scale_out[7:0];
                end
            end
        end
	end

    assign odata = data8 + zero_point;

endmodule