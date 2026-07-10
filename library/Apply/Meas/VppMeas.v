// 峰峰值（Vpp）测量模块
// 测量输入信号的最大峰峰值幅度

`timescale 1 ns / 1 ns

/*
*   Date : 2025-06-24
*   Author : nitcloud
*   Module Name:   VppMeas.v - VppMeas
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description : Vpp Measure module.
*   Dependencies: MaxMin
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

module VppMeas #(
        parameter   DWIDTH  = 4'd12,
        parameter   RWIDTH  = 4'd10
    ) (
        input                      clock,
        input                      reset,

        input         [RWIDTH-1:0] range,

        input                      ivalid,
        input  signed [DWIDTH-1:0] idata,

        output                     ovalid,
        output signed [DWIDTH-1:0] odata
    );

    wire maxDone;
    wire [DWIDTH-1:0] maxValue;
    wire minDone;
    wire [DWIDTH-1:0] minValue;

    MaxMin #(
        .MODESET 	("MAX"  ),
        .DWIDTH  	(DWIDTH ),
        .RWIDTH  	(RWIDTH ))
    u_Max(
        .clock 	(clock    ),
        .reset 	(reset    ),
        .range 	(range    ),
        .ivalid (ivalid   ),
        .idata 	(idata    ),
        .ovalid (maxDone  ),
        .odata 	(maxValue )
    );

    MaxMin #(
        .MODESET 	("MIN"  ),
        .DWIDTH  	(DWIDTH ),
        .RWIDTH  	(RWIDTH ))
    u_Min(
        .clock 	(clock    ),
        .reset 	(reset    ),
        .range 	(range    ),
        .ivalid (ivalid   ),
        .idata 	(idata    ),
        .ovalid (minDone  ),
        .odata 	(minValue )
    );

    reg valid;

    always @(posedge clock or posedge reset)begin
        if(reset)begin
            valid <= 1'b0;
        end
        else begin
            valid <= maxDone && minDone;
        end
    end

    assign ovalid = valid;

    reg [DWIDTH:0] Vpp_Value;

    always @(posedge clock or posedge reset) begin
        if(reset)begin
            Vpp_Value <= 1'b0;
        end
        else if(maxDone && minDone)begin
            Vpp_Value <= $signed(maxValue) - $signed(minValue);
        end
    end

    assign odata = Vpp_Value >>> 1;

endmodule
