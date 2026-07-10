`timescale 1 ns / 1 ns

/*
*   Date : 2025-06-23
*   Author : nitcloud
*   Module Name:   DDC.v - DDC
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description : 数字下变频模块。
*   Dependencies: accuml, cordic, CIC
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

module DDC #(
        //LO_OUTPUR_parameter
        parameter LWIDTH = 12,
        parameter PWIDTH = 32,
        //CIC_Filter_parameter
        parameter OSCALE = 0,
        parameter FACTOR = 8,
        //IQ_MIXED_parameter
        parameter IWIDTH = 12,
        parameter OWIDTH = 12
    ) (
        input               clock,
        input               reset,
        
        input  [PWIDTH-1:0] fre_word,
        input  [IWIDTH-1:0] wave_in,

        output              clk_div,  
        output [OWIDTH-1:0] i_wave,
        output [OWIDTH-1:0] q_wave
    );

    wire [PWIDTH-1:0] Q;
    accuml #(
        .WIDTH 		( PWIDTH    ))
    u_accuml(
        // 端口
        .clock      ( clock     ),
        .reset      ( reset     ),
        .clr        ( reset     ),
        .add_sub    ( 1'b0      ),
        .D          ( fre_word  ),
        .Q          ( Q         )
    );

    wire                     iq_ovalid;
    wire signed [LWIDTH-1:0] cos_wave;
    wire signed [LWIDTH-1:0] sin_wave;
    wire signed [PWIDTH-1:0] pha_diff;

    cordic #(
        .XY_BITS      		( LWIDTH        ),
        .PH_BITS      		( PWIDTH        ),
        .ITERATIONS   		( 16            ),
        .STYLE       		( "ROTATE"      ),
        .CALMODE    		( "FAST"        ))
    u_cordic_iq(
        // 端口
        .clock     		( clock          ),
        .reset     		( reset          ),
        .ivalid    		( ~reset         ),
        .x_i       		( 2**(LWIDTH-1)  ),
        .y_i       		( 0              ),
        .z_i    		( Q              ),
        .ovalid    		( iq_ovalid      ),
        .x_o       		( sin_wave       ),
        .y_o       		( cos_wave       ),
        .z_o     		( pha_diff       )
    );

    reg  signed [IWIDTH+LWIDTH-1:0] i_sig;
    reg  signed [IWIDTH+LWIDTH-1:0] q_sig;
    always @(posedge clock) begin
        if (reset) begin
            i_sig <= 0;
            q_sig <= 0;
        end
        else begin
            i_sig <= $signed(wave_in) * $signed(cos_wave);
            q_sig <= $signed(wave_in) * $signed(sin_wave);
        end
    end

    wire [11:0] i;
    wire [11:0] q;

    assign i = i_sig[IWIDTH+LWIDTH-1:IWIDTH+LWIDTH-12];
    assign q = q_sig[IWIDTH+LWIDTH-1:IWIDTH+LWIDTH-12];

    wire 	ivalid;
    wire 	qvalid;
    CIC #(
        .CSTAGE 		( 3  		),
        .FACTOR 		( FACTOR 	),
        .IWIDTH 		( 12 		),
        .OSCALE 		( OSCALE  	),
        .OWIDTH 		( OWIDTH 	))
    u_CIC_i(
        // 端口
        .clock      		( clock      ),
        .reset      		( reset      ),
        .ivalid     		( 1'b1       ),
        .filter_in  		( i  		 ),
        .ovalid     		( ivalid     ),
        .filter_out 		( i_wave 	 )
    );

    CIC #(
        .CSTAGE 		( 3  		),
        .FACTOR 		( FACTOR 	),
        .IWIDTH 		( 12 		),
        .OSCALE 		( OSCALE  	),
        .OWIDTH 		( OWIDTH 	))
    u_CIC_q(
        // 端口
        .clock      		( clock      ),
        .reset      		( reset      ),
        .ivalid     		( 1'b1       ),
        .filter_in  		( q  		 ),
        .ovalid     		( qvalid     ),
        .filter_out 		( q_wave 	 )
    );

    assign clk_div = ivalid & qvalid;

endmodule