`timescale 1 ns / 1 ns

/*
*   Date : 2025-06-23
*   Author : nitcloud
*   Module Name:   AnalogDemod.v - AnalogDemod
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description : Analog Demodulation module.
*   Dependencies: DDC, cordic
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

module AnalogDemod #(
        parameter PWIDTH = 32,
        parameter OSCALE = 0,
        parameter FACTOR = 4,
        parameter IWIDTH = 8,
        parameter OWIDTH = 12
    ) (
        input               clock,
        input               reset,

        input  [PWIDTH-1:0] fre_word,

        input               ivalid,
        input  [IWIDTH-1:0] wave_in,

        output              clk_div,
        output              ovalid,
        output [OWIDTH-1:0] FM_Demodule,
        output [OWIDTH-1:0] PM_Demodule,
        output [OWIDTH-1:0] AM_Demodule
    );

    // IQ_MIXED Outputs
    wire  [OWIDTH - 1 : 0]  i_wave;
    wire  [OWIDTH - 1 : 0]  q_wave;

    DDC #(
        .LWIDTH     ( 12      ),
        .PWIDTH     ( PWIDTH  ),
        .OSCALE     ( OSCALE  ),
        .FACTOR     ( FACTOR  ),
        .IWIDTH     ( IWIDTH  ),
        .OWIDTH     ( OWIDTH  )) 
    u_DDC (
        .clock                  ( clock      ),
        .reset                  ( reset      ),

        .fre_word               ( fre_word   ),
        .wave_in                ( wave_in    ),

        .clk_div                ( clk_div    ),
        .i_wave                 ( i_wave     ),
        .q_wave                 ( q_wave     )
    );

    wire                     iqvalid;
    wire signed [OWIDTH-1:0] x_vec;
    wire signed [OWIDTH-1:0] y_vec;
    wire signed [OWIDTH-1:0] z_vec;
    cordic #(
        .XY_BITS      		( OWIDTH        ),
        .PH_BITS      		( OWIDTH        ),
        .ITERATIONS   		( 16            ),
        .STYLE       		( "VECTOR"      ),
        .CALMODE    		( "FAST"        ))
    u_cordic_vec(
        // 端口
        .clock     		( clk_div   ),
        .reset     		( reset     ),

        .ivalid    		( ivalid    ),
        .x_i       		( q_wave    ),
        .y_i       		( i_wave    ),
        .z_i    		( 0         ),
        
        .ovalid    		( iqvalid   ),
        .x_o       		( x_vec     ),
        .y_o       		( y_vec     ),
        .z_o     		( z_vec 	)
    );

    reg signed [OWIDTH-1:0] pha_buf;
    always @(posedge clk_div) begin
        if(iqvalid) begin
            pha_buf <= z_vec;
        end
        else begin
            pha_buf <= 0;
        end
    end

    assign ovalid = iqvalid;
    assign FM_Demodule = z_vec - pha_buf;
    assign AM_Demodule = x_vec;
    assign PM_Demodule = z_vec;

endmodule
