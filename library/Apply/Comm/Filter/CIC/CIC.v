`timescale 1 ns / 1 ns

/*
*   Date : 2025-06-23
*   Author : nitcloud
*   Module Name:   CIC.v - CIC
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description :  CIC Digital Down Sample Filter.
*   Dependencies: CIC_DOWN_S2, CIC_DOWN_S3
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

module CIC #(
        parameter CSTAGE = 3,
        parameter FACTOR = 12,
        parameter IWIDTH = 12,
        parameter OSCALE = 1,
        parameter OWIDTH = 15
    ) (
        input   					 clock,
        input   					 reset,
 
        input   					 ivalid,
        input   signed [IWIDTH-1:0]  filter_in,
 
        output  				     ovalid,
        output  signed [OWIDTH-1:0]  filter_out
    );

    generate 
        if(CSTAGE == 2) begin : CIC_DOWN_S2        
            CIC_DOWN_S2 #(
                .FACTOR 		( FACTOR 		),
                .IWIDTH 		( IWIDTH 		),
                .OWIDTH 		( OWIDTH 		),
                .OSCALE         ( OSCALE        ))
            u_CIC_DOWN_S2(
                // 端口
                .clock      	( clock      		),
                .reset      	( reset      		),

                .ivalid     	( ivalid     		),
                .filter_in  	( filter_in  		),

                .ovalid     	( ovalid     		),
                .filter_out 	( filter_out 		)
            );
        end
        else if(CSTAGE == 3) begin : CIC_DOWN_S2
            CIC_DOWN_S3 #(
                .FACTOR 		( FACTOR 		),
                .IWIDTH 		( IWIDTH 		),
                .OWIDTH 		( OWIDTH 		),
                .OSCALE         ( OSCALE        ))
            u_CIC_DOWN_S3(
                // 端口
                .clock      	( clock      		),
                .reset      	( reset      		),

                .ivalid     	( ivalid     		),
                .filter_in  	( filter_in  		),

                .ovalid     	( ovalid     		),
                .filter_out 	( filter_out 		)
            );
        end
    endgenerate

endmodule