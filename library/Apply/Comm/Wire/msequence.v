`timescale 1ns / 1ps

/*
*   Date : 2025-06-23
*   Author : nitcloud
*   Module Name:   msequence.v - msequence
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description : m-sequence generator module.
*   Dependencies: 
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

module msequence #(  
        // Width of the m-sequence
        parameter WIDTH = 4'd8,
        // Initial value for the m-sequence
        parameter MINIT = 4'd1,
        // Polynomial for the m-sequence
        // Example: 8'b10001110 corresponds to x^8+x^4+x^3+x^2+1
        parameter MPOLY = 8'b10001110
    ) (   
        input  clock,
        input  reset,
        output omseq
    );

    reg [WIDTH-1:0] sreg;
    assign omseq = sreg[WIDTH-1];

    always@(posedge clock or posedge reset) begin
        if(reset) begin
            sreg <= MINIT;
        end
        else begin
            sreg[0] <= ^(sreg & MPOLY);
            sreg[WIDTH-1:1] <= sreg;
        end
    end

endmodule
