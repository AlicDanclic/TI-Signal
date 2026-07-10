`timescale 1ns/100ps

module lvds_clk_out #(
        parameter   BUFTYPE = "SERIES7"
    ) (
        input      clk,
        output     clk_out_p,
        output     clk_out_n
    );

    // wires
    wire      clk_ibuf_s;

    generate
        if (BUFTYPE == "VIRTEX6") begin : VIRTEX6_BUF
            BUFR #(
                .BUFR_DIVIDE( "BYPASS" )) 
            i_clk_rbuf (
                .CLR    ( 1'b0       ),
                .CE     ( 1'b1       ),
                .I      ( clk        ),
                .O      ( clk_ibuf_s )
            );
        end
        else if (BUFTYPE == "SERIES7") begin : SERIES7_BUF
            BUFG i_clk_gbuf (
                .I ( clk        ),
                .O ( clk_ibuf_s )
            );
        end
    endgenerate

    
    // instantiations
    OBUFDS #(
        .IOSTANDARD("DEFAULT"), // Specify the output I/O standard
        .SLEW("FAST")           // Specify the output slew rate
    ) OBUFDS_inst (
        .O(clk_out_p),          // Diff_p output (connect directly to top-level port)
        .OB(clk_out_n),         // Diff_n output (connect directly to top-level port)
        .I(clk_ibuf_s)          // Buffer input
    );

endmodule
