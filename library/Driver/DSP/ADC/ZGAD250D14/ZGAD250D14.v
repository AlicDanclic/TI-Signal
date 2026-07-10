// ZGAD250D14数据接收模块
// 正点原子双路250MSPS高速ADC数据接收与解串

`timescale 1ns/100ps

module ZGAD250D14 (
        // user interface outputs
        input                   clock,
        input                   reset,
        output                  adc_clk,
        output  reg [15:0]      adc_data_CHA,
        output  reg [15:0]      adc_data_CHB,  
        output  reg             adc_or_n,
        output  reg             adc_or_p,

        // adc interface (clk, data+over-range)
        input                   adc_clk_in_p,
        input                   adc_clk_in_n,
        output                  adc_clk_out_p,
        output                  adc_clk_out_n,
        input       [6:0]       adc_dataA_in_p,
        input       [6:0]       adc_dataA_in_n,
        input       [6:0]       adc_dataB_in_p,
        input       [6:0]       adc_dataB_in_n,
        input                   adc_data_or_p,
        input                   adc_data_or_n,

        output                  spi_cs,
        output                  spi_clk,
        output                  spi_mosi
    ); 

    // internal registers
    reg     [6:0]  adc_dmuxA_a = 7'd0;
    reg     [6:0]  adc_dmuxA_b = 7'd0;
    reg     [6:0]  adc_dmuxB_a = 7'd0;
    reg     [6:0]  adc_dmuxB_b = 7'd0;

    wire           adc_or_p_s;
    wire           adc_or_n_s;

    // internal signals
    wire    [6:0]  adc_dataA_p_s;
    wire    [6:0]  adc_dataA_n_s;
    wire    [6:0]  adc_dataB_p_s;
    wire    [6:0]  adc_dataB_n_s;

    // output declaration of module ZGAD250D14_cfg    
    ZGAD250D14_cfg u_ZGAD250D14_cfg(
        .clock      (clock     ),
        .reset    	(reset     ),
        .spi_cs   	(spi_cs    ),
        .spi_clk  	(spi_clk   ),
        .spi_mosi 	(spi_mosi  )
    );
    
    always @(posedge adc_clk) begin
        //Channel A
        adc_dmuxA_a <=  adc_dataA_p_s;
        adc_dmuxA_b <=  adc_dataA_n_s;
    
        adc_data_CHA[15] <= 1'b0;
        adc_data_CHA[14] <= 1'b0;
        adc_data_CHA[13] <= adc_dmuxA_a[6];
        adc_data_CHA[12] <= adc_dmuxA_b[6];
        adc_data_CHA[11] <= adc_dmuxA_a[5];
        adc_data_CHA[10] <= adc_dmuxA_b[5];
        adc_data_CHA[9]  <= adc_dmuxA_a[4];
        adc_data_CHA[8]  <= adc_dmuxA_b[4];
        adc_data_CHA[7]  <= adc_dmuxA_a[3];
        adc_data_CHA[6]  <= adc_dmuxA_b[3];
        adc_data_CHA[5]  <= adc_dmuxA_a[2];
        adc_data_CHA[4]  <= adc_dmuxA_b[2];
        adc_data_CHA[3]  <= adc_dmuxA_a[1];
        adc_data_CHA[2]  <= adc_dmuxA_b[1];
        adc_data_CHA[1]  <= adc_dmuxA_a[0];
        adc_data_CHA[0]  <= adc_dmuxA_b[0];

        //Channel B
        adc_dmuxB_a <= adc_dataB_p_s;
        adc_dmuxB_b <= adc_dataB_n_s;

        adc_data_CHB[15] <= 1'b0;
        adc_data_CHB[14] <= 1'b0;
        adc_data_CHB[13] <= adc_dmuxB_a[6];
        adc_data_CHB[12] <= adc_dmuxB_b[6];
        adc_data_CHB[11] <= adc_dmuxB_a[5];
        adc_data_CHB[10] <= adc_dmuxB_b[5];
        adc_data_CHB[9]  <= adc_dmuxB_a[4];
        adc_data_CHB[8]  <= adc_dmuxB_b[4];
        adc_data_CHB[7]  <= adc_dmuxB_a[3];
        adc_data_CHB[6]  <= adc_dmuxB_b[3];
        adc_data_CHB[5]  <= adc_dmuxB_a[2];
        adc_data_CHB[4]  <= adc_dmuxB_b[2];
        adc_data_CHB[3]  <= adc_dmuxB_a[1];
        adc_data_CHB[2]  <= adc_dmuxB_b[1];
        adc_data_CHB[1]  <= adc_dmuxB_a[0];
        adc_data_CHB[0]  <= adc_dmuxB_b[0];
    
        // adc_or
        adc_or_n <= adc_or_n_s;
        adc_or_p <= adc_or_p_s;
    end  

    // data interface
    genvar l_inst;
    generate for (l_inst = 0; l_inst <= 6; l_inst = l_inst + 1) begin : g_adc_if
        //Channel A
        lvds_in i_adc_data (
            .rx_clk       (adc_clk),
            .rx_data_in_p (adc_dataA_in_p[l_inst]),
            .rx_data_in_n (adc_dataA_in_n[l_inst]),
            .rx_data_p    (adc_dataA_p_s[l_inst]),
            .rx_data_n    (adc_dataA_n_s[l_inst])
        );
        //Channel B
        lvds_in i_adc_data2 (
            .rx_clk       (adc_clk),
            .rx_data_in_p (adc_dataB_in_p[l_inst]),
            .rx_data_in_n (adc_dataB_in_n[l_inst]),
            .rx_data_p    (adc_dataB_p_s[l_inst]),
            .rx_data_n    (adc_dataB_n_s[l_inst])
        );
    end
    endgenerate

    lvds_in i_adc_or (
        .rx_clk       (adc_clk),
        .rx_data_in_p (adc_data_or_p),
        .rx_data_in_n (adc_data_or_n),
        .rx_data_p    (adc_or_p_s),
        .rx_data_n    (adc_or_n_s)
    );

    lvds_clk_in i_adc_clk (
        .clk_in_p (adc_clk_in_p),
        .clk_in_n (adc_clk_in_n),
        .clk      (adc_clk)
    );
    
    lvds_clk_out u_lvds_clk_out(
        .clk       	(clock         ),
        .clk_out_p 	(adc_clk_out_p ),
        .clk_out_n 	(adc_clk_out_n )
    );
    
endmodule
