`timescale 1 ns / 1 ns

/*
*   Date : 2025-06-23
*   Author : nitcloud
*   Module Name:   AnalogMod.v - AnalogMod
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description : Arbitrary frequency generator module.
*   Dependencies: accuml, Sin
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

module AnalogMod #(
        parameter MODULE_TYPE  = "AMP",
        parameter ISDSBSC_SET  = 0,
        parameter INPUT_WIDTH  = 12,
        parameter PHASE_WIDTH  = 32,
        parameter MODUL_WIDTH  = 16,
        parameter OUTAMP_WIDTH = 16,
        parameter OUTPUT_WIDTH = 12
    ) (
        input                     clk_cw,
        input                     clk_sw,
        input                     reset,

        input  [INPUT_WIDTH-1:0]  wave_in,

        //(fre*2^PHASE_WIDTH)/Fc
        input  [PHASE_WIDTH-1:0]  fre_word,     

        //initial phase word
        input  [PHASE_WIDTH-1:0]  pha_word, 

        //AM: (2 ^ `MODUL_WIDTH` - 1) * percent
        //FM: (2 ^ `(PHASE_WIDTH-INPUT_WIDTH+1)`) * Δf / clock
        //    note: In FM Mode, need INPUT_WIDTH + MODUL_WIDTH < PHASE_WIDTH
        input  [MODUL_WIDTH-1:0]  mod_word,

        //amplitude word 
        //Max: 2 ^ `MODUL_WIDTH` - 1
        input  [OUTAMP_WIDTH-1:0] amp_word,

        output [OUTPUT_WIDTH-1:0] mod_wave
    );

    localparam SIGNAL_WIDTH = INPUT_WIDTH + MODUL_WIDTH;
    localparam OSHIFT_WIDTH = OUTAMP_WIDTH - OUTPUT_WIDTH + 17;
    localparam PSHIFT_WIDTH = INPUT_WIDTH + MODUL_WIDTH - PHASE_WIDTH;

    reg  signed [SIGNAL_WIDTH:0] wave;
    reg  [PHASE_WIDTH-1:0] pha;
    reg  [PHASE_WIDTH-1:0] fre;
    reg  [9:0]  addr;

    wire signed [15:0] sin;
    wire signed [15:0] out;
    wire [MODUL_WIDTH-1:0] modul_wave;
    wire [PHASE_WIDTH-1:0] Q;

    always @(posedge clk_sw or posedge reset) begin
        if(reset) begin
            wave <= 0;
        end
        else begin
            wave <= $signed(wave_in) * $signed({1'b0, mod_word});
        end
    end

    generate 
        if(MODULE_TYPE == "AMP") begin : AMP
            // add DC value or not
            wire [MODUL_WIDTH-1:0] modul_am__wave = {~wave[SIGNAL_WIDTH-1], wave[SIGNAL_WIDTH-2:INPUT_WIDTH]};
            wire [MODUL_WIDTH-1:0] modul_dsb_wave = wave[SIGNAL_WIDTH-1:INPUT_WIDTH];
            assign modul_wave = ISDSBSC_SET ? modul_dsb_wave : modul_am__wave;
            
            // frequence
            always @(posedge clk_cw or posedge reset) begin
                if(reset) begin
                    fre <= 0;
                end
                else begin
                    fre <= fre_word;
                end
            end

            // phase
            always @(posedge clk_cw or posedge reset) begin
                if(reset) begin
                    pha <= 0;
                end
                else begin
                    pha <= Q + pha_word;
                end
            end

            // amplitude
            reg signed [MODUL_WIDTH+16:0] modul_out;
            always @(posedge clk_cw or posedge reset) begin
                if(reset) begin
                    modul_out <= 0;
                end
                else begin
                    modul_out <= $signed({1'b0, modul_wave}) * $signed(sin);
                end
            end
            assign out = modul_out[MODUL_WIDTH+15:MODUL_WIDTH];

            // mult u0_mult(
            //     .CLK ( clk_cw       ),
            //     .A   ( sin          ),
            //     .B   ( modul_wave   ),
            //     .P   ( out          )
            // );
        end
        else if(MODULE_TYPE == "FRE") begin : FRE
            wire signed [PHASE_WIDTH-1:0] modul_fm__wave = wave;
            assign modul_wave = 0;
            // frequence
            always @(posedge clk_sw or posedge reset) begin
                if(reset) begin
                    fre <= 0;
                end
                else begin
                    fre <= $signed(modul_fm__wave) + $signed({1'd0,fre_word});
                end
            end

            // phase
            always @(posedge clk_cw or posedge reset) begin
                if(reset) begin
                    pha <= 0;
                end
                else begin
                    pha <= Q + pha_word;
                end
            end

            // amplitude
            assign out = sin;
        end
        else if(MODULE_TYPE == "PHA") begin : PHA
            wire signed [PHASE_WIDTH-1:0] modul_pm__wave = wave;
            assign modul_wave = 0;
            // frequence
            always @(posedge clk_cw or posedge reset) begin
                if(reset) begin
                    fre <= 0;
                end
                else begin
                    fre <= fre_word;
                end
            end

            // phase
            reg signed [MODUL_WIDTH+16:0] modul_out;
            always @(posedge clk_sw or posedge reset) begin
                if(reset) begin
                    modul_out <= 0;
                end
                else begin
                    modul_out <= pha_word + modul_pm__wave;
                end
            end

            always @(posedge clk_cw or posedge reset) begin
                if(reset) begin
                    pha <= 0;
                end
                else begin
                    pha <= Q + modul_out;
                end
            end

            // amplitude
            assign out = sin;
        end
    endgenerate
    
    accuml #(
        .WIDTH 		( PHASE_WIDTH   ))
    u_accuml(
        // 端口
        .clock   		( clk_cw    ),
        .reset   		( reset     ),
        .clr     		( reset     ),
        .add_sub 		( 1'b0      ),
        .D       		( fre       ),
        .Q       		( Q         )
    );

    always @(posedge clk_cw or posedge reset) begin
        if (reset) begin
            addr <= 0;
        end
        else begin
            addr <= pha[PHASE_WIDTH-1:PHASE_WIDTH-10];
        end
    end

    Sin u_Sin(
        // 端口
        .clka  		( clk_cw    ),
        .rsta  		( reset     ),
        .addra 		( addr      ),
        .douta 		( sin       )
    );

    // wire signed [15:0] amp_out;
    // mult u1_mult(
    //     .CLK ( clk_cw       ),
    //     .A   ( out          ),
    //     .B   ( amp_word     ),
    //     .P   ( amp_out      )
    // );

    // assign mod_wave = (amp_out >>> (16-OUTPUT_WIDTH));

    reg signed [OUTAMP_WIDTH+16:0] amp_out;
    always @(posedge clk_cw or posedge reset) begin
        if(reset) begin
            amp_out <= 0;
        end
        else begin
            amp_out <= $signed(out) * $signed({1'b0, amp_word});
        end
    end

    assign mod_wave = (OSHIFT_WIDTH > 0) ? ((amp_out) >>>   OSHIFT_WIDTH) : 
                                           ((amp_out) <<< (-OSHIFT_WIDTH));

endmodule
