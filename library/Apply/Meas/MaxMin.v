// 最大最小值测量模块
// 在采样窗口内跟踪并输出信号的最大值和最小值

`timescale 1 ns / 1 ns

/*
*   Date : 2025-06-24
*   Author : nitcloud
*   Module Name:   MaxMin.v - MaxMin
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
*   Description : Max or Min Measure module.
*   Dependencies: none
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

module MaxMin #(
        parameter   MODESET = "MAX",
        parameter   DWIDTH  = 4'd12,
        parameter   RWIDTH  = 4'd10
    )(
        input               clock,
        input               reset,
        
        input  [RWIDTH-1:0] range,

        input               ivalid,
        input  [DWIDTH-1:0] idata,

        output              ovalid,
        output [DWIDTH-1:0] odata
    );

    localparam state_output    = 3'b010;
    localparam state_initial   = 3'b000;
    localparam state_detection = 3'b001;

    reg                     done_reg;
    reg                     done_buf;
    reg signed [DWIDTH-1:0] odata_buf;
    reg signed [DWIDTH-1:0] odata_reg;
    reg [2:0]  state = 3'b000;
    reg 	   test_sig;
    reg 	   test_sig_buf;
    wire 	   test_done_sig = ~test_sig &  test_sig_buf;
    wire 	   test_start_sig = test_sig & ~test_sig_buf;

    /***************************************************/
    //define the time counter
    reg [RWIDTH-1:0] count = 1;

    always@(posedge clock or posedge reset) begin
        if (reset) begin
            test_sig <= 1'b0;
            count <= 0;
        end
        else begin
            if (ivalid) begin
                if (range == 0) begin
                    test_sig <= 1'b1;
                    count <= count + 1'b1;
                end
                else begin
                    if (count == range) begin
                        count <= 0;                    
                        test_sig <= 1'd0;
                    end
                    else begin
                        test_sig <= 1'd1;                  
                        count <= count + 1'b1;
                    end
                end
            end     
        end
    end
    /***************************************************/

    always@(posedge clock or posedge reset) begin
        if (reset) begin
            test_sig_buf <= 1'b0;
        end
        else begin
            if (range == 0) begin
                test_sig_buf <= 1'b0;
            end
            else begin
                test_sig_buf <= test_sig;
            end
        end
    end

    always@(posedge clock or posedge reset) begin
        if(reset) begin
            done_reg  <= 0;
            done_buf  <= 0;
            odata_reg <= 0;
            odata_buf <= 0;
        end
        else begin
            case(state)
                state_initial: begin
                    if(test_start_sig) begin
                        state <= state_detection;
                        odata_reg <= idata;
                    end
                end
                state_detection: begin
                    if(test_done_sig) begin
                        state <= state_output;
                    end
                    else begin
                        if (MODESET == "MAX") begin
                            if($signed(idata) > $signed(odata_reg)) begin
                                odata_reg <= idata;
                                if (range == 0) begin
                                    odata_buf <= odata_reg;
                                end
                            end
                        end 
                        else if (MODESET == "MIN") begin
                            if($signed(idata) < $signed(odata_reg)) begin
                                odata_reg <= idata;
                                if (range == 0) begin
                                    odata_buf <= odata_reg;
                                end
                            end
                        end                   
                    end
                end
                state_output: begin
                    odata_buf <= odata_reg;
                    state <= state_initial;
                end
            endcase
            done_reg <= test_done_sig;
            done_buf <= done_reg;
        end
    end

    assign ovalid  = done_buf;
    assign odata = odata_buf;

endmodule
