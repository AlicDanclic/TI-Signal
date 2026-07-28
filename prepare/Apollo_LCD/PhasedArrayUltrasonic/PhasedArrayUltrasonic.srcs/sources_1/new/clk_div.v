`timescale 1ns / 1ps
// ============================================================================
// Clock-enable generator for the stage-2 top.
//
// The design keeps logic in the 50 MHz clock domain. Divided clocks are exposed
// only for legacy compatibility with older modules.
// ============================================================================

module clk_div (
    input  wire clk_50m,
    input  wire rst_n,

    output reg  en_40k,
    output reg  en_audio,
    output reg  en_1k,
    output reg  clk_spi_adc,
    output reg  clk_spi_595
);

    localparam DIV_40K      = 12'd1250;
    localparam DIV_AUDIO    = 12'd1250;
    localparam DIV_1K       = 6'd40;
    localparam DIV_ADC      = 5'd25;
    localparam ADC_HIGH_CNT = 5'd13;

    reg [11:0] cnt_40k;
    reg [11:0] cnt_audio;
    reg [5:0]  cnt_1k;
    reg [4:0]  cnt_adc;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            cnt_40k <= 12'd0;
            en_40k  <= 1'b0;
        end else if (cnt_40k == DIV_40K - 1'b1) begin
            cnt_40k <= 12'd0;
            en_40k  <= 1'b1;
        end else begin
            cnt_40k <= cnt_40k + 1'b1;
            en_40k  <= 1'b0;
        end
    end

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            cnt_audio <= 12'd0;
            en_audio  <= 1'b0;
        end else if (cnt_audio == DIV_AUDIO - 1'b1) begin
            cnt_audio <= 12'd0;
            en_audio  <= 1'b1;
        end else begin
            cnt_audio <= cnt_audio + 1'b1;
            en_audio  <= 1'b0;
        end
    end

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            cnt_1k <= 6'd0;
            en_1k  <= 1'b0;
        end else if (en_40k) begin
            if (cnt_1k == DIV_1K - 1'b1) begin
                cnt_1k <= 6'd0;
                en_1k  <= 1'b1;
            end else begin
                cnt_1k <= cnt_1k + 1'b1;
                en_1k  <= 1'b0;
            end
        end else begin
            en_1k <= 1'b0;
        end
    end

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            cnt_adc     <= 5'd0;
            clk_spi_adc <= 1'b0;
        end else begin
            if (cnt_adc == DIV_ADC - 1'b1)
                cnt_adc <= 5'd0;
            else
                cnt_adc <= cnt_adc + 1'b1;

            clk_spi_adc <= (cnt_adc < ADC_HIGH_CNT);
        end
    end

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            clk_spi_595 <= 1'b0;
        else
            clk_spi_595 <= ~clk_spi_595;
    end

endmodule
