`timescale 1ns / 1ps

module adda_data_path (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [9:0] ad_ch1_data,
    input  wire [9:0] ad_ch2_data,
    output wire [9:0] ad_ch1_sample,
    output wire [9:0] ad_ch2_sample,
    output wire [9:0] da_ch1_data,
    output wire [9:0] da_ch2_data
);

    // Keep the 50 MHz capture registers beside the JP2 input pins. This is
    // required to preserve the small source-synchronous setup window.
    (* IOB = "TRUE" *) reg [9:0] ad_ch1_sample_iob;
    (* IOB = "TRUE" *) reg [9:0] ad_ch2_sample_iob;

    assign ad_ch1_sample = ad_ch1_sample_iob;
    assign ad_ch2_sample = ad_ch2_sample_iob;

    // Capture each ADC bus halfway through its output clock period.
    always @(negedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ad_ch1_sample_iob <= 10'd512;
            ad_ch2_sample_iob <= 10'd512;
        end else begin
            ad_ch1_sample_iob <= ad_ch1_data;
            ad_ch2_sample_iob <= ad_ch2_data;
        end
    end

    // Data changes just after the falling edge and is stable for about half a
    // cycle before the externally forwarded DAC clock rises.
    assign da_ch1_data = ~ad_ch1_sample;
    assign da_ch2_data = ~ad_ch2_sample;

endmodule
