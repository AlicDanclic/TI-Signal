`timescale 1ns / 1ps

module top_ad_da_uart #(
    parameter integer CLK_FREQ_HZ           = 50_000_000,
    parameter integer BAUD_RATE             = 115200,
    parameter integer FREQ_GATE_CYCLES      = CLK_FREQ_HZ,
    parameter integer FRAME_INTERVAL_CYCLES = CLK_FREQ_HZ / 25,
    parameter integer ZERO_WAIT_CYCLES      = CLK_FREQ_HZ / 25
) (
    input  wire       clk_50m,
    input  wire       rst_n,

    input  wire [9:0] ad_ch1_data,
    output wire       ad_clk1,
    output wire       ad_oe1,
    input  wire [9:0] ad_ch2_data,
    output wire       ad_clk2,
    output wire       ad_oe2,

    output wire [9:0] da_ch1_data,
    output wire       da_clk1,
    output wire [9:0] da_ch2_data,
    output wire       da_clk2,

    output wire       uart_tx,
    output wire [3:0] led
);

    wire [9:0] ad_ch1_sample;
    wire [9:0] ad_ch2_sample;

    wire [31:0] frequency_hz;
    wire [31:0] last_period_cycles;
    wire [31:0] adaptive_sample_div;
    wire        signal_present;
    wire        gate_toggle;
    wire        crossing_pulse;

    wire          frame_valid;
    wire          frame_done;
    wire          frame_busy;
    wire [31:0]   frame_frequency_hz;
    wire [31:0]   frame_sample_div;
    wire [31:0]   frame_period_cycles;
    wire [7:0]    frame_flags;
    wire [1815:0] frame_samples;

    (* ASYNC_REG = "TRUE" *) reg [1:0] reset_release_sync;
    wire core_rst_n = reset_release_sync[1];

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            reset_release_sync <= 2'b00;
        else
            reset_release_sync <= {reset_release_sync[0], 1'b1};
    end

    assign ad_oe1 = 1'b0;
    assign ad_oe2 = 1'b0;

    // Four source-synchronous clocks are forwarded in phase with clk_50m.
    ODDR #(
        .DDR_CLK_EDGE ("SAME_EDGE"),
        .INIT         (1'b0),
        .SRTYPE       ("ASYNC")
    ) u_ad_clk1_oddr (
        .Q  (ad_clk1),
        .C  (clk_50m),
        .CE (1'b1),
        .D1 (1'b1),
        .D2 (1'b0),
        .R  (~rst_n),
        .S  (1'b0)
    );

    ODDR #(
        .DDR_CLK_EDGE ("SAME_EDGE"),
        .INIT         (1'b0),
        .SRTYPE       ("ASYNC")
    ) u_ad_clk2_oddr (
        .Q  (ad_clk2),
        .C  (clk_50m),
        .CE (1'b1),
        .D1 (1'b1),
        .D2 (1'b0),
        .R  (~rst_n),
        .S  (1'b0)
    );

    ODDR #(
        .DDR_CLK_EDGE ("SAME_EDGE"),
        .INIT         (1'b0),
        .SRTYPE       ("ASYNC")
    ) u_da_clk1_oddr (
        .Q  (da_clk1),
        .C  (clk_50m),
        .CE (1'b1),
        .D1 (1'b1),
        .D2 (1'b0),
        .R  (~rst_n),
        .S  (1'b0)
    );

    ODDR #(
        .DDR_CLK_EDGE ("SAME_EDGE"),
        .INIT         (1'b0),
        .SRTYPE       ("ASYNC")
    ) u_da_clk2_oddr (
        .Q  (da_clk2),
        .C  (clk_50m),
        .CE (1'b1),
        .D1 (1'b1),
        .D2 (1'b0),
        .R  (~rst_n),
        .S  (1'b0)
    );

    adda_data_path u_adda_data_path (
        .clk           (clk_50m),
        .rst_n         (core_rst_n),
        .ad_ch1_data   (ad_ch1_data),
        .ad_ch2_data   (ad_ch2_data),
        .ad_ch1_sample (ad_ch1_sample),
        .ad_ch2_sample (ad_ch2_sample),
        .da_ch1_data   (da_ch1_data),
        .da_ch2_data   (da_ch2_data)
    );

    adc_monitor_capture #(
        .CLK_FREQ_HZ           (CLK_FREQ_HZ),
        .GATE_CYCLES           (FREQ_GATE_CYCLES),
        .FRAME_INTERVAL_CYCLES (FRAME_INTERVAL_CYCLES),
        .ZERO_WAIT_CYCLES      (ZERO_WAIT_CYCLES),
        .LOW_THRESHOLD         (496),
        .HIGH_THRESHOLD        (528)
    ) u_monitor_capture (
        .clk                (clk_50m),
        .rst_n              (core_rst_n),
        .sample_data        (ad_ch1_sample),
        .frame_done         (frame_done),
        .frequency_hz       (frequency_hz),
        .last_period_cycles (last_period_cycles),
        .adaptive_sample_div(adaptive_sample_div),
        .signal_present     (signal_present),
        .gate_toggle        (gate_toggle),
        .crossing_pulse     (crossing_pulse),
        .frame_valid        (frame_valid),
        .frame_frequency_hz (frame_frequency_hz),
        .frame_sample_div   (frame_sample_div),
        .frame_period_cycles(frame_period_cycles),
        .frame_flags        (frame_flags),
        .frame_samples      (frame_samples)
    );

    wave_uart_frame_tx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) u_frame_tx (
        .clk               (clk_50m),
        .rst_n             (core_rst_n),
        .frame_valid       (frame_valid),
        .frame_frequency_hz(frame_frequency_hz),
        .frame_sample_div  (frame_sample_div),
        .frame_period_cycles(frame_period_cycles),
        .frame_flags       (frame_flags),
        .frame_samples     (frame_samples),
        .uart_tx_o         (uart_tx),
        .frame_done        (frame_done),
        .frame_busy        (frame_busy),
        .debug_byte_valid  (),
        .debug_byte        (),
        .debug_byte_index  ()
    );

    // The four board LEDs are active low.
    assign led[0] = ~signal_present;
    assign led[1] = ~frame_busy;
    assign led[2] = ~gate_toggle;
    assign led[3] = core_rst_n;

endmodule
