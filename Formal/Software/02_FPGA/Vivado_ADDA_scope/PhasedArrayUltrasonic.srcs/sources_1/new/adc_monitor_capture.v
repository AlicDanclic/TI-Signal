`timescale 1ns / 1ps

module adc_monitor_capture #(
    parameter integer CLK_FREQ_HZ          = 50_000_000,
    parameter integer GATE_CYCLES          = 50_000_000,
    parameter integer FRAME_INTERVAL_CYCLES = 2_000_000,
    parameter integer ZERO_WAIT_CYCLES     = 2_000_000,
    parameter integer ENVELOPE_CYCLES      = CLK_FREQ_HZ / 100,
    parameter integer MIN_SIGNAL_RANGE     = 32,
    parameter integer MIN_PERIOD_CYCLES    = 64,
    parameter integer MAX_PERIOD_CYCLES    = CLK_FREQ_HZ,
    parameter integer OVERVIEW_SAMPLE_RATE_HZ = 100_000,
    parameter integer LOW_THRESHOLD        = 496,
    parameter integer HIGH_THRESHOLD       = 528
) (
    input  wire          clk,
    input  wire          rst_n,
    input  wire [9:0]    sample_data,
    input  wire          frame_done,
    output reg  [31:0]   frequency_hz,
    output reg  [31:0]   last_period_cycles,
    output reg  [31:0]   adaptive_sample_div,
    output reg           signal_present,
    output reg           gate_toggle,
    output wire          crossing_pulse,
    output reg           frame_valid,
    output reg  [31:0]   frame_frequency_hz,
    output reg  [31:0]   frame_sample_div,
    output reg  [31:0]   frame_period_cycles,
    output reg  [7:0]    frame_flags,
    output reg  [1815:0] frame_samples
);

    reg        crossing_armed;
    reg        have_previous_crossing;
    reg        period_locked;
    reg [31:0] period_counter;
    reg [31:0] period_candidate;
    reg [1:0]  good_period_count;
    reg [1:0]  bad_period_count;
    reg [31:0] no_good_period_counter;
    reg [31:0] gate_counter;
    reg [31:0] gate_cross_count;

    reg [31:0] envelope_counter;
    reg [9:0]  envelope_min;
    reg [9:0]  envelope_max;
    reg [9:0]  dynamic_low_threshold;
    reg [9:0]  dynamic_high_threshold;
    reg        envelope_signal_valid;
    reg [9:0]  sample_delay_one;
    reg [9:0]  sample_delay_two;
    reg [1:0]  sample_filter_warmup;

    reg [31:0] frame_interval_counter;
    reg [31:0] history_div_counter;
    reg [1809:0] history_samples;
    reg [1809:0] triggered_samples;
    reg          triggered_samples_valid;

    localparam [31:0] OVERVIEW_SAMPLE_DIV =
        (CLK_FREQ_HZ > OVERVIEW_SAMPLE_RATE_HZ) ?
        (CLK_FREQ_HZ / OVERVIEW_SAMPLE_RATE_HZ) : 32'd1;

    wire [9:0] sample_pair_min =
        (sample_data < sample_delay_one) ? sample_data : sample_delay_one;
    wire [9:0] sample_pair_max =
        (sample_data > sample_delay_one) ? sample_data : sample_delay_one;
    wire [9:0] filtered_sample =
        (sample_delay_two < sample_pair_min) ? sample_pair_min :
        ((sample_delay_two > sample_pair_max) ? sample_pair_max : sample_delay_two);

    wire [9:0] envelope_min_with_sample =
        (filtered_sample < envelope_min) ? filtered_sample : envelope_min;
    wire [9:0] envelope_max_with_sample =
        (filtered_sample > envelope_max) ? filtered_sample : envelope_max;
    wire [10:0] envelope_range =
        {1'b0, envelope_max_with_sample} - {1'b0, envelope_min_with_sample};
    wire [12:0] envelope_range_times_three = envelope_range * 3'd3;
    wire [13:0] envelope_range_times_five  = envelope_range * 3'd5;
    wire [10:0] next_low_threshold =
        {1'b0, envelope_min_with_sample} + (envelope_range_times_three >> 3);
    wire [10:0] next_high_threshold =
        {1'b0, envelope_min_with_sample} + (envelope_range_times_five >> 3);

    assign crossing_pulse = envelope_signal_valid && crossing_armed &&
                            (filtered_sample >= dynamic_high_threshold);

    wire [31:0] period_reference =
        period_locked ? last_period_cycles : period_candidate;
    wire [31:0] period_difference =
        (period_counter >= period_reference) ?
        (period_counter - period_reference) :
        (period_reference - period_counter);
    wire [31:0] scaled_period_tolerance = period_reference >> 3;
    wire [31:0] period_tolerance =
        (scaled_period_tolerance < 32'd8) ? 32'd8 : scaled_period_tolerance;
    wire period_in_range =
        (period_counter >= MIN_PERIOD_CYCLES) &&
        (period_counter <= MAX_PERIOD_CYCLES);
    wire period_consistent = period_in_range && (period_reference != 0) &&
                             (period_difference <= period_tolerance);
    wire [32:0] period_average_sum =
        {1'b0, period_reference} + {1'b0, period_counter};
    wire [31:0] averaged_period = period_average_sum[32:1];
    wire accepted_period_pulse = crossing_pulse && have_previous_crossing &&
                                 period_locked && period_consistent;

    wire [31:0] scaled_loss_limit =
        (last_period_cycles > 32'h1fff_ffff) ?
        32'hffff_ffff : (last_period_cycles << 3);
    wire [31:0] period_loss_limit =
        (scaled_loss_limit < ZERO_WAIT_CYCLES) ?
        ZERO_WAIT_CYCLES : scaled_loss_limit;

    wire [32:0] gate_cross_total = {1'b0, gate_cross_count} +
                                    (accepted_period_pulse ? 33'd1 : 33'd0);

    // A three-point median removes isolated mixed-code ADC samples before the
    // display/frequency path. The direct ADC-to-DAC path remains untouched.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_delay_one <= 10'd512;
            sample_delay_two <= 10'd512;
            sample_filter_warmup <= 2'd0;
        end else begin
            sample_delay_one <= sample_data;
            sample_delay_two <= sample_delay_one;
            if (sample_filter_warmup != 2'd3)
                sample_filter_warmup <= sample_filter_warmup + 1'b1;
        end
    end

    // Track the filtered input envelope over 10 ms (default) and derive
    // Schmitt thresholds around its midpoint for bipolar and offset inputs.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            envelope_counter        <= 32'd0;
            envelope_min            <= 10'd1023;
            envelope_max            <= 10'd0;
            dynamic_low_threshold   <= LOW_THRESHOLD;
            dynamic_high_threshold  <= HIGH_THRESHOLD;
            envelope_signal_valid   <= 1'b0;
        end else if (sample_filter_warmup != 2'd3) begin
            envelope_counter      <= 32'd0;
            envelope_min          <= 10'd1023;
            envelope_max          <= 10'd0;
            envelope_signal_valid <= 1'b0;
        end else if (envelope_counter >= ENVELOPE_CYCLES - 1) begin
            envelope_counter <= 32'd0;
            envelope_min     <= filtered_sample;
            envelope_max     <= filtered_sample;
            if (envelope_range >= MIN_SIGNAL_RANGE) begin
                dynamic_low_threshold  <= next_low_threshold[9:0];
                dynamic_high_threshold <= next_high_threshold[9:0];
                envelope_signal_valid <= 1'b1;
            end else begin
                envelope_signal_valid <= 1'b0;
            end
        end else begin
            envelope_counter <= envelope_counter + 1'b1;
            if (filtered_sample < envelope_min)
                envelope_min <= filtered_sample;
            if (filtered_sample > envelope_max)
                envelope_max <= filtered_sample;
        end
    end

    // Lock only after three mutually consistent periods. One isolated ADC
    // glitch normally creates two bad intervals, so three bad intervals are
    // required to drop an existing lock.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            crossing_armed         <= 1'b0;
            have_previous_crossing <= 1'b0;
            period_locked          <= 1'b0;
            period_counter         <= 32'd0;
            period_candidate       <= 32'd0;
            good_period_count      <= 2'd0;
            bad_period_count       <= 2'd0;
            no_good_period_counter <= 32'd0;
            last_period_cycles     <= 32'd0;
            adaptive_sample_div    <= OVERVIEW_SAMPLE_DIV;
            gate_counter           <= 32'd0;
            gate_cross_count       <= 32'd0;
            frequency_hz           <= 32'd0;
            signal_present         <= 1'b0;
            gate_toggle            <= 1'b0;
        end else begin
            if (!envelope_signal_valid) begin
                crossing_armed         <= 1'b0;
                have_previous_crossing <= 1'b0;
                period_locked          <= 1'b0;
                period_counter         <= 32'd0;
                period_candidate       <= 32'd0;
                good_period_count      <= 2'd0;
                bad_period_count       <= 2'd0;
                no_good_period_counter <= 32'd0;
                last_period_cycles     <= 32'd0;
                adaptive_sample_div    <= OVERVIEW_SAMPLE_DIV;
                signal_present         <= 1'b0;
            end else begin
                if (filtered_sample <= dynamic_low_threshold)
                    crossing_armed <= 1'b1;
                else if (crossing_pulse)
                    crossing_armed <= 1'b0;

                if (crossing_pulse) begin
                    period_counter <= 32'd1;
                    if (!have_previous_crossing) begin
                        have_previous_crossing <= 1'b1;
                    end else if (!period_locked) begin
                        if (!period_in_range) begin
                            period_candidate  <= 32'd0;
                            good_period_count <= 2'd0;
                        end else if (period_candidate == 0 ||
                                     !period_consistent) begin
                            period_candidate  <= period_counter;
                            good_period_count <= 2'd1;
                        end else if (good_period_count >= 2'd2) begin
                            period_candidate       <= averaged_period;
                            good_period_count      <= 2'd3;
                            bad_period_count       <= 2'd0;
                            no_good_period_counter <= 32'd0;
                            last_period_cycles     <= averaged_period;
                            adaptive_sample_div    <=
                                (averaged_period < 32'd64) ? 32'd1 :
                                ((averaged_period + 32'd32) >> 6);
                            period_locked          <= 1'b1;
                            signal_present         <= 1'b1;
                        end else begin
                            period_candidate  <= averaged_period;
                            good_period_count <= good_period_count + 1'b1;
                        end
                    end else if (period_consistent) begin
                        last_period_cycles     <= averaged_period;
                        adaptive_sample_div    <=
                            (averaged_period < 32'd64) ? 32'd1 :
                            ((averaged_period + 32'd32) >> 6);
                        bad_period_count       <= 2'd0;
                        no_good_period_counter <= 32'd0;
                        signal_present         <= 1'b1;
                    end else if (bad_period_count >= 2'd2 ||
                                 no_good_period_counter >=
                                 period_loss_limit - 1'b1) begin
                        period_locked          <= 1'b0;
                        signal_present         <= 1'b0;
                        last_period_cycles     <= 32'd0;
                        adaptive_sample_div    <= OVERVIEW_SAMPLE_DIV;
                        period_candidate       <=
                            period_in_range ? period_counter : 32'd0;
                        good_period_count      <= period_in_range ? 2'd1 : 2'd0;
                        bad_period_count       <= 2'd0;
                        no_good_period_counter <= 32'd0;
                    end else begin
                        bad_period_count <= bad_period_count + 1'b1;
                        if (no_good_period_counter != 32'hffff_ffff)
                            no_good_period_counter <=
                                no_good_period_counter + 1'b1;
                    end
                end else begin
                    if (have_previous_crossing &&
                        period_counter != 32'hffff_ffff)
                        period_counter <= period_counter + 1'b1;

                    if (period_locked) begin
                        if (no_good_period_counter >=
                            period_loss_limit - 1'b1) begin
                            period_locked          <= 1'b0;
                            signal_present         <= 1'b0;
                            have_previous_crossing <= 1'b0;
                            period_counter         <= 32'd0;
                            period_candidate       <= 32'd0;
                            good_period_count      <= 2'd0;
                            bad_period_count       <= 2'd0;
                            no_good_period_counter <= 32'd0;
                            last_period_cycles     <= 32'd0;
                            adaptive_sample_div    <= OVERVIEW_SAMPLE_DIV;
                        end else if (no_good_period_counter != 32'hffff_ffff) begin
                            no_good_period_counter <=
                                no_good_period_counter + 1'b1;
                        end
                    end else if (have_previous_crossing &&
                                 period_counter >= MAX_PERIOD_CYCLES) begin
                        have_previous_crossing <= 1'b0;
                        period_counter         <= 32'd0;
                        period_candidate       <= 32'd0;
                        good_period_count      <= 2'd0;
                    end
                end
            end

            if (gate_counter >= GATE_CYCLES - 1) begin
                gate_counter     <= 32'd0;
                gate_cross_count <= 32'd0;
                frequency_hz     <= gate_cross_total[31:0];
                gate_toggle      <= ~gate_toggle;
            end else begin
                gate_counter <= gate_counter + 1'b1;
                if (accepted_period_pulse)
                    gate_cross_count <= gate_cross_count + 1'b1;
            end
        end
    end

    // Continuously maintain a 181-sample history. Only a validated crossing
    // latches a trigger snapshot, while frames are emitted at a fixed cadence.
    // Low-frequency signals therefore never stall the UART link while their
    // history window is still filling.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_interval_counter <= 32'd0;
            history_div_counter    <= 32'd0;
            history_samples        <= {181{10'd512}};
            triggered_samples      <= {181{10'd512}};
            triggered_samples_valid <= 1'b0;
            frame_valid            <= 1'b0;
            frame_frequency_hz     <= 32'd0;
            frame_sample_div       <= OVERVIEW_SAMPLE_DIV;
            frame_period_cycles    <= 32'd0;
            frame_flags            <= 8'd0;
            frame_samples          <= {1816{1'b0}};
        end else begin
            if (history_div_counter >=
                ((adaptive_sample_div == 0) ? 32'd0 : adaptive_sample_div - 1'b1)) begin
                history_div_counter <= 32'd0;
                history_samples <= {filtered_sample, history_samples[1809:10]};
            end else begin
                history_div_counter <= history_div_counter + 1'b1;
            end

            if (!signal_present) begin
                triggered_samples_valid <= 1'b0;
            end else if (accepted_period_pulse) begin
                triggered_samples <= {filtered_sample, history_samples[1809:10]};
                triggered_samples_valid <= 1'b1;
            end

            if (frame_interval_counter < FRAME_INTERVAL_CYCLES - 1)
                frame_interval_counter <= frame_interval_counter + 1'b1;

            if (frame_valid) begin
                if (frame_done)
                    frame_valid <= 1'b0;
            end else if (frame_interval_counter >= FRAME_INTERVAL_CYCLES - 1) begin
                frame_interval_counter <= 32'd0;
                frame_frequency_hz  <= frequency_hz;
                frame_sample_div    <= (adaptive_sample_div == 0) ?
                                       32'd1 : adaptive_sample_div;
                frame_period_cycles <= last_period_cycles;
                frame_flags         <= {7'd0, signal_present};
                frame_samples       <= triggered_samples_valid ?
                                       {6'd0, triggered_samples} :
                                       {6'd0, history_samples};
                frame_valid         <= 1'b1;
            end
        end
    end

endmodule
