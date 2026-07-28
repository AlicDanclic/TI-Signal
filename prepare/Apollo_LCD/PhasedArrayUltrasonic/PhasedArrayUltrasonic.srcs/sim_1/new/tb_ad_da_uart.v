`timescale 1ns / 1ps

module tb_ad_da_uart;

    reg clk;
    reg rst_n;
    integer failures;

    reg  [9:0] ad_ch1_data;
    reg  [9:0] ad_ch2_data;
    wire [9:0] ad_ch1_sample;
    wire [9:0] ad_ch2_sample;
    wire [9:0] da_ch1_data;
    wire [9:0] da_ch2_data;

    reg  [9:0] monitor_sample;
    reg        monitor_glitch;
    wire [31:0] frequency_hz;
    wire [31:0] last_period_cycles;
    wire [31:0] adaptive_sample_div;
    wire        signal_present;
    wire        gate_toggle;
    wire        crossing_pulse;
    wire        monitor_frame_valid;
    reg         monitor_frame_done;
    wire [31:0] monitor_frame_frequency;
    wire [31:0] monitor_frame_div;
    wire [31:0] monitor_frame_period;
    wire [7:0]  monitor_frame_flags;
    wire [1815:0] monitor_frame_samples;

    reg           tx_frame_valid;
    reg  [1815:0] tx_frame_samples;
    wire          serial_tx;
    wire          tx_frame_done;
    wire          tx_frame_busy;
    wire          debug_byte_valid;
    wire [7:0]    debug_byte;
    wire [7:0]    debug_byte_index;
    wire [7:0]    serial_byte;
    wire          serial_byte_valid;

    integer wave_phase;
    integer sample_index;
    integer debug_count;
    integer serial_count;
    reg [15:0] expected_crc;
    reg datapath_done;
    reg frequency_done;
    reg transmitter_done;
    reg monitor_cadence_done;
    reg glitch_rejection_done;

    always #5 clk = ~clk;

    adda_data_path u_data_path (
        .clk           (clk),
        .rst_n         (rst_n),
        .ad_ch1_data   (ad_ch1_data),
        .ad_ch2_data   (ad_ch2_data),
        .ad_ch1_sample (ad_ch1_sample),
        .ad_ch2_sample (ad_ch2_sample),
        .da_ch1_data   (da_ch1_data),
        .da_ch2_data   (da_ch2_data)
    );

    adc_monitor_capture #(
        .CLK_FREQ_HZ           (1000),
        .GATE_CYCLES           (1000),
        .FRAME_INTERVAL_CYCLES (200),
        .ZERO_WAIT_CYCLES      (40),
        .ENVELOPE_CYCLES       (1000),
        .LOW_THRESHOLD         (496),
        .HIGH_THRESHOLD        (528)
    ) u_monitor (
        .clk                (clk),
        .rst_n              (rst_n),
        .sample_data        (monitor_sample),
        .frame_done         (monitor_frame_done),
        .frequency_hz       (frequency_hz),
        .last_period_cycles (last_period_cycles),
        .adaptive_sample_div(adaptive_sample_div),
        .signal_present     (signal_present),
        .gate_toggle        (gate_toggle),
        .crossing_pulse     (crossing_pulse),
        .frame_valid        (monitor_frame_valid),
        .frame_frequency_hz (monitor_frame_frequency),
        .frame_sample_div   (monitor_frame_div),
        .frame_period_cycles(monitor_frame_period),
        .frame_flags        (monitor_frame_flags),
        .frame_samples      (monitor_frame_samples)
    );

    wave_uart_frame_tx #(
        .CLK_FREQ_HZ (100),
        .BAUD_RATE   (10)
    ) u_frame_tx (
        .clk               (clk),
        .rst_n             (rst_n),
        .frame_valid       (tx_frame_valid),
        .frame_frequency_hz(32'h1234_5678),
        .frame_sample_div  (32'h0102_0304),
        .frame_period_cycles(32'hA1B2_C3D4),
        .frame_flags       (8'h01),
        .frame_samples     (tx_frame_samples),
        .uart_tx_o         (serial_tx),
        .frame_done        (tx_frame_done),
        .frame_busy        (tx_frame_busy),
        .debug_byte_valid  (debug_byte_valid),
        .debug_byte        (debug_byte),
        .debug_byte_index  (debug_byte_index)
    );

    uart_rx #(
        .CLK_FREQ_HZ (100),
        .BAUD_RATE   (10)
    ) u_serial_loopback (
        .clk        (clk),
        .rst_n      (rst_n),
        .rx         (serial_tx),
        .data_out   (serial_byte),
        .data_valid (serial_byte_valid)
    );

    function [15:0] crc16_ccitt_next;
        input [15:0] crc_in;
        input [7:0]  data_in;
        integer bit_number;
        reg [15:0] crc_work;
        begin
            crc_work = crc_in ^ {data_in, 8'h00};
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                if (crc_work[15])
                    crc_work = (crc_work << 1) ^ 16'h1021;
                else
                    crc_work = crc_work << 1;
            end
            crc16_ccitt_next = crc_work;
        end
    endfunction

    function [7:0] expected_payload_byte;
        input [7:0] index;
        begin
            case (index)
                8'd0:  expected_payload_byte = 8'hA5;
                8'd1:  expected_payload_byte = 8'h5A;
                8'd2:  expected_payload_byte = 8'h02;
                8'd3:  expected_payload_byte = 8'h00;
                8'd4:  expected_payload_byte = 8'h78;
                8'd5:  expected_payload_byte = 8'h56;
                8'd6:  expected_payload_byte = 8'h34;
                8'd7:  expected_payload_byte = 8'h12;
                8'd8:  expected_payload_byte = 8'h04;
                8'd9:  expected_payload_byte = 8'h03;
                8'd10: expected_payload_byte = 8'h02;
                8'd11: expected_payload_byte = 8'h01;
                8'd12: expected_payload_byte = 8'hB5;
                8'd13: expected_payload_byte = 8'h01;
                8'd14: expected_payload_byte = 8'hD4;
                8'd15: expected_payload_byte = 8'hC3;
                8'd16: expected_payload_byte = 8'hB2;
                8'd17: expected_payload_byte = 8'hA1;
                default: expected_payload_byte = tx_frame_samples[(index-8'd18)*8 +: 8];
            endcase
        end
    endfunction

    always @(negedge clk) begin
        if (!rst_n) begin
            wave_phase    = 0;
            monitor_sample = 10'd650;
        end else begin
            if (wave_phase == 199)
                wave_phase = 0;
            else
                wave_phase = wave_phase + 1;

            if (monitor_glitch)
                monitor_sample = 10'd1023;
            else if (wave_phase < 100)
                monitor_sample = 10'd650;
            else
                monitor_sample = 10'd850;
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            debug_count  <= 0;
            expected_crc <= 16'hffff;
        end else if (debug_byte_valid) begin
            if (debug_byte_index !== debug_count[7:0]) begin
                $display("FAIL: byte index %0d, expected %0d", debug_byte_index, debug_count);
                failures = failures + 1;
            end

            if (debug_byte_index <= 8'd244) begin
                if (debug_byte !== expected_payload_byte(debug_byte_index)) begin
                    $display("FAIL: byte[%0d]=%02x expected %02x",
                             debug_byte_index, debug_byte,
                             expected_payload_byte(debug_byte_index));
                    failures = failures + 1;
                end
                expected_crc <= crc16_ccitt_next(expected_crc, debug_byte);
            end else if (debug_byte_index == 8'd245) begin
                if (debug_byte !== expected_crc[7:0]) begin
                    $display("FAIL: CRC low=%02x expected %02x",
                             debug_byte, expected_crc[7:0]);
                    failures = failures + 1;
                end
            end else if (debug_byte_index == 8'd246) begin
                if (debug_byte !== expected_crc[15:8]) begin
                    $display("FAIL: CRC high=%02x expected %02x",
                             debug_byte, expected_crc[15:8]);
                    failures = failures + 1;
                end
            end
            debug_count <= debug_count + 1;
        end
    end

    // Decode the real UART wire as well as observing the frame producer's
    // debug handshake. This checks start/data/stop bit order and baud timing.
    always @(posedge clk) begin
        if (!rst_n) begin
            serial_count <= 0;
        end else if (serial_byte_valid) begin
            if (serial_count <= 244) begin
                if (serial_byte !== expected_payload_byte(serial_count[7:0])) begin
                    $display("FAIL: serial byte[%0d]=%02x expected %02x",
                             serial_count, serial_byte,
                             expected_payload_byte(serial_count[7:0]));
                    failures = failures + 1;
                end
            end else if (serial_count == 245) begin
                if (serial_byte !== expected_crc[7:0]) begin
                    $display("FAIL: serial CRC low=%02x expected %02x",
                             serial_byte, expected_crc[7:0]);
                    failures = failures + 1;
                end
            end else if (serial_count == 246) begin
                if (serial_byte !== expected_crc[15:8]) begin
                    $display("FAIL: serial CRC high=%02x expected %02x",
                             serial_byte, expected_crc[15:8]);
                    failures = failures + 1;
                end
            end
            serial_count <= serial_count + 1;
        end
    end

    initial begin
        clk              = 1'b0;
        rst_n            = 1'b0;
        failures         = 0;
        ad_ch1_data      = 10'd512;
        ad_ch2_data      = 10'd512;
        monitor_sample   = 10'd650;
        monitor_glitch   = 1'b0;
        tx_frame_valid   = 1'b0;
        tx_frame_samples = {1816{1'b0}};
        wave_phase       = 0;
        debug_count      = 0;
        serial_count     = 0;
        expected_crc     = 16'hffff;
        datapath_done    = 1'b0;
        frequency_done   = 1'b0;
        transmitter_done = 1'b0;
        monitor_cadence_done = 1'b0;
        glitch_rejection_done = 1'b0;
        monitor_frame_done = 1'b0;

        for (sample_index = 0; sample_index < 181; sample_index = sample_index + 1)
            tx_frame_samples[sample_index*10 +: 10] = sample_index[9:0] ^ 10'h15A;

        #42;
        rst_n = 1'b1;
    end

    initial begin
        wait (rst_n);
        @(posedge clk);
        ad_ch1_data = 10'h155;
        ad_ch2_data = 10'h2A3;
        @(negedge clk);
        #1;
        if (da_ch1_data !== ~10'h155 || da_ch2_data !== ~10'h2A3) begin
            $display("FAIL: inverted ADC-to-DAC path after falling edge");
            failures = failures + 1;
        end
        #3;
        if (da_ch1_data !== ~10'h155 || da_ch2_data !== ~10'h2A3) begin
            $display("FAIL: DAC data was not stable before rising clock edge");
            failures = failures + 1;
        end
        @(posedge clk);
        #1;
        if (da_ch1_data !== ~10'h155 || da_ch2_data !== ~10'h2A3) begin
            $display("FAIL: DAC data changed at its rising clock edge");
            failures = failures + 1;
        end
        datapath_done = 1'b1;
    end

    initial begin
        wait (rst_n);
        // The first envelope window learns thresholds. Measure the following
        // complete gate so this also verifies an offset waveform whose low
        // level is above the legacy fixed high threshold (528).
        // Three matching periods establish the lock; the following complete
        // one-second gate must contain all five accepted crossings.
        repeat (3100) @(posedge clk);
        #1;
        if (frequency_hz !== 32'd5) begin
            $display("FAIL: frequency=%0d expected 5", frequency_hz);
            failures = failures + 1;
        end
        if (!signal_present) begin
            $display("FAIL: signal_present was not asserted");
            failures = failures + 1;
        end
        if (last_period_cycles !== 32'd200 || adaptive_sample_div !== 32'd3) begin
            $display("FAIL: period/div=%0d/%0d expected 200/3",
                     last_period_cycles, adaptive_sample_div);
            failures = failures + 1;
        end
        frequency_done = 1'b1;
    end

    initial begin
        wait (frequency_done);
        @(posedge clk);
        monitor_glitch = 1'b1;
        @(posedge clk);
        monitor_glitch = 1'b0;
        repeat (300) @(posedge clk);
        #1;
        if (!signal_present || last_period_cycles !== 32'd200 ||
            adaptive_sample_div !== 32'd3) begin
            $display("FAIL: one-sample glitch disturbed lock/period/div=%0d/%0d/%0d",
                     signal_present, last_period_cycles, adaptive_sample_div);
            failures = failures + 1;
        end
        glitch_rejection_done = 1'b1;
    end

    initial begin
        wait (rst_n);
        wait (monitor_frame_valid);
        @(posedge clk);
        monitor_frame_done = 1'b1;
        @(posedge clk);
        monitor_frame_done = 1'b0;
        wait (!monitor_frame_valid);
        wait (monitor_frame_valid);
        if (^monitor_frame_samples === 1'bx) begin
            $display("FAIL: continuous monitor frame contains unknown sample bits");
            failures = failures + 1;
        end
        monitor_cadence_done = 1'b1;
    end

    initial begin
        wait (rst_n);
        @(posedge clk);
        tx_frame_valid = 1'b1;
        wait (tx_frame_done);
        #1;
        if (debug_count !== 247) begin
            $display("FAIL: frame length=%0d expected 247", debug_count);
            failures = failures + 1;
        end
        if (serial_count !== 247) begin
            $display("FAIL: decoded serial length=%0d expected 247", serial_count);
            failures = failures + 1;
        end
        tx_frame_valid = 1'b0;
        transmitter_done = 1'b1;
    end

    initial begin
        wait (datapath_done && frequency_done && transmitter_done &&
              monitor_cadence_done && glitch_rejection_done);
        #20;
        if (failures == 0)
            $display("PASS: AD/DA path, frequency gate, 247-byte frame, and CRC verified");
        else
            $display("FAIL: %0d self-check(s) failed", failures);
        $finish;
    end

    initial begin
        repeat (30000) @(posedge clk);
        $display("FAIL: simulation timeout");
        $finish;
    end

endmodule
