/**
 * @file tb_clk_div.v
 * @brief Testbench for clk_div module.
 * @details Generates VCD with only clk_in, rst_n, clk_out signals.
 *          Uses default parameters (50MHz -> 25MHz).
 *          Simulation runs for 1000 clock cycles then finishes.
 */
`timescale 1ns / 1ps

module tb_clk_div;

    // ---------- Signals ----------
    reg  clk_in;
    reg  rst_n;
    wire clk_out;

    // ---------- Instantiate DUT ----------
    // Use default parameters (INPUT_FREQ = 50MHz, OUTPUT_FREQ = 25MHz)
    clk_div uut (
        .clk_in  (clk_in),
        .rst_n   (rst_n),
        .clk_out (clk_out)
    );

    // ---------- Clock generation ----------
    // 50 MHz clock => period = 20 ns
    always #10 clk_in = ~clk_in;

    // ---------- Reset generation ----------
    initial begin
        clk_in = 0;
        rst_n  = 0;          // Assert reset
        #40;                 // Hold for 2 clock cycles
        rst_n  = 1;          // Release reset
    end

    // ---------- VCD dumping (only the three required signals) ----------
    initial begin
        $dumpfile("clk_div.vcd");
        // Dump only the signals listed below (no hierarchy)
        $dumpvars(0, clk_in, rst_n, clk_out);
    end

    // ---------- Simulation control ----------
    initial begin
        // Run for 1000 clock cycles (20 ns each => 20 us)
        #20000;              // 1000 * 20 ns = 20,000 ns
        $display("Simulation finished.");
        $finish;
    end

endmodule