`timescale 1ns / 1ps

module tb_sin_rom();

    localparam CLK_PERIOD = 20;       // 50 MHz
    localparam ADDR_WIDTH = 12;
    localparam DEPTH      = 1 << ADDR_WIDTH;   // 4096
    localparam LOOPS      = 100;      // 足够长的循环次数

    reg                  clk;
    reg                  rst_n;
    reg  [ADDR_WIDTH-1:0] addr;
    wire [7:0]           data_out;

    sin_rom u_sin_rom (
        .clka  (clk),
        .addra (addr),
        .douta (data_out)
    );

    always #(CLK_PERIOD/2) clk = ~clk;

    integer i, j;

    initial begin
        clk = 0;
        rst_n = 0;
        addr = 0;
        #100;
        rst_n = 1;

        for (j = 0; j < LOOPS; j = j + 1) begin
            for (i = 0; i < DEPTH; i = i + 1) begin
                @(posedge clk);
                addr = i;
            end
        end

        #1000;
        $finish;
    end

endmodule