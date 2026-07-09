`timescale 1ns / 1ps

module tb_uart_command_parser;
    // 参数
    localparam CLK_FREQ_HZ = 50_000_000;
    localparam BAUD_RATE   = 115200;
    localparam BIT_CNT     = CLK_FREQ_HZ / BAUD_RATE;  // 434

    // 信号
    reg         clk;
    reg         rst_n;
    reg         uart_rx_i;
    wire [7:0]  TT, AA, BB, CC;
    wire        parser_valid;

    // 实例化被测模块（内部已含 uart_rx）
    uart_command_parser #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .uart_rx_i    (uart_rx_i),
        .TT           (TT),
        .AA           (AA),
        .BB           (BB),
        .CC           (CC),
        .parser_valid (parser_valid)
    );

    // 时钟
    always #10 clk = ~clk;  // 50 MHz

    // ---------- 同步 UART 字节发送任务 ----------
    task send_byte;
        input [7:0] data;
        integer i;
    begin
        uart_rx_i = 1'b0;          // 起始位
        repeat (BIT_CNT) @(posedge clk);
        for (i = 0; i < 8; i = i + 1) begin
            uart_rx_i = data[i];
            repeat (BIT_CNT) @(posedge clk);
        end
        uart_rx_i = 1'b1;          // 停止位
        repeat (BIT_CNT) @(posedge clk);
    end
    endtask

    // ---------- 等待 parser_valid 脉冲（超时保护） ----------
    task wait_parser;
        integer timeout;
    begin
        timeout = 0;
        while (!parser_valid && timeout < 5000) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        @(posedge clk); // 额外等待，但数值会保持
    end
    endtask

    // ---------- 测试序列 ----------
    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        uart_rx_i = 1'b1;
        #100;
        rst_n = 1'b1;
        #200;

        $display("========== Test Start ==========");

        // ---------- 测试 1：标准格式 @3A|01|0F|FF# ----------
        $display("Test 1: @3A|01|0F|FF#");
        send_byte(8'h40); send_byte(8'h33); send_byte(8'h41);
        send_byte(8'h7C); send_byte(8'h30); send_byte(8'h31);
        send_byte(8'h7C); send_byte(8'h30); send_byte(8'h46);
        send_byte(8'h7C); send_byte(8'h46); send_byte(8'h46);
        send_byte(8'h23); // #
        wait_parser();
        if (TT === 8'h3A && AA === 8'h01 && BB === 8'h0F && CC === 8'hFF)
            $display("  PASS (values correct)");
        else
            $display("  FAIL: TT=%h AA=%h BB=%h CC=%h", TT, AA, BB, CC);
        #(BIT_CNT * 20);

        // ---------- 测试 2：新格式（多一个 '|'） @3A|01|0F|FF|# ----------
        $display("Test 2: @3A|01|0F|FF|# (extra pipe)");
        send_byte(8'h40); send_byte(8'h33); send_byte(8'h41);
        send_byte(8'h7C); send_byte(8'h30); send_byte(8'h31);
        send_byte(8'h7C); send_byte(8'h30); send_byte(8'h46);
        send_byte(8'h7C); send_byte(8'h46); send_byte(8'h46);
        send_byte(8'h7C); // 额外的 '|'
        send_byte(8'h23); // #
        wait_parser();
        if (TT === 8'h3A && AA === 8'h01 && BB === 8'h0F && CC === 8'hFF)
            $display("  PASS (extra pipe supported)");
        else
            $display("  FAIL");
        #(BIT_CNT * 20);

        // ---------- 测试 3：小写十六进制 @5b|0a|1f|2c# ----------
        $display("Test 3: @5b|0a|1f|2c#");
        send_byte(8'h40); send_byte(8'h35); send_byte(8'h62);
        send_byte(8'h7C); send_byte(8'h30); send_byte(8'h61);
        send_byte(8'h7C); send_byte(8'h31); send_byte(8'h66);
        send_byte(8'h7C); send_byte(8'h32); send_byte(8'h63);
        send_byte(8'h23);
        wait_parser();
        if (TT === 8'h5B && AA === 8'h0A && BB === 8'h1F && CC === 8'h2C)
            $display("  PASS");
        else
            $display("  FAIL");
        #(BIT_CNT * 20);

        // ---------- 测试 4：缺失 '#'（不应解析） ----------
        $display("Test 4: Missing '#' -> should not assert");
        send_byte(8'h40); send_byte(8'h30); send_byte(8'h30);
        send_byte(8'h7C); send_byte(8'h30); send_byte(8'h30);
        send_byte(8'h7C); send_byte(8'h30); send_byte(8'h30);
        send_byte(8'h7C); send_byte(8'h30); send_byte(8'h30);
        // 无 '#'
        wait_parser();
        if (!parser_valid)
            $display("  PASS");
        else
            $display("  FAIL");
        #(BIT_CNT * 20);

        // ---------- 测试 5：非法字符（'G'） ----------
        $display("Test 5: Invalid hex 'G' -> should not parse");
        send_byte(8'h40); send_byte(8'h47); send_byte(8'h30); // G0
        send_byte(8'h7C); send_byte(8'h30); send_byte(8'h30);
        send_byte(8'h7C); send_byte(8'h30); send_byte(8'h30);
        send_byte(8'h7C); send_byte(8'h30); send_byte(8'h30);
        send_byte(8'h23);
        wait_parser();
        if (!parser_valid)
            $display("  PASS");
        else
            $display("  FAIL");
        #(BIT_CNT * 20);

        $display("========== Test Finished ==========");
        $finish;
    end

    // VCD 波形导出
    initial begin
        $dumpfile("tb_uart_command_parser.vcd");
        $dumpvars(0, tb_uart_command_parser);
    end

endmodule