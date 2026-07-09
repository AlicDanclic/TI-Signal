`timescale 1ns / 1ps

module tb_uart_cmd_sender();
    // 参数
    localparam CLK_FREQ_HZ = 50_000_000;
    localparam BAUD_RATE   = 115200;
    localparam BIT_TIME    = 1_000_000_000 / BAUD_RATE;
    localparam BIT_CNT     = CLK_FREQ_HZ / BAUD_RATE;

    // 信号
    reg         clk;
    reg         rst_n;
    reg  [7:0]  tt, aa, bb, cc;
    reg         start;
    wire        uart_tx;
    wire        done;

    // 测试用变量（模块级）
    reg [7:0] expected [0:13];
    reg [7:0] rx_byte;
    integer i, err_cnt;

    // 实例化 DUT
    uart_cmd_sender #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) dut (
        .clk     (clk),
        .rst_n   (rst_n),
        .tt      (tt),
        .aa      (aa),
        .bb      (bb),
        .cc      (cc),
        .start   (start),
        .uart_tx (uart_tx),
        .done    (done)
    );

    // 时钟生成
    always #10 clk = ~clk;

    // UART 接收任务（用于仿真验证）
    task receive_byte;
        output [7:0] data;
        integer j;
    begin
        @(negedge uart_tx);               // 等待起始位
        #(BIT_TIME / 2);                  // 移到中间采样
        data = 0;
        for (j = 0; j < 8; j = j + 1) begin
            #(BIT_TIME);
            data[j] = uart_tx;            // 采样数据位
        end
        #(BIT_TIME);                      // 跳过停止位
    end
    endtask

    // 测试序列
    initial begin
        // 初始化
        clk = 1'b0;
        rst_n = 1'b0;
        tt = 8'h00;
        aa = 8'h00;
        bb = 8'h00;
        cc = 8'h00;
        start = 1'b0;

        #100;
        rst_n = 1'b1;
        #200;

        $display("========== Test Start ==========");

        // ---------- 测试 1：发送 @3A|01|0F|FF|# ----------
        $display("Test 1: Send @3A|01|0F|FF|#");
        tt = 8'h3A;
        aa = 8'h01;
        bb = 8'h0F;
        cc = 8'hFF;
        start = 1'b1;
        @(posedge clk);
        start = 1'b0;

        wait (done == 1'b1);
        #(BIT_TIME * 2);  // 等待最后一个字节完全发出

        expected[0]  = 8'h40;
        expected[1]  = 8'h33;
        expected[2]  = 8'h41;
        expected[3]  = 8'h7C;
        expected[4]  = 8'h30;
        expected[5]  = 8'h31;
        expected[6]  = 8'h7C;
        expected[7]  = 8'h30;
        expected[8]  = 8'h46;
        expected[9]  = 8'h7C;
        expected[10] = 8'h46;
        expected[11] = 8'h46;
        expected[12] = 8'h7C;
        expected[13] = 8'h23;

        err_cnt = 0;
        for (i = 0; i < 14; i = i + 1) begin
            receive_byte(rx_byte);
            if (rx_byte !== expected[i]) begin
                $display("  Byte %0d mismatch: expected 0x%h, got 0x%h", i, expected[i], rx_byte);
                err_cnt = err_cnt + 1;
            end
        end

        if (err_cnt == 0)
            $display("  PASS: All bytes match");
        else
            $display("  FAIL: %0d byte(s) mismatched", err_cnt);

        #(BIT_TIME * 20);

        // ---------- 测试 2：发送 @5B|0A|1F|2C|# ----------
        $display("Test 2: Send @5B|0A|1F|2C|#");
        tt = 8'h5B;
        aa = 8'h0A;
        bb = 8'h1F;
        cc = 8'h2C;
        start = 1'b1;
        @(posedge clk);
        start = 1'b0;

        wait (done == 1'b1);
        #(BIT_TIME * 2);

        expected[0]  = 8'h40;
        expected[1]  = 8'h35;
        expected[2]  = 8'h42;
        expected[3]  = 8'h7C;
        expected[4]  = 8'h30;
        expected[5]  = 8'h41;
        expected[6]  = 8'h7C;
        expected[7]  = 8'h31;
        expected[8]  = 8'h46;
        expected[9]  = 8'h7C;
        expected[10] = 8'h32;
        expected[11] = 8'h43;
        expected[12] = 8'h7C;
        expected[13] = 8'h23;

        err_cnt = 0;
        for (i = 0; i < 14; i = i + 1) begin
            receive_byte(rx_byte);
            if (rx_byte !== expected[i]) begin
                $display("  Byte %0d mismatch: expected 0x%h, got 0x%h", i, expected[i], rx_byte);
                err_cnt = err_cnt + 1;
            end
        end

        if (err_cnt == 0)
            $display("  PASS: All bytes match");
        else
            $display("  FAIL: %0d byte(s) mismatched", err_cnt);

        #(BIT_TIME * 20);

        $display("========== Test Finished ==========");
        $finish;
    end

    // VCD 波形导出
    initial begin
        $dumpfile("tb_uart_cmd_sender.vcd");
        $dumpvars(0, tb_uart_cmd_sender);
    end

endmodule