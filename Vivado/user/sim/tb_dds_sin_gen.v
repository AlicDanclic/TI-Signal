`timescale 1ns / 1ps

module tb_dds_sin_gen();

    // 1. 输入信号定义
    reg        clk;
    reg        rst_n;
    reg [11:0] fre_ctrl;

    // 2. 输出信号定义
    wire [7:0] wave_out;

    // 3. 实例化 DUT（被测模块）
    dds_sin_gen u_dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .fre_ctrl (fre_ctrl),
        .wave_out (wave_out)
    );

    // 4. 时钟生成：50MHz（周期 20ns）
    initial begin
        clk = 0;
        forever #10 clk = ~clk;  // 半周期 10ns
    end

    // 5. 激励行为逻辑
    initial begin
        // 初始化
        rst_n = 0;
        fre_ctrl = 12'd0;
        
        // 释放复位（保持复位 40ns，即 2 个时钟周期）
        #40 rst_n = 1;
        #40;

        // --- 测试阶段 1：低频率（fre_ctrl = 1）---
        // 输出频率 = 50MHz / 4096 ≈ 12.207 kHz
        $display(">> [50MHz] 低频率控制字 K=1（输出约 12.2 kHz）");
        fre_ctrl = 12'd1;
        #400_000; // 观察 400us

        // --- 测试阶段 2：中等频率（fre_ctrl = 500）---
        // 输出频率 = (50MHz * 500) / 4096 ≈ 6.103 MHz
        $display(">> [50MHz] 中等频率控制字 K=500（输出约 6.1 MHz）");
        fre_ctrl = 12'd500;
        #400_000;

        // --- 测试阶段 3：高频（fre_ctrl = 1024）---
        // 输出频率 = (50MHz * 1024) / 4096 = 12.5 MHz
        $display(">> [50MHz] 高频控制字 K=1024（输出 12.5 MHz）");
        fre_ctrl = 12'd1024;
        #400_000;

        // 测试完成
        $display(">> 测试完成");
        $stop;
    end

endmodule