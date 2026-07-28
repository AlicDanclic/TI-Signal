//****************************************Copyright (c)***********************************//
//原子哥在线教学平台：www.yuanzige.com
//技术支持：http://www.openedv.com/forum.php
//淘宝店铺：https://zhengdianyuanzi.tmall.com
//关注微信公众平台微信号："正点原子"，免费获取ZYNQ & FPGA & STM32 & LINUX资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2023-2033
//All rights reserved                                  
//----------------------------------------------------------------------------------------
// File name:           hs_dual_ad
// Created by:          正点原子
// Created date:        2025年10月8日18:06:00
// Version:             V1.0
// Descriptions:        双路AD实验顶层模块
//
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module hs_dual_ad(
    input          sys_clk    ,  //系统时钟
    input          sys_rst_n  ,  //系统复位
    //第一路ADC
    input   [9:0]  ad_data_1  ,  //第一路ADC数据
    input          ad_otr_1   ,  //第一路ADC输入电压超过量程标志
    output         ad_clk_1   ,  //第一路ADC驱动时钟
    output         ad_oe_1    ,  //第一路ADC输出使能
    //第二路ADC
    input   [9:0]  ad_data_2  ,  //第二路ADC数据
    input          ad_otr_2   ,  //第二路ADC输入电压超过量程标志
    output         ad_clk_2   ,  //第二路ADC驱动时钟
    output         ad_oe_2       //第二路ADC输出使能
    );

//wire define 
wire             clk_50m          ;  //50MHz时钟
wire             clk_50m_deg180   ;  //相位偏移180的50MHz时钟

//*****************************************************
//**                    main code
//*****************************************************

assign  ad_oe_1 =  1'b0;
assign  ad_oe_2 =  1'b0;
assign  ad_clk_1 = clk_50m;
assign  ad_clk_2 = clk_50m;

//pll
clk_wiz_0 u_clk_wiz_0(
    .clk_out1    (clk_50m       ),     
    .clk_out2    (clk_50m_deg180), 
    .clk_in1     (sys_clk       )
    );      

ila_0 u_ila_0 (
    .clk     (clk_50m_deg180 ),  // input wire clk
    .probe0  (ad_otr_1       ),  // input wire [0:0]  probe0  
    .probe1  (ad_data_1      ),  // input wire [9:0]  probe1
    .probe2  (ad_otr_2       ),  // input wire [0:0]  probe2  
    .probe3  (ad_data_2      )   // input wire [9:0]  probe3
);

endmodule
