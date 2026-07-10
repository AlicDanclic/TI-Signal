// ***************************************************************************
//                    NDA AND NEED-TO-KNOW REQUIRED
// ***************************************************************************
// Copyright (C) 2022 - 2025 Zynalog Co.Ltd. All rights reserved.
//
// This file contains information that is proprietary to Zynalog
// Co.Ltd ("Zynalog"). The holder of this file shall treat all
// information contained herein as confidential, shall use the
// information only for its intended purpose, and shall not duplicate,
// disclose, or disseminate any of this information in any manner unless
// Zynalog has otherwise provided express, written permission.
// Use of the materials may require a license of intellectual property
// from a third party or from Zynalog. Receipt or possession of this
// file conveys no express or implied licenses to any intellectual
// property rights belonging to Zynalog.
//
// ***************************************************************************
`timescale 1ns/100ps

module si549(
    input           clock,
    input           reset,
    output  reg     scl,
    inout           sda
);

	localparam  cnt_clk_max = 10'd599;
	localparam  data1 = 8'b10101010;

	wire    [7:0]data2[0:10];
	wire    [7:0]data3[0:10];

	assign  data2[0] = 8'd23;
	assign  data2[1] = 8'd24;
	assign  data2[2] = 8'd26;
	assign  data2[3] = 8'd27;
	assign  data2[4] = 8'd28;
	assign  data2[5] = 8'd29;
	assign  data2[6] = 8'd30;
	assign  data2[7] = 8'd31;
	assign  data2[8] = 8'd231;
	assign  data2[9] = 8'd232;
	assign  data2[10] = 8'd233;

	/* 250MHz parameter */
	/*---------------------------*/
	assign  data3[0] = 8'h2C; 
	assign  data3[1] = 8'h0;
	assign  data3[2] = 8'h34;
	assign  data3[3] = 8'h1f;
	assign  data3[4] = 8'h79;
	assign  data3[5] = 8'h15;
	assign  data3[6] = 8'h48;
	assign  data3[7] = 8'h0;
	assign  data3[8] = 8'h14;
	assign  data3[9] = 8'hdf;
	assign  data3[10] = 8'h68;
	/*---------------------------*/

	localparam  idle = 4'd0;
	localparam  s0 = 4'd1;
	localparam  s1 = 4'd2;
	localparam  s2 = 4'd3;
	localparam  s3 = 4'd4;
	localparam  s4 = 4'd5;
	localparam  s5 = 4'd6;
	localparam  s6 = 4'd7;
	localparam  s7 = 4'd8;
	localparam  s8 = 4'd9;

	reg [3:0]state;
	reg [31:0]cnt_wait;
	reg [9:0]cnt_clk;
	reg [2:0]cnt_bit;
	reg [3:0]cnt_all;
	reg sda_reg;

	always @(posedge clock or posedge reset) begin
		if(reset)
			cnt_wait <= 0;
		else if(state == idle)
			cnt_wait <= cnt_wait + 1'b1;
		else if(cnt_clk == 32'd50000)
			cnt_wait <= 0;
		else
			cnt_wait <= 0;
	end

	always @(posedge clock or posedge reset) begin
		if(reset)
			cnt_clk <= 0;
		else if(state == idle)
			cnt_clk <= 0;
		else if(cnt_clk == cnt_clk_max)
			cnt_clk <= 0;
		else
			cnt_clk <= cnt_clk + 1'b1;
	end

	always @(posedge clock or posedge reset) begin
		if(reset)
			cnt_bit <= 0;
		else if((state == idle)||(state == s0)||(state == s2)||(state == s4)||(state == s6)||(state == s7))
			cnt_bit <= 0;
		else if((state != idle)&&(cnt_clk == cnt_clk_max))
			cnt_bit <= cnt_bit + 1'b1;
		else
			cnt_bit <= cnt_bit;
	end

	always @(posedge clock or posedge reset) begin
		if(reset)
			cnt_all <= 0;
		else if((state == s7)&&(cnt_clk == cnt_clk_max))
			cnt_all <= cnt_all + 1'b1;
		else if(cnt_all == 11)
			cnt_all <= 11;
		else
			cnt_all <= cnt_all;
	end

	always @(posedge clock or posedge reset) begin
		if(reset)
			state <= idle;
		else
		begin
			case(state)
			idle:if(cnt_wait == 32'd50000)
					state <= s0;
				else
					state <= idle;
			s0:if(cnt_clk == cnt_clk_max)
					state <= s1;
				else
					state <= s0;
			s1:if((cnt_bit == 7)&&(cnt_clk == cnt_clk_max))
					state <= s2;
				else
					state <= s1;
			s2:if(cnt_clk == cnt_clk_max)
					state <= s3;
				else
					state <= s2;
			s3:if((cnt_bit == 7)&&(cnt_clk == cnt_clk_max))
					state <= s4;
				else
					state <= s3;
			s4:if(cnt_clk == cnt_clk_max)
					state <= s5;
				else
					state <= s4;
			s5:if((cnt_bit == 7)&&(cnt_clk == cnt_clk_max))
					state <= s6;
				else
					state <= s5;
			s6:if(cnt_clk == cnt_clk_max)
					state <= s7;
				else
					state <= s6;
			s7:if(cnt_clk == cnt_clk_max)
					state <= s8;
				else
					state <= s7;
			s8:if(cnt_all == 11)
					state <= s8;
			else
					state <= idle;
			default:state <= idle;
			endcase
		end
	end

	always @(posedge clock or posedge reset) begin
		if(reset)
			scl <= 1;
		else if(state == idle)
			scl <= 1;
		else if(state == s7)
			scl <= 1;
		else if(state == s0)
		begin
			if(cnt_clk == 3*cnt_clk_max/4)
				scl <= ~scl;
			else
				scl <= scl;
		end
		else if(cnt_clk == cnt_clk_max/4)
			scl <= ~scl;
		else if(cnt_clk == 3*cnt_clk_max/4)
			scl <= ~scl;
	end

	always @(posedge clock or posedge reset) begin
		if(reset)
			sda_reg <= 1;
		else if(state == s1)
			sda_reg <= data1[7 - cnt_bit];
		else if(state == s3)
			sda_reg <= data2[cnt_all][7 - cnt_bit];
		else if(state == s5)
			sda_reg <= data3[cnt_all][7 - cnt_bit];
		else if(state == s0)
		begin
			if(cnt_clk == cnt_clk_max/2)
				sda_reg <= 0;
			else
				sda_reg <= sda_reg;
		end
		else if(state == s7)
		begin
			if(cnt_clk >= cnt_clk_max/2)
				sda_reg <= 1;
			else
				sda_reg <= 0;
		end
		else
			sda_reg <= 1;
	end

	assign   sda = ((state == s2)||(state == s4)||(state == s6)) ? 1'bz : sda_reg;

endmodule