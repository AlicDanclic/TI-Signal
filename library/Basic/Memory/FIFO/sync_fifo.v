`timescale 1ns / 1ps

/*
*   Date : 2024-07-02
*   Author : cjh
*   Module Name:   sync_fifo.v - sync_fifo
*   Target Device: [Target FPGA and ASIC Device]
*   Tool versions: vivado 18.3 & DC 2016
*   Revision Historyc :
*   Revision :
*       Revision 0.01 - File Created
        Revision 0.02 - upload a stable version
*   Description : The synchronous dual-port SRAM has A, B ports to access the same memory location. 
*                 Both ports can be independently read or written from the memory array.
*                 1. In Vivado, EDA can directly use BRAM for synthesis.
*                 2. The module continuously outputs data when enabled, and when disabled, 
*                    it outputs the last data.
*                 3. When writing data to the same address on ports A and B simultaneously, 
*                    the write operation from port B will take precedence.
*                 4. In write mode, the current data input takes precedence for writing, 
*                    and the data from the address input at the previous clock cycle is read out. 
*                    In read mode, the data from the address input at the current clock cycle 
*                    is directly read out. In write mode, when writing to different addresses, 
*                    the data corresponding to the current address input at the current clock cycle 
*                    is directly read out.
*   Dependencies: none(FPGA) auto for BRAM in vivado | RAM_IP with IC 
*   Company : ncai Technology .Inc
*   Copyright(c) 1999, ncai Technology Inc, All right reserved
*/

/* @wavedrom
{signal: [
  {name: 'clock', wave: '010101010101010101'},
  {name: 'reset', wave: '1.0...............'},
  {name: 'wr_en', wave: '01...0...'},
  {name: 'wr_ready', wave: 'x3...3.x.', data: ['addr0','addr2']},
  {name: 'din', wave: 'x4.4.x...', data: ['data0','data1']},
  {name: 'rd_en', wave: 'x..5.5.x.', data: ['data0','data2']},
  {name: 'valid', wave: 'x..5.5.x.', data: ['data0','data2']},
  {name: 'dout', wave: 'x..5.5.x.', data: ['data0','data2']},
  {name: 'full', wave: 'x..5.5.x.', data: ['data0','data2']},
  {name: 'empty', wave: 'x..5.5.x.', data: ['data0','data2']},
  {name: 'wr_data_count', wave: 'x..5.5.x.', data: ['data0','data2']},
  {name: 'wr_data_space', wave: 'x..5.5.x.', data: ['data0','data2']},
  {name: 'rd_data_count', wave: 'x..5.5.x.', data: ['data0','data2']},
  {name: 'rd_data_space', wave: 'x..5.5.x.', data: ['data0','data2']},
]}
*/

module sync_fifo #(
    //写数据位宽参数
    parameter        INPUT_WIDTH       = 128,
    // 读数据位宽参数
    parameter        OUTPUT_WIDTH      = 16,
    // 写内存深度参数,if INPUT_WIDTH < OUTPUT_WIDTH, WR_DEPTH = (OUTPUT_WIDTH/INPUT_WIDTH) * RD_DEPTH
    parameter        WR_DEPTH          = 1024,
    // 读内存深度参数,if INPUT_WIDTH > OUTPUT_WIDTH, RD_DEPTH = (INPUT_WIDTH/OUTPUT_WIDTH) * WR_DEPTH
    parameter        RD_DEPTH          = 8192,
    // 读取方式参数
    parameter        MODE              = "FWFT",
    // 数据从高位还是低位存储
    parameter        DIRECTION         = "MSB",
    // 设置纠错功能,INPUT_WIDTH must equal to OUTPUT_WIDTH
    parameter        ECC_MODE          = "no_ecc",
    //Specify the value of the programmable null threshold
    parameter        PROG_EMPTY_THRESH = 10,
    //Specify the value of programmable full threshold
    parameter        PROG_FULL_THRESH  = 10,
    //Enable the corresponding signal
    parameter [15:0] USE_ADV_FEATURES  = 16'h1F1F
) (

    //系统时钟输入
    input                              clock,
    //系统复位，高电平有效，无同步操作
    input                              reset,

    //写端口使能，高电平有效
    input                              wr_en,
    //写端口准备好接收数据
    output                             wr_ready,
    //写端口数据输入
    input   [INPUT_WIDTH - 1 : 0]      din,

    //读端口

    //读端口使能，高电平有效
    input                              rd_en,
    //读端口数据有效，高电平有效
    output  reg                        valid,
    //读端口数据输出
    output  reg [OUTPUT_WIDTH - 1 : 0] dout,

    output                             full,
    output                             empty,

    //写端口输入数据数量
    output  reg [$clog2(WR_DEPTH) : 0] wr_data_count,
    //写端口剩余空间
    output      [$clog2(WR_DEPTH) : 0] wr_data_space,
    //读端口输出数据数量
    output  reg [$clog2(RD_DEPTH) : 0] rd_data_count,
    //读端口剩余数据数量
    output      [$clog2(RD_DEPTH) : 0] rd_data_space,
    //fifo将满，只能再写入一次，写入后fifo变满
    output                             almost_full,
    //fifo将空，只能再读取一次，读取后fifo变空
    output                             almost_empty,
    //fifo中数据量大于等于可编程满阈值时，信号拉高
    output                             prog_full,
    //fifo中数据量小于等于可编程空阈值时，信号拉高
    output                             prog_empty,
    //上一周期的写请求被拒绝，因为fifo已满
    output  reg                        overflow,
    //上一周期的读请求被拒绝，因为fifo已空
    output  reg                        underflow,
    //上一周期的写请求成功
    output                             wr_ack,
    //ECC解码器检测到单比特错误并已纠正
    output                             sbiterr,
    //ECC解码器检测到双比特错误，FIFO中的数据已损坏
    output                             dbiterr
);

//当ECC模式使能时，ECC编码数据位宽
localparam DW = INPUT_WIDTH;

//当ECC模式使能时，ECC编码校验位宽
localparam PW = $clog2(1+DW+$clog2(1+DW));

//需要操作的RAM数量
localparam RAM_NUM      = (INPUT_WIDTH >= OUTPUT_WIDTH) ? (INPUT_WIDTH/OUTPUT_WIDTH) : (OUTPUT_WIDTH/INPUT_WIDTH);

//每个操作的RAM位宽
localparam RAM_WIDTH    = (ECC_MODE == "en_ecc") ? (DW+PW+1) : (INPUT_WIDTH >= OUTPUT_WIDTH) ? OUTPUT_WIDTH : INPUT_WIDTH;

//每个操作的RAM深度
localparam RAM_DEPTH    = (INPUT_WIDTH >= OUTPUT_WIDTH) ? WR_DEPTH : (WR_DEPTH/RAM_NUM);

//ram写数据位宽
localparam RAM_WR_WIDTH = (ECC_MODE == "en_ecc") ? (DW+PW+1) : INPUT_WIDTH;

//ram读数据位宽
localparam RAM_RD_WIDTH = (ECC_MODE == "en_ecc") ? (DW+PW+1) : (INPUT_WIDTH >= OUTPUT_WIDTH) ? INPUT_WIDTH : OUTPUT_WIDTH;

//读写指针二进制计数器
reg  [$clog2(WR_DEPTH) : 0]      wr_ptr;         //标记fifo写入位置

reg  [$clog2(RD_DEPTH) : 0]      rd_ptr;         //标记fifo读取位置

reg  [$clog2(RD_DEPTH) : 0]      rd_ptr_next;    //标记fifo读取下一位置

reg  [$clog2(RD_DEPTH) : 0]      rd_ptr_pre;    //标记预读取前fifo中数据位置
reg  [$clog2(RD_DEPTH) : 0]      rd_ptr_fwft;   //fwft模式下，ram地址变化需使用rd_ptr_fwft

//DPRAM端口
wire [RAM_NUM - 1 : 0]           ram_wr_en;     //DPRAM写使能信号
wire [RAM_NUM - 1 : 0]           ram_rd_en;     //DPRAM读使能信号

wire [$clog2(RAM_DEPTH)     : 0] ram_wr_addr;   //DPRAM写地址
wire [$clog2(RAM_DEPTH)     : 0] ram_rd_addr;   //DPRAM读地址

wire [RAM_WR_WIDTH      - 1 : 0] ram_wr_data;   //DPRAM写数据
wire [RAM_RD_WIDTH      - 1 : 0] ram_rd_data;   //DPRAM读数据

//数据计数
wire [$clog2(RD_DEPTH) : 0]      rd_addr;           //标记fifo读取位置

reg                              empty_d1;         // 空标志缓冲
reg                              pre_valid;        // 记录预读取后正式读取前的周期
wire                             pre_read;         // fwft模式下,pre-acquire data from fifo

//写端口准备好接收数据
assign wr_ready = ~full;

//写端口剩余空间
assign wr_data_space = WR_DEPTH - wr_data_count;

//读端口剩余数据数量
assign rd_data_space = RD_DEPTH - rd_data_count;

//mark the position of writing,use binary counter
always @(posedge clock or posedge reset) begin
    if(reset == 1'b1)begin
        wr_ptr <= 'd0;
    end
    else if(wr_en & (~full))begin
        wr_ptr <= wr_ptr + 1'b1;
    end
    else begin
        wr_ptr <= wr_ptr;
    end
end

//when in standard mode,mark the position of reading,use binary  counter
always @(posedge clock or posedge reset) begin
    if(reset == 1'b1)begin
        rd_ptr <= 'd0;
    end
    else if(rd_en & (~empty))begin
        rd_ptr <= rd_ptr + 1'b1;
    end
    else begin
        rd_ptr <= rd_ptr;
    end
end

// fwft模式下,rd_ptr need to add one to pre-extract from fifo,use binary counter
always @(posedge clock or posedge reset) begin
    if(reset == 1'b1)begin
        rd_ptr_next <= 'd0;
    end
    else if((rd_en | pre_read) & (~empty))begin
        rd_ptr_next <= rd_ptr_next + 1'b1;
    end
    else begin
        rd_ptr_next <= rd_ptr_next;
    end
end

// 记录预读取后正式读取前的周期
always @(posedge clock or posedge reset) begin
    if(reset == 1'b1)begin
        pre_valid <= 1'b0;
    end
    else if(rd_en)begin
        pre_valid <= 1'b0;
    end
    else if(pre_read)begin
        pre_valid <= 1'b1;
    end
    else begin
        pre_valid <= pre_valid;
    end
end

//mark the position of data pre-read from fifo
always @(posedge clock or posedge reset) begin
    if(reset == 1'b1)begin
        rd_ptr_pre <= 'd0;
    end
    else if(pre_read)begin
        rd_ptr_pre <= rd_ptr_next;
    end
    else begin
        rd_ptr_pre <= rd_ptr_pre;
    end
end

//fwft模式下，ram地址变化需使用rd_ptr_fwft
always @(*) begin
    //when reading formally,rd_ptr need to add one to pre-extract from fifo
    if(rd_en)begin
        rd_ptr_fwft = rd_ptr_next;
    end
    else begin
        //before formal reading after pre reading,read pre fetched data from FIFO
        if(pre_valid)begin
            rd_ptr_fwft = rd_ptr_pre;
        end
        //When it is not officially read and the FIFO is not empty, there is no need to read it in advance
        else begin
            rd_ptr_fwft = rd_ptr_next;
        end
    end
end

//while fifo is not empty,pre-read data from fifo
assign pre_read = (~empty) & empty_d1;

always @(posedge clock or posedge reset) begin
    if(reset == 1'b1)begin
        empty_d1 <= 1'b1;
    end
    else begin
        empty_d1 <= empty;
    end
end

genvar i;

//ram_rd_en and valid
generate if(MODE == "Standard") begin : standard_mode_read

    always @(posedge clock or posedge reset) begin
        if(reset == 1'b1)begin
            valid <= 1'b0;
        end
        else begin
            valid <= rd_en & (~empty);
        end
    end

end
endgenerate
//assign valid = (reset) ? 1'b0 :  ~empty_d1;
generate if(MODE == "FWFT") begin : fwft_mode_read

always @(*) begin

        valid = ~empty_d1;
    end
end
endgenerate

generate if(INPUT_WIDTH >= OUTPUT_WIDTH) begin : BIG_TO_SMALL_RAM

    reg  [RAM_NUM - 1 : 0]      ram_sel;    // 选择哪个ram的读数据输出
    wire [DW - 1 : 0]           decode_data;// ECC解码输出

    if(RAM_NUM >= 2)begin

        if(MODE == "Standard")begin
            
            always @(posedge clock or posedge reset) begin
                if(reset == 1'b1)begin
                    ram_sel[RAM_NUM - 1]     <= 1'b1;
                    ram_sel[RAM_NUM - 2 : 0] <= 'd0;
                end
                else if(rd_en & (~empty))begin
                    ram_sel <= {ram_sel[RAM_NUM - 2 : 0],ram_sel[RAM_NUM - 1]};
                end
                else begin
                    ram_sel <= ram_sel;
                end
            end

        end
        else if(MODE == "FWFT")begin
            
            always @(posedge clock or posedge reset) begin
                if(reset == 1'b1)begin
                    ram_sel <= 1;
                end
                else if(rd_en & (~empty))begin
                    ram_sel <= {ram_sel[RAM_NUM - 2 : 0],ram_sel[RAM_NUM - 1]};
                end
                else begin
                    ram_sel <= ram_sel;
                end
            end

        end

    end

    assign ram_wr_addr = wr_ptr;

    for(i=0;i<RAM_NUM;i=i+1)begin : B2S_DUAL_PORT_RAM

        assign ram_wr_en[i] = wr_en & (~full);

        if(ECC_MODE == "no_ecc")begin
            assign ram_wr_data = din;
            assign sbiterr     = 1'b0;
            assign dbiterr     = 1'b0;
        end
        else if(ECC_MODE == "en_ecc")begin
            // ECC编码r
            ecc_encode #(
                .DW     (DW),
                .PW     (PW)) 
            u_ecc_encode(
                .data_i (din),
                .data_o (ram_wr_data));

            // ECC解码r
                ecc_decode #(
                    .DW (DW),
                    .PW (PW)) 
                u_ecc_decode(
                    .data_i  (ram_rd_data),
                    .data_o  (decode_data),
                    .sbiterr (sbiterr),
                    .dbiterr (dbiterr));
        end

        if(MODE == "Standard")begin
            assign ram_rd_en[i] = rd_en & (~empty);
        end
        else if(MODE == "FWFT")begin
            assign ram_rd_en[i] = (rd_en | pre_read) & (~empty); // fwft模式下需从fifo预读取 when fifo is not empty
        end

        if(DIRECTION == "LSB")begin

            DPRAM #(
	            .WIDTH 	( RAM_WIDTH  ),
	            .DEPTH 	( RAM_DEPTH  ))
            u_DPRAM(
	            .clka  	( clock       ),
	            .ena   	( 1'b1        ),
	            .wea   	( ram_wr_en[i]),
	            .addra 	( ram_wr_addr[$clog2(RAM_DEPTH) - 1 : 0]),
	            .dina  	( ram_wr_data[(i+1) * RAM_WIDTH - 1 : i * RAM_WIDTH]),
	            .douta 	(             ),
	            .clkb  	( clock       ),
	            .enb   	( ram_rd_en[i]),
	            .web   	( 1'b0        ),
	            .addrb 	( ram_rd_addr[$clog2(RAM_DEPTH) - 1 : 0]),
	            .dinb  	( {RAM_WIDTH{1'b1}}),
	            .doutb 	( ram_rd_data[(i+1) * RAM_WIDTH - 1 : i * RAM_WIDTH]));
        end
        else if(DIRECTION == "MSB")begin

            DPRAM #(
	            .WIDTH 	( RAM_WIDTH  ),
	            .DEPTH 	( RAM_DEPTH  ))
            u_DPRAM(
	            .clka  	( clock       ),
	            .ena   	( 1'b1        ),
	            .wea   	( ram_wr_en[i]),
	            .addra 	( ram_wr_addr[$clog2(RAM_DEPTH) - 1 : 0]),
	            .dina  	( ram_wr_data[(i+1) * RAM_WIDTH - 1 : i * RAM_WIDTH]),
	            .douta 	(             ),
	            .clkb  	( clock       ),
	            .enb   	( ram_rd_en[i]),
	            .web   	( 1'b0        ),
	            .addrb 	( ram_rd_addr[$clog2(RAM_DEPTH) - 1 : 0]),
	            .dinb  	( {RAM_WIDTH{1'b1}}),
	            .doutb 	( ram_rd_data[RAM_RD_WIDTH - i * RAM_WIDTH - 1 : RAM_RD_WIDTH - (i+1) * RAM_WIDTH]));
        end
    end

    //ram_rd_addr
    if(MODE == "Standard")begin
        assign ram_rd_addr = rd_ptr >> $clog2(RAM_NUM);
    end
    else if(MODE == "FWFT")begin
        assign ram_rd_addr = rd_ptr_fwft >> $clog2(RAM_NUM);
    end

    //rd_addr
    if(MODE == "Standard")begin
        assign rd_addr = rd_ptr;
    end
    else if(MODE == "FWFT")begin
        assign rd_addr = rd_ptr_fwft;
    end

    //rd_data
    case(RAM_NUM)
        4'd1:begin
            if(ECC_MODE == "no_ecc")begin
             always @(*) begin
                    dout = ram_rd_data;
                end
            end
            else if(ECC_MODE == "en_ecc")begin
              always @(*) begin
                    dout = decode_data;
                end
            end
        end

        4'd2:begin
            always @(*) begin
                case(ram_sel)
                    2'b01 : dout = ram_rd_data[OUTPUT_WIDTH - 1 : 0];
                    2'b10 : dout = ram_rd_data[OUTPUT_WIDTH * 2 - 1 : OUTPUT_WIDTH];
                    default:begin
                        dout = ram_rd_data[OUTPUT_WIDTH - 1 : 0];
                    end
                endcase
            end
        end
        
        4'd4:begin
            always @(*) begin
                case(ram_sel)
                    4'b0001 : dout = ram_rd_data[OUTPUT_WIDTH - 1 : 0];
                    4'b0010 : dout = ram_rd_data[OUTPUT_WIDTH * 2 - 1 : OUTPUT_WIDTH];
                    4'b0100 : dout = ram_rd_data[OUTPUT_WIDTH * 3 - 1 : OUTPUT_WIDTH * 2];
                    4'b1000 : dout = ram_rd_data[OUTPUT_WIDTH * 4 - 1 : OUTPUT_WIDTH * 3];
                    default : begin
                              dout = ram_rd_data[OUTPUT_WIDTH - 1 : 0];
                    end
                endcase
            end
        end
          
        4'd8:begin
            always @(*) begin
                case(ram_sel)
                    8'b0000_0001 : dout = ram_rd_data[OUTPUT_WIDTH - 1 : 0];
                    8'b0000_0010 : dout = ram_rd_data[OUTPUT_WIDTH * 2 - 1 : OUTPUT_WIDTH];
                    8'b0000_0100 : dout = ram_rd_data[OUTPUT_WIDTH * 3 - 1 : OUTPUT_WIDTH * 2];
                    8'b0000_1000 : dout = ram_rd_data[OUTPUT_WIDTH * 4 - 1 : OUTPUT_WIDTH * 3];
                    8'b0001_0000 : dout = ram_rd_data[OUTPUT_WIDTH * 5 - 1 : OUTPUT_WIDTH * 4];
                    8'b0010_0000 : dout = ram_rd_data[OUTPUT_WIDTH * 6 - 1 : OUTPUT_WIDTH * 5];
                    8'b0100_0000 : dout = ram_rd_data[OUTPUT_WIDTH * 7 - 1 : OUTPUT_WIDTH * 6];
                    8'b1000_0000 : dout = ram_rd_data[OUTPUT_WIDTH * 8 - 1 : OUTPUT_WIDTH * 7];
                    default:begin
                                   dout = ram_rd_data[OUTPUT_WIDTH - 1 : 0];
                    end
                endcase
            end
        end
    endcase

    //wr_data_count
    always @(posedge clock or posedge reset) begin
        if(reset == 1'b1)begin
            wr_data_count <= 'd0;
        end
        else if(ram_wr_addr[$clog2(RAM_DEPTH)] ^ ram_rd_addr[$clog2(RAM_DEPTH)])begin
            wr_data_count <= {1'b1,ram_wr_addr[$clog2(RAM_DEPTH) - 1 : 0]} - {1'b0,ram_rd_addr[$clog2(RAM_DEPTH) - 1 : 0]};
        end
        else begin
            wr_data_count <= ram_wr_addr - ram_rd_addr;
        end
    end

    //rd_data_count
    always @(posedge clock or posedge reset) begin
        if(reset == 1'b1)begin
            rd_data_count <= 'd0;
        end
        else if(ram_wr_addr[$clog2(RAM_DEPTH)] ^ ram_rd_addr[$clog2(RAM_DEPTH)])begin
            rd_data_count <= RAM_NUM * {1'b1,ram_wr_addr[$clog2(RAM_DEPTH) - 1 : 0]} - {1'b0,rd_addr[$clog2(RD_DEPTH) - 1 : 0]};
        end
        else begin
            rd_data_count <= RAM_NUM * ram_wr_addr - rd_addr;
        end
    end

end
endgenerate
                                
generate if(INPUT_WIDTH < OUTPUT_WIDTH) begin : SMALL_TO_BIG_RAM
    
    reg  [RAM_NUM - 1 : 0]           wr_en_d1;      // 当input_width<output_width时,enable each memory sequentially

    always @(posedge clock or posedge reset) begin
        if(reset == 1'b1)begin
            wr_en_d1 <= 'd1;
        end
        else if(wr_en)begin
            wr_en_d1 <= {wr_en_d1[RAM_NUM - 2 : 0],wr_en_d1[RAM_NUM - 1]};
        end
        else begin
            wr_en_d1 <= wr_en_d1;
        end
    end

    assign ram_wr_addr = wr_ptr >> $clog2(RAM_NUM);

    for(i=0;i<RAM_NUM;i=i+1)begin : S2B_DUAL_PORT_RAM

        assign ram_wr_en[i] = wr_en & wr_en_d1[i] & (~full);

        if(MODE == "Standard")begin
            assign ram_rd_en[i] = rd_en & (~empty);
        end
        else if(MODE == "FWFT")begin
            assign ram_rd_en[i] = (rd_en | pre_read) & (~empty); // fwft模式下需从fifo预读取 when fifo is not empty
        end

        if(DIRECTION == "LSB")begin
            DPRAM #(
	            .WIDTH 	( RAM_WIDTH  ),
	            .DEPTH 	( RAM_DEPTH  ))
            u_DPRAM(
	            .clka  	( clock       ),
	            .ena   	( 1'b1        ),
	            .wea   	( ram_wr_en[i]),
	            .addra 	( ram_wr_addr[$clog2(RAM_DEPTH) - 1 : 0]),
	            .dina  	( din         ),
	            .douta 	(             ),
	            .clkb  	( clock       ),
	            .enb   	( ram_rd_en[i]),
	            .web   	( 1'b0        ),
	            .addrb 	( ram_rd_addr[$clog2(RAM_DEPTH) - 1 : 0]),
	            .dinb  	( {RAM_WIDTH{1'b1}}),
	            .doutb 	( ram_rd_data[(i+1) * RAM_WIDTH - 1 : i * RAM_WIDTH]));
        end
        else if(DIRECTION == "MSB")begin
            DPRAM #(
	            .WIDTH 	( RAM_WIDTH  ),
	            .DEPTH 	( RAM_DEPTH  ))
            u_DPRAM(
	            .clka  	( clock       ),
	            .ena   	( 1'b1        ),
	            .wea   	( ram_wr_en[i]),
	            .addra 	( ram_wr_addr[$clog2(RAM_DEPTH) - 1 : 0]),
	            .dina  	( din         ),
	            .douta 	(             ),
	            .clkb  	( clock       ),
	            .enb   	( ram_rd_en[i]),
	            .web   	( 1'b0        ),
	            .addrb 	( ram_rd_addr[$clog2(RAM_DEPTH) - 1 : 0]),
	            .dinb  	( {RAM_WIDTH{1'b1}}),
	            .doutb 	( ram_rd_data[RAM_RD_WIDTH - i * RAM_WIDTH - 1 : RAM_RD_WIDTH - (i+1) * RAM_WIDTH]));
        end
    end

    //ram_rd_addr
    if(MODE == "Standard")begin
        assign ram_rd_addr = rd_ptr;
    end
    else if(MODE == "FWFT")begin
        always @(*) begin
            rd_ptr_fwft;
        end
    end

    //rd_addr
    if(MODE == "Standard")begin
        assign rd_addr = rd_ptr;
    end
    else if(MODE == "FWFT")begin
        assign rd_addr = rd_ptr_fwft;
    end

    //rd_data
    assign dout = ram_rd_data;

    //wr_data_count
    always @(posedge clock or posedge reset) begin
        if(reset == 1'b1)begin
            wr_data_count <= 'd0;
        end
        else if(ram_wr_addr[$clog2(RAM_DEPTH)] ^ ram_rd_addr[$clog2(RAM_DEPTH)])begin
            wr_data_count <= {1'b1,wr_ptr[$clog2(WR_DEPTH) - 1 : 0]} - RAM_NUM * {1'b0,ram_rd_addr[$clog2(RAM_DEPTH) - 1 : 0]};
        end
        else begin
            wr_data_count <= wr_ptr - RAM_NUM * ram_rd_addr;
        end
    end

    //rd_data_count
    always @(posedge clock or posedge reset) begin
        if(reset == 1'b1)begin
            rd_data_count <=  'd0;
        end
        else if(ram_wr_addr[$clog2(RAM_DEPTH)] ^ ram_rd_addr[$clog2(RAM_DEPTH)])begin
            rd_data_count <= {1'b1,ram_wr_addr[$clog2(RAM_DEPTH) - 1 : 0]} - {1'b0,rd_addr[$clog2(RD_DEPTH) - 1 : 0]};
        end
        else begin
            rd_data_count <= ram_wr_addr - rd_addr;
        end
    end
    
end
endgenerate

//full and empty operation
assign full  = (ram_wr_addr - ram_rd_addr == RAM_DEPTH) ? 1'b1 : 1'b0;

assign empty = (ram_wr_addr == ram_rd_addr) ? 1'b1 : 1'b0;

//fifo is about to be full
generate if(USE_ADV_FEATURES[3] == 1'b1) begin : ALMOST_FULL_ENABLE

    assign almost_full = (ram_wr_addr - ram_rd_addr >= RAM_DEPTH - 1) ? 1'b1 : 1'b0;

end
endgenerate

generate if(USE_ADV_FEATURES[3] == 1'b0) begin : ALMOST_FULL_DISABLE

    assign almost_full = 1'b0;

end
endgenerate

//fifo is about to be empty
generate if(USE_ADV_FEATURES[11] == 1'b1) begin : ALMOST_EMPTY_ENABLE

    assign almost_empty = (ram_wr_addr - ram_rd_addr <= 1) ? 1'b1 : 1'b0;

end
endgenerate

generate if(USE_ADV_FEATURES[11] == 1'b0) begin : ALMOST_EMPTY_DISABLE

    assign almost_empty = 1'b0;

end
endgenerate

// fifo中数据量大于等于 the programmable full threshold
generate if(USE_ADV_FEATURES[1] == 1'b1) begin : PROG_FULL_ENABLE

    assign prog_full = (wr_data_count >= PROG_FULL_THRESH) ? 1'b1 : 1'b0;

end
endgenerate

generate if(USE_ADV_FEATURES[1] == 1'b0) begin : PROG_FULL_DISABLE
    
    assign prog_full = 1'b0;

end
endgenerate

// fifo中数据量小于等于 the programmable null threshold
generate if(USE_ADV_FEATURES[9] == 1'b1) begin : PROG_EMPTY_ENABLE
    
    assign prog_empty = (rd_data_count <= PROG_EMPTY_THRESH) ? 1'b1 : 1'b0;

end
endgenerate

generate if(USE_ADV_FEATURES[9] == 1'b0) begin : PROG_EMPTY_DISABLE
    
    assign prog_empty = 1'b0;

end
endgenerate

// 上一周期写请求被拒绝 because the FIFO is now full
generate if(USE_ADV_FEATURES[0] == 1'b1) begin : OVERFLOW_ENABLE
    
    always @(posedge clock or posedge reset) begin
        if(reset == 1'b1)begin
            overflow <= 1'b0;
        end
        else if(full)begin
            overflow <= wr_en;
        end
        else begin
            overflow <= 1'b0;
        end
    end

end
endgenerate

generate if(USE_ADV_FEATURES[0] == 1'b0) begin : OVERFLOW_DISABLE
    
    always @(*) begin
        overflow = 1'b0;
    end
end
endgenerate

// 上一周期读请求被拒绝 because the FIFO is now empty
generate if(USE_ADV_FEATURES[8] == 1'b1) begin : UNDERFLOW_ENABLE
    
    always @(posedge clock or posedge reset) begin
        if(reset == 1'b1)begin
            underflow <= 1'b0;
        end
        else if(empty)begin
            underflow <= rd_en;
        end
        else begin
            underflow <= 1'b0;
        end
    end

end
endgenerate

generate if(USE_ADV_FEATURES[8] == 1'b0) begin : UNDERFLOW_DISABLE
    
    always @(*) begin
        underflow = 1'b0;
    end

end
endgenerate

//上一周期的写请求成功
generate if(USE_ADV_FEATURES[4] == 1'b1) begin : WR_ACK_ENABLE
    
    assign wr_ack = wr_en & (~full);

end
endgenerate

generate if(USE_ADV_FEATURES[4] == 1'b0) begin : WR_ACK_DISABLE
    
    assign wr_ack = 1'b0;

end
endgenerate

endmodule  //sync_fifo
