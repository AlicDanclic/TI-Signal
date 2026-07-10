// ZGAD250D14配置模块
// 正点原子双路250MSPS ADC模块的SPI寄存器配置

module  ZGAD250D14_cfg(
        input           clock,
        input           reset,
        output  reg     spi_cs,
        output  reg     spi_clk,
        output  reg     spi_mosi
    );

    reg     [8:0]cnt_bit;
    reg     [3:0]cnt_ctrl;
    reg     [6:0]state;

    localparam  idle       = 7'b0_000_001;
    localparam  setup_time = 7'b0_000_010;
    localparam  ctrl       = 7'b0_000_100;
    localparam  hold_time  = 7'b0_001_000;
    localparam  wait_time  = 7'b0_010_000;
    localparam  compare    = 7'b0_100_000;
    localparam  stop       = 7'b1_000_000;

    wire    [15:0]data_reg;

    /* ADC register set */
    assign  data_reg = {1'b0,7'h00,8'h80};   // soft reset

    always @(posedge clock or posedge reset) begin
        if(reset)
            state <= idle;
        else
        begin
        case(state)
            idle:state <= setup_time;
            setup_time:state <= ctrl;
            ctrl:
                if((cnt_ctrl == 15)&&(cnt_bit == 399))
                    state <= hold_time;
                else
                    state <= state;
            hold_time:state <= wait_time;
            wait_time:state <= compare;
            compare:state <= stop;
            stop:state <= stop;
        endcase
        end
    end

    always @(posedge clock or posedge reset) begin
        if(reset)
            cnt_bit <= 0;    
        else if((state == setup_time)||(state == ctrl)||(state == hold_time))
            cnt_bit <= cnt_bit + 1'b1;
        else
            cnt_bit <= 0;
    end

    always @(posedge clock or posedge reset) begin
        if(reset)
            cnt_ctrl <= 0;
        else if((cnt_ctrl == 15)&&(cnt_bit == 399))
            cnt_ctrl <= 0;
        else if((state == ctrl)&&(cnt_bit == 399))
            cnt_ctrl <= cnt_ctrl + 1'b1;
        else
            cnt_ctrl <= cnt_ctrl;
    end

    always @(posedge clock or posedge reset) begin
        if(reset)
            spi_cs <= 1;
        else if((state == setup_time)||(state == ctrl)||(state == hold_time))
            spi_cs <= 0;
        else
            spi_cs <= 1;
    end

    always @(posedge clock or posedge reset) begin
        if(reset)
            spi_clk <= 0;
        else if(cnt_bit == 0)
            spi_clk <= 0;
        else if(cnt_bit == 199)
            spi_clk <= 1;
        else
            spi_clk <= spi_clk;
    end

    always @(posedge clock or posedge reset) begin
        if(reset)
            spi_mosi <= 0;
        else if(state != ctrl)
            spi_mosi <= 0;
        else if((data_reg[15] == 1)&&(cnt_ctrl > 7))
            spi_mosi <= 1'bz;
        else if((data_reg[15] == 0)||(data_reg[15] == 1))
            spi_mosi <= data_reg[15-cnt_ctrl];
        else
            spi_mosi <= spi_mosi;
    end

endmodule