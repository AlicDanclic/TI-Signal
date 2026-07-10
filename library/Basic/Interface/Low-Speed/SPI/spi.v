//      可修改分频参数以产生目标频率，最小分频系数为2;
//      可设置CPOL和CPHA来设定通信模式;
//      1位片选信号，可简单修改位宽来设定位多片选信号

`default_nettype none                        // 取消wire类型隐式声明

module spi #(   
        parameter         VALUE_DIVIDE = 2   // 分频系数（最小为2）
    ) (
        // -----------------内部接口------------------
        input   wire        clock,           // 输入时钟
        input   wire        reset,           // 复位

        input   wire        CPOL,            // 时钟极性
        input   wire        CPHA,            // 时钟相位
        input   wire        CS_input,        // 片选信号

        input   wire        valid,          
        input   wire [7:0]  data_send,       

        output  reg         ready,           
        output  reg  [7:0]  data_receive,     
        // ------------------外部接口------------------
        output  reg         spi_clk,         // 输出时钟

        output  reg         spi_mosi,        // 主机输出从机输入
        input   wire        spi_miso,        // 主机输入从机输出

        output  wire        CS_output        // 片选信号输出
    );
 
    reg         act_flag;                    // 动作标志寄存器
    reg [9:0]   cnt_divide;                  // 分频计数器
    reg [7:0]   data_send_reg;               // 待发送数据寄存器
    reg [4:0]   cnt_pulse;                   // 脉冲计数器
    
    always @(posedge clock or posedge reset) begin
        if(reset) begin
            act_flag        <= 0;
            ready           <= 0;
            data_send_reg   <= 0;
            cnt_divide      <= 0;
            cnt_pulse       <= 0;
            if(CPOL == 1) begin
                spi_clk     <= 1;
                spi_mosi    <= 1;
                data_receive <= 0;
            end else begin
                spi_clk     <= 0;
                spi_mosi    <= 1;
                data_receive <= 0;
            end
        end
    end

    always @(posedge clock) begin 
        if(valid == 1) begin
            act_flag <= 1;
        end else if(cnt_divide == VALUE_DIVIDE/2 - 1 & act_flag == 1 & cnt_pulse == 16) begin
            act_flag <= 0;
        end else begin
            act_flag <= act_flag;
        end
    end
    
    always @(posedge clock) begin
        if(cnt_divide == VALUE_DIVIDE/2 - 1 & act_flag == 1 & cnt_pulse == 16) begin
            ready <= 1;
        end else begin
            ready <= 0;
        end
    end
    
    always @(posedge clock) begin
        if(valid == 1) begin
            data_send_reg <= data_send;
        end else begin
            data_send_reg <= data_send_reg;
        end
    end
    
    always @(posedge clock) begin 
        if(cnt_divide == VALUE_DIVIDE/2 - 1 & act_flag == 1) begin
            cnt_divide <= 0;
        end else if(act_flag == 1) begin
            cnt_divide <= cnt_divide + 1'b1;
        end else begin
            cnt_divide <= 0;
        end
    end
     
    always @(posedge clock) begin // 以目标时钟的两倍频率生成cnt_pulse
        if(cnt_divide == VALUE_DIVIDE/2 - 1 & act_flag == 1 & cnt_pulse == 16) begin
            cnt_pulse <= 0;
        end else if(cnt_divide == VALUE_DIVIDE/2 - 1 & act_flag == 1) begin
            cnt_pulse <= cnt_pulse + 1'b1;
        end else if(act_flag == 1) begin
            cnt_pulse <= cnt_pulse;
        end else begin
            cnt_pulse <= 0;
        end
    end
    
    always @(posedge clock) begin
        if(cnt_divide == VALUE_DIVIDE/2 - 1 & act_flag == 1) begin
            if(CPHA == 0) begin
                case(cnt_pulse)
                    0:begin  
                        spi_clk         <= spi_clk;
                        spi_mosi        <= data_send_reg[7];
                        data_receive    <= data_receive;
                    end
                    1:begin
                        spi_clk         <= ~spi_clk;
                        spi_mosi        <= spi_mosi;
                        data_receive[7] <= spi_miso;
                    end
                    2:begin          
                        spi_clk         <= ~spi_clk;            
                        spi_mosi        <= data_send_reg[6];  
                        data_receive    <= data_receive;    
                    end            
                    3:begin          
                        spi_clk         <= ~spi_clk;         
                        spi_mosi        <= spi_mosi;        
                        data_receive[6] <= spi_miso;  
                    end            
                    4:begin          
                        spi_clk         <= ~spi_clk;            
                        spi_mosi        <= data_send_reg[5];  
                        data_receive    <= data_receive;    
                    end            
                    5:begin          
                        spi_clk         <= ~spi_clk;         
                        spi_mosi        <= spi_mosi;        
                        data_receive[5] <= spi_miso;  
                    end            
                    6:begin          
                        spi_clk         <= ~spi_clk;            
                        spi_mosi        <= data_send_reg[4];  
                        data_receive    <= data_receive;    
                    end            
                    7:begin          
                        spi_clk         <= ~spi_clk;         
                        spi_mosi        <= spi_mosi;        
                        data_receive[4] <= spi_miso;  
                    end            
                    8:begin          
                        spi_clk         <= ~spi_clk;            
                        spi_mosi        <= data_send_reg[3];  
                        data_receive    <= data_receive;    
                    end            
                    9:begin          
                        spi_clk         <= ~spi_clk;         
                        spi_mosi        <= spi_mosi;        
                        data_receive[3] <= spi_miso;  
                    end            
                    10:begin          
                        spi_clk         <= ~spi_clk;           
                        spi_mosi        <= data_send_reg[2]; 
                        data_receive    <= data_receive;   
                    end            
                    11:begin          
                        spi_clk         <= ~spi_clk;         
                        spi_mosi        <= spi_mosi;        
                        data_receive[2] <= spi_miso;  
                    end            
                    12:begin          
                        spi_clk         <= ~spi_clk;            
                        spi_mosi        <= data_send_reg[1];  
                        data_receive    <= data_receive;    
                    end            
                    13:begin          
                        spi_clk         <= ~spi_clk;         
                        spi_mosi        <= spi_mosi;        
                        data_receive[1] <= spi_miso;  
                    end            
                    14:begin          
                        spi_clk         <= ~spi_clk;            
                        spi_mosi        <= data_send_reg[0];  
                        data_receive    <= data_receive;    
                    end            
                    15:begin          
                        spi_clk         <= ~spi_clk;         
                        spi_mosi        <= spi_mosi;        
                        data_receive[0] <= spi_miso;  
                    end
                    16:begin
                        spi_clk         <= ~spi_clk;       
                        spi_mosi        <= 1;      
                        data_receive    <= data_receive;
                    end
                    default:;
                endcase
            end else begin
                case(cnt_pulse)
                    0:begin  
                        spi_clk         <= ~spi_clk;
                        spi_mosi        <= data_send_reg[7];
                        data_receive    <= data_receive;
                    end
                    1:begin
                        spi_clk         <= ~spi_clk;
                        spi_mosi        <= spi_mosi;
                        data_receive[7] <= spi_miso;
                    end
                    2:begin          
                        spi_clk         <= ~spi_clk;         
                        spi_mosi        <= data_send_reg[6];
                        data_receive    <= data_receive;  
                    end            
                    3:begin
                        spi_clk         <= ~spi_clk;       
                        spi_mosi        <= spi_mosi;      
                        data_receive[6] <= spi_miso;
                    end            
                    4:begin          
                        spi_clk         <= ~spi_clk;         
                        spi_mosi        <= data_send_reg[5];
                        data_receive    <= data_receive;  
                    end            
                    5:begin          
                        spi_clk         <= ~spi_clk;       
                        spi_mosi        <= spi_mosi;      
                        data_receive[5] <= spi_miso;
                    end            
                    6:begin          
                        spi_clk         <= ~spi_clk;           
                        spi_mosi        <= data_send_reg[4];  
                        data_receive    <= data_receive;    
                    end            
                    7:begin          
                        spi_clk         <= ~spi_clk;         
                        spi_mosi        <= spi_mosi;        
                        data_receive[4] <= spi_miso;  
                    end            
                    8:begin          
                        spi_clk         <= ~spi_clk;         
                        spi_mosi        <= data_send_reg[3];
                        data_receive    <= data_receive;  
                    end            
                    9:begin          
                        spi_clk         <= ~spi_clk;         
                        spi_mosi        <= spi_mosi;        
                        data_receive[3] <= spi_miso; 
                    end            
                    10:begin          
                        spi_clk         <= ~spi_clk;         
                        spi_mosi        <= data_send_reg[2];
                        data_receive    <= data_receive;  
                    end            
                    11:begin          
                        spi_clk         <= ~spi_clk;       
                        spi_mosi        <= spi_mosi;      
                        data_receive[2] <= spi_miso;
                    end            
                    12:begin          
                        spi_clk         <= ~spi_clk;         
                        spi_mosi        <= data_send_reg[1];
                        data_receive    <= data_receive;  
                    end            
                    13:begin          
                        spi_clk         <= ~spi_clk;       
                        spi_mosi        <= spi_mosi;      
                        data_receive[1] <= spi_miso;
                    end            
                    14:begin          
                        spi_clk         <= ~spi_clk;         
                        spi_mosi        <= data_send_reg[0];
                        data_receive    <= data_receive;  
                    end            
                    15:begin          
                        spi_clk         <= ~spi_clk;       
                        spi_mosi        <= spi_mosi;      
                        data_receive[0] <= spi_miso;
                    end
                    16:begin                       
                        spi_clk         <= spi_clk;       
                        spi_mosi        <= 1;      
                        data_receive    <= data_receive;
                    end                          
                    default:;
                endcase     
            end   
        end
    end
    
    assign CS_output = CS_input;
    
endmodule
 