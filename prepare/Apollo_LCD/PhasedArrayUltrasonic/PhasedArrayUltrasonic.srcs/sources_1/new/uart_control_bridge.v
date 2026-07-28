`timescale 1ns / 1ps

module uart_control_bridge #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD_RATE   = 115200,
    parameter integer AUDIO_SAMPLE_RATE = 4000
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        uart_rx_i,
    output wire        uart_tx_o,

    input  wire [7:0]  adc_env,
    input  wire [1:0]  local_mode,
    input  wire [1:0]  local_preset,
    input  wire        active_remote_enable,
    input  wire        active_remote_env_adc,
    input  wire        apply_pending,

    output reg         shadow_remote_enable,
    output reg         shadow_remote_env_adc,
    output reg [199:0] shadow_phase_flat,
    output reg [199:0] shadow_amp_flat,
    output reg         apply_req_pulse,
    output reg [7:0]   duty_limit,
    output reg [7:0]   current_task,

    output wire        uart_tx_busy,
    output reg         rx_activity,
    output wire        apply_trigger          // 应用命令触发指示（电平翻转）
);

    localparam [7:0] CHAR_AT   = 8'h40;   // '@'
    localparam [7:0] CHAR_HASH = 8'h23;   // '#'
    localparam [7:0] CHAR_PIPE = 8'h7C;   // '|'

    localparam [3:0] RX_IDLE  = 0, RX_TT_HI = 1, RX_TT_LO = 2, RX_SEP1 = 3,
                     RX_AA_HI = 4, RX_AA_LO = 5, RX_SEP2 = 6, RX_BB_HI = 7,
                     RX_BB_LO = 8, RX_SEP3 = 9, RX_CC_HI =10, RX_CC_LO =11,
                     RX_END   =12;

    localparam [1:0] TX_IDLE = 0, TX_LOAD = 1, TX_WAIT = 2;

    localparam integer AUDIO_TICK_DIV   = CLK_FREQ_HZ / AUDIO_SAMPLE_RATE;
    localparam integer AUDIO_FIFO_DEPTH = 256;
    localparam [8:0]   AUDIO_FIFO_FULL  = 9'd256;

    // UART 子模块信号
    wire [7:0] rx_data;
    wire       rx_data_valid;
    reg  [7:0] tx_data;
    reg        tx_data_valid;
    wire       tx_ready;

    reg [3:0]  rx_state;
    reg [7:0]  r_tt_hi, r_tt_lo, r_aa_hi, r_aa_lo;
    reg [7:0]  r_bb_hi, r_bb_lo, r_cc_hi, r_cc_lo;

    reg [1:0]  tx_state;
    reg [7:0]  tx_buf [0:12];
    reg [7:0]  ack_buf [0:12];
    reg [3:0]  tx_idx;
    reg        cmd_valid;
    reg        ack_pending;
    reg [15:0] tx_timeout;

    reg [7:0]  task_amp   [0:24];
    reg [7:0]  task_phase [0:24];
    reg        audio_stream_mode;
    reg [7:0]  audio_fifo [0:AUDIO_FIFO_DEPTH-1];
    reg [7:0]  audio_wr_ptr;
    reg [7:0]  audio_rd_ptr;
    reg [8:0]  audio_fifo_count;
    reg [15:0] audio_tick_cnt;
    reg        audio_push;
    reg        audio_stop;
    reg        audio_pop;
    reg [7:0]  audio_push_data;

    integer i;

    // ---------- 辅助函数 ----------
    function [3:0] hex_val;
        input [7:0] ch;
        begin
            if (ch >= 8'h30 && ch <= 8'h39)       hex_val = ch[3:0];
            else if (ch >= 8'h41 && ch <= 8'h46)  hex_val = ch[3:0] + 4'd9;
            else if (ch >= 8'h61 && ch <= 8'h66)  hex_val = ch[3:0] + 4'd9;
            else                                  hex_val = 4'd0;
        end
    endfunction

    function is_hex;
        input [7:0] ch;
        begin
            is_hex = ((ch >= 8'h30 && ch <= 8'h39) ||
                       (ch >= 8'h41 && ch <= 8'h46) ||
                       (ch >= 8'h61 && ch <= 8'h66));
        end
    endfunction

    // ---------- 通道编号解析（十六进制，1-based） ----------
    wire [7:0] idx_1based = {4'b0, hex_val(r_bb_hi)} * 8'd16 + {4'b0, hex_val(r_bb_lo)};
    wire       idx_valid  = (idx_1based >= 8'd1) && (idx_1based <= 8'd25);

    // UART 收发实例
    uart_rx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) u_uart_rx (
        .clk       (clk),
        .rst_n     (rst_n),
        .rx        (uart_rx_i),
        .data_out  (rx_data),
        .data_valid(rx_data_valid)
    );

    uart_tx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) u_uart_tx (
        .clk       (clk),
        .rst_n     (rst_n),
        .data_in   (tx_data),
        .data_valid(tx_data_valid),
        .tx        (uart_tx_o),
        .ready     (tx_ready)
    );

    assign uart_tx_busy = (tx_state != TX_IDLE);

    // 应用触发寄存器
    reg apply_trig_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state <= RX_IDLE;
            {r_tt_hi,r_tt_lo,r_aa_hi,r_aa_lo,r_bb_hi,r_bb_lo,r_cc_hi,r_cc_lo} <= 0;
            tx_state   <= TX_IDLE;
            tx_data    <= 0;
            tx_data_valid <= 0;
            tx_idx     <= 0;
            cmd_valid  <= 0;
            ack_pending <= 0;
            tx_timeout <= 0;
            shadow_remote_enable  <= 0;
            shadow_remote_env_adc <= 0;
            shadow_phase_flat     <= 0;
            shadow_amp_flat       <= {200{1'b1}};
            apply_req_pulse       <= 0;
            duty_limit            <= 8'd128;
            current_task          <= 0;
            rx_activity           <= 0;
            apply_trig_reg        <= 1'b0;
            audio_stream_mode     <= 1'b0;
            audio_wr_ptr          <= 8'd0;
            audio_rd_ptr          <= 8'd0;
            audio_fifo_count      <= 9'd0;
            audio_tick_cnt        <= 16'd0;
            audio_push            = 1'b0;
            audio_stop            = 1'b0;
            audio_pop             = 1'b0;
            audio_push_data       = 8'd0;
            for (i = 0; i < 25; i = i + 1) begin
                task_amp[i]   <= 8'hFF;
                task_phase[i] <= 0;
            end
            for (i = 0; i < AUDIO_FIFO_DEPTH; i = i + 1) begin
                audio_fifo[i] <= 8'd128;
            end
            for (i = 0; i < 13; i = i + 1) begin
                tx_buf[i] <= 0;
                ack_buf[i] <= 0;
            end
        end else begin
            tx_data_valid <= 0;
            apply_req_pulse <= 0;
            cmd_valid       <= 0;
            audio_push      = 1'b0;
            audio_stop      = 1'b0;
            audio_pop       = 1'b0;
            audio_push_data = rx_data;

            // rx_activity 翻转指示数据到达
            if (rx_data_valid)
                rx_activity <= ~rx_activity;

            // RX 状态机
            if (rx_data_valid && audio_stream_mode) begin
                if (rx_data == 8'hFF)
                    audio_stop = 1'b1;
                else begin
                    audio_push = 1'b1;
                    audio_push_data = rx_data;
                end
            end else if (rx_data_valid) begin
                case (rx_state)
                    RX_IDLE:  if (rx_data == CHAR_AT) rx_state <= RX_TT_HI;
                    RX_TT_HI: if (is_hex(rx_data)) begin r_tt_hi <= rx_data; rx_state <= RX_TT_LO; end else rx_state <= RX_IDLE;
                    RX_TT_LO: if (is_hex(rx_data)) begin r_tt_lo <= rx_data; rx_state <= RX_SEP1; end else rx_state <= RX_IDLE;
                    RX_SEP1:  rx_state <= (rx_data == CHAR_PIPE) ? RX_AA_HI : RX_IDLE;
                    RX_AA_HI: if (is_hex(rx_data)) begin r_aa_hi <= rx_data; rx_state <= RX_AA_LO; end else rx_state <= RX_IDLE;
                    RX_AA_LO: if (is_hex(rx_data)) begin r_aa_lo <= rx_data; rx_state <= RX_SEP2; end else rx_state <= RX_IDLE;
                    RX_SEP2:  rx_state <= (rx_data == CHAR_PIPE) ? RX_BB_HI : RX_IDLE;
                    RX_BB_HI: if (is_hex(rx_data)) begin r_bb_hi <= rx_data; rx_state <= RX_BB_LO; end else rx_state <= RX_IDLE;
                    RX_BB_LO: if (is_hex(rx_data)) begin r_bb_lo <= rx_data; rx_state <= RX_SEP3; end else rx_state <= RX_IDLE;
                    RX_SEP3:  rx_state <= (rx_data == CHAR_PIPE) ? RX_CC_HI : RX_IDLE;
                    RX_CC_HI: if (is_hex(rx_data)) begin r_cc_hi <= rx_data; rx_state <= RX_CC_LO; end else rx_state <= RX_IDLE;
                    RX_CC_LO: if (is_hex(rx_data)) begin r_cc_lo <= rx_data; rx_state <= RX_END;  end else rx_state <= RX_IDLE;
                    RX_END: begin
                        if (rx_data == CHAR_PIPE) begin
                            rx_state <= RX_END;
                        end else begin
                        if (rx_data == CHAR_HASH) begin
                            // -----------------------------------------------------------
                            // 任务握手：@TT|02|00|00#
                            if (r_aa_hi == 8'h30 && r_aa_lo == 8'h32 && r_bb_hi == 8'h30 && r_bb_lo == 8'h30 && r_cc_hi == 8'h30 && r_cc_lo == 8'h30) begin
                                cmd_valid <= 1;
                                ack_pending <= 1;
                                current_task <= {hex_val(r_tt_hi), hex_val(r_tt_lo)};
                                ack_buf[0]=CHAR_AT; ack_buf[1]=r_tt_hi; ack_buf[2]=r_tt_lo; ack_buf[3]=CHAR_PIPE;
                                ack_buf[4]=8'h32; ack_buf[5]=8'h35; ack_buf[6]=CHAR_PIPE;
                                ack_buf[7]=8'h32; ack_buf[8]=8'h35; ack_buf[9]=CHAR_PIPE;
                                ack_buf[10]=8'h30; ack_buf[11]=8'h30; ack_buf[12]=CHAR_HASH;
                            end
                            // -----------------------------------------------------------
                            // 设置幅度：@TT|00|II|DD#
                            else if (r_aa_hi == 8'h30 && r_aa_lo == 8'h30 && !(r_bb_hi == 8'h30 && r_bb_lo == 8'h30)) begin
                                cmd_valid <= 1;
                                ack_pending <= 1;
                                if (idx_valid)
                                    task_amp[idx_1based - 1] <= {hex_val(r_cc_hi), hex_val(r_cc_lo)};
                                ack_buf[0]=CHAR_AT; ack_buf[1]=r_tt_hi; ack_buf[2]=r_tt_lo; ack_buf[3]=CHAR_PIPE;
                                ack_buf[4]=8'h30; ack_buf[5]=8'h30; ack_buf[6]=CHAR_PIPE;
                                ack_buf[7]=r_bb_hi; ack_buf[8]=r_bb_lo; ack_buf[9]=CHAR_PIPE;
                                ack_buf[10]=8'h30; ack_buf[11]=8'h31; ack_buf[12]=CHAR_HASH;
                            end
                            // -----------------------------------------------------------
                            // 设置相位：@TT|01|II|DD#
                            else if (r_aa_hi == 8'h30 && r_aa_lo == 8'h31 && !(r_bb_hi == 8'h30 && r_bb_lo == 8'h30)) begin
                                cmd_valid <= 1;
                                ack_pending <= 1;
                                if (idx_valid)
                                    task_phase[idx_1based - 1] <= {hex_val(r_cc_hi), hex_val(r_cc_lo)};
                                ack_buf[0]=CHAR_AT; ack_buf[1]=r_tt_hi; ack_buf[2]=r_tt_lo; ack_buf[3]=CHAR_PIPE;
                                ack_buf[4]=8'h30; ack_buf[5]=8'h31; ack_buf[6]=CHAR_PIPE;
                                ack_buf[7]=r_bb_hi; ack_buf[8]=r_bb_lo; ack_buf[9]=CHAR_PIPE;
                                ack_buf[10]=8'h30; ack_buf[11]=8'h31; ack_buf[12]=CHAR_HASH;
                            end
                            // -----------------------------------------------------------
                            // 应用参数：@TT|01|00|00#
                            else if (r_aa_hi == 8'h30 && r_aa_lo == 8'h31 && r_bb_hi == 8'h30 && r_bb_lo == 8'h30 && r_cc_hi == 8'h30 && r_cc_lo == 8'h30) begin
                                cmd_valid <= 1;
                                ack_pending <= 1;
                                for (i = 0; i < 25; i = i + 1) begin
                                    shadow_amp_flat[i*8 +: 8]   <= task_amp[i];
                                    shadow_phase_flat[i*8 +: 8] <= task_phase[i];
                                end
                                shadow_remote_enable <= 1;
                                shadow_remote_env_adc <= 0;
                                apply_req_pulse <= 1;
                                apply_trig_reg <= ~apply_trig_reg;   // 翻转触发信号
                                ack_buf[0]=CHAR_AT; ack_buf[1]=r_tt_hi; ack_buf[2]=r_tt_lo; ack_buf[3]=CHAR_PIPE;
                                ack_buf[4]=8'h30; ack_buf[5]=8'h31; ack_buf[6]=CHAR_PIPE;
                                ack_buf[7]=8'h30; ack_buf[8]=8'h30; ack_buf[9]=CHAR_PIPE;
                                ack_buf[10]=8'h30; ack_buf[11]=8'h30; ack_buf[12]=CHAR_HASH;
                            end
                            // -----------------------------------------------------------
                            // 启动音频占空比流：@TT|03|01|00#，之后原始字节 0..254 更新上限，0xFF 停止
                            else if (r_aa_hi == 8'h30 && r_aa_lo == 8'h33 && r_bb_hi == 8'h30 && r_bb_lo == 8'h31 && r_cc_hi == 8'h30 && r_cc_lo == 8'h30) begin
                                cmd_valid <= 1;
                                ack_pending <= 1;
                                audio_stream_mode <= 1'b1;
                                audio_wr_ptr <= 8'd0;
                                audio_rd_ptr <= 8'd0;
                                audio_fifo_count <= 9'd0;
                                audio_tick_cnt <= 16'd0;
                                duty_limit <= 8'd128;
                                ack_buf[0]=CHAR_AT; ack_buf[1]=r_tt_hi; ack_buf[2]=r_tt_lo; ack_buf[3]=CHAR_PIPE;
                                ack_buf[4]=8'h30; ack_buf[5]=8'h33; ack_buf[6]=CHAR_PIPE;
                                ack_buf[7]=8'h30; ack_buf[8]=8'h31; ack_buf[9]=CHAR_PIPE;
                                ack_buf[10]=8'h30; ack_buf[11]=8'h30; ack_buf[12]=CHAR_HASH;
                            end
                            // -----------------------------------------------------------
                            // 设置占空比上限：@TT|03|00|DD#
                            else if (r_aa_hi == 8'h30 && r_aa_lo == 8'h33 && r_bb_hi == 8'h30 && r_bb_lo == 8'h30) begin
                                cmd_valid <= 1;
                                ack_pending <= 1;
                                duty_limit <= {hex_val(r_cc_hi), hex_val(r_cc_lo)};
                                ack_buf[0]=CHAR_AT; ack_buf[1]=r_tt_hi; ack_buf[2]=r_tt_lo; ack_buf[3]=CHAR_PIPE;
                                ack_buf[4]=8'h30; ack_buf[5]=8'h33; ack_buf[6]=CHAR_PIPE;
                                ack_buf[7]=8'h30; ack_buf[8]=8'h30; ack_buf[9]=CHAR_PIPE;
                                ack_buf[10]=r_cc_hi; ack_buf[11]=r_cc_lo; ack_buf[12]=CHAR_HASH;
                            end
                            // -----------------------------------------------------------
                            // 关闭远程：@TT|00|00|FF#
                            else if (r_aa_hi == 8'h30 && r_aa_lo == 8'h30 && r_bb_hi == 8'h30 && r_bb_lo == 8'h30 && r_cc_hi == 8'h46 && r_cc_lo == 8'h46) begin
                                cmd_valid <= 1;
                                ack_pending <= 1;
                                shadow_remote_enable <= 0;
                                ack_buf[0]=CHAR_AT; ack_buf[1]=r_tt_hi; ack_buf[2]=r_tt_lo; ack_buf[3]=CHAR_PIPE;
                                ack_buf[4]=8'h30; ack_buf[5]=8'h30; ack_buf[6]=CHAR_PIPE;
                                ack_buf[7]=8'h30; ack_buf[8]=8'h30; ack_buf[9]=CHAR_PIPE;
                                ack_buf[10]=8'h46; ack_buf[11]=8'h46; ack_buf[12]=CHAR_HASH;
                            end
                            // -----------------------------------------------------------
                            // 状态查询：@TT|00|00|00#
                            else if (r_aa_hi == 8'h30 && r_aa_lo == 8'h30 && r_bb_hi == 8'h30 && r_bb_lo == 8'h30 && r_cc_hi == 8'h30 && r_cc_lo == 8'h30) begin
                                cmd_valid <= 1;
                                ack_pending <= 1;
                                ack_buf[0]=CHAR_AT; ack_buf[1]=r_tt_hi; ack_buf[2]=r_tt_lo; ack_buf[3]=CHAR_PIPE;
                                ack_buf[4]=8'h30; ack_buf[5]=8'h30; ack_buf[6]=CHAR_PIPE;
                                ack_buf[7]=8'h30; ack_buf[8]=8'h30; ack_buf[9]=CHAR_PIPE;
                                ack_buf[10]=8'h30; ack_buf[11]=8'h30; ack_buf[12]=CHAR_HASH;
                            end
                        end
                        rx_state <= RX_IDLE;
                        end
                    end
                    default: rx_state <= RX_IDLE;
                endcase
            end

            // 音频流播放：UART/BLE 收到的字节先进 FIFO，再以固定 AUDIO_SAMPLE_RATE 节拍更新 duty_limit。
            if (audio_stop) begin
                audio_stream_mode <= 1'b0;
                audio_wr_ptr <= 8'd0;
                audio_rd_ptr <= 8'd0;
                audio_fifo_count <= 9'd0;
                audio_tick_cnt <= 16'd0;
                duty_limit <= 8'd128;
            end else if (audio_stream_mode) begin
                if (audio_tick_cnt == AUDIO_TICK_DIV - 1) begin
                    audio_tick_cnt <= 16'd0;
                    if (audio_fifo_count != 9'd0) begin
                        duty_limit <= audio_fifo[audio_rd_ptr];
                        audio_rd_ptr <= audio_rd_ptr + 1'b1;
                        audio_pop = 1'b1;
                    end
                end else begin
                    audio_tick_cnt <= audio_tick_cnt + 1'b1;
                end

                if (audio_push && audio_fifo_count != AUDIO_FIFO_FULL) begin
                    audio_fifo[audio_wr_ptr] <= audio_push_data;
                    audio_wr_ptr <= audio_wr_ptr + 1'b1;
                end

                case ({audio_push && audio_fifo_count != AUDIO_FIFO_FULL, audio_pop})
                    2'b10: audio_fifo_count <= audio_fifo_count + 1'b1;
                    2'b01: audio_fifo_count <= audio_fifo_count - 1'b1;
                    default: audio_fifo_count <= audio_fifo_count;
                endcase
            end else begin
                audio_tick_cnt <= 16'd0;
            end

            // TX 应答状态机
            case (tx_state)
                TX_IDLE: begin
                    tx_timeout <= 0;
                    if (ack_pending) begin
                        for (i = 0; i < 13; i = i + 1) begin
                            tx_buf[i] <= ack_buf[i];
                        end
                        tx_idx <= 0;
                        ack_pending <= 0;
                        tx_state <= TX_LOAD;
                    end
                end

                TX_LOAD: begin
                    if (tx_timeout > 16'd2500) begin
                        tx_state <= TX_IDLE;
                    end else if (tx_ready) begin
                        tx_data    <= tx_buf[tx_idx];
                        tx_data_valid <= 1'b1;
                        tx_state   <= TX_WAIT;
                        tx_timeout <= 0;
                    end else begin
                        tx_timeout <= tx_timeout + 1'b1;
                    end
                end

                TX_WAIT: begin
                    if (tx_ready && !tx_data_valid) begin
                        if (tx_idx == 4'd12)
                            tx_state <= TX_IDLE;
                        else begin
                            tx_idx   <= tx_idx + 1'd1;
                            tx_state <= TX_LOAD;
                            tx_timeout <= 0;
                        end
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    assign apply_trigger = apply_trig_reg;

endmodule
