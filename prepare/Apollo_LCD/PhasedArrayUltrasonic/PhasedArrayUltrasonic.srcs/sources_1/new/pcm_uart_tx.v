`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//            XOR = L2 ^ L1 ^ L0
//////////////////////////////////////////////////////////////////////////////////

module pcm_uart_tx_24b #(
    parameter CLK_FREQ_HZ = 50_000_000,
    parameter BAUD_RATE    = 9600,
    parameter DOWNSAMPLE   = 200
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       bck,
    input  wire       lrck,
    input  wire       dout,
    output wire       uart_tx,
    output wire [23:0] dbg_left,
    output wire        dbg_valid
);

    //==========================================================================
    //==========================================================================
    wire [23:0] left_data;
    wire [23:0] right_data;
    wire        pcm_valid;

    pcm1808_slave_rx #(
        .FS_BCK_RATIO (64),
        .DATA_WIDTH   (24)
    ) u_pcm_rx (
        .clk             (clk),
        .rst_n           (rst_n),
        .bck             (bck),
        .lrck            (lrck),
        .dout            (dout),
        .left_chan_data  (left_data),
        .right_chan_data (right_data),
        .data_valid      (pcm_valid)
    );

    //==========================================================================
    //==========================================================================
    reg [$clog2(DOWNSAMPLE)-1:0] ds_cnt;
    reg [23:0] samp_left;
    reg        samp_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ds_cnt      <= 'd0;
            samp_left   <= 24'd0;
            samp_valid  <= 1'b0;
        end else begin
            samp_valid <= 1'b0;
            if (pcm_valid) begin
                if (ds_cnt == DOWNSAMPLE - 1) begin
                    ds_cnt     <= 'd0;
                    samp_left  <= left_data;
                    samp_valid <= 1'b1;
                end else begin
                    ds_cnt <= ds_cnt + 1'b1;
                end
            end
        end
    end

    //==========================================================================
    //==========================================================================
    localparam [1:0] TX_IDLE = 2'd0;
    localparam [1:0] TX_LOAD = 2'd1;
    localparam [1:0] TX_WAIT = 2'd2;

    reg [1:0]  tx_state;
    reg [7:0]  tx_buf [0:4];
    reg [2:0]  tx_idx;
    reg [7:0]  tx_data;
    reg        tx_data_valid;
    wire       tx_ready;

    uart_tx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) u_uart_tx (
        .clk        (clk),
        .rst_n      (rst_n),
        .data_in    (tx_data),
        .data_valid (tx_data_valid),
        .tx         (uart_tx),
        .ready      (tx_ready)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state      <= TX_IDLE;
            tx_idx        <= 3'd0;
            tx_data       <= 8'd0;
            tx_data_valid <= 1'b0;
        end else begin
            tx_data_valid <= 1'b0;

            case (tx_state)
                TX_IDLE: begin
                    if (samp_valid) begin
                        tx_buf[0] <= 8'hAA;
                        tx_buf[1] <= samp_left[23:16];    // L2 MSB
                        tx_buf[2] <= samp_left[15:8];     // L1
                        tx_buf[3] <= samp_left[7:0];      // L0 LSB
                        tx_buf[4] <= samp_left[23:16] ^ samp_left[15:8] ^ samp_left[7:0];
                        tx_idx    <= 3'd0;
                        tx_state  <= TX_LOAD;
                    end
                end

                TX_LOAD: begin
                    if (tx_ready) begin
                        tx_data       <= tx_buf[tx_idx];
                        tx_data_valid <= 1'b1;
                        tx_state      <= TX_WAIT;
                    end
                end

                TX_WAIT: begin
                    if (tx_ready && !tx_data_valid) begin
                        if (tx_idx == 3'd4)
                            tx_state <= TX_IDLE;
                        else begin
                            tx_idx   <= tx_idx + 1'b1;
                            tx_state <= TX_LOAD;
                        end
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    assign dbg_left  = samp_left;
    assign dbg_valid = samp_valid;

endmodule
