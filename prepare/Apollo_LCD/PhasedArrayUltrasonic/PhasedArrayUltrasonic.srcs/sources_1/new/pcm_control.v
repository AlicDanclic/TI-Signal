//////////////////////////////////////////////////////////////////////////////////
//
//
//////////////////////////////////////////////////////////////////////////////////

module pcm1808_slave_rx #(
    parameter FS_BCK_RATIO = 64,
    parameter DATA_WIDTH   = 24
) (
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     bck,
    input  wire                     lrck,
    input  wire                     dout,
    output reg  [DATA_WIDTH-1:0]    left_chan_data,
    output reg  [DATA_WIDTH-1:0]    right_chan_data,
    output reg                      data_valid
);

    //==============================================================================
    //==============================================================================
    reg  [2:0] bck_sync;
    reg  [2:0] lrck_sync;
    reg        lrck_d;

    wire bck_rising;
    wire lrck_rising;
    wire lrck_falling;
    wire lrck_change;

    reg  [$clog2(FS_BCK_RATIO)-1:0] bit_cnt;

    reg  [DATA_WIDTH-1:0] shift_reg;
    reg  [DATA_WIDTH-1:0] shift_cnt;

    reg  [DATA_WIDTH-1:0] left_reg, right_reg;

    //==============================================================================
    //==============================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bck_sync  <= 3'b000;
            lrck_sync <= 3'b000;
            lrck_d    <= 1'b0;
        end else begin
            bck_sync  <= {bck_sync[1:0], bck};
            lrck_sync <= {lrck_sync[1:0], lrck};
            lrck_d    <= lrck_sync[2];
        end
    end

    assign bck_rising   = (bck_sync[2:1] == 2'b01);
    assign lrck_rising  = (lrck_sync[2] && !lrck_d);
    assign lrck_falling = (!lrck_sync[2] && lrck_d);
    assign lrck_change  = lrck_rising | lrck_falling;

    //==============================================================================
    //==============================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt <= 'd0;
        end else begin
            if (lrck_change) begin
                bit_cnt <= 'd0;
            end else if (bck_rising) begin
                if (bit_cnt == FS_BCK_RATIO/2 - 1)
                    bit_cnt <= 'd0;
                else
                    bit_cnt <= bit_cnt + 1'b1;
            end
        end
    end

    //==============================================================================
    //==============================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg <= {DATA_WIDTH{1'b0}};
            shift_cnt <= 'd0;
        end else begin
            if (lrck_change) begin
                shift_reg <= {DATA_WIDTH{1'b0}};
                shift_cnt <= 'd0;
            end else if (bck_rising) begin
                if (bit_cnt >= 1 && bit_cnt <= DATA_WIDTH) begin
                    shift_reg <= {shift_reg[DATA_WIDTH-2:0], dout};
                    shift_cnt <= shift_cnt + 1'b1;
                end
            end
        end
    end

    //==============================================================================
    //==============================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            left_chan_data  <= {DATA_WIDTH{1'b0}};
            right_chan_data <= {DATA_WIDTH{1'b0}};
            left_reg        <= {DATA_WIDTH{1'b0}};
            right_reg       <= {DATA_WIDTH{1'b0}};
            data_valid      <= 1'b0;
        end else begin
            data_valid <= 1'b0;

            if (bck_rising && (bit_cnt == DATA_WIDTH + 1)) begin
                if (lrck_sync[2]) begin
                    left_reg  <= shift_reg;
                end else begin
                    right_reg <= shift_reg;
                end
            end

            if (lrck_rising) begin
                left_chan_data  <= left_reg;
                right_chan_data <= right_reg;
                data_valid      <= 1'b1;
            end
        end
    end

endmodule
