`timescale 1ns / 1ps

module wave_uart_frame_tx #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD_RATE   = 115200
) (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          frame_valid,
    input  wire [31:0]   frame_frequency_hz,
    input  wire [31:0]   frame_sample_div,
    input  wire [31:0]   frame_period_cycles,
    input  wire [7:0]    frame_flags,
    input  wire [1815:0] frame_samples,
    output wire          uart_tx_o,
    output reg           frame_done,
    output reg           frame_busy,
    output reg           debug_byte_valid,
    output reg  [7:0]    debug_byte,
    output reg  [7:0]    debug_byte_index
);

    localparam [2:0] TX_IDLE         = 3'd0;
    localparam [2:0] TX_LOAD         = 3'd1;
    localparam [2:0] TX_WAIT         = 3'd2;
    localparam [2:0] TX_WAIT_RELEASE = 3'd3;

    reg [2:0]  tx_state;
    reg [7:0]  byte_index;
    reg [7:0]  sequence;
    reg [15:0] crc_reg;
    reg [7:0]  tx_data;
    reg        tx_data_valid;
    wire       tx_ready;
    reg [7:0]  current_byte;

    function [15:0] crc16_ccitt_next;
        input [15:0] crc_in;
        input [7:0]  data_in;
        integer bit_number;
        reg [15:0] crc_work;
        begin
            crc_work = crc_in ^ {data_in, 8'h00};
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                if (crc_work[15])
                    crc_work = (crc_work << 1) ^ 16'h1021;
                else
                    crc_work = crc_work << 1;
            end
            crc16_ccitt_next = crc_work;
        end
    endfunction

    always @* begin
        case (byte_index)
            8'd0:  current_byte = 8'hA5;
            8'd1:  current_byte = 8'h5A;
            8'd2:  current_byte = 8'h02;
            8'd3:  current_byte = sequence;
            8'd4:  current_byte = frame_frequency_hz[7:0];
            8'd5:  current_byte = frame_frequency_hz[15:8];
            8'd6:  current_byte = frame_frequency_hz[23:16];
            8'd7:  current_byte = frame_frequency_hz[31:24];
            8'd8:  current_byte = frame_sample_div[7:0];
            8'd9:  current_byte = frame_sample_div[15:8];
            8'd10: current_byte = frame_sample_div[23:16];
            8'd11: current_byte = frame_sample_div[31:24];
            8'd12: current_byte = 8'hB5;
            8'd13: current_byte = frame_flags;
            8'd14: current_byte = frame_period_cycles[7:0];
            8'd15: current_byte = frame_period_cycles[15:8];
            8'd16: current_byte = frame_period_cycles[23:16];
            8'd17: current_byte = frame_period_cycles[31:24];
            8'd245: current_byte = crc_reg[7:0];
            8'd246: current_byte = crc_reg[15:8];
            default: begin
                if (byte_index >= 8'd18 && byte_index <= 8'd244)
                    current_byte = frame_samples[(byte_index-8'd18)*8 +: 8];
                else
                    current_byte = 8'h00;
            end
        endcase
    end

    uart_tx #(
        .CLK_FREQ_HZ (CLK_FREQ_HZ),
        .BAUD_RATE   (BAUD_RATE)
    ) u_uart_tx (
        .clk        (clk),
        .rst_n      (rst_n),
        .data_in    (tx_data),
        .data_valid (tx_data_valid),
        .tx         (uart_tx_o),
        .ready      (tx_ready)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state          <= TX_IDLE;
            byte_index        <= 8'd0;
            sequence          <= 8'd0;
            crc_reg           <= 16'hffff;
            tx_data           <= 8'd0;
            tx_data_valid     <= 1'b0;
            frame_done        <= 1'b0;
            frame_busy        <= 1'b0;
            debug_byte_valid  <= 1'b0;
            debug_byte        <= 8'd0;
            debug_byte_index  <= 8'd0;
        end else begin
            tx_data_valid    <= 1'b0;
            frame_done       <= 1'b0;
            debug_byte_valid <= 1'b0;

            case (tx_state)
                TX_IDLE: begin
                    frame_busy <= 1'b0;
                    if (frame_valid) begin
                        byte_index <= 8'd0;
                        crc_reg    <= 16'hffff;
                        frame_busy <= 1'b1;
                        tx_state   <= TX_LOAD;
                    end
                end

                TX_LOAD: begin
                    if (tx_ready) begin
                        tx_data          <= current_byte;
                        tx_data_valid    <= 1'b1;
                        debug_byte       <= current_byte;
                        debug_byte_index <= byte_index;
                        debug_byte_valid <= 1'b1;
                        if (byte_index <= 8'd244)
                            crc_reg <= crc16_ccitt_next(crc_reg, current_byte);
                        tx_state <= TX_WAIT;
                    end
                end

                TX_WAIT: begin
                    if (tx_ready && !tx_data_valid) begin
                        if (byte_index == 8'd246) begin
                            sequence   <= sequence + 1'b1;
                            frame_done <= 1'b1;
                            frame_busy <= 1'b0;
                            tx_state   <= TX_WAIT_RELEASE;
                        end else begin
                            byte_index <= byte_index + 1'b1;
                            tx_state   <= TX_LOAD;
                        end
                    end
                end

                TX_WAIT_RELEASE: begin
                    frame_busy <= 1'b0;
                    if (!frame_valid)
                        tx_state <= TX_IDLE;
                end

                default: begin
                    frame_busy <= 1'b0;
                    tx_state   <= TX_IDLE;
                end
            endcase
        end
    end

endmodule
