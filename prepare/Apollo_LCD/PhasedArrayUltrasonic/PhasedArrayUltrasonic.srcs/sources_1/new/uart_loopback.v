`timescale 1ns / 1ps

// ============================================================================
// UART Loopback (Echo) Module
// 
// Function: Receives bytes via UART and sends back the same byte immediately.
// 
// Handshake:
//   - uart_rx: data_valid pulse, data_out[7:0]
//   - uart_tx: ready (high when can accept new data), data_in, data_valid pulse
// 
// Single-byte buffer is used to handle the case when a new byte arrives
// while the transmitter is busy. If buffer is already full, incoming byte
// is dropped (simple policy, can be extended to FIFO if needed).
// 
// Clock: 50 MHz, Reset: active low asynchronously
// Baud rate: 115200, 8N1
// ============================================================================

module uart_loopback (
    input  wire clk,          // 50 MHz system clock
    input  wire rst_n,        // asynchronous active-low reset
    input  wire uart_rx_i,    // UART receive pin
    output wire uart_tx_o     // UART transmit pin
);

    // ------------------------------------------------------------
    // Internal signals
    // ------------------------------------------------------------
    wire       rx_data_valid;
    wire [7:0] rx_data;

    wire       tx_ready;
    reg  [7:0] tx_data;
    reg        tx_data_valid;

    // Buffer for one pending byte
    reg        pending_valid;
    reg  [7:0] pending_data;

    // ------------------------------------------------------------
    // UART Receiver instance
    // ------------------------------------------------------------
    uart_rx #(
        .CLK_FREQ_HZ (50_000_000),
        .BAUD_RATE   (9600)
    ) u_rx (
        .clk         (clk),
        .rst_n       (rst_n),
        .rx          (uart_rx_i),
        .data_out    (rx_data),
        .data_valid  (rx_data_valid)
    );

    // ------------------------------------------------------------
    // UART Transmitter instance
    // ------------------------------------------------------------
    uart_tx #(
        .CLK_FREQ_HZ (50_000_000),
        .BAUD_RATE   (9600)
    ) u_tx (
        .clk         (clk),
        .rst_n       (rst_n),
        .data_in     (tx_data),
        .data_valid  (tx_data_valid),
        .tx          (uart_tx_o),
        .ready       (tx_ready)
    );

    // ------------------------------------------------------------
    // Loopback control logic with single-byte buffer
    // ------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pending_valid <= 1'b0;
            pending_data  <= 8'd0;
            tx_data       <= 8'd0;
            tx_data_valid <= 1'b0;
        end else begin
            // Default: no new transmit request
            tx_data_valid <= 1'b0;

            // Case 1: Transmitter is ready and there is a pending byte -> send it
            if (tx_ready && pending_valid) begin
                tx_data       <= pending_data;
                tx_data_valid <= 1'b1;
                pending_valid <= 1'b0;
            end

            // Case 2: New byte arrives from UART receiver
            if (rx_data_valid) begin
                // If transmitter is ready right now, send directly (avoid buffer)
                if (tx_ready) begin
                    tx_data       <= rx_data;
                    tx_data_valid <= 1'b1;
                    // pending_valid remains unchanged (buffer not used)
                end
                // Else if buffer is empty, store the byte for later
                else if (!pending_valid) begin
                    pending_data  <= rx_data;
                    pending_valid <= 1'b1;
                end
                // Else (buffer full and transmitter busy) -> drop incoming byte
                // (Simple policy: no overflow handling)
            end
        end
    end

endmodule