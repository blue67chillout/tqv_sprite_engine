/*
 * Copyright (c) 2025 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// Change the name of this module to something that reflects its functionality and includes your name for uniqueness
// For example tqvp_yourname_spi for an SPI peripheral.
// Then edit tt_wrapper.v line 41 and change tqvp_example to your chosen module name.
module tqvp_example (
    input         clk,          // Clock - the TinyQV project clock is normally set to 64MHz.
    input         rst_n,        // Reset_n - low to reset.

    input  [7:0]  ui_in,        // The input PMOD, always available.  Note that ui_in[7] is normally used for UART RX.
                                // The inputs are synchronized to the clock, note this will introduce 2 cycles of delay on the inputs.

    output [7:0]  uo_out,       // The output PMOD.  Each wire is only connected if this peripheral is selected.
                                // Note that uo_out[0] is normally used for UART TX.

    input [5:0]   address,      // Address within this peripheral's address space
    input [31:0]  data_in,      // Data in to the peripheral, bottom 8, 16 or all 32 bits are valid on write.

    // Data read and write requests from the TinyQV core.
    input [1:0]   data_write_n, // 11 = no write, 00 = 8-bits, 01 = 16-bits, 10 = 32-bits
    input [1:0]   data_read_n,  // 11 = no read,  00 = 8-bits, 01 = 16-bits, 10 = 32-bits
    
    output [31:0] data_out,     // Data out from the peripheral, bottom 8, 16 or all 32 bits are valid on read when data_ready is high.
    output        data_ready,

    output        user_interrupt  // Dedicated interrupt request for this peripheral
);

        // -----------------------------
    // Registers
    // -----------------------------
    reg [7:0]  control_reg;

    reg [7:0]  spr0_x, spr0_y;
    reg [7:0]  spr1_x, spr1_y;

    reg [63:0] spr0_bmp, spr1_bmp;

    // -----------------------------
    // Register write decode
    // -----------------------------

wire write_8  = (data_write_n == 2'b00);
wire write_16 = (data_write_n == 2'b01);
wire write_32 = (data_write_n == 2'b10);
wire write_any = ~(&data_write_n); // anything but "11"
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            control_reg <= 8'h00;
            spr0_x <= 8'h00; spr0_y <= 8'h00;
            spr1_x <= 8'h00; spr1_y <= 8'h00;
            spr0_bmp <= 64'h0;
            spr1_bmp <= 64'h0;
        end else if (write_16 || write_8) begin
            case (address)
                // control_reg (8-bit only)
                6'h00: control_reg <= data_in[7:0];

                // spr0 coords {Y[7:0], X[7:0]}
                6'h04: begin
                    spr0_x <= data_in[7:0];
                    spr0_y <= data_in[15:8];
                end

                // spr0 bitmap (64-bit, 4 words)
                6'h06: spr0_bmp[15:0]   <= data_in;
                6'h08: spr0_bmp[31:16]  <= data_in;
                6'h0A: spr0_bmp[47:32]  <= data_in;
                6'h0C: spr0_bmp[63:48]  <= data_in;

                // spr1 coords {Y[7:0], X[7:0]}
                6'h0E: begin
                    spr1_x <= data_in[7:0];
                    spr1_y <= data_in[15:8];
                end

                // spr1 bitmap (64-bit, 4 words)
                6'h10: spr1_bmp[15:0]   <= data_in;
                6'h12: spr1_bmp[31:16]  <= data_in;
                6'h14: spr1_bmp[47:32]  <= data_in;
                6'h16: spr1_bmp[63:48]  <= data_in;

                default: ;
            endcase
        end
    end

    // -----------------------------
    // Register readback
    // -----------------------------
    always @(*) begin
        case (address)
            6'h00: data_out = {8'h00, control_reg};

            6'h04: data_out = {spr0_y, spr0_x};
            6'h06: data_out = spr0_bmp[15:0];
            6'h08: data_out = spr0_bmp[31:16];
            6'h0A: data_out = spr0_bmp[47:32];
            6'h0C: data_out = spr0_bmp[63:48];

            6'h0E: data_out = {spr1_y, spr1_x};
            6'h10: data_out = spr1_bmp[15:0];
            6'h12: data_out = spr1_bmp[31:16];
            6'h14: data_out = spr1_bmp[47:32];
            6'h16: data_out = spr1_bmp[63:48];

            default: data_out = 16'h0000;
        endcase
    end

    // -----------------------------
    // XGA Timing (1024x768 @ 60 Hz)
    // -----------------------------
    localparam H_ACTIVE = 1024;
    localparam H_FP     = 24;
    localparam H_SYNC   = 136;
    localparam H_BP     = 160;
    localparam H_TOTAL  = H_ACTIVE + H_FP + H_SYNC + H_BP; // 1344

    localparam V_ACTIVE = 768;
    localparam V_FP     = 3;
    localparam V_SYNC   = 6;
    localparam V_BP     = 29;
    localparam V_TOTAL  = V_ACTIVE + V_FP + V_SYNC + V_BP; // 806

    reg [10:0] h_cnt;
    reg [9:0]  v_cnt;
    reg hsync ;
    reg vsync ;
    
    wire video_active = (h_cnt < H_ACTIVE) && (v_cnt < V_ACTIVE);

    always @(posedge clk ) begin
        if (!rst_n) begin
            h_cnt <= 0;
            v_cnt <= 0;
            hsync <= 1;
            vsync <= 1;
        end else if (control_reg[0]) begin // streaming enabled
            if (h_cnt == H_TOTAL-1) begin
                h_cnt <= 0;
                if (v_cnt == V_TOTAL-1)
                    v_cnt <= 0;
                else
                    v_cnt <= v_cnt + 1;
            end else begin
                h_cnt <= h_cnt + 1;
            end

            // sync pulses
            hsync <= ~((h_cnt >= (H_ACTIVE + H_FP)) &&
                       (h_cnt <  (H_ACTIVE + H_FP + H_SYNC)));
            vsync <= ~((v_cnt >= (V_ACTIVE + V_FP)) &&
                       (v_cnt <  (V_ACTIVE + V_FP + V_SYNC)));
        end
    end

    // -----------------------------
    // Sprite rendering
    // -----------------------------
    // Scale logical (256x192) → physical (1024x768) by factor 4
    wire [7:0] lx = h_cnt[9:2]; // divide by 4
    wire [7:0] ly = v_cnt[8:2]; // divide by 4

    reg pixel_on;

    wire [2:0] spr0_row = ly - spr0_y; // 0..7
    wire [2:0] spr0_col = lx - spr0_x; // 0..7
    wire [5:0] spr0_idx = {spr0_row, spr0_col}; // row*8 + col
    
    wire [2:0] spr1_row = ly - spr1_y;
    wire [2:0] spr1_col = lx - spr1_x;
    wire [5:0] spr1_idx = {spr1_row, spr1_col};

    always @(*) begin
        pixel_on = 1'b0;
        if (video_active) begin
            if (lx >= spr0_x && lx < spr0_x+8 &&
                ly >= spr0_y && ly < spr0_y+8) begin
                pixel_on = spr0_bmp[spr0_idx];
            end
            else if (lx >= spr1_x && lx < spr1_x+8 &&
                     ly >= spr1_y && ly < spr1_y+8) begin
                pixel_on = spr1_bmp[spr1_idx];
            end
        end
   end

    always @(*) begin
        if (video_active && pixel_on)
            rgb = 8'hFF; // white pixel
        else
            rgb = 8'h00; // black background
    end

endmodule
