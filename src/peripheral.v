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
    
    output reg [31:0] data_out,     // Data out from the peripheral, bottom 8, 16 or all 32 bits are valid on read when data_ready is high.
    output        data_ready,

    output        user_interrupt  // Dedicated interrupt request for this peripheral
);

     // Write decode
    wire write_8  = (data_write_n == 2'b00);
    wire write_16 = (data_write_n == 2'b01);
    wire write_32 = (data_write_n == 2'b10);
    wire write_any = (data_write_n != 2'b11);
    assign data_ready = 1'b1;

    // --- Registers
    reg [2:0] control_reg;
    reg [2:0] spr0_ctrl, spr1_ctrl;         // [1:0]=palette_sel, =flip
    reg [7:0] spr0_x, spr0_y, spr1_x, spr1_y;
    reg [143:0] spr0_bmp, spr1_bmp;
    reg irq_flag;

    // --- Register Write Handling
    always @(posedge clk) begin
        if (!rst_n) begin
            control_reg <= 3'd0; spr0_ctrl <= 3'd0; spr1_ctrl <= 3'd0;
            spr0_x <= 0; spr0_y <= 0;
            spr1_x <= 0; spr1_y <= 0;
            spr0_bmp <= 144'd0; spr1_bmp <= 144'd0;
        end else begin
            if (write_any && (address == 6'h00))
                control_reg[2:0] <= data_in[2:0];
            if (write_any && (address == 6'h01))
                spr0_ctrl <= data_in[2:0]; // palette:1:0, flip:2
            if (write_any && (address == 6'h02))
                spr1_ctrl <= data_in[2:0];
            if (!control_reg && write_16) begin // only config when not streaming
                case (address)
                    6'h04: begin spr0_x <= data_in[7:0]; spr0_y <= data_in[15:8]; end
                    6'h06: spr0_bmp[15:0]    <= data_in[15:0];
                    6'h08: spr0_bmp[31:16]   <= data_in[15:0];
                    6'h0A: spr0_bmp[47:32]   <= data_in[15:0];
                    6'h0C: spr0_bmp[63:48]   <= data_in[15:0];
                    6'h0E: spr0_bmp[79:64]   <= data_in[15:0];
                    6'h10: spr0_bmp[95:80]   <= data_in[15:0];
                    6'h12: spr0_bmp[111:96]  <= data_in[15:0];
                    6'h14: spr0_bmp[127:112] <= data_in[15:0];
                    6'h16: spr0_bmp[143:128] <= data_in[15:0];
                    6'h1A: begin spr1_x <= data_in[7:0]; spr1_y <= data_in[15:8]; end
                    6'h1C: spr1_bmp[15:0]    <= data_in[15:0];
                    6'h1E: spr1_bmp[31:16]   <= data_in[15:0];
                    6'h20: spr1_bmp[47:32]   <= data_in[15:0];
                    6'h22: spr1_bmp[63:48]   <= data_in[15:0];
                    6'h24: spr1_bmp[79:64]   <= data_in[15:0];
                    6'h26: spr1_bmp[95:80]   <= data_in[15:0];
                    6'h28: spr1_bmp[111:96]  <= data_in[15:0];
                    6'h2A: spr1_bmp[127:112] <= data_in[15:0];
                    6'h2C: spr1_bmp[143:128] <= data_in[15:0];
                endcase
            end
        end
    end

    // --- Readback
    always @(*) begin
        case(address)
            6'h00: data_out = {29'h0, control_reg};
            6'h01: data_out = {29'h0, spr0_ctrl};
            6'h02: data_out = {29'h0, spr1_ctrl};
            6'h04: data_out = {16'h0, spr0_y, spr0_x};
            6'h06: data_out = {16'h0, spr0_bmp[15:0]};
            6'h08: data_out = {16'h0, spr0_bmp[31:16]};
            6'h0A: data_out = {16'h0, spr0_bmp[47:32]};
            6'h0C: data_out = {16'h0, spr0_bmp[63:48]};
            6'h0E: data_out = {16'h0, spr0_bmp[79:64]};
            6'h10: data_out = {16'h0, spr0_bmp[95:80]};
            6'h12: data_out = {16'h0, spr0_bmp[111:96]};
            6'h14: data_out = {16'h0, spr0_bmp[127:112]};
            6'h16: data_out = {16'h0, spr0_bmp[143:128]};
            6'h1A: data_out = {16'h0, spr1_y, spr1_x};
            6'h1C: data_out = {16'h0, spr1_bmp[15:0]};
            6'h1E: data_out = {16'h0, spr1_bmp[31:16]};
            6'h20: data_out = {16'h0, spr1_bmp[47:32]};
            6'h22: data_out = {16'h0, spr1_bmp[63:48]};
            6'h24: data_out = {16'h0, spr1_bmp[79:64]};
            6'h26: data_out = {16'h0, spr1_bmp[95:80]};
            6'h28: data_out = {16'h0, spr1_bmp[111:96]};
            6'h2A: data_out = {16'h0, spr1_bmp[127:112]};
            6'h2C: data_out = {16'h0, spr1_bmp[143:128]};
            default: data_out = 32'h0;
        endcase
    end

    // --- Palette Table: 4 entries × (R,G,B) 2b each
    localparam [23:0] PALETTE = {
      6'b11_11_11, // white
      6'b11_00_00, // red
      6'b00_11_00, // green
      6'b00_00_11  // blue
    };

    function [5:0] get_palette(input [1:0] sel);
        case(sel)
            2'd0: get_palette = PALETTE[5:0];
            2'd1: get_palette = PALETTE[11:6];
            2'd2: get_palette = PALETTE[17:12];
            2'd3: get_palette = PALETTE[23:18];
        endcase
    endfunction

    // --- XGA timing logic (unchanged): h_cnt, v_cnt, hsync_r, vsync_r, visible_r, etc.
    // --- IRQ logic as in your base

    // --- Rendering logic
    wire [9:0] pix_x = h_cnt[9:0];
    wire [9:0] pix_y = v_cnt[9:0];
    wire video_active = visible_r; 
    wire [7:0] lx = pix_x[9:2];
    wire [7:0] ly = pix_y[9:2];

    // Sprite 0
    wire [7:0] s0_dx = lx - spr0_x, s0_dy = ly - spr0_y;
    wire s0_in = (lx >= spr0_x) && (lx < spr0_x+12) && (ly >= spr0_y) && (ly < spr0_y+12);
    wire [3:0] s0_col = s0_dx[3:0];  // for 12
    wire [3:0] s0_row = s0_dy[3:0];
    wire [7:0] s0_idx = {s0_row[3:0], s0_col[3:0]}; // gives up to 16x16 but only [0:143] valid

    // Y-flip logic
    wire s0_flip = spr0_ctrl;
    wire [7:0] s0_cmp_idx  = {s0_row[3:0], (s0_flip ? (11 - s0_col[3:0]) : s0_col[3:0])};
    wire s0_pixel = video_active && s0_in && spr0_bmp[s0_cmp_idx];

    // Optional: Mirror copy at X+12
    wire s0_m_in = (lx >= spr0_x+12) && (lx < spr0_x+24) && (ly >= spr0_y) && (ly < spr0_y+12);
    wire [3:0] s0_m_col = lx - (spr0_x + 12);
    wire [7:0] s0_m_idx = {s0_row[3:0], (s0_flip ? (11-s0_m_col[3:0]) : s0_m_col[3:0])};
    wire s0_flip_enable = spr0_ctrl;
    wire s0_m_pixel = video_active && s0_flip_enable && s0_m_in && spr0_bmp[s0_m_idx];

    // Same logic for sprite 1 (use respective registers)

    // --- Palette
    wire [1:0] s0_palette = spr0_ctrl[1:0];
    wire [5:0] s0_rgb = get_palette(s0_palette);
    wire [1:0] s1_palette = spr1_ctrl[1:0];
    wire [5:0] s1_rgb = get_palette(s1_palette);

    // --- Output composition and prioritization, for example:
    wire sprite1_on = ...; // logic as above for s1
    // Priority: sprite1 > sprite0 > mirror
    wire [5:0] final_rgb = sprite1_on ? s1_rgb : (s0_pixel ? s0_rgb : (s0_m_pixel ? s0_rgb : 6'b0));
    assign uo_out = {vsync_r, hsync_r, final_rgb};

    assign user_interrupt = irq_flag;
    wire _unused_ok = &{1'b0, ui_in, data_read_n};

endmodule
