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
// Sprite registers for shadow/write
reg [7:0] spr0_xw, spr0_yw, spr1_xw, spr1_yw;

always @(posedge clk) begin
        if (!rst_n) begin
            control_reg      <= 3'd0;
            spr0_ctrl        <= 3'd0;
            spr1_ctrl        <= 3'd0;
            spr0_xw <= 8'd0; spr0_yw <= 8'd0;
            spr1_xw <= 8'd0; spr1_yw <= 8'd0;
            spr0_bmp <= 144'd0;
            spr1_bmp <= 144'd0;
        end else begin
            if (write_any && (address == 6'h00))
                control_reg <= data_in[2:0];
            if (write_any && (address == 6'h01))
                spr0_ctrl <= data_in[2:0];
            if (write_any && (address == 6'h02))
                spr1_ctrl <= data_in[2:0];
            if (!control_reg[0] && write_16) begin
                case (address)
                    6'h04: begin
                        spr0_xw <= data_in[7:0];
                        spr0_yw <= data_in[15:8];
                    end
                    6'h06: spr0_bmp[15:0]    <= data_in[15:0];
                    6'h08: spr0_bmp[31:16]   <= data_in[15:0];
                    6'h0A: spr0_bmp[47:32]   <= data_in[15:0];
                    6'h0C: spr0_bmp[63:48]   <= data_in[15:0];
                    6'h0E: spr0_bmp[79:64]   <= data_in[15:0];
                    6'h10: spr0_bmp[95:80]   <= data_in[15:0];
                    6'h12: spr0_bmp[111:96]  <= data_in[15:0];
                    6'h14: spr0_bmp[127:112] <= data_in[15:0];
                    6'h16: spr0_bmp[143:128] <= data_in[15:0];
                    6'h1A: begin
                        spr1_xw <= data_in[7:0];
                        spr1_yw <= data_in[15:8];
                    end
                    6'h1C: spr1_bmp[15:0]    <= data_in[15:0];
                    6'h1E: spr1_bmp[31:16]   <= data_in[15:0];
                    6'h20: spr1_bmp[47:32]   <= data_in[15:0];
                    6'h22: spr1_bmp[63:48]   <= data_in[15:0];
                    6'h24: spr1_bmp[79:64]   <= data_in[15:0];
                    6'h26: spr1_bmp[95:80]   <= data_in[15:0];
                    6'h28: spr1_bmp[111:96]  <= data_in[15:0];
                    6'h2A: spr1_bmp[127:112] <= data_in[15:0];
                    6'h2C: spr1_bmp[143:128] <= data_in[15:0];
                    default: ;
                endcase
            end
        end
    end

    // Register Readback
    always @(*) begin
        case(address)
            6'h00: data_out = {29'h0, control_reg};
            6'h01: data_out = {29'h0, spr0_ctrl};
            6'h02: data_out = {29'h0, spr1_ctrl};
            6'h04: data_out = {16'h0, spr0_yw, spr0_xw};
            6'h06: data_out = {16'h0, spr0_bmp[15:0]};
            6'h08: data_out = {16'h0, spr0_bmp[31:16]};
            6'h0A: data_out = {16'h0, spr0_bmp[47:32]};
            6'h0C: data_out = {16'h0, spr0_bmp[63:48]};
            6'h0E: data_out = {16'h0, spr0_bmp[79:64]};
            6'h10: data_out = {16'h0, spr0_bmp[95:80]};
            6'h12: data_out = {16'h0, spr0_bmp[111:96]};
            6'h14: data_out = {16'h0, spr0_bmp[127:112]};
            6'h16: data_out = {16'h0, spr0_bmp[143:128]};
            6'h1A: data_out = {16'h0, spr1_yw, spr1_xw};
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

    //--- Palette Table: 4 entries × (R,G,B) 2b each
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

    // -----------------------------
    // XGA Timing (1024x768 @60), gated by control_reg[0] (stream enable)
    // -----------------------------
    localparam H_ACTIVE = 1024;
    localparam H_FP     = 24;
    localparam H_SYNC   = 136;
    //localparam H_BP     = 160;
    localparam H_TOTAL  = 1344;
    localparam V_ACTIVE = 768;
    localparam V_FP     = 3;
    localparam V_SYNC   = 6;
    //localparam V_BP     = 29;
    localparam V_TOTAL  = 806;
    reg [10:0] h_cnt;
    reg [9:0]  v_cnt;
    reg        hsync_r;
    reg        vsync_r;
    reg        visible_r;
    reg        last_vsync_irq;
    reg        last_vsync_buf;
    
    always @(posedge clk) begin
    if (!rst_n) begin
        // Timing/logical frame
        h_cnt         <= 0;
        v_cnt         <= 0;
        hsync_r       <= 0;
        vsync_r       <= 0;
        visible_r     <= 0;

        // Sprite coordinate buffer (visible on-screen position)
        spr0_x        <= 8'd0; 
        spr0_y        <= 8'd0;
        spr1_x        <= 8'd0;
        spr1_y        <= 8'd0;

        // VSYNC edge detection for buffering and IRQ
        last_vsync_buf <= 1'b0;
        last_vsync_irq <= 1'b0;

        // Interrupt logic
        irq_flag      <= 1'b0;
    end else begin
        // --- XGA TIMING ---
        if (control_reg[0]) begin
            if (h_cnt == H_TOTAL - 1) begin
                h_cnt <= 0;
                if (v_cnt == V_TOTAL - 1) v_cnt <= 0;
                else v_cnt <= v_cnt + 1;
            end else h_cnt <= h_cnt + 1;
            hsync_r   <= (h_cnt >= (H_ACTIVE+H_FP)) && (h_cnt < (H_ACTIVE+H_FP+H_SYNC));
            vsync_r   <= (v_cnt >= (V_ACTIVE+V_FP)) && (v_cnt < (V_ACTIVE+V_FP+V_SYNC));
            visible_r <= (h_cnt < H_ACTIVE) && (v_cnt < V_ACTIVE);
        end else begin
            hsync_r   <= 0;
            vsync_r   <= 0;
            visible_r <= 0;
        end

        // --- SPRITE POSITION BUFFERING AT VSYNC RISING EDGE ---
        last_vsync_buf <= vsync_r;
        if (~last_vsync_buf && vsync_r) begin
            spr0_x <= spr0_xw;
            spr0_y <= spr0_yw;
            spr1_x <= spr1_xw;
            spr1_y <= spr1_yw;
        end

        // --- INTERRUPT LOGIC ---
        last_vsync_irq <= vsync_r;
        if (~last_vsync_irq && vsync_r && control_reg[1])
            irq_flag <= 1'b1;
        // Clear IRQ: W1C via write to control with bit2
        if (write_any && (address == 6'h00) && data_in[2])
            irq_flag <= 1'b0;
    end
end

    wire [9:0] pix_x = h_cnt[9:0];
    wire [9:0] pix_y = v_cnt[9:0];
    wire video_active = visible_r; 
    wire [7:0] lx = pix_x[9:2];
    wire [7:0] ly = pix_y[9:2];

    // -- Sprite 0 non-flip
    wire s0_in           = (lx >= spr0_x) && (lx < spr0_x+12) && (ly >= spr0_y) && (ly < spr0_y+12);
    wire [3:0] s0_col    = lx - spr0_x;
    wire [3:0] s0_row    = ly - spr0_y;
    wire [7:0] s0_idx_nf = {s0_row, s0_col}; // non-flip index
    wire s0_pixel_nf     = video_active && s0_in && !spr0_ctrl[2] && spr0_bmp[s0_idx_nf];
    
    // -- Sprite 0 flip & mirror (if enabled)
    wire s0_flip         = spr0_ctrl[2];
    wire s0_in_flip      = s0_flip && s0_in;
    wire [3:0] s0_col_f  = 11 - (lx - spr0_x);
    wire [7:0] s0_idx_f  = {s0_row, s0_col_f};
    wire s0_pixel_f      = video_active && s0_in_flip && spr0_bmp[s0_idx_f];
    
    // -- Sprite 0 mirror (shows at x+12 when flip enabled)
    wire s0_m_in         = (lx >= spr0_x+12) && (lx < spr0_x+24) && (ly >= spr0_y) && (ly < spr0_y+12);
    wire [3:0] s0_m_col  = lx - (spr0_x+12);
    wire [7:0] s0_m_idx_nf = {s0_row, s0_m_col};
    wire [3:0] s0_m_col_f  = 11 - s0_m_col;
    wire [7:0] s0_m_idx_f  = {s0_row, s0_m_col_f};
    wire s0_m_pixel_nf  = video_active && s0_m_in && !spr0_ctrl[2] && spr0_bmp[s0_m_idx_nf];
    wire s0_m_pixel_f   = video_active && s0_m_in && s0_flip && spr0_bmp[s0_m_idx_f];
    
    // Combine sprite 0 result (flip, non-flip, mirror)
    wire s0_pixel      = s0_pixel_nf || s0_pixel_f;
    wire s0_m_pixel    = s0_m_pixel_nf || s0_m_pixel_f;
    
    // -- Sprite 1 non-flip
    wire s1_in           = (lx >= spr1_x) && (lx < spr1_x+12) && (ly >= spr1_y) && (ly < spr1_y+12);
    wire [3:0] s1_col    = lx - spr1_x;
    wire [3:0] s1_row    = ly - spr1_y;
    wire [7:0] s1_idx_nf = {s1_row, s1_col}; // non-flip index
    wire s1_pixel_nf     = video_active && s1_in && !spr1_ctrl[2] && spr1_bmp[s1_idx_nf];
    
    // -- Sprite 1 flip & mirror (if enabled)
    wire s1_flip         = spr1_ctrl[2];
    wire s1_in_flip      = s1_flip && s1_in;
    wire [3:0] s1_col_f  = 11 - (lx - spr1_x);
    wire [7:0] s1_idx_f  = {s1_row, s1_col_f};
    wire s1_pixel_f      = video_active && s1_in_flip && spr1_bmp[s1_idx_f];
    
    // -- Sprite 1 mirror (shows at x+12 when flip enabled)
    wire s1_m_in         = (lx >= spr1_x+12) && (lx < spr1_x+24) && (ly >= spr1_y) && (ly < spr1_y+12);
    wire [3:0] s1_m_col  = lx - (spr1_x+12);
    wire [7:0] s1_m_idx_nf = {s1_row, s1_m_col};
    wire [3:0] s1_m_col_f  = 11 - s1_m_col;
    wire [7:0] s1_m_idx_f  = {s1_row, s1_m_col_f};
    wire s1_m_pixel_nf  = video_active && s1_m_in && !spr1_ctrl[2] && spr1_bmp[s1_m_idx_nf];
    wire s1_m_pixel_f   = video_active && s1_m_in && s1_flip && spr1_bmp[s1_m_idx_f];
    
    // Combine sprite 1 result (flip, non-flip, mirror)
    wire s1_pixel      = s1_pixel_nf || s1_pixel_f;
    wire s1_m_pixel    = s1_m_pixel_nf || s1_m_pixel_f;
    
    // -- Palette/color select
    wire [5:0] s0_rgb = get_palette(spr0_ctrl[1:0]);
    wire [5:0] s1_rgb = get_palette(spr1_ctrl[1:0]);

    // wire [5:0] s0_rgb = 6'b111111 ;
    // wire [5:0] s1_rgb = 6'b111111 ;
    
    // -- Output composition, priority: s1 > s0 > mirror0 background
    wire [5:0] final_rgb =
          s1_pixel     ? s1_rgb :
          s1_m_pixel   ? s1_rgb :
          s0_pixel     ? s0_rgb :
          s0_m_pixel   ? s0_rgb :
                        6'b000000;

    assign uo_out = {vsync_r, hsync_r, final_rgb};

    assign user_interrupt = irq_flag;
    wire _unused_ok = &{1'b0, ui_in, data_read_n};

endmodule
