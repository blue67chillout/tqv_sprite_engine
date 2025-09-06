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
    // write-size decode
    // -----------------------------
    wire write_8  = (data_write_n == 2'b00);
    wire write_16 = (data_write_n == 2'b01);
    wire write_32 = (data_write_n == 2'b10);
    wire write_any = (data_write_n != 2'b11);

    assign data_ready = 1'b1; // immediate

    // -----------------------------
    // Registers (compact)
    // -----------------------------
    reg [7:0]  control_reg;   // [0]=stream_enable, [1]=vsync_irq_en, [2]=IRQ flag (readback from irq_flag)
    reg        irq_flag;      // set on vsync rising edge, cleared by CPU via write to control_reg bit2

    // logical coords (8-bit)
    reg [7:0]  spr0_x;
    reg [7:0]  spr0_y;
    reg [7:0]  spr1_x;
    reg [7:0]  spr1_y;

    // 8x8 bitmaps (row-major), LSB = row0,col0
    reg [63:0] spr0_bmp;
    reg [63:0] spr1_bmp;

    // -----------------------------
    // Register write handling
    // CONTROL (addr 0x00) always writable (low byte used)
    // Other regs: expect 16-bit writes at base addresses (unpack as requested)
    // -----------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            control_reg <= 8'h00;
            irq_flag    <= 1'b0;

            spr0_x <= 8'h00; spr0_y <= 8'h00;
            spr1_x <= 8'h00; spr1_y <= 8'h00;
            spr0_bmp <= 64'h0;
            spr1_bmp <= 64'h0;
        end else begin
            // CONTROL base at address 0x00: accept any size, low byte used
            if (write_any && (address == 6'h00)) begin
                // write low byte into control_reg
                control_reg[1:0] <= data_in[1:0]; // only bits 0..1 are writable here
                // W1C: if CPU writes bit2=1, clear irq_flag
                if (data_in[2])
                    irq_flag <= 1'b0;
            end

            // Sprite config: accept only 16-bit writes at base addresses (as you requested)
            // spr0 coords at 0x04: data_in[7:0] -> spr0_x, data_in[15:8] -> spr0_y
            if (write_16 && (address == 6'h04) && !control_reg[0]) begin
                spr0_x <= data_in[7:0];
                spr0_y <= data_in[15:8];
            end

            // spr0 bitmap words (16-bit each) stored into 64-bit in little-endian halves
            // addresses: 0x06 -> bits[15:0], 0x08 -> [31:16], 0x0A -> [47:32], 0x0C -> [63:48]
            if (write_16 && !control_reg[0]) begin
                case (address)
                    6'h06: spr0_bmp[15:0]  <= data_in[15:0];
                    6'h08: spr0_bmp[31:16] <= data_in[15:0];
                    6'h0A: spr0_bmp[47:32] <= data_in[15:0];
                    6'h0C: spr0_bmp[63:48] <= data_in[15:0];
                    default: ;
                endcase
            end

            // spr1 coords at 0x0E
            if (write_16 && (address == 6'h0E) && !control_reg[0]) begin
                spr1_x <= data_in[7:0];
                spr1_y <= data_in[15:8];
            end

            // spr1 bitmap words
            if (write_16 && !control_reg[0]) begin
                case (address)
                    6'h10: spr1_bmp[15:0]  <= data_in[15:0];
                    6'h12: spr1_bmp[31:16] <= data_in[15:0];
                    6'h14: spr1_bmp[47:32] <= data_in[15:0];
                    6'h16: spr1_bmp[63:48] <= data_in[15:0];
                    default: ;
                endcase
            end
        end
    end

    // -----------------------------
    // Readback (combinational)
    // control read should reflect irq_flag in bit2
    // others return lower 16 bits valid
    // -----------------------------
    always @(*) begin
        case (address)
            6'h00: begin
                // construct readable control byte with irq_flag reflected into bit2
                data_out = 32'h0;
                data_out[7:0] = { control_reg[7:3], irq_flag, control_reg[1:0] };
            end

            6'h04: begin
                data_out = 32'h0;
                data_out[15:0] = { spr0_y, spr0_x }; // high byte = y, low byte = x
            end

            6'h06: begin data_out = {16'h0, spr0_bmp[15:0]}; end
            6'h08: begin data_out = {16'h0, spr0_bmp[31:16]}; end
            6'h0A: begin data_out = {16'h0, spr0_bmp[47:32]}; end
            6'h0C: begin data_out = {16'h0, spr0_bmp[63:48]}; end

            6'h0E: begin data_out = 32'h0; data_out[15:0] = { spr1_y, spr1_x }; end

            6'h10: begin data_out = {16'h0, spr1_bmp[15:0]}; end
            6'h12: begin data_out = {16'h0, spr1_bmp[31:16]}; end
            6'h14: begin data_out = {16'h0, spr1_bmp[47:32]}; end
            6'h16: begin data_out = {16'h0, spr1_bmp[63:48]}; end

            default: data_out = 32'h0;
        endcase
    end

    // -----------------------------
    // XGA Timing (1024x768 @60) — gated by stream_enable (control_reg[0])
    // Also detect VSYNC rising edge to set irq_flag when enabled
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
    reg        hsync_r;
    reg        vsync_r;
    reg        visible_r;
    reg        last_vsync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h_cnt <= 11'd0;
            v_cnt <= 10'd0;
            hsync_r <= 1'b0;
            vsync_r <= 1'b0;
            visible_r <= 1'b0;
            last_vsync <= 1'b0;
        end else begin
            if (control_reg[0]) begin
                // run counters when streaming enabled
                if (h_cnt == H_TOTAL - 1) begin
                    h_cnt <= 11'd0;
                    if (v_cnt == V_TOTAL - 1)
                        v_cnt <= 10'd0;
                    else
                        v_cnt <= v_cnt + 10'd1;
                end else begin
                    h_cnt <= h_cnt + 11'd1;
                end

                hsync_r <= (h_cnt >= (H_ACTIVE + H_FP)) && (h_cnt < (H_ACTIVE + H_FP + H_SYNC));
                vsync_r <= (v_cnt >= (V_ACTIVE + V_FP)) && (v_cnt < (V_ACTIVE + V_FP + V_SYNC));
                visible_r <= (h_cnt < H_ACTIVE) && (v_cnt < V_ACTIVE);
            end else begin
                // streaming disabled -> hold idle syncs and freeze counters
                h_cnt <= h_cnt;
                v_cnt <= v_cnt;
                hsync_r <= 1'b0;
                vsync_r <= 1'b0;
                visible_r <= 1'b0;
            end

            // VSYNC rising detection & IRQ set (separate from control_reg writes)
            if (control_reg[1] && (!last_vsync) && vsync_r) begin
                irq_flag <= 1'b1;
            end
            last_vsync <= vsync_r;
        end
    end

    // -----------------------------
    // Rendering: scale logical 256x192 -> physical 1024x768 by 4x
    // compute logical pixel coords (lx,ly) by dividing physical coords by 4
    // no multiply in the hot path; index = {row, col}
    // -----------------------------
    // physical pixel coords derived from counters
    wire [9:0] pix_x = h_cnt[9:0];
    wire [9:0] pix_y = v_cnt[9:0];
    wire video_active = visible_r;

    // logical coordinates (divide by 4 = shift right 2)
    wire [7:0] lx = pix_x[9:2];
    wire [7:0] ly = pix_y[9:2];

    // compute deltas safely then slice for 3-bit indices
    wire [7:0] spr0_dx = lx - spr0_x; // valid only if inside box
    wire [7:0] spr0_dy = ly - spr0_y;
    wire [2:0] spr0_col = spr0_dx[2:0];
    wire [2:0] spr0_row = spr0_dy[2:0];
    wire [5:0] spr0_idx = {spr0_row, spr0_col};

    wire [7:0] spr1_dx = lx - spr1_x;
    wire [7:0] spr1_dy = ly - spr1_y;
    wire [2:0] spr1_col = spr1_dx[2:0];
    wire [2:0] spr1_row = spr1_dy[2:0];
    wire [5:0] spr1_idx = {spr1_row, spr1_col};

    wire spr0_in = (lx >= spr0_x) && (lx < (spr0_x + 8)) && (ly >= spr0_y) && (ly < (spr0_y + 8));
    wire spr1_in = (lx >= spr1_x) && (lx < (spr1_x + 8)) && (ly >= spr1_y) && (ly < (spr1_y + 8));

    reg pixel_on;
    always @(*) begin
        pixel_on = 1'b0;
        if (video_active) begin
            // sprite1 has priority over sprite0; check sprite1 first
            if (spr1_in) begin
                pixel_on = spr1_bmp[spr1_idx];
            end else if (spr0_in) begin
                pixel_on = spr0_bmp[spr0_idx];
            end
        end
    end

    // color levels: sprite1 -> full (11), sprite0 -> mid (10). We simplified to single pixel_on:
    // to keep the earlier intensity difference we'd need per-sprite detection; do that now:
    wire spr1_pixel = video_active && spr1_in && spr1_bmp[spr1_idx];
    wire spr0_pixel = video_active && (~spr1_pixel) && spr0_in && spr0_bmp[spr0_idx];

    wire [1:0] color_lv = spr1_pixel ? 2'b11 : (spr0_pixel ? 2'b10 : 2'b00);

    wire [1:0] R = color_lv;
    wire [1:0] G = color_lv;
    wire [1:0] B = color_lv;

    // pack into uo_out: {vsync, hsync, B[1:0], G[1:0], R[1:0]}
    assign uo_out = { vsync_r, hsync_r, B, G, R };

    // user interrupt output mirrors irq_flag
    assign user_interrupt = irq_flag;

    // tie-off unused signals to avoid lint warnings
    wire _unused = &{ ui_in, data_read_n[0], data_write_n[0] };


endmodule
