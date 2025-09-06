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

    // -----------------------------
    // write-size decode (we keep them so doc/comments match)
    // -----------------------------
    wire write_8  = (data_write_n == 2'b00);
    wire write_16 = (data_write_n == 2'b01);
    wire write_32 = (data_write_n == 2'b10);
    wire write_any = (data_write_n != 2'b11);

    assign data_ready = 1'b1; // immediate completion for all reads

    // -----------------------------
    // Compact registers
    // -----------------------------
    reg [7:0]  control_reg;   // [0]=stream_enable, [1]=vsync_irq_en, [2]=IRQ flag (readback from irq_flag)
    reg        irq_flag;

    reg [7:0]  spr0_x;
    reg [7:0]  spr0_y;
    reg [7:0]  spr1_x;
    reg [7:0]  spr1_y;

    reg [63:0] spr0_bmp;
    reg [63:0] spr1_bmp;

    // -----------------------------
    // Register write handling
    // CONTROL (addr 0x00) low byte writable any time
    // Other regs: expect 16-bit writes at base addresses; config writes blocked when streaming active
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
            // CONTROL at address 0x00: low byte used; allow clearing IRQ by writing bit2=1
            if (write_any && (address == 6'h00)) begin
                control_reg[1:0] <= data_in[1:0]; // keep only bits 0..1 writable here
                if (data_in[2]) irq_flag <= 1'b0; // W1C
            end

            // Only accept config writes when not streaming (control_reg[0]==0)
            if (!control_reg[0] && write_16) begin
                case (address)
                    // spr0 coords (base address 0x04) : data_in[7:0] -> spr0_x, data_in[15:8] -> spr0_y
                    6'h04: begin
                        spr0_x <= data_in[7:0];
                        spr0_y <= data_in[15:8];
                    end

                    // spr0 bitmap 16-bit words (little-endian mapping)
                    6'h06: spr0_bmp[15:0]  <= data_in[15:0];
                    6'h08: spr0_bmp[31:16] <= data_in[15:0];
                    6'h0A: spr0_bmp[47:32] <= data_in[15:0];
                    6'h0C: spr0_bmp[63:48] <= data_in[15:0];

                    // spr1 coords at 0x0E
                    6'h0E: begin
                        spr1_x <= data_in[7:0];
                        spr1_y <= data_in[15:8];
                    end

                    // spr1 bitmap
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
    // Readback (combinational) - data_out is reg driven in always_comb style above
    // control returns irq_flag reflected into bit2
    // others return 16-bit chunks in low half of data_out
    // -----------------------------
    always @(*) begin
        case (address)
            6'h00: data_out = {24'h0, control_reg | {5'b0, irq_flag, 2'b0}}; // bit2 shows irq_flag

            6'h04: data_out = {16'h0, spr0_y, spr0_x};
            6'h06: data_out = {16'h0, spr0_bmp[15:0]};
            6'h08: data_out = {16'h0, spr0_bmp[31:16]};
            6'h0A: data_out = {16'h0, spr0_bmp[47:32]};
            6'h0C: data_out = {16'h0, spr0_bmp[63:48]};

            6'h0E: data_out = {16'h0, spr1_y, spr1_x};
            6'h10: data_out = {16'h0, spr1_bmp[15:0]};
            6'h12: data_out = {16'h0, spr1_bmp[31:16]};
            6'h14: data_out = {16'h0, spr1_bmp[47:32]};
            6'h16: data_out = {16'h0, spr1_bmp[63:48]};

            default: data_out = 32'h0;
        endcase
    end

    // -----------------------------
    // XGA Timing (1024x768 @60), gated by control_reg[0] (stream enable)
    // -----------------------------
    localparam H_ACTIVE = 1024;
    localparam H_FP     = 24;
    localparam H_SYNC   = 136;
    localparam H_BP     = 160;
    localparam H_TOTAL  = 1344;

    localparam V_ACTIVE = 768;
    localparam V_FP     = 3;
    localparam V_SYNC   = 6;
    localparam V_BP     = 29;
    localparam V_TOTAL  = 806;

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
                // streaming disabled: keep counters frozen and blank outputs
                hsync_r <= 1'b0;
                vsync_r <= 1'b0;
                visible_r <= 1'b0;
            end

            // VSYNC rising detection - set irq_flag if irq enabled
            if (control_reg[1] && (!last_vsync) && vsync_r) begin
                irq_flag <= 1'b1;
            end
            last_vsync <= vsync_r;
        end
    end

    // -----------------------------
    // Rendering: logical 256x192 -> physical 1024x768 (scale 4x)
    // -----------------------------
    wire [9:0] pix_x = h_cnt[9:0];
    wire [9:0] pix_y = v_cnt[9:0];
    wire       video_active = visible_r;

    // logical coords are physical >> 2
    wire [7:0] lx = pix_x[9:2];
    wire [7:0] ly = pix_y[9:2];

    // safe deltas and indices (no multiply)
    wire [7:0] spr0_dx = lx - spr0_x;
    wire [7:0] spr0_dy = ly - spr0_y;
    wire [2:0] spr0_col = spr0_dx[2:0];
    wire [2:0] spr0_row = spr0_dy[2:0];
    wire [5:0] spr0_idx = {spr0_row, spr0_col};

    wire [7:0] spr1_dx = lx - spr1_x;
    wire [7:0] spr1_dy = ly - spr1_y;
    wire [2:0] spr1_col = spr1_dx[2:0];
    wire [2:0] spr1_row = spr1_dy[2:0];
    wire [5:0] spr1_idx = {spr1_row, spr1_col};

    wire spr0_in = (lx >= spr0_x) && (lx < spr0_x + 8) && (ly >= spr0_y) && (ly < spr0_y + 8);
    wire spr1_in = (lx >= spr1_x) && (lx < spr1_x + 8) && (ly >= spr1_y) && (ly < spr1_y + 8);

    // priority + per-sprite pixel
    wire spr1_pixel = video_active && spr1_in && spr1_bmp[spr1_idx];
    wire spr0_pixel = video_active && (~spr1_pixel) && spr0_in && spr0_bmp[spr0_idx];

    wire [1:0] color_lv = spr1_pixel ? 2'b11 : (spr0_pixel ? 2'b10 : 2'b00);

    wire [1:0] R = color_lv;
    wire [1:0] G = color_lv;
    wire [1:0] B = color_lv;

    assign uo_out = { vsync_r, hsync_r, B, G, R };

    assign user_interrupt = irq_flag;

    // tie off truly-unused inputs so lint is quieter (we reference them in _unused)
    wire _unused_ok = &{ 1'b0, ui_in, data_read_n, data_write_n };

endmodule
