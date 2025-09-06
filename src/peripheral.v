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

    // -----------------------
    // XGA timing constants (1024x768 @60 timing)
    // -----------------------
    localparam H_VISIBLE = 1024;
    localparam H_FP      = 24;
    localparam H_SYNC    = 136;
    localparam H_BP      = 160;
    localparam H_TOTAL   = 1344;

    localparam V_VISIBLE = 768;
    localparam V_FP      = 3;
    localparam V_SYNC    = 6;
    localparam V_BP      = 29;
    localparam V_TOTAL   = 806;

    // -----------------------
    // Internal timing counters
    // -----------------------
    reg [10:0] h_cnt;
    reg [9:0]  v_cnt;

    // internal timing signals
    reg last_vsync; // will be updated in the same always block
    wire hsync = (h_cnt >= (H_VISIBLE + H_FP)) && (h_cnt < (H_VISIBLE + H_FP + H_SYNC));
    wire vsync = (v_cnt >= (V_VISIBLE + V_FP)) && (v_cnt < (V_VISIBLE + V_FP + V_SYNC));
    wire visible = (h_cnt < H_VISIBLE) && (v_cnt < V_VISIBLE);
    wire [9:0] pix_x = h_cnt[9:0];
    wire [9:0] pix_y = v_cnt[9:0];

    // -----------------------
    // Registers (driven inside single always block)
    // -----------------------
    reg [9:0]  spr0_x;
    reg [9:0]  spr0_y;
    reg [63:0] spr0_bmp;

    reg [9:0]  spr1_x;
    reg [9:0]  spr1_y;
    reg [63:0] spr1_bmp;

    // control bits: [0]=stream_enable, [1]=vsync_irq_enable, [2]=vsync_irq_flag (W1C)
    reg [2:0] control;

    // convenience decode of write size
    wire write_8  = (data_write_n == 2'b00);
    wire write_16 = (data_write_n == 2'b01);
    wire write_32 = (data_write_n == 2'b10);
    wire write_any = (data_write_n != 2'b11);

    // data_ready always 1 for this simple peripheral
    assign data_ready = 1'b1;

    // user interrupt is mirror of control[2]
    assign user_interrupt = control[2];

    // -----------------------
    // Single synchronous block:
    // - advances h_cnt/v_cnt
    // - applies bus writes (respecting stream_enable)
    // - detects VSYNC rising edge and sets W1C flag if enabled
    // Keeping everything in one block avoids multiple drivers on 'control'
    // -----------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // timing reset
            h_cnt <= 11'd0;
            v_cnt <= 10'd0;
            last_vsync <= 1'b0;
            // sprite resets
            spr0_x <= 10'd0;
            spr0_y <= 10'd0;
            spr0_bmp <= 64'h0;
            spr1_x <= 10'd0;
            spr1_y <= 10'd0;
            spr1_bmp <= 64'h0;
            control <= 3'b000;
        end else begin
            // ----------------
            // advance counters
            // ----------------
            if (h_cnt == H_TOTAL - 1) begin
                h_cnt <= 11'd0;
                if (v_cnt == V_TOTAL - 1)
                    v_cnt <= 10'd0;
                else
                    v_cnt <= v_cnt + 10'd1;
            end else begin
                h_cnt <= h_cnt + 11'd1;
            end

            // capture last_vsync (previous value)
            last_vsync <= vsync;

            // ----------------
            // handle CONTROL writes (always allowed)
            // ----------------
            if (write_any && (address == 6'h08)) begin
                // stream_enable -> control[0]
                if (write_32 || write_16 || write_8) begin
                    control[0] <= data_in[0];
                    control[1] <= data_in[1];
                    // W1C clear for bit2: if CPU writes a 1 to bit2, clear the flag
                    if (data_in[2])
                        control[2] <= 1'b0;
                end
            end

            // ----------------
            // config writes blocked while streaming (control[0]==1)
            // ----------------
            if (!control[0] && write_any) begin
                case (address)
                    // spr0_x (10 bits)
                    6'h00: begin
                        if (write_32) begin
                            spr0_x <= data_in[9:0];
                        end else if (write_16) begin
                            spr0_x <= data_in[9:0];
                        end else if (write_8) begin
                            // update low byte; keep top 2 bits unchanged
                            spr0_x <= {spr0_x[9:8], data_in[7:0]};
                        end
                    end
                    // spr0_y
                    6'h01: begin
                        if (write_32) begin
                            spr0_y <= data_in[9:0];
                        end else if (write_16) begin
                            spr0_y <= data_in[9:0];
                        end else if (write_8) begin
                            spr0_y <= {spr0_y[9:8], data_in[7:0]};
                        end
                    end
                    // spr0_bmp low 32 bits
                    6'h02: begin
                        if (write_32) begin
                            spr0_bmp[31:0] <= data_in;
                        end else if (write_16) begin
                            spr0_bmp[15:0] <= data_in[15:0];
                        end else if (write_8) begin
                            spr0_bmp[7:0] <= data_in[7:0];
                        end
                    end
                    // spr0_bmp high 32 bits
                    6'h03: begin
                        if (write_32) begin
                            spr0_bmp[63:32] <= data_in;
                        end else if (write_16) begin
                            spr0_bmp[47:32] <= data_in[15:0]; // writing low half of this word -> map into bits [47:32]
                        end else if (write_8) begin
                            spr0_bmp[39:32] <= data_in[7:0]; // map to the low byte of the high word (keeping ordering deterministic)
                        end
                    end

                    // spr1_x
                    6'h04: begin
                        if (write_32) begin
                            spr1_x <= data_in[9:0];
                        end else if (write_16) begin
                            spr1_x <= data_in[9:0];
                        end else if (write_8) begin
                            spr1_x <= {spr1_x[9:8], data_in[7:0]};
                        end
                    end
                    // spr1_y
                    6'h05: begin
                        if (write_32) begin
                            spr1_y <= data_in[9:0];
                        end else if (write_16) begin
                            spr1_y <= data_in[9:0];
                        end else if (write_8) begin
                            spr1_y <= {spr1_y[9:8], data_in[7:0]};
                        end
                    end
                    // spr1_bmp low 32 bits
                    6'h06: begin
                        if (write_32) begin
                            spr1_bmp[31:0] <= data_in;
                        end else if (write_16) begin
                            spr1_bmp[15:0] <= data_in[15:0];
                        end else if (write_8) begin
                            spr1_bmp[7:0] <= data_in[7:0];
                        end
                    end
                    // spr1_bmp high 32 bits
                    6'h07: begin
                        if (write_32) begin
                            spr1_bmp[63:32] <= data_in;
                        end else if (write_16) begin
                            spr1_bmp[47:32] <= data_in[15:0];
                        end else if (write_8) begin
                            spr1_bmp[39:32] <= data_in[7:0];
                        end
                    end
                    default: begin
                        // ignore other addresses
                    end
                endcase
            end // config writes

            // ----------------
            // VSYNC rising edge detection (set W1C flag) — do this after writes so CPU can enable in same cycle if needed
            // ----------------
            if (control[1] && (!last_vsync) && vsync) begin
                control[2] <= 1'b1; // set IRQ flag (W1C)
            end
            // note: CONTROL[2] cleared by CPU writing bit2 = 1 at address 0x08 (handled above)
        end
    end // always

    // -----------------------
    // Readback (combinational)
    // -----------------------
    reg [31:0] data_out_r;
    always @(*) begin
        case (address)
            6'h00: data_out_r = {22'b0, spr0_x};
            6'h01: data_out_r = {22'b0, spr0_y};
            6'h02: data_out_r = spr0_bmp[31:0];
            6'h03: data_out_r = spr0_bmp[63:32];
            6'h04: data_out_r = {22'b0, spr1_x};
            6'h05: data_out_r = {22'b0, spr1_y};
            6'h06: data_out_r = spr1_bmp[31:0];
            6'h07: data_out_r = spr1_bmp[63:32];
            6'h08: data_out_r = {29'b0, control};
            default: data_out_r = 32'h0;
        endcase
    end
    assign data_out = data_out_r;

    // -----------------------
    // Sprite evaluation (combinational)
    // - keep width-safe arithmetic; index bits explicitly sliced
    // -----------------------
    // sprite0
    wire spr0_in_x = (pix_x >= spr0_x) && (pix_x < (spr0_x + 10'd8));
    wire spr0_in_y = (pix_y >= spr0_y) && (pix_y < (spr0_y + 10'd8));
    wire spr0_hit  = visible && spr0_in_x && spr0_in_y;
    wire [9:0] spr0_dx = pix_x - spr0_x;
    wire [9:0] spr0_dy = pix_y - spr0_y;
    wire [2:0] spr0_col = spr0_dx[2:0];
    wire [2:0] spr0_row = spr0_dy[2:0];
    wire [5:0] spr0_bitidx = {spr0_row, spr0_col};
    wire spr0_pixel = spr0_hit ? spr0_bmp[spr0_bitidx] : 1'b0;

    // sprite1
    wire spr1_in_x = (pix_x >= spr1_x) && (pix_x < (spr1_x + 10'd8));
    wire spr1_in_y = (pix_y >= spr1_y) && (pix_y < (spr1_y + 10'd8));
    wire spr1_hit  = visible && spr1_in_x && spr1_in_y;
    wire [9:0] spr1_dx = pix_x - spr1_x;
    wire [9:0] spr1_dy = pix_y - spr1_y;
    wire [2:0] spr1_col = spr1_dx[2:0];
    wire [2:0] spr1_row = spr1_dy[2:0];
    wire [5:0] spr1_bitidx = {spr1_row, spr1_col};
    wire spr1_pixel = spr1_hit ? spr1_bmp[spr1_bitidx] : 1'b0;

    // final draw: sprite1 over sprite0
    wire draw1 = spr1_pixel;
    wire draw0 = (~draw1) && spr0_pixel;

    // color levels (2 bits per channel)
    wire [1:0] color_lv = draw1 ? 2'b11 : (draw0 ? 2'b10 : 2'b00);
    wire [1:0] R = visible ? color_lv : 2'b00;
    wire [1:0] G = visible ? color_lv : 2'b00;
    wire [1:0] B = visible ? color_lv : 2'b00;

    assign uo_out = {vsync, hsync, B, G, R};

    // avoid unused warnings
    wire _unused = &{ui_in, data_read_n[0]};

endmodule
