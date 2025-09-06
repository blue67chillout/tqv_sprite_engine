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

        // -------------------------
    // XGA timing generator
    // -------------------------
    localparam H_VISIBLE = 1024, H_FP = 24, H_SYNC = 136, H_BP = 160, H_TOTAL = 1344;
    localparam V_VISIBLE = 768,  V_FP = 3,  V_SYNC = 6,   V_BP = 29,  V_TOTAL = 806;

    reg [10:0] h_cnt;
    reg [9:0]  v_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h_cnt <= 0;
            v_cnt <= 0;
        end else begin
            if (h_cnt == H_TOTAL-1) begin
                h_cnt <= 0;
                if (v_cnt == V_TOTAL-1)
                    v_cnt <= 0;
                else
                    v_cnt <= v_cnt + 1;
            end else begin
                h_cnt <= h_cnt + 1;
            end
        end
    end

    wire hsync = ~((h_cnt >= H_VISIBLE+H_FP) && (h_cnt < H_VISIBLE+H_FP+H_SYNC));
    wire vsync = ~((v_cnt >= V_VISIBLE+V_FP) && (v_cnt < V_VISIBLE+V_FP+V_SYNC));
    wire visible = (h_cnt < H_VISIBLE) && (v_cnt < V_VISIBLE);

    wire [9:0] pix_x = h_cnt[9:0];
    wire [9:0] pix_y = v_cnt[9:0];

    // -------------------------
    // Two sprites, same as before
    // -------------------------
    reg [9:0] spr0_x, spr0_y, spr1_x, spr1_y;
    reg [31:0] spr0_bmp0, spr0_bmp1;
    reg [31:0] spr1_bmp0, spr1_bmp1;

    reg [7:0] control_reg;
    reg irq_flag;
    assign user_interrupt = irq_flag;

    wire [63:0] spr0_bitmap = {spr0_bmp1, spr0_bmp0};
    wire [63:0] spr1_bitmap = {spr1_bmp1, spr1_bmp0};

    // write/read logic identical to previous version
    // (not rewriting here for brevity — copy from last implementation, with address map unchanged)

    // -------------------------
    // VSYNC interrupt
    // -------------------------
    reg last_vsync;
    always @(posedge clk) begin
        if (!rst_n) begin
            last_vsync <= 0;
            irq_flag <= 0;
            control_reg[2] <= 0;
        end else begin
            last_vsync <= vsync;
            if (control_reg[1] && !last_vsync && vsync) begin
                irq_flag <= 1;
                control_reg[2] <= 1;
            end
        end
    end

    // -------------------------
    // Sprite pixel logic
    // -------------------------
    wire spr0_in = (pix_x >= spr0_x && pix_x < spr0_x+8 && pix_y >= spr0_y && pix_y < spr0_y+8);
    wire spr1_in = (pix_x >= spr1_x && pix_x < spr1_x+8 && pix_y >= spr1_y && pix_y < spr1_y+8);

    wire [2:0] spr0_col = pix_x - spr0_x;
    wire [2:0] spr0_row = pix_y - spr0_y;
    wire [2:0] spr1_col = pix_x - spr1_x;
    wire [2:0] spr1_row = pix_y - spr1_y;

    wire spr0_bit = spr0_in ? spr0_bitmap[{spr0_row, spr0_col}] : 1'b0;
    wire spr1_bit = spr1_in ? spr1_bitmap[{spr1_row, spr1_col}] : 1'b0;

    wire draw1 = visible && spr1_bit;
    wire draw0 = visible && ~draw1 && spr0_bit;

    wire [1:0] R = draw1 ? 2'b11 : draw0 ? 2'b10 : 2'b00;
    wire [1:0] G = R;
    wire [1:0] B = R;

    assign uo_out = {vsync, hsync, B, G, R};

    assign data_out = 32'h0;  // fill in same read logic as before
    assign data_ready = 1'b1;

    wire _unused = &{ui_in, data_read_n, 1'b0};

endmodule
