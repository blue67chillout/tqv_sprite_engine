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
    wire write_any = (data_write_n != 2'b11);
    assign data_ready = 1'b1;

    // Registers
    reg [2:0] control_reg;
    reg       spr0_ctrl, spr1_ctrl; // [0]=flip
    reg [7:0] spr0_x, spr0_y, spr1_x, spr1_y;
    reg [63:0] spr0_bmp, spr1_bmp;
    reg  irq_flag;
    // Shadow/write regs for buffering
    reg [7:0] spr0_xw, spr0_yw, spr1_xw, spr1_yw;

    // Register Write Handling
    always @(posedge clk) begin
        if (!rst_n) begin
            control_reg <= 3'd0;
            spr0_ctrl   <= 1'd0;
            spr1_ctrl   <= 1'd0;
            spr0_xw <= 8'd0; spr0_yw <= 8'd0;
            spr1_xw <= 8'd0; spr1_yw <= 8'd0;
            spr0_bmp <= 64'd0;
            spr1_bmp <= 64'd0;
        end else begin
            if (write_any && (address == 6'h00))
                control_reg <= data_in[2:0];
            if (write_any && (address == 6'h01))
                spr0_ctrl <= data_in[0];
            if (write_any && (address == 6'h02))
                spr1_ctrl <= data_in[0];
            if (!control_reg[0] && write_16) begin
                case (address)
                    6'h04: begin
                        spr0_xw <= data_in[7:0];
                        spr0_yw <= data_in[15:8];
                    end
                    6'h06: spr0_bmp[15:0]  <= data_in[15:0];
                    6'h08: spr0_bmp[31:16] <= data_in[15:0];
                    6'h0A: spr0_bmp[47:32] <= data_in[15:0];
                    6'h0C: spr0_bmp[63:48] <= data_in[15:0];
                    6'h10: begin
                        spr1_xw <= data_in[7:0];
                        spr1_yw <= data_in[15:8];
                    end
                    6'h12: spr1_bmp[15:0]  <= data_in[15:0];
                    6'h14: spr1_bmp[31:16] <= data_in[15:0];
                    6'h16: spr1_bmp[47:32] <= data_in[15:0];
                    6'h18: spr1_bmp[63:48] <= data_in[15:0];
                    default: ;
                endcase
            end
        end
    end

    // Register Readback
    always @(*) begin
        case(address)
            6'h00: data_out = {29'd0, control_reg};
            6'h01: data_out = {31'd0, spr0_ctrl};
            6'h02: data_out = {31'd0, spr1_ctrl};
            6'h04: data_out = {16'd0, spr0_yw, spr0_xw};
            6'h06: data_out = {16'd0, spr0_bmp[15:0]};
            6'h08: data_out = {16'd0, spr0_bmp[31:16]};
            6'h0A: data_out = {16'd0, spr0_bmp[47:32]};
            6'h0C: data_out = {16'd0, spr0_bmp[63:48]};
            6'h10: data_out = {16'd0, spr1_yw, spr1_xw};
            6'h12: data_out = {16'd0, spr1_bmp[15:0]};
            6'h14: data_out = {16'd0, spr1_bmp[31:16]};
            6'h16: data_out = {16'd0, spr1_bmp[47:32]};
            6'h18: data_out = {16'd0, spr1_bmp[63:48]};
            default: data_out = 32'd0;
        endcase
    end

    // XGA Timing and Rendering
    localparam H_ACTIVE = 1024, H_FP = 24, H_SYNC = 136, H_TOTAL = 1344;
    localparam V_ACTIVE = 768,  V_FP = 3,  V_SYNC = 6,  V_TOTAL = 806;
    reg [10:0] h_cnt;
    reg [9:0]  v_cnt;
    reg        hsync_r, vsync_r, visible_r;
    reg        last_vsync_irq, last_vsync_buf;
    always @(posedge clk) begin
        if (!rst_n) begin
            h_cnt     <= 0; v_cnt <= 0;
            hsync_r   <= 0; vsync_r <= 0; visible_r <= 0;
            spr0_x    <= 8'd0; spr0_y <= 8'd0;
            spr1_x    <= 8'd0; spr1_y <= 8'd0;
            last_vsync_buf <= 1'b0;
            last_vsync_irq <= 1'b0;
            irq_flag <= 1'b0;
        end else begin
            // VGA timing
            if (control_reg[0]) begin
                if (h_cnt == H_TOTAL-1) begin
                    h_cnt <= 0;
                    if (v_cnt == V_TOTAL-1)
                        v_cnt <= 0;
                    else
                        v_cnt <= v_cnt + 1;
                end else h_cnt <= h_cnt + 1;
                hsync_r   <= (h_cnt >= (H_ACTIVE+H_FP)) && (h_cnt < (H_ACTIVE+H_FP+H_SYNC));
                vsync_r   <= (v_cnt >= (V_ACTIVE+V_FP)) && (v_cnt < (V_ACTIVE+V_FP+V_SYNC));
                visible_r <= (h_cnt < H_ACTIVE) && (v_cnt < V_ACTIVE);
            end else begin
                hsync_r   <= 0;
                vsync_r   <= 0;
                visible_r <= 0;
            end
            // Sprite buffering on vsync
            last_vsync_buf <= vsync_r;
            if (~last_vsync_buf && vsync_r) begin
                spr0_x <= spr0_xw; spr0_y <= spr0_yw;
                spr1_x <= spr1_xw; spr1_y <= spr1_yw;
            end
            // IRQ
            last_vsync_irq <= vsync_r;
            if (~last_vsync_irq && vsync_r && control_reg[1])
                irq_flag <= 1'b1;
            if (write_any && (address==6'h00) && data_in[2])
                irq_flag <= 1'b0;
        end
    end
    assign user_interrupt = irq_flag;

    // Rendering
    wire [9:0] pix_x = h_cnt;
    wire [9:0] pix_y = v_cnt;
    wire video_active = visible_r;
    wire [7:0] lx = pix_x[9:2];
    wire [7:0] ly = pix_y[9:2];

    // -- Sprite 0 non-flip
    wire s0_in       = (lx >= spr0_x) && (lx < spr0_x+8) && (ly >= spr0_y) && (ly < spr0_y+8);
    wire [2:0] s0_col  = lx - spr0_x;
    wire [2:0] s0_row  = ly - spr0_y;
    wire [5:0] s0_idx_nf = {s0_row, s0_col};
    wire s0_pixel_nf = video_active && s0_in && !spr0_ctrl && spr0_bmp[s0_idx_nf];
    // -- Sprite 0 flip
    wire s0_flip     = spr0_ctrl;
    wire s0_in_flip  = s0_flip && s0_in;
    wire [2:0] s0_col_f = 7 - (lx - spr0_x);
    wire [5:0] s0_idx_f = {s0_row, s0_col_f};
    wire s0_pixel_f  = video_active && s0_in_flip && spr0_bmp[s0_idx_f];
    // -- Sprite 0 mirror
    wire s0_m_in     = (lx >= spr0_x+8) && (lx < spr0_x+16) && (ly >= spr0_y) && (ly < spr0_y+8);
    wire [2:0] s0_m_col = lx - (spr0_x + 8);
    wire [5:0] s0_m_idx_nf = {s0_row, s0_m_col};
    wire [2:0] s0_m_col_f  = 7 - s0_m_col;
    wire [5:0] s0_m_idx_f  = {s0_row, s0_m_col_f};
    wire s0_m_pixel_nf = video_active && s0_m_in && !spr0_ctrl && spr0_bmp[s0_m_idx_nf];
    wire s0_m_pixel_f  = video_active && s0_m_in && s0_flip && spr0_bmp[s0_m_idx_f];
    wire s0_pixel = s0_pixel_nf || s0_pixel_f;
    wire s0_m_pixel = s0_m_pixel_nf || s0_m_pixel_f;

    // -- Sprite 1 non-flip/flip/mirror
    wire s1_in       = (lx >= spr1_x) && (lx < spr1_x+8) && (ly >= spr1_y) && (ly < spr1_y+8);
    wire [2:0] s1_col  = lx - spr1_x;
    wire [2:0] s1_row  = ly - spr1_y;
    wire [5:0] s1_idx_nf = {s1_row, s1_col};
    wire s1_pixel_nf = video_active && s1_in && !spr1_ctrl && spr1_bmp[s1_idx_nf];
    wire s1_flip     = spr1_ctrl;
    wire s1_in_flip  = s1_flip && s1_in;
    wire [2:0] s1_col_f = 7 - (lx - spr1_x);
    wire [5:0] s1_idx_f = {s1_row, s1_col_f};
    wire s1_pixel_f  = video_active && s1_in_flip && spr1_bmp[s1_idx_f];
    wire s1_m_in     = (lx >= spr1_x+8) && (lx < spr1_x+16) && (ly >= spr1_y) && (ly < spr1_y+8);
    wire [2:0] s1_m_col = lx - (spr1_x + 8);
    wire [5:0] s1_m_idx_nf = {s1_row, s1_m_col};
    wire [2:0] s1_m_col_f  = 7 - s1_m_col;
    wire [5:0] s1_m_idx_f  = {s1_row, s1_m_col_f};
    wire s1_m_pixel_nf = video_active && s1_m_in && !spr1_ctrl && spr1_bmp[s1_m_idx_nf];
    wire s1_m_pixel_f  = video_active && s1_m_in && s1_flip && spr1_bmp[s1_m_idx_f];
    wire s1_pixel = s1_pixel_nf || s1_pixel_f;
    wire s1_m_pixel = s1_m_pixel_nf || s1_m_pixel_f;

    wire [5:0] s0_rgb = 6'b111111; // All sprites are white unless palette is used
    wire [5:0] s1_rgb = 6'b111111;

    // Output composition (priority)
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
