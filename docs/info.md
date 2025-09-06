# Tiny Sprite Engine

Author: IRIS Labs Hardware (NITK)

Peripheral index: nn

---

## What it does

The **Tiny Sprite Engine** is a 2-sprite graphics peripheral for the TinyQV SoC.  
It generates **1024×768 @ 60 Hz video output** with two independent 8×8 sprites.  

- Each sprite has configurable `spr*_x` / `spr*_y` position registers.  
- Each sprite has a 64-bit bitmap stored in `spr*_bmp` registers.  
- Logical resolution is 256×192, scaled ×4 to physical 1024×768.  
- `spr1_bmp` has priority over `spr0_bmp`.  
- Provides a VSYNC interrupt via `irq_flag`.  
- Outputs 6-bit grayscale RGB + HSYNC/VSYNC on `uo_out`.  

---

## Register map

| Address | Register         | Access | Description                                                                 |
|---------|------------------|--------|-----------------------------------------------------------------------------|
| 0x00    | control_reg      | R/W    | **[0]** stream_enable<br>**[1]** vsync_irq_en<br>**[2]** irq_flag (set on VSYNC rising, write-1-to-clear) |
| 0x04    | spr0_x           | R/W    | X co-ordinate of the first sprite                                           |
| 0x05    | spr0_y           | R/W    | Y co-ordinate of the first sprite                                                                    |
| 0x06    | spr0_bmp[15:0]   | R/W    | Lower 16 bits of `spr0_bmp`                                                 |
| 0x08    | spr0_bmp[31:16]  | R/W    | Bits [31:16] of `spr0_bmp`                                                  |
| 0x0A    | spr0_bmp[47:32]  | R/W    | Bits [47:32] of `spr0_bmp`                                                  |
| 0x0C    | spr0_bmp[63:48]  | R/W    | Upper 16 bits of `spr0_bmp`                                                 |
| 0x0E    | spr1_x           | R/W    | X co-ordinate of the second sprite                                          |
| 0x0F    | spr1_y           | R/W    | Y co-ordinate of the second sprite                                          |
| 0x10    | spr1_bmp[15:0]   | R/W    | Lower 16 bits of `spr1_bmp`                                                 |
| 0x12    | spr1_bmp[31:16]  | R/W    | Bits [31:16] of `spr1_bmp`                                                  |
| 0x14    | spr1_bmp[47:32]  | R/W    | Bits [47:32] of `spr1_bmp`                                                  |
| 0x16    | spr1_bmp[63:48]  | R/W    | Upper 16 bits of `spr1_bmp`                                                 |

**Notes:**  
- Writes to sprite registers are only allowed when `control_reg[0]` (stream_enable) = 0.  
- Reads always complete immediately (`data_ready = 1`).  

---

## How to test

1. Reset the system (keep streaming disabled).  
2. Write sprite coordinates into `spr0_x` / `spr0_y` and `spr1_x`/`spr1_y`.  
3. Write the 64-bit bitmaps into the corresponding `spr*_bmp` registers in 16-bit chunks.  
4. Enable streaming by setting `control_reg[0] = 1`.  
5. Connect a VGA/XGA monitor and observe output.  
6. (Optional) Enable interrupts by setting `control_reg[1] = 1`.  
   - IRQ is signaled on `user_interrupt` whenever VSYNC rises.  
   - Clear by writing `control_reg[2] = 1`.  

---

## External hardware

- **VGA/XGA monitor** or capture device.  
- Output signals on `uo_out`:  
  - `uo_out[7]` = VSYNC  
  - `uo_out[6]` = HSYNC  
  - `uo_out[5:4]` = Blue (2 bits)  
  - `uo_out[3:2]` = Green (2 bits)  
  - `uo_out[1:0]` = Red (2 bits)  
