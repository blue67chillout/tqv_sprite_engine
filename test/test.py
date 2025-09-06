# SPDX-FileCopyrightText: © 2025 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge
from tqv import TinyQV

PERIPHERAL_NUM = 0

def xga_pix(row, col, scale=4):
    # Returns all logical (h,v) positions covered by a single sprite pixel at (row,col), scaled up.
    px = []
    for y in range(row*scale, (row+1)*scale):
        for x in range(col*scale, (col+1)*scale):
            px.append((y, x))
    return px

@cocotb.test()
async def test_project(dut):
    dut._log.info("Starting testbench for dual 8x8 sprite XGA peripheral.")

    clock = Clock(dut.clk, 10, units="ns")  # 100 MHz - adjust if needed
    cocotb.start_soon(clock.start())

    tqv = TinyQV(dut, PERIPHERAL_NUM)

    # --- Reset and idle state ---
    await tqv.reset()
    await ClockCycles(dut.clk, 5)
    assert (await tqv.read_word_reg(0)) == 0  # control reg

    # === Test register writes and reads ===
    await tqv.write_word_reg(0x00, 0x00000001)     # enable stream
    await tqv.write_byte_reg(0x01, 0b0)           # spr0_ctrl: no flip
    await tqv.write_byte_reg(0x02, 0b0)           # spr1_ctrl: no flip

    # Sprite0: place at (20,45). Sprite1: at (30,80).
    await tqv.write_hword_reg(0x04, (20<<8)|45)   # spr0_y,spr0_x
    await tqv.write_hword_reg(0x10, (30<<8)|80)   # spr1_y,spr1_x

    # Sprite0: write a plus in bits [4, 28, 32] (center, up, left)
    plus = 0
    plus |= (1 << (3*8 + 3))  # pixel at row=3, col=3 (center)
    plus |= (1 << (0*8 + 3))  # up
    plus |= (1 << (3*8 + 0))  # left
    await tqv.write_hword_reg(0x06, plus & 0xFFFF)
    await tqv.write_hword_reg(0x08, (plus >> 16) & 0xFFFF)
    await tqv.write_hword_reg(0x0A, (plus >> 32) & 0xFFFF)
    await tqv.write_hword_reg(0x0C, (plus >> 48) & 0xFFFF)

    # Sprite1: a single dot at (4,6)
    dot = 1 << (4*8 + 6)
    await tqv.write_hword_reg(0x12, dot & 0xFFFF)
    await tqv.write_hword_reg(0x14, (dot >> 16) & 0xFFFF)
    await tqv.write_hword_reg(0x16, (dot >> 32) & 0xFFFF)
    await tqv.write_hword_reg(0x18, (dot >> 48) & 0xFFFF)

    # === Wait for VSYNC to register buffered sprite positions ===
    # At minimum, need one XGA frame, which is about 1344*806 clk cycles.
    await ClockCycles(dut.clk, 1344*10)  # cover at least 10 lines

    # === Verify sprite placement for plus (Sprite 0) ===
    # You'll likely want a pixel sample from the output during a time when the
    # scaled position equals the target.
    scale = 4
    center_sy = 20 * scale + 3 * scale
    center_sx = 45 * scale + 3 * scale
    # hsync/vsync are not strictly needed for data but you could mask for output

    found = False
    # We'll scan one frame for the expected pattern
    for _ in range(1344 * 806):
        await RisingEdge(dut.clk)
        # Decode PMOD output (uo_out): top 2 bits are sync, rest is "pixel"
        rgb = dut.uo_out.value.integer & 0x3F
        # You're outputting white for set pixels
        if rgb == 0x3F:
            # Clock counters recovered from your DUT
            h = dut.h_cnt.value.integer
            v = dut.v_cnt.value.integer
            # Only check sprite center
            if v in range(center_sy, center_sy+scale) and h in range(center_sx, center_sx+scale):
                found = True
                break
    assert found, "Sprite 0 plus center not rendered at expected XGA location!"

    # === Test flipping (Sprite 0), mirrored placement ===
    await tqv.write_byte_reg(0x01, 0b1)     # spr0_ctrl: flip enable
    await ClockCycles(dut.clk, 1344*806)
    # For a horizontal flip, the leftmost set pixel should show up mirrored
    expected_flipped_sx = 45 * scale + (7-0) * scale  # originally col=0, now col=7
    found_flip = False
    for _ in range(1344 * 806):
        await RisingEdge(dut.clk)
        rgb = dut.uo_out.value.integer & 0x3F
        h = dut.h_cnt.value.integer
        v = dut.v_cnt.value.integer
        if rgb == 0x3F and v in range(center_sy, center_sy+scale) and h in range(expected_flipped_sx, expected_flipped_sx+scale):
            found_flip = True
            break
    assert found_flip, "Sprite 0 flipped pixel not rendered at expected mirrored XGA location!"

    # === Test interrupt ===
    # Enable IRQ, wait for vsync
    await tqv.write_word_reg(0x00, 0b11)   # [1]=irq_enable, [0]=stream_enable, [2]=irq clear
    await ClockCycles(dut.clk, 1344*2)
    assert await tqv.is_interrupt_asserted(), "IRQ not asserted after vsync!"

    # Clear IRQ using W1C (write 1 to bit2)
    await tqv.write_word_reg(0x00, 0b100) # should clear irqs
    await ClockCycles(dut.clk, 5)
    assert not await tqv.is_interrupt_asserted(), "IRQ did not clear with W1C!"

    # === Timing: check frame counter increments predictably (bonus) ===
    old_vs = int(dut.vsync_r.value)
    vcnt = 0
    for _ in range(10000):
        await RisingEdge(dut.clk)
        new_vs = int(dut.vsync_r.value)
        if not old_vs and new_vs:
            vcnt += 1
        old_vs = new_vs
    # There should be at least one VSYNC edge over 1344*806 clocks @ 64MHz
    assert vcnt >= 1, "VSYNC was not produced in XGA timing."

    dut._log.info("All XGA sprite and peripheral features verified!")
